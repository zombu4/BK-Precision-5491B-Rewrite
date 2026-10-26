"""Compare the flash tables in src/tables.c with the original image.

Reads src/tables.c, parses every table definition, and checks it against the bytes of the original
firmware's flash image at the address the table was taken from. The image is NOT part of this repository:
the user supplies their own dump of a 5491B's flash (131072 bytes) as the one argument. Prints PASS or
FAIL per table and a total; the exit status is non-zero when any table fails.

Run with:  python -I tools/check_tables.py PATH_TO_FLASH_DUMP.bin
"""

import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TABLES_C = ROOT / "src" / "tables.c"

FUNC = {
    "FUNC_DCV": 0,
    "FUNC_ACV": 1,
    "FUNC_DCA": 2,
    "FUNC_ACA": 3,
    "FUNC_OHM": 4,
    "FUNC_FREQ": 5,
    "FUNC_PERIOD": 6,
    "FUNC_CONT": 7,
    "FUNC_DIODE": 8,
    "FUNC_ACDCV": 9,
    "FUNC_ACDCI": 10,
}
FUNC_COUNT = 11
RANGE_COLUMNS = 6
TRIPLE = r"\{\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\}"


class Mismatch(Exception):
    """Raised when a table does not match the image."""


def require(condition, message):
    if not condition:
        raise Mismatch(message)


def search_or_fail(pattern, text, message):
    """re.search that raises Mismatch instead of returning None."""
    found = re.search(pattern, text)
    if found is None:
        raise Mismatch(message)
    return found


def fullmatch_or_fail(pattern, text, message, flags=0):
    """re.fullmatch that raises Mismatch instead of returning None."""
    found = re.fullmatch(pattern, text, flags=flags)
    if found is None:
        raise Mismatch(message)
    return found


# ---- reading the C source -------------------------------------------------------------------------


def strip_comments(text):
    return re.sub(r"/\*.*?\*/", "", text, flags=re.S)


def match_brace(text, open_index):
    """Return the index of the brace closing the one at open_index, skipping string literals."""
    depth = 0
    i = open_index
    in_string = False
    while i < len(text):
        c = text[i]
        if in_string:
            if c == "\\":
                i += 1
            elif c == '"':
                in_string = False
        elif c == '"':
            in_string = True
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise Mismatch("unbalanced braces")


def split_top_level(body):
    """Split an initialiser body at the commas that are not nested in braces or strings."""
    items = []
    depth = 0
    in_string = False
    start = 0
    i = 0
    while i < len(body):
        c = body[i]
        if in_string:
            if c == "\\":
                i += 1
            elif c == '"':
                in_string = False
        elif c == '"':
            in_string = True
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
        elif c == "," and depth == 0:
            items.append(body[start:i].strip())
            start = i + 1
        i += 1
    tail = body[start:].strip()
    if tail:
        items.append(tail)
    return items


class Source:
    def __init__(self, path):
        self.text = strip_comments(path.read_text(encoding="utf-8"))
        self.strings = {
            m.group(1): m.group(2)
            for m in re.finditer(r'static\s+const\s+char\s+(\w+)\s*\[\]\s*TBL_FLASH\s*=\s*"([^"]*)"\s*;', self.text)
        }

    def body(self, name):
        pattern = r"\b%s\s*(?:\[[^=;{}]*\])+\s*TBL_FLASH\s*=\s*\{" % re.escape(name)
        m = search_or_fail(pattern, self.text, "definition of %s not found" % name)
        end = match_brace(self.text, m.end() - 1)
        return self.text[m.end():end]

    def items(self, name):
        return split_top_level(self.body(name))

    def number(self, name):
        pattern = r"\bconst\s+uint8_t\s+%s\s*=\s*(\w+)\s*;" % re.escape(name)
        m = search_or_fail(pattern, self.text, "constant %s not found" % name)
        return int(m.group(1), 0)

    def scalar(self, name):
        """The initialiser text of a table that is a single value, e.g. 'const float x TBL_FLASH = 2.3f;'."""
        pattern = r"\bconst\s+\w+\s+%s\s+TBL_FLASH\s*=\s*([^;{}]+);" % re.escape(name)
        return search_or_fail(pattern, self.text, "scalar definition of %s not found" % name).group(1).strip()

    def defined_tables(self):
        """Names of every flash table the file defines, except the helper strings."""
        names = re.findall(r"\bconst\s+[\w\s*]*?\b(\w+)\s*(?:\[[^\]=;]*\])*\s+TBL_FLASH\s*=\s*[^;]", self.text)
        return [n for n in names if n not in self.strings]

    def string(self, ident):
        require(ident in self.strings, "string %s is not defined" % ident)
        return self.strings[ident]


# ---- reading the image ----------------------------------------------------------------------------


class Image:
    def __init__(self, path):
        self.data = path.read_bytes()

    def u16(self, addr):
        return self.data[addr] | (self.data[addr + 1] << 8)

    def f32_bytes(self, addr):
        return self.data[addr:addr + 4]

    def cstring(self, addr):
        end = self.data.index(0, addr)
        return self.data[addr:end].decode("latin-1")

    def record(self, addr, size):
        return self.data[addr:addr + size]


def float_bytes(literal):
    token = literal.strip()
    require(token.endswith("f"), "float literal without suffix: %r" % token)
    return struct.pack("<f", float(token[:-1]))


def padded(text, width):
    raw = text.encode("latin-1")
    require(len(raw) <= width, "text %r longer than %d" % (text, width))
    return raw + b"\0" * (width - len(raw))


# ---- the checks -----------------------------------------------------------------------------------


def check_words(src, img, name, addr, count):
    items = src.items(name)
    require(len(items) == count, "%d entries, expected %d" % (len(items), count))
    for i, item in enumerate(items):
        want = img.u16(addr + 2 * i)
        require(int(item, 0) == want, "entry %d is %s, image has 0x%04X" % (i, item, want))


def check_glyph_digit(src, img):
    check_words(src, img, "glyph_digit", 0x0F5, 12)


def check_glyph_alpha(src, img):
    check_words(src, img, "glyph_alpha", 0x10D, 26)


def check_glyph_symbol(src, img):
    check_words(src, img, "glyph_symbol", 0x141, 11)


def check_annun(src, img):
    items = src.items("annun_table")
    require(len(items) == 24, "%d rows, expected 24" % len(items))
    for i, item in enumerate(items):
        m = fullmatch_or_fail(r'\{\s*"([^"]*)"\s*,\s*(\w+)\s*\}', item, "row %d unparsable: %s" % (i, item))
        want = img.record(0x50A + 7 * i, 7)
        got = padded(m.group(1), 6) + bytes([int(m.group(2), 0)])
        require(got == want, "row %d is %r, image has %r" % (i, got, want))


def check_range_max(src, img):
    items = src.items("range_max_index")
    require(len(items) == FUNC_COUNT, "%d entries" % len(items))
    for f, item in enumerate(items):
        want = img.u16(0x5B9 + 2 * f)
        require(int(item, 0) == want, "function %d is %s, image has %d" % (f, item, want))


def check_si(src, img):
    mult = src.items("si_multiplier")
    require(len(mult) == 6, "%d multipliers" % len(mult))
    for i, item in enumerate(mult):
        require(float_bytes(item) == img.f32_bytes(0x5D1 + 4 * i), "multiplier %d (%s) differs" % (i, item))
    chars = re.findall(r"'(.)'", src.body("si_character"))
    want = img.record(0x5E9, 6).decode("latin-1")
    require("".join(chars) == want, "characters %r, image has %r" % ("".join(chars), want))


def check_nplc(src, img):
    items = src.items("nplc_value")
    require(len(items) == 3, "%d entries" % len(items))
    for i, item in enumerate(items):
        name = img.record(0x1546 + 6 * i, 5).rstrip(b"\0").decode("latin-1")
        want = struct.pack("<f", float(name))
        require(float_bytes(item) == want, "entry %d (%s) differs from NPLC name %r" % (i, item, name))
        require(img.data[0x1546 + 6 * i + 5] == i, "NPLC name record %d has index %d" % (i, img.data[0x1546 + 6 * i + 5]))


def check_float_rows(src, img, name, sources):
    """sources maps a function name to (image address, number of real columns)."""
    rows = {}
    for item in src.items(name):
        m = fullmatch_or_fail(r"\[(\w+)\]\s*=\s*\{(.*)\}", item, "row unparsable: %s" % item, flags=re.S)
        require(m.group(1) not in rows, "duplicate row %s" % m.group(1))
        rows[m.group(1)] = split_top_level(m.group(2))
    require(set(rows) == set(sources), "rows %s, expected %s" % (sorted(rows), sorted(sources)))
    for func, values in rows.items():
        addr, columns = sources[func]
        require(len(values) == RANGE_COLUMNS, "%s has %d columns" % (func, len(values)))
        for c, literal in enumerate(values):
            want = img.f32_bytes(addr + 4 * c) if c < columns else struct.pack("<f", 0.0)
            require(float_bytes(literal) == want, "%s column %d (%s) differs" % (func, c, literal))


SCALE_SOURCES = {
    "FUNC_DCV": (0x71B, 5),
    "FUNC_ACV": (0x72F, 5),
    "FUNC_DCA": (0x757, 5),
    "FUNC_ACA": (0x743, 5),
    "FUNC_OHM": (0x76B, 6),
    "FUNC_CONT": (0x76B, 6),
}
UP_SOURCES = {
    "FUNC_DCV": (0x61B, 5),
    "FUNC_ACV": (0x657, 5),
    "FUNC_DCA": (0x67F, 5),
    "FUNC_ACA": (0x6A7, 5),
    "FUNC_OHM": (0x6CF, 6),
    "FUNC_ACDCV": (0x61B, 5),
    "FUNC_ACDCI": (0x67F, 5),
}
DOWN_SOURCES = {
    "FUNC_DCV": (0x62F, 5),
    "FUNC_ACV": (0x66B, 5),
    "FUNC_DCA": (0x693, 5),
    "FUNC_ACA": (0x6BB, 5),
    "FUNC_OHM": (0x6E7, 6),
    "FUNC_ACDCV": (0x62F, 5),
    "FUNC_ACDCI": (0x693, 5),
}


def check_scale(src, img):
    check_float_rows(src, img, "range_scale", SCALE_SOURCES)


def check_up(src, img):
    check_float_rows(src, img, "autorange_up", UP_SOURCES)


def check_down(src, img):
    check_float_rows(src, img, "autorange_down", DOWN_SOURCES)


def check_root_keywords(src, img):
    items = src.items("root_keyword")
    count = src.number("root_keyword_count")
    require(count == len(items) - 1, "count %d, but %d entries before the sentinel" % (count, len(items) - 1))
    mine = []
    for i, item in enumerate(items):
        m = fullmatch_or_fail(TRIPLE, item, "entry %d unparsable: %s" % (i, item))
        long_form, short_form, code = src.string(m.group(1)), src.string(m.group(2)), int(m.group(3), 0)
        if i == count:
            require(long_form == "" and short_form == "" and code == 0x13, "sentinel is not {\"\", \"\", 0x13}")
            continue
        mine.append((long_form, code))
        if short_form != long_form:
            mine.append((short_form, code))
    theirs = []
    for i in range(34):
        rec = img.record(0x138C + 13 * i, 13)
        name = rec[:12].rstrip(b"\0").decode("latin-1")
        if i == 33:
            require(name == "" and rec[12] == 0x13, "image sentinel record is %r" % rec)
        else:
            theirs.append((name, rec[12]))
    require(sorted(mine) == sorted(theirs), "keyword sets differ: only here %s, only in image %s" % (
        sorted(set(mine) - set(theirs)), sorted(set(theirs) - set(mine))))


RANGE_SOURCES = {
    "FUNC_DCV": (0x156A, 17),
    "FUNC_ACDCI": (0x1625, 17),
    "FUNC_ACV": (0x16E0, 17),
    "FUNC_FREQ": (0x16E0, 17),
    "FUNC_PERIOD": (0x16E0, 17),
    "FUNC_DCA": (0x179B, 17),
    "FUNC_ACA": (0x1856, 17),
    "FUNC_OHM": (0x1911, 19),
    "FUNC_CONT": (0x1911, 19),
    "FUNC_ACDCV": (0x19E2, 17),
    # FUNC_DIODE (0x1A9D, 13) is deliberately not ported: ORIGINAL_BUGS #44, see tables.c.
}


def check_range_words(src, img):
    items = src.items("range_word")
    require(src.number("range_word_count") == len(items), "count %d, %d entries" % (
        src.number("range_word_count"), len(items)))
    per_func = {f: [] for f in RANGE_SOURCES}
    for i, item in enumerate(items):
        m = fullmatch_or_fail(TRIPLE, item, "entry %d unparsable: %s" % (i, item))
        require(m.group(2) in per_func, "entry %d names unknown function %s" % (i, m.group(2)))
        per_func[m.group(2)].append((src.string(m.group(1)), int(m.group(3), 0)))
    for func, (addr, records) in RANGE_SOURCES.items():
        want = []
        for r in range(records - 1):
            rec = img.record(addr + 11 * r, 11)
            want.append((rec[:10].rstrip(b"\0").decode("latin-1"), rec[10]))
        require(img.record(addr + 11 * (records - 1), 11) == b"\0" * 11, "%s: image table not zero-ended" % func)
        require(per_func[func] == want, "%s: spellings differ from image table at 0x%X" % (func, addr))


def name_records(src, img, name, addr, total, terminators_in_count):
    items = src.items(name)
    require(len(items) == total, "%d records, expected %d" % (len(items), total))
    sentinels = 0
    for i, item in enumerate(items):
        m = fullmatch_or_fail(r'\{\s*"([^"]*)"\s*,\s*(\w+)\s*\}', item, "record %d unparsable: %s" % (i, item))
        got = padded(m.group(1), 10) + bytes([int(m.group(2), 0)])
        want = img.record(addr + 11 * i, 11)
        require(got == want, "record %d is %r, image has %r" % (i, got, want))
        if got[10] == 0x13:
            sentinels += 1
    count = src.number(name + "_count")
    expected = total if terminators_in_count else total - sentinels
    require(count == expected, "count %d, expected %d" % (count, expected))
    return sentinels


def check_func_names(src, img):
    sentinels = name_records(src, img, "func_name", 0x787, 10, False)
    require(sentinels == 1, "%d sentinels" % sentinels)


def check_second_display(src, img):
    sentinels = name_records(src, img, "second_display_name", 0x7F5, 29, True)
    require(sentinels == 7, "%d sentinels, expected 7" % sentinels)


def check_error_fragments(src, img):
    items = src.items("error_fragment")
    require(len(items) == 14, "error_fragment has %d entries, expected 14" % len(items))
    for i, ident in enumerate(items):
        want = img.cstring(img.u16(0x135E + 2 * i))
        require(src.string(ident) == want, "error_fragment[%d] is %r, image has %r" % (i, src.string(ident), want))


def check_overload(src, img):
    """One float per function at 0x5EF, written with designated initialisers [FUNC_x] = value."""
    seen = {}
    for item in src.items("overload_limit"):
        m = fullmatch_or_fail(r"\[(\w+)\]\s*=\s*(.+)", item, "entry unparsable: %s" % item)
        require(m.group(1) in FUNC, "unknown function %s" % m.group(1))
        require(m.group(1) not in seen, "duplicate entry %s" % m.group(1))
        seen[m.group(1)] = m.group(2)
    require(set(seen) == set(FUNC), "functions %s, expected all %d" % (sorted(seen), FUNC_COUNT))
    for func, literal in seen.items():
        want = img.f32_bytes(0x5EF + 4 * FUNC[func])
        require(float_bytes(literal) == want, "%s (%s) differs from image at 0x%X" % (func, literal, 0x5EF + 4 * FUNC[func]))


def check_diode_limit(src, img):
    require(float_bytes(src.scalar("diode_limit")) == img.f32_bytes(0x6FF), "diode_limit differs from image at 0x6FF")


def check_continuity(src, img):
    items = src.items("continuity_threshold")
    require(len(items) == 6, "%d thresholds, expected 6" % len(items))
    for i, item in enumerate(items):
        require(float_bytes(item) == img.f32_bytes(0x703 + 4 * i), "threshold %d (%s) differs from image" % (i, item))


# The menu strings are no longer the vendor's table (see docs/MENU_TEXTS.md): the texts were
# rewritten so that they fit the display, and the strings no menu shows any more were dropped. This check
# still walks the original's 95 strings at 0xBCC-0xEE6, in order, so that every string that was KEPT is
# compared with the image exactly as before, and every change is a named row below, with the vendor's text
# beside ours. Nothing is skipped silently: a string that is neither kept, nor listed here, fails.

# Vendor strings the port no longer has, by their position in the image (0-94), with the text that must
# still be there.
MENU_TEXT_DROPPED = {
    2: "      ",
    38: "RETURN:   ", 39: "SOUND:    ", 40: "R.F:      ", 41: "BAUD:     ", 42: "RDGS:     ",
    43: "HOLD:     ", 44: "          ", 45: "REF:      ", 46: "TERM:     ", 47: "BEEP:     ",
    48: "SOURCE:   ", 49: "LO:       ", 50: "HI:       ",
    51: "RANG: 2.3V ", 63: "RANG: 750V",
    69: "dB", 70: "AAD", 71: "VAD", 72: "AD.A", 73: "AA.D", 74: "VD.A", 75: "VA.D", 76: "dBm",
    77: "dB ", 78: "%  ", 79: "   ", 81: "SEC", 82: "HZ ", 84: "AAC", 85: "ADC", 86: "VAC", 87: "VDC",
    88: "  GOING. .  ", 89: "CLEAR. . . . ", 93: "004", 94: "001",
}

# Vendor strings the port changed: identifier in tables.c -> (the vendor's text, ours).
MENU_TEXT_CHANGED = {
    "mt_cr": ("CR", "RETURN"),
    "mt_lf": ("LF", "LINE FEED"),
    "mt_man": ("MAN", "MANUAL"),
    "mt_imm": ("IMM", "IMMEDIATE"),
    "mt_item_zero_cal": ("7:ZERO CAL", "ZERO CAL"),
    "mt_item_revision": ("6:REVISION", "REVISION"),
    "mt_item_key_sound": ("5:KEY SONG", "KEY SOUND"),
    "mt_item_echo": ("4:RETURN  ", "ECHO"),
    "mt_item_line_end": ("3:TX TERM ", "LINE END"),
    "mt_item_baud": ("2:BAUD RAT", "BAUD RATE"),
    "mt_item_beeper": ("1:BEEP STA", "BEEPER"),
    "mt_item_hold_cnt": ("3:HOLD CNT", "HOLD CNT"),
    "mt_item_hold_win": ("2:HOLD WIN", "HOLD WIN"),
    "mt_item_trig_mode": ("1:TRIG MOD", "TRIG MODE"),
    "mt_item_dbm_ref": ("5:dBm REF ", "dBm REF"),
    "mt_item_db_ref": ("4:dB REF  ", "dB REF"),
    "mt_item_perc_ref": ("3:PERC REF", "PERC REF"),
    "mt_item_lo_limit": ("2:LO LIMIT", "LO LIMIT"),
    "mt_item_hi_limit": ("1:HI LIMIT", "HI LIMIT"),
    "mt_menu_sys": ("C:SYS  MEU", "SYS MENU"),
    "mt_menu_trig": ("B:TRIG MEU", "TRIG MENU"),
    "mt_menu_math": ("A:MATH MEU", "MATH MENU"),
    "mt_range_500r": ("RANG: 500&", "500&"),
    "mt_range_5k": ("RANG: 5k& ", "5k&"),
    "mt_range_50k": ("RANG: 50k&", "50k&"),
    "mt_range_500k": ("RANG:500k&", "500k&"),
    "mt_range_5m": ("RANG: 5M& ", "5M&"),
    "mt_range_50m": ("RANG: 50M&", "50M&"),
    "mt_range_5ma": ("RANG: 5mA ", "5mA"),
    "mt_range_50ma": ("RANG: 50mA", "50mA"),
    "mt_range_500ma": ("RANG:500mA", "500mA"),
    "mt_range_5a": ("RANG: 5A  ", "5A"),
    "mt_range_20a": ("RANG: 20A ", "20A"),
    "mt_range_500mv": ("RANG:500mV", "500mV"),
    "mt_range_5v": ("RANG: 5V  ", "5V"),
    "mt_range_50v": ("RANG: 50V ", "50V"),
    "mt_range_500v": ("RANG: 500V", "500V"),
    "mt_range_1000v": ("RANG:1000V", "1000V"),
}

VENDOR_MENU_TEXT_COUNT = 95


def check_menu_text(src, img):
    items = src.items("menu_text")
    require(src.number("menu_text_count") == len(items), "count %d, %d entries" % (
        src.number("menu_text_count"), len(items)))
    require(len(items) + len(MENU_TEXT_DROPPED) == VENDOR_MENU_TEXT_COUNT, "%d entries and %d dropped, the image has %d" % (
        len(items), len(MENU_TEXT_DROPPED), VENDOR_MENU_TEXT_COUNT))
    addr = 0xBCC
    next_item = 0
    changed_seen = set()
    for index in range(VENDOR_MENU_TEXT_COUNT):
        want = img.cstring(addr)
        if index in MENU_TEXT_DROPPED:
            require(want == MENU_TEXT_DROPPED[index], "dropped entry %d at 0x%X is %r in the image, expected %r" % (
                index, addr, want, MENU_TEXT_DROPPED[index]))
        else:
            ident = items[next_item]
            next_item += 1
            if ident in MENU_TEXT_CHANGED:
                vendor, ours = MENU_TEXT_CHANGED[ident]
                require(want == vendor, "%s at 0x%X: the image has %r, expected the vendor's %r" % (ident, addr, want, vendor))
                require(src.string(ident) == ours, "%s is %r, expected our %r" % (ident, src.string(ident), ours))
                changed_seen.add(ident)
            else:
                require(src.string(ident) == want, "entry %d at 0x%X is %r, image has %r and no change is listed" % (
                    index, addr, src.string(ident), want))
        addr += len(want) + 1
    require(addr == 0xEE7, "strings end at 0x%X, expected 0xEE7" % addr)
    require(next_item == len(items), "%d entries are left over after the image ran out" % (len(items) - next_item))
    require(changed_seen == set(MENU_TEXT_CHANGED), "listed as changed but absent from tables.c: %s" % (
        sorted(set(MENU_TEXT_CHANGED) - changed_seen)))


CHECKS = [
    ("glyph_digit", check_glyph_digit, ("glyph_digit",)),
    ("glyph_alpha", check_glyph_alpha, ("glyph_alpha",)),
    ("glyph_symbol", check_glyph_symbol, ("glyph_symbol",)),
    ("annun_table", check_annun, ("annun_table",)),
    ("range_max_index", check_range_max, ("range_max_index",)),
    ("si_multiplier/si_character", check_si, ("si_multiplier", "si_character")),
    ("nplc_value", check_nplc, ("nplc_value",)),
    ("range_scale", check_scale, ("range_scale",)),
    ("overload_limit", check_overload, ("overload_limit",)),
    ("diode_limit", check_diode_limit, ("diode_limit",)),
    ("continuity_threshold", check_continuity, ("continuity_threshold",)),
    ("autorange_up", check_up, ("autorange_up",)),
    ("autorange_down", check_down, ("autorange_down",)),
    ("root_keyword", check_root_keywords, ("root_keyword",)),
    ("range_word", check_range_words, ("range_word",)),
    ("func_name", check_func_names, ("func_name",)),
    ("second_display_name", check_second_display, ("second_display_name",)),
    ("error_fragment", check_error_fragments, ("error_fragment",)),
    ("menu_text", check_menu_text, ("menu_text",)),
]


def main():
    if len(sys.argv) != 2:
        print("usage: python -I tools/check_tables.py PATH_TO_FLASH_DUMP.bin")
        return 2
    src = Source(TABLES_C)
    img = Image(Path(sys.argv[1]))
    passed = 0
    failed = 0
    for label, check, _tables in CHECKS:
        try:
            check(src, img)
        except Exception as exc:  # any failure to find, parse or match a table is a FAIL, never a skip
            print("FAIL  %-28s %s: %s" % (label, type(exc).__name__, exc))
            failed += 1
        else:
            print("PASS  %s" % label)
            passed += 1
    covered = {name for _label, _check, tables in CHECKS for name in tables}
    for name in src.defined_tables():
        if name not in covered:
            print("FAIL  %-28s defined in tables.c but no check covers it" % name)
            failed += 1
    for _label, _check, tables in CHECKS:
        for name in tables:
            if name not in src.defined_tables():
                print("FAIL  %-28s check refers to a table tables.c does not define" % name)
                failed += 1
    print("TOTAL %d/%d tables match, %d failures" % (passed, len(CHECKS), failed))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
