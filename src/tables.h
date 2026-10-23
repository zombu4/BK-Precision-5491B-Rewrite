/* tables.h -- types and accessors for the hand-written flash tables.
 *
 * FROZEN FOUNDATION HEADER. Modules code against this; changing it changes every module.
 *
 * The data lives in src/tables.c as typed C initialisers. The display code (glyphs, annunciators), the
 * remote code (keywords, ranges, NPLC) and the menu code (menu text) read it.
 *
 * RAM RULE: every table in this header lives in FLASH. Declare with TBL_FLASH and read
 * through the accessors below -- never copy a whole table to RAM.
 */
#ifndef BK_TABLES_H
#define BK_TABLES_H

#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>

#include "state.h"

#define TBL_FLASH PROGMEM

/* ---- Display glyphs (flash 0xF5-0x156) ------------------------------------------------------------
 * 16-bit words: 14 segments + decimal point + colon. 12 entries '0'..';', 26 'A'..'Z',
 * 11 symbols (m k u @ - + % / & n d).
 */
#define GLYPH_DIGIT_COUNT 12u
#define GLYPH_ALPHA_COUNT 26u
#define GLYPH_SYMBOL_COUNT 11u

extern const uint16_t glyph_digit[GLYPH_DIGIT_COUNT] TBL_FLASH;
extern const uint16_t glyph_alpha[GLYPH_ALPHA_COUNT] TBL_FLASH;
extern const uint16_t glyph_symbol[GLYPH_SYMBOL_COUNT] TBL_FLASH;

static inline uint16_t tbl_glyph_digit(uint8_t i) {
    return pgm_read_word(&glyph_digit[i]);
}
static inline uint16_t tbl_glyph_alpha(uint8_t i) {
    return pgm_read_word(&glyph_alpha[i]);
}
static inline uint16_t tbl_glyph_symbol(uint8_t i) {
    return pgm_read_word(&glyph_symbol[i]);
}

/* ---- Text on the panel -----------------------------------------------------------------------------
 * The one rule that turns a string into display cells. The firmware's printers use it, and so does the
 * host check that every menu text fits its line (host/test_menu_text.c), which is why it lives here beside
 * the glyph tables and not inside the display driver.
 *
 * Characters with a glyph: letters A-Z (J has none: its word is 0), digits, space, and the symbols
 * m k u @ - + % / & n d. Anything else is a blank cell that still takes its place. A '.' or ':' directly
 * after a letter or a digit sets the decimal point or colon of THAT cell and takes none of its own; after
 * any other character it is such an "anything else" and takes a blank cell, so a text that needs the
 * merge must put the mark right behind a letter or digit.
 *
 * TEXT_PRINTER_NUMBER differs in one rule: it lets a '-' take a following '.' or ':' too. */
typedef enum { TEXT_PRINTER_NUMBER, TEXT_PRINTER_TEXT } text_printer_t;

/* Writes the glyph of each cell of `s` to `out` (at most `max_cells` of them) and returns how many cells the
 * text took. `in_flash` says which memory `s` is in. */
/* The cell that character `char_index` lands in, under the same rule as tbl_text_cells. For blinking a
 * cell rather than substituting a space for a character -- see the comment at the definition. */
uint8_t tbl_cell_of_char(const char *s, bool in_flash, text_printer_t printer, uint8_t char_index);

uint8_t tbl_text_cells(const char *s, bool in_flash, text_printer_t printer, uint16_t *out,
                       uint8_t max_cells);

/* ---- Annunciators (24-row table at flash 0x4F2-0x5B8): {legend[6], VFD display-RAM address} --------
 * Written as a whole byte 0x00 (off) / 0xFF (on) to the given display address.
 */
#define ANNUN_LEGEND_LEN 6u
#define ANNUN_COUNT 24u

typedef struct {
    char legend[ANNUN_LEGEND_LEN];
    uint8_t vfd_addr;
} annun_t;

extern const annun_t annun_table[ANNUN_COUNT] TBL_FLASH;

/* Annunciator ids ARE `annun_table` row indices -- rows 0-11 live on the main controller, 12-23 on the
 * secondary (the address sequence 0x02,0x05...0x1D repeats identically in both halves).
 *
 * A lamp is a WHOLE BYTE at a 3k+2 display address, 0xFF on / 0x00 off. There is
 * no bit mask, so `annun_t` needs no mask field and a whole-byte write is correct.
 *
 * Five table rows are NEVER referenced by the firmware and so have no name here: rows 1 and 2 (a second
 * "Dc"/"Ac" on main) and rows 13, 14 and 19 ("Dc"/"Ac"/"Auto" on secondary). That is also why the three
 * rows sharing address 0x1D never collide: 0x1D is only ever ANN_FAST (main) or ANN_COMP (secondary).
 * Do NOT renumber -- these values index flash.
 */
typedef enum {
    /* main controller, rows 0-11 */
    ANN_FAST = 0,
    ANN_MED = 3,
    ANN_SLOW = 4,
    ANN_TRIG = 5,
    ANN_HOLD = 6,
    ANN_AUTO = 7,
    ANN_REL = 8,
    ANN_MATH = 9,
    ANN_BEEP = 10,
    ANN_DIOD = 11,
    /* secondary controller, rows 12-23 */
    ANN_COMP = 12,
    ANN_HI = 15,
    ANN_IN = 16,
    ANN_LO = 17,
    ANN_RMT = 18,
    ANN_MAX = 20,
    ANN_MIN = 21,
    ANN_ERR = 22,
    ANN_SHIFT = 23
} annun_id_t;

/* ---- Per-function limits and ranges ---------------------------------------------------------------
 * Max range index per function (flash 0x5B9-0x5CE): {4,4,4,4,5,4,4,5,0,4,4}.
 */
extern const uint8_t range_max_index[FUNC_COUNT] TBL_FLASH;
static inline uint8_t tbl_range_max(func_t f) {
    return pgm_read_byte(&range_max_index[f]);
}

/* Range scale factors: volts-at-ADC -> input units, per function and range (flash 0x5EF-0x783). */
extern const float range_scale[FUNC_COUNT][RANGE_MAX_INDEX + 1u] TBL_FLASH;
static inline float tbl_range_scale(func_t f, uint8_t r) {
    float v;
    memcpy_P(&v, &range_scale[f][r], sizeof v);
    return v;
}

/* Overload limit per function and range, and the autorange up/down thresholds (flash 0x5EF-0x783). */
/* The image holds ONE limit per function (11 floats at flash 0x5EF), NOT one per range.
 *
 * DEAD DATA -- DO NOT USE IT FOR THE OVERLOAD TEST. The table is real flash content and its
 * per-function order is certain, because the eleven values identify their own functions (1 % over each
 * function's top range, and the diode entry equals `diode_limit` independently). But it has
 * ZERO references in the original image: the firmware's overload test uses the autorange UPPER table indexed by
 * RANGE (0x061B, 0x0657, 0x067F, 0x06A7, 0x06CF, 0x06FF). Using this table instead would trip at a
 * different point from the original on every range but the top one. It stays declared and checked because
 * the EEPROM/flash cross-check wants it, and for no other reason. */
extern const float overload_limit[FUNC_COUNT] TBL_FLASH;
extern const float autorange_up[FUNC_COUNT][RANGE_MAX_INDEX + 1u] TBL_FLASH;
extern const float autorange_down[FUNC_COUNT][RANGE_MAX_INDEX + 1u] TBL_FLASH;

/* Diode and continuity thresholds (flash 0x6FF-0x71A). These two are genuinely SHARED tables, so one
 * definition here keeps the display code from carrying its own copy.
 *
 * Decoded from the original flash image at those addresses: 0x6FF is one float, 2.3; 0x703 onwards is six
 * floats, 1e6, 1e5, 1e4, 1e3, 100, 10. tools/check_tables.py covers the definitions.
 *
 * Other constant regions of the original are NOT tables and get no type:
 *   0x157-0x4F1 and 0xA07-0xB35 are the ICCAVR constant pool, one slot per use site, so in
 *     hand-written C each value is a literal at the site that uses it. There is nothing to share.
 *   0x124F is the printf format strings. The port has no printf, so they are gone with it.
 *   0x111D-0x1122 are the idata defaults, which in C are plain initialisers.
 *   0x71B-0x783 is the RANGE SCALE block, already typed above as range_scale. It is not the
 *     autorange thresholds; the code says so. cal_load_dcv 0x0CAEC reads 0x71B
 *     and cal_load_acv 0x0CD70 reads 0x72F, while autorange reads a different block: meas_autorange_dcv
 *     0x0B3E4 uses 0x61B as its upper limit and 0x62F as its lower, and meas_sec_vacdc 0x0A0EA uses 0x61B
 *     and 0x657. So autorange_up / autorange_down live at 0x61B-0x6FE, which is where they already were.
 *   0x133D, the ohm segment-switch table, is typed where its consumer is written. */
extern const float diode_limit TBL_FLASH;
#define CONTINUITY_RANGE_COUNT 6u
extern const float continuity_threshold[CONTINUITY_RANGE_COUNT] TBL_FLASH;

/* SI prefix multipliers 1e6..1e-9 and their characters "Mk mun" (flash 0x5D1-0x5EE). */
#define SI_PREFIX_COUNT 6u
extern const float si_multiplier[SI_PREFIX_COUNT] TBL_FLASH;
extern const char si_character[SI_PREFIX_COUNT] TBL_FLASH;

/* NPLC table (idata image, RAM 0x036F region in the original; here it stays in flash). */
#define NPLC_COUNT 3u
extern const float nplc_value[NPLC_COUNT] TBL_FLASH;

/* ---- SCPI keyword tables --------------------------------------------------------------------------
 * The original stores long/short keyword pairs. `code` is the node code the executor switches on;
 * its numbering is protocol-visible through the dispatch table and must not be renumbered.
 */
typedef struct {
    const char *long_form;  /* flash string */
    const char *short_form; /* flash string */
    uint8_t code;
} keyword_t;

extern const keyword_t root_keyword[] TBL_FLASH;
extern const uint8_t root_keyword_count;

/* Range spelling tables used by :<func>:RANGe parsing and replies. */
typedef struct {
    const char *text;
    func_t func;
    uint8_t range_index;
} range_word_t;

extern const range_word_t range_word[] TBL_FLASH;
extern const uint8_t range_word_count;

/* ---- Function / second-display name records (flash 0x787-0x933): {name[10], index} ---------------- */
#define FUNC_NAME_LEN 10u

typedef struct {
    char name[FUNC_NAME_LEN];
    uint8_t index;
} name_record_t;

extern const name_record_t func_name[] TBL_FLASH;
extern const uint8_t func_name_count;
extern const name_record_t second_display_name[] TBL_FLASH;
extern const uint8_t second_display_name_count;

/* ---- Text ------------------------------------------------------------------------------------------
 * Error fragments (flash 0xB36-0xBD1) are combined in pairs, e.g. "BUS: BAD " + " COMMAND.".
 * Menu strings, prompts, range texts and status messages: the table at the end of this file.
 * All are flash strings; use the _P string functions.
 */
/* The original has ONE table of 14 fragments (flash 0x135E); callers combine two indexes, e.g. (3,4) =
 * "BUS: BAD " + " COMMAND.". Entries 14-22 of that table (display words) are deliberately excluded.
 * tables.c and remote/reply.c both use this single array. */
#define ERROR_FRAGMENT_COUNT 14u
extern const char *const error_fragment[ERROR_FRAGMENT_COUNT] TBL_FLASH;
/* Index of each entry of menu_text[], in the order of the mt_* symbols in tables.c.
 *
 * ui/menu.c addresses the shared table through these names, so there is one source of truth for every
 * string the panel shows.
 *
 * The table began as the vendor's 95 strings (flash 0x0BCC-0x0EE6). It is no longer that table: the
 * item, page, choice and range texts were rewritten so that they fit the display (see
 * docs/MENU_TEXTS.md), and the 37 vendor strings the menu no longer shows were dropped. tools/check_tables.py
 * lists every one of those changes, so the strings that remain the vendor's are still checked against
 * the original image. */
typedef enum {
    MT_CR = 0,
    MT_LF = 1,
    MT_PCT10 = 2,
    MT_PCT1 = 3,
    MT_PCT0_1 = 4,
    MT_PCT0_01 = 5,
    MT_BAUD600 = 6,
    MT_BAUD1200 = 7,
    MT_BAUD2400 = 8,
    MT_BAUD4800 = 9,
    MT_BAUD9600 = 10,
    MT_BAUD19K2 = 11,
    MT_BAUD38K4 = 12,
    MT_ON = 13,
    MT_OFF = 14,
    MT_BUS = 15,
    MT_MAN = 16,
    MT_IMM = 17,
    MT_ITEM_ZERO_CAL = 18,
    MT_ITEM_REVISION = 19,
    MT_ITEM_KEY_SOUND = 20,
    MT_ITEM_ECHO = 21,
    MT_ITEM_LINE_END = 22,
    MT_ITEM_BAUD = 23,
    MT_ITEM_BEEPER = 24,
    MT_ITEM_HOLD_CNT = 25,
    MT_ITEM_HOLD_WIN = 26,
    MT_ITEM_TRIG_MODE = 27,
    MT_ITEM_DBM_REF = 28,
    MT_ITEM_DB_REF = 29,
    MT_ITEM_PERC_REF = 30,
    MT_ITEM_LO_LIMIT = 31,
    MT_ITEM_HI_LIMIT = 32,
    MT_MENU_SYS = 33,
    MT_MENU_TRIG = 34,
    MT_MENU_MATH = 35,
    MT_PROMPT_CAL_SHORT = 36,
    MT_RANGE_500R = 37,
    MT_RANGE_5K = 38,
    MT_RANGE_50K = 39,
    MT_RANGE_500K = 40,
    MT_RANGE_5M = 41,
    MT_RANGE_50M = 42,
    MT_RANGE_5MA = 43,
    MT_RANGE_50MA = 44,
    MT_RANGE_500MA = 45,
    MT_RANGE_5A = 46,
    MT_RANGE_20A = 47,
    MT_RANGE_500MV = 48,
    MT_RANGE_5V = 49,
    MT_RANGE_50V = 50,
    MT_RANGE_500V = 51,
    MT_RANGE_1000V = 52,
    MT_LBL_DCV = 53,
    MT_LBL_OHM = 54,
    MT_MSG_SAVED = 55,
    MT_MSG_TOO_SMALL = 56,
    MT_MSG_TOO_LARGE = 57,
    MT_COUNT = 58
} menu_text_id_t;

extern const char *const menu_text[] TBL_FLASH;
extern const uint8_t menu_text_count;

#endif /* BK_TABLES_H */
