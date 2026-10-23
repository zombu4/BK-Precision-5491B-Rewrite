# MENU_TEXTS -- every text the front panel shows, old and new (2026-10-23)

The menu texts did not fit the display. The design therefore has no "A:" / "1:" prefixes anywhere, words spelled
out, and the vendor's misspelling "MEU" (for MENU) corrected. This file lists **every** text of the menu
system with its cell count, the old text and the new one. `host/test_menu_text.c` measures the same texts,
and every screen the menu builds from them, on each host test run (`python -I build.py hosttests`).

## 1. The rules the counts come from

**Cell rule.** One character takes one cell, except that a `.` or `:` standing directly behind a letter or a
digit sets the point or colon of that cell and takes none. The rule is `tbl_text_cells()` in `src/tables.c`
(moved there from `ui/display.c` so that the display and the check share it). Behind any other character
(space, `-`, a symbol) a `.` or `:` is a blank cell of its own, so no shipped text uses one there; the check
fails a text that does.

**Showable characters.** Space, `A`-`Z` except `J`, `0`-`9`, and `m k u @ - + % / & n d`. This is the rule
the BOOT MSG picker uses (`picker_slot_char`, menu.c): a character is showable when its glyph word is not
zero. Evidence in `tables.c`: `glyph_alpha[9]` ('J') is `0x0000`; every other letter and digit is non-zero;
`glyph_symbol` is `{0x802B m, 0x0C0C k, 0x0A03 u, 0x8001 @, 0x8001 -, 0x8809 +, 0xAC1B %, 0x0410 /,
0xA315 &, 0x8028 n, 0x8263 d}`. Lower-case `d`, `m`, `k`, `u`, `n` therefore render; every other lower-case
letter has no glyph. `dB`, `dBm`, `dB REF`, `dBm REF` render: `d` is `glyph_symbol[10]` = 0x8263, `m` is
`glyph_symbol[0]` = 0x802B, `B` is `glyph_alpha[1]` = 0x0B4B.

**Main line: 9 cells.** `VFD_TEXT_CELLS` = 9 (`hal/vfd.h:14`); `show_line` (display.c) writes exactly nine and
blank-fills.

**Second line: 9 cells.** Established from the code:
- The original's streaming text printer for controller 2, `0x2B4E`, ends its loop at `0x2D2E`
  (`cpi r24, 0xDB`). 0xDB is `0xC0 | 0x1B`: it stops when the next cell address is 0x1B, that is after cells
  0-8 (three bytes per cell; 0x1B / 3 = cell 9). The loop is at 0x2B4E-0x2D50 of the original; the sequence at
  `0x2D2A` adds the cell offset to the start address before the compare.
- The measurement screen on the second controller uses the same nine: a six-cell number at cells 0-5
  (`VFD_NUM_CELLS`) and a three-cell label at `LABEL_CELL` 6 (display.c), and cell 9 (`FLAG_CELL`) is its flag
  byte. `display.c` asserts `DISPLAY_TEXT_CELLS == FLAG_CELL` for both lines.
- The original's text printers stop after 9 cells. The controller's memory holds 11 cells
  (`SECOND_RAM_BYTES` 0x21 = 33 bytes at three per cell, `vfd.c:59`); cell 10 is never written by any text path.

**Main-line cells 9-11 while a menu is open: blank.** `MAIN_RAM_BYTES` 0x24 = 36 bytes = 12 cells
(`vfd.c:58`). `menu_enter()` calls `display_clear_all()`, which clears both controllers' whole memory
(`vfd_clear` writes all 0x24 bytes). While a menu is open `display_update()` and `display_annunciators()`
return at once, so nothing rewrites the flag byte at 0x1B (cell 9); no text path writes past cell 8; lamps
sit at the third byte of cells 0-9 and the menu writes only SHIFT and TRIG. The one exception is the SELF TEST
lamp phase, which lights all 12 cells (`lamp_test_on`) and is cleared by `display_clear_all()` in
`start_key_test`. So cells 9-11 are blank and no text may use them: the printers stop at 9.

**The tube has nine grids per line and no more (counted on the glass, 2026-10-23).** The SELF TEST lamp
phase with every segment lit was photographed.
Counted on the glass, each line carries exactly **nine** 14-segment character cells, and **every** cell
including the ninth has its own small `:` and `.` to its right, between it and the next cell. There is no
visible grid at main cells 10-11 or at second cell 10 — those addresses exist in the controller's RAM and
drive nothing on the panel, which is consistent with 0x1B being the flag byte. The
earlier UNVERIFIED on this point is answered and withdrawn. Consequence for text: nine cells is the hard
limit on both lines, not a convention, and the marks after the ninth cell cannot be used to carry a
character.

**Two lines.** A name that does not fit one line in full takes both: main line, then second line, each at most
9 cells. An editor shows its value on the main line and the name of what it edits on the second line.
`display_show_second_P()` (display.h) writes the second line; NULL clears it.

## 2. Pages, items and submenu entries

| Where shown | Old | New | Cells | Line split |
|---|---|---|---|---|
| Page 0 | `A:MATH MEU` | `MATH MENU` | 9 | one line |
| Page 1 | `B:TRIG MEU` | `TRIG MENU` | 9 | one line |
| Page 2 | `C:SYS  MEU` | `SYS MENU` | 8 | one line |
| MATH item 1 | `1:HI LIMIT` | `HI LIMIT` | 8 | one line |
| MATH item 2 | `2:LO LIMIT` | `LO LIMIT` | 8 | one line |
| MATH item 3 | `3:PERC REF` | `PERC REF` | 8 | one line |
| MATH item 4 | `4:dB REF` | `dB REF` | 6 | one line |
| MATH item 5 | `5:dBm REF` | `dBm REF` | 7 | one line |
| TRIG item 1 | `1:TRIG MOD` | `TRIG MODE` | 9 | one line |
| TRIG item 2 | `2:HOLD WIN` | `HOLD WIN` | 8 | one line |
| TRIG item 3 | `3:HOLD CNT` | `HOLD CNT` | 8 | one line |
| SYS item 1 | `1:BEEP STA` | `BEEPER` | 6 | one line |
| SYS item 2 | `2:BAUD RAT` | `BAUD RATE` | 9 | one line |
| SYS item 3 | `3:TX TERM` | `LINE END` | 8 | one line |
| SYS item 4 | `4:RETURN` | `ECHO` | 4 | one line (the vendor's RETURN is the echo setting) |
| SYS item 5 | `5:KEY SONG` | `KEY SOUND` | 9 | one line |
| SYS item 6 | `6:REVISION` | `REVISION` | 8 | one line |
| SYS item 7 | `7:ZERO CAL` | `ZERO CAL` | 8 | one line |
| SYS item 8 | `8:RLY EXER` | `RELAY` / `EXERCISER` | 5 / 9 | main / second |
| SYS item 9 | `9:RLY BOOT` | `RELAY` / `POWER ON` | 5 / 8 | main / second |
| SYS item 10 | `10:LINE` | `LINE FREQ` | 9 | one line |
| SYS item 11 | `11:CAL` | `CAL` | 3 | one line |
| SYS item 12 | `12:CAL CHK` | `CAL CHECK` | 9 | one line |
| SYS item 13 | `13:INFO` | `INFO` | 4 | one line |
| SYS item 14 | `14:SELFTST` | `SELF TEST` | 9 | one line |
| SYS item 15 | `15:BRIGHT` | `BRIGHT` | 6 | one line |
| SYS item 16 | `16:CODE` | `CODE` | 4 | one line |
| SYS item 17 | `17:RESET` | `RESET` | 5 | one line |
| SYS item 18 | `18:BOOTMSG` | `BOOT MSG` | 8 | one line |
| CAL entry 1 | `1:ZERO` | `ZERO` | 4 | one line |
| CAL entry 2 | `2:OPEN` | `OPEN` | 4 | one line |
| CAL entry 3 | `3:RANGE` | `RANGE` | 5 | one line |
| CAL entry 4 | `4:VERIFY` | `VERIFY` | 6 | one line |
| CAL entry 5 | `5:UNDO` | `UNDO` | 4 | one line |
| CAL entry 6 | `6:DATE` | `DATE` | 4 | one line |
| CAL entry 7 | `7:COUNT` | `COUNT` | 5 | one line |

Page, item and CAL-entry titles are static. The original blinked the first character (the page letter or item
digit); with no prefix there is no such character. Prompts that wait for ENTER now blink as a whole instead.

The 36 texts first proposed were all counted with the rule above and all fit; none was altered. `RELAY EX` and
`RELAY PWR` were replaced by two-line forms for the item titles; `RELAY PWR` (9) remains
as the second-line name of the RELAY POWER ON editor, where the value is on the main line.

## 3. Editors: value on the main line, name on the second

The original put a prompt before the value (`HI:`, `R.F:`, `RDGS:`) and padded to a fixed cell. That left two
to four cells for the value, and `R.F:` for "reference" is the vendor's squeeze to fit. Now every editor shows
the value from cell 0 and the name on the second line, except the two reference editors, which put `REF:` first (below).

| Editor | Old screen (prompt + value) | New main line | New second line | Main cells | Second cells |
|---|---|---|---|---|---|
| HI LIMIT | `HI:` + number | number (`+1.0000k`) | `HI LIMIT` | 7 | 8 |
| LO LIMIT | `LO:` + number | number | `LO LIMIT` | 7 | 8 |
| PERC REF | blank + number | number | `PERC REF` | 7 | 8 |
| dB REF | `R.F:` + number | `REF:` + number without its sign (`REF:1.0000^`) | `dB REF` | 9 | 6 |
| dBm REF | `REF:` + 4 digits + `&` | `REF:` + 4 digits + `&` (`REF:0600&`) | `dBm REF` | 8 | 7 |
| HOLD CNT | `RDGS:` + 3 digits | `004` | `HOLD CNT` | 3 | 8 |
| CAL reference value | `R.F:` + number | number | `REFERENCE` | 7 | 9 |
| TRIG MODE | `SOURCE:` + `IMM` / `MAN` / `BUS` | `IMMEDIATE` / `MANUAL` / `BUS` | `TRIG MODE` | 9 / 6 / 3 | 9 |
| HOLD WIN | `HOLD:` + `0.01%` ... `10%` | `0.01%`, `0.1%`, `1%`, `10%` | `HOLD WIN` | 4 / 3 / 2 / 3 | 8 |
| BEEPER | `BEEP:` + `ON`/`OFF` | `ON` / `OFF` | `BEEPER` | 2 / 3 | 6 |
| BAUD RATE | `BAUD:` + rate | `38.4K` `19.2K` `9600` `4800` `2400` `1200` `600` | `BAUD RATE` | 3-4 | 9 |
| LINE END | `TERM:` + `LF`/`CR` | `LINE FEED` / `RETURN` | `LINE END` | 9 / 6 | 8 |
| ECHO | `RETURN:` + `ON`/`OFF` | `ON` / `OFF` | `ECHO` | 2 / 3 | 4 |
| KEY SOUND | `SOUND:` + `ON`/`OFF` | `ON` / `OFF` | `KEY SOUND` | 2 / 3 | 9 |
| RELAY POWER ON | `BOOT:` + `ON`/`OFF` | `ON` / `OFF` | `RELAY PWR` | 2 / 3 | 9 |
| LINE FREQ | `LINE:` + `60HZ`/`50HZ` | `60 HZ` / `50 HZ` | `LINE FREQ` | 5 | 9 |
| BRIGHT | `LVL:` + `10/16` ... `14/16` | `1` ... `8` (eight choices, one per dimming code of the display chip; `1` dimmest, `8` brightest) | `BRIGHT` | 1 | 6 |
| CAL function | `FUNC:` + `DCV`/`DCA`/`OHM` | `DCV` / `DCA` / `OHM` | `FUNCTION` | 3 | 8 |
| CAL range (V) | `RANG:1000V` ... `RANG:500mV` (9 cells) | `1000V` `500V` `50V` `5V` `500mV` | `RANGE` | 2-5 | 5 |
| CAL range (A) | `RANG: 20A` ... `RANG: 5mA` | `20A` `5A` `500mA` `50mA` `5mA` | `RANGE` | 2-5 | 5 |
| CAL range (ohm) | `RANG: 50M&` ... `RANG: 500&` | `50M&` `5M&` `500k&` `50k&` `5k&` `500&` | `RANGE` | 3-5 | 5 |
| Service code entry | ` KEY:` + 4 digits | 4 digits | `CODE` | 4 | 4 |
| New service code | ` NEW:` + 4 digits | 4 digits | `NEW CODE` | 4 | 8 |
| Repeat new code | `AGAIN:` + 4 digits | 4 digits | `CONFIRM` | 4 | 7 |

`&` is the ohm glyph (`glyph_symbol[8]` = 0xA315). The number editors show `sign, five digits, a point, the
SI prefix` = 7 cells (the point shares a cell); the prefix `^` means "no prefix" and is a blank cell.

**The two reference editors carry the label `REF:`** on the main line in front of the value, because the
second line names the editor (`dB REF` or `dBm REF`) and the label says what the number is. Cell counts by the
rule in section 1, `:` sharing the cell of the `F`:

| Row | Characters | Cells |
|---|---|---|
| dB REF | `REF:` 3 + five digits 5 (the point shares the cell before it) + the prefix cell 1 | 9 |
| dB REF with the unit `V` | the row above + `V` 1 | 10, over the 9-cell line, so the unit is **not shown** |
| dBm REF | `REF:` 3 + four digits 4 + the ohm glyph 1 | 8 |

The dB reference is a voltage and is never negative, so its editor has **no sign position**: the cursor starts
on the first digit and never reaches the sign cell, and the row starts with `REF:` and not with `+`. HI LIMIT,
LO LIMIT and PERC REF keep their sign cell. `host/test_menu_text.c` (`reference_editors()`) checks both rows, the
sign cell and the cursor limits.

**Lamp rule, one for every editor, whether it is opened from the menu or from SHIFT.** The panel is cleared on
entry (`display_clear_all()`, original `0xE31C`), so no measurement lamp, no minus sign and no secondary text
stays lit behind an editor. Inside the editor only two lamps are ever lit: SHIFT while the operator has pressed
SHIFT inside the menu, and TRIG after a TRIG MODE choice was entered. When the editor closes, confirmed or
cancelled, `release_display()` redraws the whole measurement screen. Evidence from the original: after
`display_clear_all()` (`0x1FD4`, which writes display memory only and leaves the shadows alone) `0xE320-0xE336`
masks the two flag-byte shadows `0x0F57` and `0x0F56` with `0x13` and writes them back to the controllers, and
the only writers of those shadows set `0x20` (minus), `0x0C` (DC), `0xC0` (AC) or `0xCC` (`0x20FC`, `0xF3E4`,
`0xF3F8`, `0xF44A`, `0xF45E`, `0x16AB8`; `0xF330` keeps bits 0 and 1 of a value that never has them), so the
mask leaves the flag byte at zero. The shortcut editors (SHIFT+ACDC, SHIFT+DIODE, SHIFT+REL) did not clear the panel before this change, and
photographs of the panel (a `-` in front of `R`) show the
measurement's minus sign behind the editor. They now clear it exactly as `menu_enter()` does. UNVERIFIED: which
lamps the original leaves lit inside its three shortcut editors -- check: trace `0x5B5C`, `0x60BA`, `0x63A4`
for a clear or a flag-byte write; the rule above is the one chosen for the rewrite.

## 4. Prompts, messages, running screens, views

| Where shown | Old | New | Cells | Line split |
|---|---|---|---|---|
| ZERO CAL / CAL ZERO prompt | `CAL:SHORT` | `CAL:SHORT` (the vendor's, kept) | 8 | one line, blinks |
| CAL OPEN prompt | `OPEN IN` | `OPEN IN` | 7 | one line, blinks |
| UNDO prompt | `UNDO` | `UNDO` | 4 | one line, blinks |
| RESET prompt | `RESET` | `RESET` | 5 | one line, blinks |
| Warm-up warning prompt | `WARM UP` | `WARM UP` | 7 | one line, blinks |
| Relay exerciser start prompt | `NO INPUT` | `NO INPUT` / `EXERCISER` | 8 / 9 | main / second, main blinks |
| Offer after SELF TEST | `RLY EXER` | `RELAY` / `EXERCISER` | 5 / 9 | main / second, main blinks |
| CAL RANGE commit | `GAIN+0.12%` | `GAIN+0.12%` | 9 | one line (widest `GAIN-10.0%`, 9) |
| Relay exerciser running | `EXER 60` | `RELAY 60` / `EXERCISER` | 8 / 9 | main / second; the seconds left count down |
| Sweep / ZERO CAL running | `RUNNING` | `RUNNING` | 7 | one line |
| SELF TEST converter step | `SELFCAL` | `SELF CAL` | 8 | one line |
| SELF TEST key test, waiting | `PRESS KEY` | `PRESS KEY` | 9 | one line |
| SELF TEST key names | `DCV ACV OHM FREQ ACDC DIODE UP DOWN LEFT RIGHT ENTER MENU MAX MIN TRIG REL RATE SHIFT` | unchanged | 2-7 | one line |
| Message: stored | `  SAVED  ` | `  SAVED  ` | 9 | one line, 1 s |
| Message: write failed | `EE ERROR` | `SAVE FAIL` | 9 | one line, 1 s |
| Message: entry too big | `TOO LARGE` | `TOO LARGE` | 9 | one line, 1 s |
| Message: entry too small | `TOO SMALL` | `TOO SMALL` | 9 | one line, 1 s |
| Message: wrong service code | `KEY ERROR` | `BAD CODE` | 8 | one line, 1 s |
| Message: new codes differ | `MISMATCH` | `MISMATCH` | 8 | one line, 1 s |
| Message: lock expired | `LOCKED` | `LOCKED` | 6 | one line, 1 s |
| Message: sweep refused | `REFUSED` | `REFUSED` | 7 | one line, 1 s |
| Message: failed | `FAILED` | `FAILED` | 6 | one line, 1 s |
| Message: aborted | `ABORTED` | `ABORTED` | 7 | one line, 1 s |
| Message: finished | `DONE` | `DONE` | 4 | one line, 1 s |
| Message: undo done | `RESTORED` | `RESTORED` | 8 | one line, 1 s |
| Message: undo refused | `NOT AVAIL` | `CANNOT` / `UNDO` | 6 / 4 | main / second, 1 s |
| Message: input live | `INPUT ON` | `INPUT ON` | 8 | one line, 1 s |
| Message: nothing wrong | `CAL OK` | `CAL OK` | 6 | one line, 1 s |
| Message: self test | `PASS` / `FAIL` | `PASS` / `FAIL` | 4 | one line, 1 s |
| Message: code state | `FACTORY` / `CHANGED` | `FACTORY` / `CHANGED` | 7 | one line, 1 s |
| VERIFY result | `PASS` / `FAIL`, then `ERROR %` and the value | unchanged | 4 / 7 / 7 | one line |
| Field not stored | `NOT SET` | `NOT SET` | 7 | one line |
| INFO captions (0.7 s) | `VERSION SERIAL SERIAL 2 CAL DATE CAL COUNT PROD DATE ON HOURS POWER UP` | same, `POWER UPS` last | 7 6 8 8 9 9 8 9 | one line |
| INFO / REVISION version | `V2.0.26.10.22` | unchanged | 9 | one line |
| INFO serial pages | up to 9 characters per page | unchanged | 9 | one line |
| CAL CHECK record name | up to 9 characters | unchanged | 9 | one line |
| CAL DATE editor | `00/00/00` | unchanged | 8 | one line |
| BOOT MSG editor | 9 characters | unchanged | 9 | one line |
| Power-up splash | `BK 5491B` and `001` on the second line | unchanged | 9 / 3 | main / second |
| Panel error text (remote errors) | `BUS: BAD`, `BUS:SYNTAX`, `BUS: DATA`, `BUS:ERROR`, `BUS:LOCKED` | unchanged | 9 | one line |

## 5. The vendor's choice values that were kept

`10%`, `1%`, `0.1%`, `0.01%` (the hold windows), `600` ... `38.4K` (baud), `ON`, `OFF`, `BUS`, `DCV`, `OHM`
are unchanged. They are stored padded with blanks in `tables.c` (`10%  `, `ON    `) as in the original image;
the menu prints them without the padding.

## 6. Strings of ui/display.c (measurement screen, not menu text)

Counted by hand with the same rule; not part of the host check, which reads the menu's and the tables' text.

| String | Cells | Where |
|---|---|---|
| ` OVL.D  ` | 7 | overload text of the number field |
| ` OPEN ` | 6 | open-circuit text of the number field |
| `--.----` | number field, six cells at most | restart dashes |
| `VDC VAC ADC AAC OHM HZ  SEC DCV VAD AAD` | 3 | function label, cells 6-8 |
| `%  `, `dB `, `dBm` | 3 | math label (`d` and `m` render, see section 1) |
| `dB` | 2 | the two-cell dB label of ACV; the third cell keeps what was there |
| `001` | 3 | splash revision on the second line, cell 4 |

`remote/reply.c` holds one more panel text, `BUS:LOCKED` (`fragment_locked`): B, U, S (carrying the colon), L,
O, C, K, E, D = 9 cells. That file is not covered by the host check.

## 7. Vendor strings that were dropped

37 strings of the vendor's 95-entry table (flash 0x0BCC-0x0EE6) are no longer in `menu_text[]`, because
nothing shows them any more: the old prompts (`RETURN:`, `SOUND:`, `R.F:`, `BAUD:`, `RDGS:`, `HOLD:`, `REF:`,
`TERM:`, `BEEP:`, `SOURCE:`, `LO:`, `HI:`, and the blank one), the `RANG: 2.3V` and `RANG: 750V` texts that no
menu reached, the 17 function-label copies that `display.c` keeps for itself, `  GOING. .  `, `CLEAR. . . . `,
`004`, `001` and the six-blank string. They were in the table but referenced by no code. `tools/check_tables.py`
lists each one by its image position with the text that must still be there.
