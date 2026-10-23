/* menu.h -- front-panel menus and SYS settings. FROZEN.
 *
 * Menu (SHIFT + MENU key): MATH MENU (HI LIMIT, LO LIMIT, PERC REF, dB REF, dBm REF),
 * TRIG MENU (TRIG MODE, HOLD WIN, HOLD CNT), SYS MENU (BEEPER, BAUD RATE, LINE END, ECHO, KEY SOUND,
 * REVISION = the firmware version, ZERO CAL, and the items this firmware added; MENU_TEXTS.md lists all
 * of them). SYS values are stored to EEPROM on ENTER (0x856 beeper, 0x854 baud, 0x863 TX terminator,
 * 0x865 echo, 0x855 key click).
 *
 * A baud / echo / terminator change is applied consistently through settings_apply(),
 * not only when it is stored (the original applied them inconsistently).
 *
 * SYS item 7 ZERO CAL: an open question. The original capture routine (0x1B76) wrote bad
 * user-zero tables on one meter and the cause is not established -- firmware defect or
 * operator/short quality. DO NOT port that path until re-analyst reports. When it is ported, entry is
 * behind a service unlock entered on the panel through the existing KEY prompt, NOT the fixed code 6666.
 *
 */
#ifndef BK_MENU_H
#define BK_MENU_H

#include <stdbool.h>
#include <stdint.h>

#include "../eeprom_map.h" /* EE_BOOT_MSG_LEN, in the boot-message prototypes below */

void menu_init(void);
bool menu_active(void);
void menu_enter(void);
void menu_key(uint8_t key);

/* Called once per MILLISECOND from the main loop. It drives the 310/290 ms cursor blink and the
 * one-second messages, so a slower cadence changes visible behaviour. */
/* One millisecond of the menu's own timekeeping.
 *
 * Returns TRUE only when that tick BLOCKED on the converter -- today that is the ZERO CAL capture and
 * nothing else. The main loop uses it to stop catching up a backlog within one pass: timekeeping can be
 * replayed eight ticks at a time harmlessly, because it is arithmetic, and eight blocking captures back to
 * back cannot, because nothing else in the loop runs meanwhile and the "any key aborts" the operator was
 * promised is not serviced until the batch ends.
 *
 * It must NOT return true for an activity that merely polls a state variable, such as a calibration sweep
 * or the relay exercise -- their work happens in `cal_sweep_task()` and `meas_task()`, from the main loop.
 * Saying otherwise stops the catch-up for no reason and stretches anything that counts ticks, which is how
 * the SELF TEST lamp phase came to run long. */
bool menu_tick(void);

/* MAIN-LOOP CONTRACT: a function key pressed inside the menu CLOSES the menu without
 * executing. Once menu_key() returns and menu_active() has newly gone false, main dispatches that key
 * normally via keys_dispatch(). display_update() and display_annunciators() draw nothing while
 * menu_active(). menu_enter() clears the panel; closing the menu redraws the measurement screen.
 *
 * menu_init() also applies the stored BRIGHT level to the display controllers, so it must run after
 * vfd_init() and before display_splash(). (The original's blank-EEPROM DEBUG: prompt, is not ported: cal_boot_defaults_init() has no interactive step.) */

/* ---- The boot message --------------------------------------------------------------------------
 * The power-up splash text. `ui_boot_text` fills `out` with EE_BOOT_MSG_LEN characters and a terminator --
 * the stored message, or the default when none was set. `ui_boot_text_set` validates and stores one,
 * returning false for a text that is too long, holds a character the display has no glyph for, or could
 * not be written. Both are here rather than in display.h because the glyph picker that decides what can be
 * shown lives in menu.c. */
void ui_boot_text(char out[EE_BOOT_MSG_LEN + 1u]);
bool ui_boot_text_set(const char *text);

/* Amendment 2026-10-22, from the key-dispatcher port. The original reaches three numeric editors directly
 * from the SHIFT layer, without going through the menu tree at all: SHIFT+REL opens the percent reference
 * (0x063A4), SHIFT+ACDC the dB reference (0x05B5C) and SHIFT+DIODE the dBm reference (0x060BA). `menu.h`
 * had no way to open one, so the dispatcher had nowhere to call.
 *
 * Each of these starts that editor and returns at once: main keeps feeding
 * `menu_key()` and `menu_tick()` until `menu_active()` goes false. On a confirmed ENTER the editor calls
 * `keys_ref_confirmed()` so the dispatcher can finish what the key started -- entering dB or dBm mode, or
 * applying the percent reference. A cancelled editor just closes and calls nothing.
 *
 * STILL MISSING, and it belongs to the menu code: the original also presets the MATH menu
 * cursor (RAM 0x0131 / 0x0E24) when it opens one of these, so that leaving the editor lands on the matching
 * menu item. There is no call for that, and these three entry points do not take a cursor position. */
void menu_edit_percent_ref(void);
void menu_edit_db_ref(void);
void menu_edit_dbm_ref(void);

#endif /* BK_MENU_H */
