/* display.h -- reading formatting, annunciators and the refresh cycle. FROZEN.
 *
 * ui/display.c owns the reading/value formatters (the 13 fmt_reading_* / fmt_value_* functions). menu.c
 * and settings.c must not write display cells directly -- they call display_show_text() instead.
 *
 * Number text comes from fmt.h (fmt_fixed); glyphs and the annunciator table come from tables.h.
 */
#ifndef BK_DISPLAY_H
#define BK_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "../state.h"

void display_init(void);
void display_splash(void); /* the boot banner, a beep and the 1.2 s hold */

/* The main refresh, called from the main loop: number, label, lamps, and the second display or the MAX/MIN
 * text. It also does the work of the original's separate hold-refresh loop (0xAEB4), which drew the same
 * frame from the held reading. Draws nothing while a menu owns the panel. */
void display_update(void);

/* Original disp_clear_all 0x1FD4: blanks both controllers. The menu calls it on entry; the next
 * display_update() after the menu closes redraws the measurement screen from scratch. */
void display_clear_all(void);

/* How many cells of text each line takes, the main line and the second one alike.
 *
 * Main line: the text printers stop at VFD_TEXT_CELLS (vfd.h); cell 9 is the flag byte and cells 10 and 11
 * of the controller's memory (0x24 bytes, three per cell) are never written by any text path, so they stay
 * blank while a menu is open.
 * Second line: the original's streaming printer for controller 2 (0x2B4E) ends when the next cell address
 * reaches 0x1B (the test at 0x2D2E is `cpi r24, 0xDB`, which is 0xC0 | 0x1B), that is after cells 0-8. The
 * measurement screen uses the same nine: six number cells and a three-cell label (display.c LABEL_CELL),
 * and cell 9 is its flag cell. display.c asserts both counts against the flag cell.
 *
 * A '.' or ':' directly after a letter or digit shares that cell and costs none; see tables.h. */
#define DISPLAY_TEXT_CELLS 9u

/* One line of menu or message text on the main display, from cell 0, blank-filled to the nine text cells so
 * that nothing of the previous screen survives. The second line is left as it is. */
void display_show_text(const char *s);

/* The same, with the cell holding character `char_index` blank -- a blinking cursor that cannot shift the
 * line. */
void display_show_text_blank_cell(const char *s, uint8_t char_index);
void display_show_text_P(const char *s_flash);

/* The second line of a two-line menu screen, from cell 0, blank-filled to its nine text cells. NULL clears
 * it, and costs nothing when it is already clear, so a menu can call it with every screen it draws.
 * display_clear_all() and the first display_update() after a menu closes clear it too. */
void display_show_second_P(const char *s_flash);

/* There is no `display_show_text_at` or `display_show_text_at_P` for column-cursor editors.
 *
 * They are not needed. `ui/menu.c` composes a whole line in its own buffer, column by column, and makes
 * ONE `display_show_text()` call per frame, which is better than a sequence of partial cell writes: the
 * panel cannot be seen half-updated, and there is one place that decides what a line looks like. If a
 * future subsystem genuinely needs to write at a column, it adds the call together with its definition.
 *
 * WITHDRAWN 2026-10-22: `display_second`, `display_show_reading` and `display_hold_refresh`.
 * Each was defined, compiled, linked and called by nobody, so the second display and the MAX/MIN text were
 * never drawn. The first is now the static draw_second() inside display_update(); the other two drew what
 * display_update() draws. */

/* The lamps, derived from g_state. Does nothing while a menu owns the panel. */
void display_annunciators(void);

#endif /* BK_DISPLAY_H */
