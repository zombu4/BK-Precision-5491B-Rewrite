/* settings.h -- operator settings mirrored between RAM and EEPROM. FROZEN.
 *
 * settings_apply() pushes baud, echo and terminator into the drivers immediately, and
 * uart1_set_baud() writes the FULL 12-bit divisor (UBRR1H and UBRR1L). The original left UBRR1H at 0,
 * which truncated the 600 / 1200 / 2400 baud divisors.
 */
#ifndef BK_SETTINGS_H
#define BK_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#include "../state.h"

void settings_load(void);  /* EEPROM -> g_state.settings */
bool settings_store(void); /* g_state.settings -> EEPROM, verified */
void settings_apply(void); /* push baud / echo / terminator / beeper into the drivers */

extern const uint32_t baud_table[];
extern const uint8_t baud_table_count;

#endif /* BK_SETTINGS_H */
