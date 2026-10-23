/* buzzer.h -- piezo on PA4 (high = on). FROZEN.
 * Steady while continuity is true; a 20 ms pulse for key clicks and error beeps.
 * Must never busy-wait for the steady tone.
 */
#ifndef BK_BUZZER_H
#define BK_BUZZER_H
#include <stdbool.h>
#define BUZZ_PULSE_MS 20u
void buzzer_init(void);
void buzzer_set(bool on);     /* steady, e.g. continuity */
void buzzer_click(void);      /* 20 ms pulse, honours settings.key_click */
void buzzer_error(void);      /* 20 ms pulse, honours settings.beeper */
void buzzer_tick_1ms(void);   /* called from the 1 ms tick to end a pulse */
#endif
