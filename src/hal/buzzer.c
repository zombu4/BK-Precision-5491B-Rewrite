/* buzzer.c -- piezo on PA4 (high = on).
 *
 * Original: beeper_ctrl 0x3556, which took 'A' (steady on), 'S' (off) or
 * anything else (a 20 ms pulse produced by busy-waiting). Here the pulse is
 * timed by the 1 ms tick, so nothing ever waits.
 */
#include "buzzer.h"

#include <avr/io.h>
#include <util/atomic.h>

#include "../board.h"
#include "../state.h"

static volatile bool steady_on;
static volatile uint8_t pulse_left_ms;

/* The pin is high while a steady tone is requested or a pulse is still running.
 */
static void drive_pin(void) {
  if (steady_on || pulse_left_ms != 0u) {
    BUZZ_PORT |= (uint8_t)(1u << BUZZ_BIT);
  } else {
    BUZZ_PORT &= (uint8_t)~(1u << BUZZ_BIT);
  }
}

static void start_pulse(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pulse_left_ms = BUZZ_PULSE_MS;
    drive_pin();
  }
}

void buzzer_init(void) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    steady_on = false;
    pulse_left_ms = 0u;
    drive_pin();
  }
}

void buzzer_set(bool on) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    steady_on = on;
    drive_pin();
  }
}

void buzzer_click(void) {
  if (g_state.settings.key_click) {
    start_pulse();
  }
}

void buzzer_error(void) {
  if (g_state.settings.beeper) {
    start_pulse();
  }
}

void buzzer_tick_1ms(void) {
  if (pulse_left_ms != 0u) {
    pulse_left_ms--;
    drive_pin();
  }
}
