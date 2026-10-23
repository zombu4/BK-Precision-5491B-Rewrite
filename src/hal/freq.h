/* freq.h -- frequency / period counter on Timer1 (ICP1, PD4) and Timer3 (T3, PE6).
 * FROZEN.
 *
 * The overflow weight is 65536, NOT the original's 65535; the software
 * 3000-iteration timeout becomes a real time-based timeout; period = 1/f is guarded against f == 0.
 * This changes readings by about 15 ppm -- a deliberate change.
 */
#ifndef BK_FREQ_H
#define BK_FREQ_H
#include <stdbool.h>
#include <stdint.h>
#define FREQ_OVERFLOW_WEIGHT 65536UL
#define FREQ_GATE_TIMEOUT_MS 3000u
void freq_init(void);
void freq_gate_start(void);

/* Abandons a gate in progress and stops both timers. Added 2026-10-22: the measurement state machine can
 * leave a gate armed -- a function change, a restart or a converter fault part-way through one -- and
 * without this it stays armed, counting input edges, until the next freq_gate_start() happens to reset it.
 * That is harmless to the NEXT reading, because arming resets everything, but it leaves the capture
 * interrupt live with no one waiting for it, and "harmless because nobody looks" is the kind of state that
 * stops being harmless the moment something else starts looking. */
void freq_gate_abort(void);
bool freq_result_ready(void);
bool freq_read(float *hz, uint16_t timeout_ms);
bool freq_period_read(float *seconds, uint16_t timeout_ms); /* guarded against zero frequency */
#endif
