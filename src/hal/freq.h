/* freq.h -- frequency / period counter on Timer1 (ICP1, PD4) and Timer3 (T3, PE6).
 *
 * The overflow weight is 65536, NOT the original's 65535; the software
 * 3000-iteration timeout becomes a real time-based timeout; period = 1/f is guarded against f == 0.
 * This changes readings by about 15 ppm -- a deliberate change.
 *
 * The timeout is part of the gate. It is armed by freq_gate_start(), enforced by the Timer1 overflow
 * interrupt, and adapts to the input: the measurement is abandoned when the gate time plus two periods of the
 * measured input have passed (see freq.c). Nothing outside this module counts time for the counter.
 */
#ifndef BK_FREQ_H
#define BK_FREQ_H
#include <stdbool.h>
#include <stdint.h>
#define FREQ_OVERFLOW_WEIGHT 65536UL
void freq_init(void);
void freq_gate_start(void);

/* Abandons a gate in progress and stops both timers. Added 2026-10-22: the measurement state machine can
 * leave a gate armed -- a function change, a restart or a converter fault part-way through one -- and
 * without this it stays armed, counting input edges, until the next freq_gate_start() happens to reset it.
 * That is harmless to the NEXT reading, because arming resets everything, but it leaves the capture
 * interrupt live with no one waiting for it, and "harmless because nobody looks" is the kind of state that
 * stops being harmless the moment something else starts looking. */
void freq_gate_abort(void);

/* Where the armed gate is. RUNNING: keep polling. DONE: freq_read() / freq_period_read() will deliver.
 * NO_SIGNAL: the deadline passed, or no gate is armed; the caller reads zero. */
typedef enum { FREQ_GATE_RUNNING = 0, FREQ_GATE_DONE, FREQ_GATE_NO_SIGNAL } freq_gate_t;
freq_gate_t freq_gate_poll(void);

/* Both consume a finished gate and do not wait. False when the gate has not finished or holds no usable
 * window. */
bool freq_read(float *hz);
bool freq_period_read(float *seconds); /* guarded against zero frequency */
#endif
