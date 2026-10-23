/* main.h -- the few things main.c owns that other modules legitimately need.
 *
 * Deliberately small. `main.c` is the integration point, so almost everything flows the other way: it
 * calls the subsystems and they do not call it. These three are the exceptions, and each exists because
 * two callers must agree on the same answer rather than each keeping its own copy.
 */
#ifndef BK_MAIN_H
#define BK_MAIN_H

#include <stdbool.h>
#include <stdint.h>

/* Power-on hours, from the wear-levelled meter. Read by the SYS INFO screen and by `:SYST:INFO?`.
 * The counter lives in main.c because main owns the tick that advances it. */
uint16_t main_power_on_hours(void);

/* True once 30 minutes have passed since power-on.
 *
 * Two callers need this and must not disagree: the panel's warm-up indication, and CAL RANGE, which warns
 * before letting someone calibrate a cold instrument. The 5491B's specification assumes a warmed-up meter,
 * so a calibration done at two minutes is a calibration to the wrong numbers -- and the operator has no way
 * to tell afterwards. One source of truth for "is it warm" is the point. */
bool main_warmed_up(void);

/* A clean restart: a watchdog reset, so the instrument re-enters through the real reset vector and every
 * peripheral is initialised again from scratch.
 *
 * `*RST` uses this. The original jumped to address zero without re-initialising the UART receive interrupt
 * or the timer enables (a defect of the original), so its soft reset left the instrument in a state that depended on
 * what it had been doing beforehand -- which is the kind of fault that appears only in the field, only
 * sometimes, and never on the bench. */
void main_soft_reset(void);

#endif /* BK_MAIN_H */
