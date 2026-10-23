/* meas.h -- the measurement chain: front-end switching, conversion, averaging, correction, statistics.
 *
 * This header follows the original's behaviour, recovered from the firmware. An earlier draft was wrong in
 * three ways that mattered, and all three are worth stating so the same assumptions are not made again:
 *
 *   - It offered `meas_read_primary()` as "applies calibration and the user zero". `meas_read_primary` is
 *     also, confusingly, what the original's SINGLE-SAMPLE routine does, which this port calls
 *     `cal_sample`. A reading is not one sample: it is N conversions averaged, and only then corrected.
 *   - It implied a blocking call. The measurement CANNOT block: one reading is 60 to 220 ms of
 *     conversions, an ohm range-0 slow reading is 800 ms, and an AC+DC V reading is nearly a second. The
 *     original did block, and that is the other half of ORIGINAL_BUGS #1 -- while it blocked, the host link was deaf and
 *     there was no watchdog. This header is a state machine for that reason.
 *   - `meas_counts_to_reading(code, f, range)` suggested the correction is a pure function of one code. It
 *     is not: the zero comes off every sample, the gain applies to the mean, and the add-on stages need the
 *     function and the range. That work belongs to cal.c and is reached through `cal_sample` and
 *     `cal_apply`.
 *
 * ---- One pass, as the instrument does it --------------------------------------
 * The converter free-runs at one conversion per 20.0 ms for every rate; the rates differ only in how many
 * conversions are averaged (3 / 6 / 11, with per-function exceptions). The multiplexer and the gain are NOT
 * touched per reading -- only when the function, range or rate changes, and then a fixed number of
 * conversions are discarded while the front end settles.
 *
 * The order after the average is fixed and is NOT the obvious one:
 *     correct -> OVERLOAD TEST -> dB/dBm -> copy to `reading.raw` -> REL -> percent -> limit -> MIN/MAX
 * so the overload test sees the corrected value, and `reading.raw` -- which is what ACQuire and the REL key
 * capture -- holds the value AFTER dB/dBm and BEFORE REL. See state.h.
 *
 * ---- The front-end switching order is kept ---------------------------------------------------------
 * UNVERIFIED: what each relay and multiplexer bit does electrically -- check: a continuity trace on the
 * board, or a schematic. There is none available. Nothing here decides anything from those meanings:
 * the original's per-bit write ORDER and its settling conversions are reproduced exactly. Do not reorder
 * them, do not batch them, and do not add break-before-make logic the original did not have -- that stays
 * BLOCKED until a continuity check of the board, because deciding which relay to open first requires knowing
 * which relay it is.
 */
#ifndef BK_MEAS_H
#define BK_MEAS_H

#include <stdbool.h>
#include <stdint.h>

#include "../cal/cal.h" /* cal_set_t: a sweep measures with its own coefficient set */
#include "../state.h"

/* ---- Settling, in CONVERSIONS rather than milliseconds -----------------------------------------------
 * The "5 conversions for AC V" figure belongs
 * to the range-only routine that only the secondary legs use. The routine used for an AC V function or
 * range change discards 3. Counting conversions rather than milliseconds also keeps these right if the line
 * frequency changes the conversion rate. */
/* These are the vendor's CONVERSION counts, and the implementation
 * treats each as the DURATION it represents at 50 Hz and converts it back to a count at whatever the line
 * frequency currently is -- rounding tDATA down, so a settle is never short. Otherwise selecting 60 Hz
 * would silently shorten every settle by a sixth, because the conversions themselves get faster. */
#define MEAS_SETTLE_DCV 5u      /* DC V function or range change */
#define MEAS_SETTLE_RANGE_ONLY 5u /* the secondary legs' range-only path */
#define MEAS_SETTLE_DEFAULT 3u  /* every other function or range change */
#define MEAS_SETTLE_RATE 3u     /* a rate change */
#define MEAS_SETTLE_SECONDARY 20u /* after switching the front end for a secondary reading */

/* ---- The pass state machine -------------------------------------------------------------------------- */

typedef enum {
    MEAS_IDLE = 0,
    MEAS_SETTLING,      /* discarding conversions after a hardware change */
    MEAS_PROBING,       /* the one-conversion autorange probe */
    MEAS_ACCUMULATING,  /* averaging the N conversions of the primary reading */
    MEAS_SECONDARY_SETTLING,
    MEAS_SECONDARY_READ,
    MEAS_AC_SETTLING,   /* the AC leg of an AC+DC reading */
    MEAS_AC_READ,
    MEAS_FREQ_GATING,   /* the counter owns the measurement; see hal/freq.h */
    MEAS_COMPLETE,      /* a reading is ready; cleared by meas_take_reading() */
    MEAS_FAULT          /* the converter stopped responding; see meas_fault_take() */
} meas_phase_t;

void meas_init(void);

/* Called from the main loop as often as it likes. It performs AT MOST one conversion's worth of work and
 * returns, so the loop keeps servicing the host link, the keys and the watchdog throughout a reading. This
 * is the structural half of ORIGINAL_BUGS #1 that does not live in uart1.c. */
void meas_task(void);

meas_phase_t meas_phase(void);

/* True once per completed pass. The caller consumes the reading from g_state.reading and this returns to
 * false until the next one, so a reply cannot be sent twice for one reading. */
bool meas_reading_ready(void);
void meas_take_reading(void);

/* ---- Settings changes --------------------------------------------------------------------------------
 * Each of these aborts the pass in progress and restarts it, because a reading that straddles a front-end
 * change is a reading of neither setting. The original did not do this -- it finished the pass it was in --
 * and that is where some of its first-reading-after-a-change errors come from. */
void meas_set_function(func_t f);
void meas_set_range(func_t f, uint8_t range, bool manual);
void meas_set_rate(rate_t r);
void meas_invalidate(void); /* discard the pass in progress and start a fresh one */

/* Writes the front-end port bits in the original order (kept as the original has it) and returns how many conversions
 * must be discarded afterwards. Split from the settling itself so the state machine can wait without
 * blocking -- the original did both in one blocking routine. */
uint8_t meas_frontend_apply(func_t f, uint8_t range);

/* ---- Autorange ---------------------------------------------------------------------------------------
 * ONE single-conversion probe decides, there is no agreement count and no iteration cap,
 * and the hysteresis is the gap between the lower and the upper threshold table. AC+DC V and AC+DC I
 * autorange on the DC component only, which is confirmed and has an electrical consequence.
 * Only meas_task() calls this; it is declared for tests. */
bool meas_autorange_step(void);

/* ---- Overload ----------------------------------------------------------------------------------------
 * The test compares the corrected magnitude against the function's AUTORANGE UPPER threshold for the
 * current range -- NOT against `overload_limit` in tables.h, which is dead flash data with
 * no reference anywhere in the image. Using the wrong table would trip at a different point
 * from the original on every range but the top one.
 *
 * There is no hardware sense line: no input pin can detect an overload or a blown fuse (ORIGINAL_BUGS
 * #106). An overload is only ever "the reading is above the limit". */
bool meas_overload(void);

/* ---- The statistics stages, in pipeline order -------------------------------------------------------
 * Declared individually because the ORDER is protocol-visible and belongs in one place rather than being
 * re-derived by each caller. meas_task() applies them in exactly this sequence. */
float meas_apply_db(float corrected, func_t f, bool *out_of_range);
float meas_apply_rel(float value, func_t f);
float meas_apply_percent(float value);
void meas_update_limits(float value, func_t f);
void meas_update_minmax(float value);
void meas_minmax_reset(void);

/* ---- HOLD -------------------------------------------------------------------------------------------
 * The original's HOLD is a DISPLAY FREEZE: it stops measuring altogether and spins in a redraw loop, which
 * is also why it locks the remote port (ORIGINAL_BUGS #9). Nothing in the image reads the WINDow or COUNt
 * settings at all, so the documented capture behaviour does not exist (#32).
 *
 * The port implements this rule: once HOLD is armed, keep measuring, and latch when COUNt
 * consecutive readings stay within WINDow -- a fraction of the first of them -- then freeze and beep if the
 * beeper is on. The port keeps running the host link throughout. */
void meas_hold_arm(void);
void meas_hold_release(void);

/* ---- The relay exercise -----------------------------------------------------------
 * Works the relays to clear contact film, which is what a meter that has sat unused needs. Driven from the
 * state machine like everything else here, one step per `meas_task()` call, so the panel countdown and the
 * host link keep running throughout.
 *
 * `quick` selects the short power-on pass (about 1.5 s, run from main when
 * RLY BOOT is enabled) rather than the full 60 s sequence the SYS menu offers.
 *
 * **It refuses to start, and aborts if already running, when the DC V reading exceeds 2 V**.
 * Exercising the relays switches a signal source across the front end, and the guard exists so that nobody
 * does that to an instrument still connected to something. The check is made before the first step and
 * again at least once a second after that, not once at the beginning and not before every pattern.
 *
 * Returns false when the guard refuses, in which case nothing moves. The function, range and relay state
 * are restored on every exit path, including the guard abort.
 *
 * BLOCKED-ON-HARDWARE: the exact patterns stay as the original writes them and nothing break-before-make
 * is added, because the board's relay continuity has not been checked. */
/* `full` selects the 60 s menu sequence; false is the ~1.5 s power-on pass. */
bool meas_relay_exercise_start(bool full);
void meas_relay_exercise_abort(void);
uint8_t meas_relay_exercise_seconds_left(void);

typedef enum {
    RLYEX_IDLE = 0,
    RLYEX_RUNNING,
    RLYEX_DONE,
    RLYEX_ABORTED,
    RLYEX_INPUT_ON, /* the 2 V guard tripped: something is still connected to the terminals */
    RLYEX_FAULT
} relay_exercise_result_t;

relay_exercise_result_t meas_relay_exercise_result(void);

/* ---- Line frequency ---------------------------------------------------------------
 * Sets the converter's decimation so that mains hum is notched -- 767 for 50 Hz, 639 for 60 Hz. Getting it
 * wrong costs noise rejection rather than accuracy, which is why the command is open and not protected.
 *
 * A change runs a converter self-calibration, so it is REFUSED (returns false) while a sweep owns the
 * converter rather than being queued: a calibration that moved the conversion rate underneath itself would
 * produce a reading nobody could account for afterwards.
 *
 * The setting is EE_SET_LINE, and only the value 1 means 50 Hz -- 0 and the erased 0xFF both mean 60 Hz.
 * That is the default rule from eeprom_map.h: an erased byte is the normal state on a meter
 * that has not run this firmware. */
bool meas_set_line(bool is_50hz);
bool meas_line_is_50hz(void);

/* ---- Converter self-calibration on demand (SELF TEST) ------------------------------------------------
 * The converter already self-calibrates on its own whenever the PGA changes or the line frequency is set;
 * these two are for the SELF TEST screen, which has to *show a result*, and that is the only thing the
 * automatic path does not provide.
 *
 * `meas_selfcal_start()` asks for one and returns false if the converter is busy with something else --
 * claimed by a sweep, or in the middle of a relay exercise. `meas_selfcal_done(&ok)` returns false until it
 * has finished and then true exactly once, with `ok` false if the converter did not answer in time.
 *
 * Non-blocking: the work happens in meas_task(), so the caller must keep letting the main loop run. */
bool meas_selfcal_start(void);
bool meas_selfcal_done(bool *ok);

/* ---- Faults -----------------------------------------------------------------------------------------
 * The original polled the converter's ready line with no timeout and no watchdog, so a
 * converter that stopped answering hung the instrument with the last reading on the display. Here the wait
 * is bounded; a timeout moves the phase to MEAS_FAULT and is reported once through the error queue. */
bool meas_fault_take(void);

/* Called once per MILLISECOND, from the same 1 ms tick as the key and buzzer ticks.
 *
 * This is what makes the converter stall detectable. Without it the stall timer never advances, so a
 * converter that stops answering is never noticed and the instrument sits with its last reading on the
 * display -- which is exactly the original's behaviour, reproduced by omission. Declared here
 * rather than left as a local prototype in meas.c so that main.c cannot forget it silently. */
void meas_tick_1ms(void);

/* ---- Exclusive use of the converter, for the calibration sweeps -------------------------------------
 * A calibration sweep measures with the IDENTITY coefficients and a zero of zero, which
 * is never the set the measurement path has loaded, so it cannot just ask for a reading. Driving the
 * converter directly through `adcbus` would be a race, not merely untidy layering: once `meas_task()` is running a pass, two pieces of code own
 * the converter, and a sweep taking conversions in the middle of someone else's average corrupts both. The
 * instrument has one converter and exactly one thing may be reading it.
 *
 * So a sweep claims it. `meas_claim()` aborts any pass in progress and refuses until released;
 * `meas_average_with()` then takes `samples` conversions and returns their corrected mean using the
 * caller's set; `meas_release()` hands the converter back and invalidates the pass so the next one reloads
 * its coefficients. A sweep that fails part-way must still release.
 *
 * `cal_ops.c` is to be switched onto these when `meas.c` exists. */
bool meas_claim(void);
void meas_release(void);
bool meas_average_with(const cal_set_t *set, uint8_t samples, float *out);

/* The 6-conversion average behind the front-panel ZERO CAL capture.
 * The original routine is arithmetically correct; a bad stored zero cannot be explained from the
 * firmware alone. What this port adds is a plausibility limit -- a captured zero that is implausible for the range is refused --
 * and the signature that the original never wrote for those tables. */
bool meas_zero_capture(func_t f, uint8_t range, float *out);

#endif /* BK_MEAS_H */
