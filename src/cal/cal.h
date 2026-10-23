/* cal.h -- calibration store: EEPROM record codec, the correction maths, and the :SCAL operations.
 * The remote layer routes :SCAL commands here and never touches EEPROM itself.
 *
 * The arithmetic was recovered from the firmware. There is no single `cal_apply(c, raw)` over
 * `{gain, offset, zero}`: the instrument does not apply a gain and an offset to a raw code.
 *
 * ---- What the instrument actually does ---------------------------------------
 * Stage 0, PER SAMPLE (0x0AC56):   s = (+/-2.86e-7 * code) * scale - zero
 *   The zero comes from the 14-byte record for this function, range AND RATE, and is subtracted from every
 *   sample before anything else.
 * Stage 1 (0x0A3F2):               m = mean of N samples, N = 3 / 6 / 11 for fast / medium / slow.
 * Stage 2, PER FUNCTION, on the MEAN -- never on a single sample. With a quadratic form the two orders are
 *   not equivalent, which is why this distinction has to be in the interface and not left to a comment.
 *
 * There are four forms, not one:
 *   LINEAR_SIGNED  DCV, ACDCV          y = m * (m >= 0 ? main.f0 : alt.f0)        f1 is never read
 *   QUAD_SIGNED    DCA, ACDCI          y = m * (p.f0*m + p.f1), p = sign-selected pair
 *   QUAD           ACV, ACA, FREQ, PERIOD, CONT, DIODE   y = m * (main.f0*m + main.f1)
 *   OHM_2SEG       OHM                 y = m * (p.f0*m + p.f1), p = main below ohm_threshold[R], else alt
 * The identity is (f0,f1) = (0,1) for every quadratic form and (1,0) for the linear one. `main.f0` is the
 * QUADRATIC coefficient and `main.f1` the LINEAR one -- `eeprom_map.h` calling f1 an "offset copy" is true
 * only of what the vendor tool happens to store in DC V pairs, where f1 equals the zero value.
 *
 * Then, only when no calibration sweep is running:
 *   - the user zero is subtracted, for DCV/ACDCV (EEPROM 0x880 + 4R) and OHM (0x898 + 4R) only, AFTER the
 *     gain stage;
 *   - a dead-band forces small readings to zero (DCV and OHM have different tables);
 *   - for ACV and ACA on the lowest range the result is REPLACED for small inputs by one of three
 *     low-range corrections.
 *
 * ---- Integrity --------------------------------------------------------------------------------
 * A record write is atomic and verified at the moment it happens: invalidate the signature, write the data,
 * read it back and verify, then write the valid signature LAST. A power loss therefore leaves a record
 * INVALID, never half-valid. The original instead leaves the OLD signature valid
 * while it rewrites the data, so an interruption there leaves a record that looks good and is not.
 *
 * :SCAL:SAVE keeps its original meaning: it increments the u16 counter at EEPROM 0x861 and does nothing
 * else. Writes are NOT staged until save; staging would silently lose a calibration whenever
 * a client never sent save.
 *
 * ---- The negative DC V pair: a deliberate difference from the original -----------------------------
 * Rule: use the negative record only when its signature is valid, otherwise fall back to the positive
 * pair. On a meter whose DC V negative records are all cleared, that reproduces its readings exactly.
 * What the original really does differs twice:
 *   1. On the DCV PRIMARY path the original reads the negative record, then overwrites it with the positive
 *      pair unconditionally at 0xCD68 -- so a negative reading is always scaled by the POSITIVE gain, even
 *      when a valid negative record exists. The negative calibration data can never take effect at all.
 *   2. For ACDCV and the DC-type secondary the original falls back to a gain of 1.0, not to the positive
 *      pair, when the negative record is invalid -- i.e. an uncalibrated negative reading comes out raw.
 * The rule is kept as implemented -- the negative pair when
 * its signature is 0x5554, otherwise the positive pair, on the primary path and on the AC+DC and secondary
 * paths alike. So both differences from the original stand, deliberately: the negative calibration table
 * becomes live rather than being dead data, and an uncalibrated negative reading comes out nearly right
 * rather than raw.
 *
 * One consequence belongs to the PC application, not to this firmware, and is that
 * because the negative records now take effect as soon as they are written, BK-Calibrator must calibrate
 * BOTH POLARITIES. Writing only the positive side would leave the negative one on whatever the vendor tool
 * last stored, which on this meter is a cleared record -- and that was harmless only while the firmware
 * ignored it.
 *
 * :SCAL:SHOR:ONCE / :SCAL:OPEN:ONCE must check function AND range before any
 * write; on a mismatch the caller reports a normal error and NOTHING is written.
 */
#ifndef BK_CAL_H
#define BK_CAL_H

#include <stdbool.h>
#include <stdint.h>

#include "../eeprom_map.h"
#include "../state.h"

/* ---- The correction set ------------------------------------------------------------------------------- */

typedef enum {
    CAL_FORM_LINEAR_SIGNED = 0, /* DCV, ACDCV */
    CAL_FORM_QUAD_SIGNED,       /* DCA, ACDCI */
    CAL_FORM_QUAD,              /* ACV, ACA, FREQ, PERIOD, CONT, DIODE */
    CAL_FORM_OHM_2SEG           /* OHM: main below the threshold, alt above */
} cal_form_t;

typedef struct {
    float f0; /* the quadratic coefficient, or the gain in the linear form */
    float f1; /* the linear coefficient; not read at all in the linear form */
} cal_pair_t;

/* Everything stage 0 and stage 2 need for the current function, range and rate. The loaders fill it. */
typedef struct {
    cal_form_t form;
    float scale;      /* per function and range, from the flash range_scale table */
    float zero;       /* per function, range AND rate, from the 14-byte record; subtracted PER SAMPLE */
    cal_pair_t main;
    cal_pair_t alt;   /* the negative pair, or the ohm upper segment */
    bool alt_valid;   /* false means the alt record's signature was not valid: see the negative-pair note above */
    cal_pair_t low[3]; /* ACV and ACA lowest range only, three brackets by input size */
    float uzero;      /* user zero for this range; DCV, ACDCV and OHM only, applied AFTER the gain */
    float ohm_threshold; /* OHM only: half of the range's nominal full scale */
} cal_set_t;

/* Flags for cal_apply. */
#define CAL_F_SWEEP 0x01u /* a SHOR / OPEN / STD sweep is running: skip the user zero, the dead-bands and
                           * the low-range correction, exactly as the original's M0B0F/10/11 do */

/* Stage 0: one sample. `code` is the magnitude of the 24-bit conversion and `negative` its sign bit, which
 * the converter reports separately. */
float cal_sample(const cal_set_t *c, uint32_t code, bool negative);

/* Stage 2: the gain or quadratic, applied to the MEAN of the samples. Never skipped, not even during a
 * sweep -- which is why the sweeps load the identity coefficients first. */
float cal_apply(const cal_set_t *c, float mean, uint8_t range, uint8_t flags);

/* The add-on stages, in the one order the instrument uses: user zero, then the dead-band, then the AC
 * low-range replacement. All three are skipped while a sweep runs.
 *
 * Separate from cal_apply because they need the FUNCTION, which cal_set_t does not carry, and because
 * keeping the order in one place is the point: `mean` is still needed here, since the AC low-range brackets
 * are tested on the pre-gain mean and not on the corrected value. Call it with the output of cal_apply. */
float cal_apply_addons(const cal_set_t *c, func_t f, float mean, float y, uint8_t range, uint8_t flags);

void cal_load(func_t f, uint8_t range, rate_t rate, cal_set_t *out);

/* ---- Record addressing -------------------------------------------------------------------------------- */

typedef enum {
    CAL_PAIR_MAIN = 0,
    CAL_PAIR_NEG,   /* DC V and DC A only */
    CAL_PAIR_OHM_Y, /* the ohm upper segment, EEPROM 0x0DC0 */
    CAL_PAIR_LOW1,  /* AC V / AC A lowest range, the three low-input brackets */
    CAL_PAIR_LOW2,
    CAL_PAIR_LOW3
} cal_variant_t;

/* Returns EE_ADDR_NONE when the combination does not exist, which is the only safe answer: the original
 * had no bounds check anywhere and the EEPROM address register wraps modulo 4096, so a bad index silently
 * landed on another record. */
#define EE_ADDR_NONE 0xFFFFu

uint16_t cal_pair_address(func_t f, uint8_t range, rate_t rate, cal_variant_t variant);
uint16_t cal_zero_address(func_t f, uint8_t range);

/* ---- Records ------------------------------------------------------------------------------------------ */

bool cal_read_pair(uint16_t addr, ee_pair_t *out);      /* false when the signature is not valid */
bool cal_pair_usable(func_t f, const ee_pair_t *p);      /* false for a zero gain or a non-finite term */
bool cal_write_pair(uint16_t addr, float f0, float f1); /* invalidate, data, verify, signature last */
bool cal_read_zero(uint16_t addr, ee_zero_t *out);
bool cal_write_zero(uint16_t addr, const float z[RATE_COUNT]);

/* ---- ZERO CAL: the user zero (SYS item 7) -------------------------------------------------
 * Stores the value `meas_zero_capture()` measured, for DC V / AC+DC V or for ohms, at one range. This is
 * the call the menu had no way to make: the capture worked and the arithmetic was settled, and then the result
 * had nowhere to go, so SYS item 7 would end in NOT AVAIL.
 *
 * Protected, like every other path that changes what the instrument reads.
 *
 * NOTE ON THE WRITE ORDER, because it differs from the records above on purpose. A user-zero slot is a
 * bare float with NO SIGNATURE of its own -- the word at EE_UZERO_DCV_SIG guards the DC V block as a whole
 * and only the boot initialiser reads it (cal.c, the loader reads the float unconditionally, exactly as
 * the original does). There is therefore no signature to invalidate first: the invalidate-write-verify-
 * revalidate dance protects a record that can be half-written and still look valid, and four bytes written
 * by `ee_write_f32`, which reads back every byte, cannot be. Writing a signature this field does not have
 * would change the format the vendor firmware expects. */
bool cal_write_user_zero(func_t f, uint8_t range, float value);

bool cal_save_counter_increment(void); /* :SCAL:SAVE -- the counter at 0x861 and nothing else */

/* ---- :SCAL:COEF ---------------------------------------------------------------------------------------
 * The remote layer splits and range-checks the parameter; this takes typed fields. The original accepted a
 * string whose field count it never checked, repeated the last field when the string was short, read
 * integers from their first character only, and wrote even after reporting an error. Hence a typed outcome rather than the original's silence. */
typedef enum {
    CAL_COEF_OK = 0,
    CAL_COEF_BAD_FUNCTION, /* the original accepted functions 5..10 and wrote nothing, with no reply */
    CAL_COEF_BAD_COUNT,
    CAL_COEF_BAD_BAND,     /* the A / B selectors; the original let them wrap */
    CAL_COEF_BAD_RANGE,
    CAL_COEF_BAD_VALUE,    /* a non-finite coefficient; see the note on cal_coef_t below */
    CAL_COEF_LOCKED,       /* the service lock refused it: added 2026-10-22, see below */
    CAL_COEF_WRITE_FAILED  /* the record did not verify: nothing valid was left behind */
} cal_coef_result_t;

/* Three points about the fields:
 *
 *  - `z[]` is NOT indexed by rate. It holds the three zero fields in WIRE order -- fields 7, 8 and 9 of the
 *    parameter, which are called Z2, Z3 and Z4. The record they go into stores them in a
 *    different order again (fast = Z3, medium = Z2, slow = Z4), and cal_coef_write does that mapping. Indexing
 *    by rate would have the remote layer pre-sort them and the mapping applied twice. The remote layer fills `z[]` in the order the bytes arrived, and nothing else.
 *  - `CAL_COEF_LOCKED` exists. Without it a refusal by the service lock had to come back as
 *    WRITE_FAILED, so a caller could not tell "you are not unlocked" from "the EEPROM did not verify" --
 *    and those need different replies and different advice to the user.
 *  - `CAL_COEF_BAD_VALUE` exists. The original validated no value at all and would happily store a
 *    NaN or an infinity as a coefficient, after which every reading on that range is
 *    nonsense and cannot be read back to diagnose. Non-finite values are refused here, in the module that
 *    owns the record, rather than being left to whichever caller remembers.
 */
typedef struct {
    float v[6];  /* the six coefficient fields, in WIRE order (medium, fast, slow pairs) */
    float z[3];  /* the three zero fields in WIRE order: fields 7, 8, 9 = Z2, Z3, Z4 */
    float x[6];  /* the extra fields: the ohm Y set, or the AC low set 2 partners */
    uint8_t a;   /* the band or range selector; meanings are per function */
    uint8_t b;
    uint8_t count; /* how many fields the caller actually parsed, so the count can be CHECKED */
} cal_coef_t;

cal_coef_result_t cal_coef_write(func_t f, const cal_coef_t *c);

/* ---- Where a sweep's result lines go -----------------------------------------------------------------
 * ADDED 2026-10-22. The sweeps produce one result line per cell -- one for a ONCE form, twelve for
 * :SCAL:SHOR:ALL, six for :SCAL:OPEN:ALL -- and the first implementation had cal_ops.c call `uart1_write`
 * directly to emit them. That is the wrong layer: the calibration module would own a reply format, it
 * could not be exercised against the test fake without a UART, and a future front-panel caller would get
 * serial output as a side effect of pressing a key.
 *
 * So a sweep reports through this instead, and knows nothing about where the line goes. The remote executor
 * registers a reporter that formats and sends; a test registers one that collects. With no reporter
 * registered a sweep still runs and still writes the EEPROM -- the measurement is the point, the lines are
 * an accessory -- so a front-panel caller simply passes nothing.
 *
 * The caller remains responsible for the trailing "!" that these commands end with:
 * that terminator belongs to the command, not to a cell. */
typedef void (*cal_report_fn)(func_t f, uint8_t range, float value);
void cal_set_reporter(cal_report_fn report); /* NULL to clear */

/* ---- Guarded sweeps, as start/poll pairs -------------------------------------------------------------
 * These could have been four blocking calls, and a full sweep is up
 * to about thirty seconds of settling and conversions. Blocking for that long is a defect of the original:
 * in the original, interrupts were off for the whole sweep and every byte a host sent was lost; here the
 * receive ring keeps filling, so nothing is lost under 256 bytes, but no command would EXECUTE for half a
 * minute. That is the worst moment to look hung -- an operator watching a calibration is the person most
 * likely to decide the instrument has died and pull the cable, and pulling it during a record write is the
 * one event that leaves a record half-valid.
 *
 * So a sweep is driven from the main loop, like a reading:
 *   1. `cal_*_start()` validates function and range, claims the converter, and returns false if the guard
 *      refuses -- in which case nothing is started and nothing is written.
 *   2. the main loop calls `cal_sweep_task()` until it stops returning CAL_SWEEP_RUNNING, servicing the
 *      host link, the keys and the watchdog in between;
 *   3. each completed cell is reported through the reporter as it happens, so a client sees progress
 *      rather than silence;
 *   4. the caller sends the trailing `!` when the state becomes CAL_SWEEP_DONE.
 *
 * All four still write nothing on a guard mismatch, and all four restore the function, range, rate and
 * coefficient state on EVERY exit path including the failures -- which the original does not: it leaves the
 * instrument on rate 2, function 0, every range one past its maximum and the identity coefficients loaded,
 * so a sweep silently changes what the meter is doing afterwards.
 *
 * `cal_sweep_abort()` is for the rule that an abort means nothing further is written. The cells
 * already written stay valid, because each is one atomic record write; the sweep simply stops where it is
 * and restores the instrument. */
typedef enum {
    CAL_SWEEP_IDLE = 0,
    CAL_SWEEP_RUNNING,
    CAL_SWEEP_DONE,
    CAL_SWEEP_FAILED
} cal_sweep_state_t;

bool cal_short_once_start(func_t f, uint8_t range);
bool cal_open_once_start(func_t f, uint8_t range);
bool cal_short_all_start(void);
bool cal_open_all_start(void);

cal_sweep_state_t cal_sweep_task(void);  /* advances one step; call from the main loop */
cal_sweep_state_t cal_sweep_state(void);
void cal_sweep_abort(void);

/* True while a sweep owns the converter. The executor refuses a conflicting command with a busy error
 * rather than interleaving it: two things driving the front end during a calibration measurement is how a
 * wrong value gets written and believed. */
bool cal_sweep_busy(void);

/* ---- Panel calibration of one range -----------------------------------------------------------
 * These two ride on the sweep machine above, so they are started here and then watched through
 * `cal_sweep_state()` exactly like a short or open sweep: one state for every long calibration operation,
 * rather than two that could disagree about whether the converter is busy.
 *
 * `reference` is what the better meter reads, collected from the operator before the flow starts. Both
 * return false if they refuse -- converter busy, service lock engaged, function out of scope (DC V, DC A
 * and ohms only), or, for a calibration, under 30 minutes since power-on.
 *
 * `cal_range_start` MEASURES the point and previews it (cal_range_preview) and writes nothing. When the state
 * reaches CAL_SWEEP_DONE, `cal_range_pending()` gives the change the point would make, for the panel to show
 * (the operator sees "GAIN +0.12%" before anything is written). Only
 * `cal_range_confirm()` writes: it commits the point that was previewed, with the value measured then, after
 * copying the old record to the UNDO slot. `cal_verify_start` writes nothing at all, ever; its answer is
 * read with `cal_verify_result()` once the state reaches CAL_SWEEP_DONE. */
/* `accept_cold` says the operator was shown the warm-up warning and chose to continue (RANGE
 * WARNS). Without it, a calibration under 30 minutes from power-on is
 * refused with CAL_RANGE_NOT_WARM. It is an argument and not a stored flag so that the acknowledgement
 * cannot outlive the request it was given for. */
bool cal_range_start(func_t f, uint8_t range, float reference, bool accept_cold);
bool cal_range_pending(float *change_fraction); /* false unless a measured point awaits confirmation */
bool cal_range_confirm(void);                   /* writes the pending point; it is spent either way */
bool cal_verify_start(func_t f, uint8_t range, float reference);
bool cal_verify_result(float *deviation, bool *within_spec);

/* ---- Read-only calibration queries ------------------------------------------------------------------- */

typedef enum { CAL_STD_DC = 0, CAL_STD_AC } cal_std_kind_t;

/* ORIGINAL_BUGS #55: the original measured ONCE at the medium rate and printed the
 * same number three times. This measures all three rates and reports all three.
 *
 * TWO ORDERS, AND THEY ARE NOT THE SAME ONE. The array this fills is indexed BY RATE -- v[0] fast,
 * v[1] medium, v[2] slow. The reply the executor prints is "<medium>:<fast>,<slow>", because field 1 of a
 * `:SCAL:STD` reply is the MEDIUM reading: the vendor utility's `ReadStd` at @51e36c reads the text before
 * the colon as medium, and the 5492B Delphi source does the same . The reply is NOT in rate order.
 * The reply LAYOUT is unchanged from the original; only which value lands in which field was ever in
 * question, and the original's three values were identical so it could not have told anyone.
 *
 * START/POLL, like the sweeps (ORIGINAL_BUGS #1). It was a blocking call of about a second; it now runs on the
 * sweep machine. `cal_scal_std_start()` claims the converter and returns false if it cannot (a sweep or a
 * panel flow running, converter in use). The main loop runs `cal_sweep_task()`; when `cal_sweep_state()`
 * reaches CAL_SWEEP_DONE, `cal_scal_std_result()` copies the three readings out (true only for a finished STD)
 * and the caller sends its reply. FAILED means nothing is to be sent but the error; the converter is released
 * on every exit. The function and range are those selected at the start. */
bool cal_scal_std_start(cal_std_kind_t kind);
bool cal_scal_std_result(float out[RATE_COUNT]);

/* :SCAL:TEST, the same way: `cal_scal_test_start()` forces the medium rate and begins a reading, and the
 * state reaches DONE with the REL-corrected value available from `cal_scal_test_result()`. The rate is put
 * back on every exit. It returns false for a function above ohms, for a trigger mode other than immediate (the
 * pass would never start), and while a sweep is running. While it runs the main loop calls the sweep task
 * instead of the measurement task, and the sweep task runs the measurement pass itself. */
bool cal_scal_test_start(void);
bool cal_scal_test_result(float *out);

/* :SCAL:DCV:ZERO and :SCAL:ACV:ZERO. Read-only for EEPROM. Functions other than those the original accepts
 * are refused rather than reading from an address computed off the end of a table. */
bool cal_zero_query(func_t f, uint8_t *range_out, float z[RATE_COUNT]);

/* ---- Panel calibration ----------------------------------------------------------------------------
 * The SYS CAL submenu. `ui/menu.c` owns the screens and the navigation; everything that touches a record
 * is here, because a calibration write must go through one path and be reviewable in one place.
 *
 * **Scope: DC V, DC A and ohms only**, stated in the menu as well. AC stays with BK-Calibrator: the AC
 * low-range corrections need three bracket sets and a reference the panel cannot drive, so offering AC here
 * would be offering something that cannot be done properly.
 *
 * **DECISION.** A panel calibration adjusts the LINEAR
 * coefficient only and leaves the quadratic term exactly as it was -- for DC V that is the gain itself,
 * which is the whole coefficient; for DC A and ohms it is `f1` with `f0` untouched. The reason is that
 * solving a quadratic needs two good points per range, and a panel user with one reference meter and a
 * decade box realistically has one. A one-point fit against a quadratic would silently refit curvature
 * from noise, which is worse than leaving the vendor's curvature alone. A full two-point fit stays with
 * BK-Calibrator, which can walk the points. */
typedef enum {
    CAL_RANGE_OK = 0,
    CAL_RANGE_UNSUPPORTED_FUNCTION, /* anything but DC V, DC A, ohms */
    CAL_RANGE_NOT_WARM,             /* under 30 minutes since power-on, and the caller did not accept it */
    CAL_RANGE_NO_READING,           /* the measurement did not complete */
    CAL_RANGE_IMPLAUSIBLE,          /* the implied change is outside the sanity limit: nothing written */
    CAL_RANGE_LOCKED,
    CAL_RANGE_WRITE_FAILED
} cal_range_result_t;

/* How far a panel calibration may move a coefficient before it is refused. A real trim is fractions of a
 * per cent; ten per cent is already far beyond any drift and is almost certainly a wrong reference reading,
 * a wrong range or a wrong lead. Refusing is the safe answer because the alternative is a plausible-looking
 * instrument that reads wrongly, and the coefficients cannot be read back to find out why. */
#define CAL_RANGE_MAX_CHANGE 0.10f

/* Computes what a calibration at this point WOULD do, and writes nothing. `measured` is the instrument's
 * own corrected reading and `reference` is what the better meter says. The caller shows the change so the
 * operator can confirm an actual number rather than a yes/no. */
cal_range_result_t cal_range_preview(func_t f, uint8_t range, rate_t rate, float measured, float reference,
                                     bool accept_cold,
                                     float *change_fraction);

/* Commits what the preview described. Saves the old record to the UNDO slot first. */
cal_range_result_t cal_range_commit(func_t f, uint8_t range, rate_t rate, float measured, float reference,
                                    bool accept_cold);

/* VERIFY: the error against the reference and whether it is inside the published specification. Writes
 * nothing, ever. The specification numbers are transcribed from the manual
 * and are never invented -- a verify that passes against
 * a made-up limit is worse than no verify at all. */
bool cal_verify(func_t f, uint8_t range, float measured, float reference, float *error_out, bool *pass_out);

/* UNDO: restores the record saved by the last calibration write, panel or remote (`:SCAL:UNDO`). One slot,
 * because it restores the LAST write. */
bool cal_undo_available(void);
bool cal_undo(void);

/* CHECK (`:SCAL:CHECK?`): walks every record, testing the signature and then plausibility -- finite values,
 * a gain near nominal, a zero inside limits. Calls `report` once per bad record with a name the panel and
 * the remote reply can both show, and returns the number of bad records (0 means everything is sound). */
typedef void (*cal_check_report_fn)(const char *name, uint16_t addr);
uint16_t cal_check_all(cal_check_report_fn report);

/* The panel's view of the same scan . `cal_check_run()` performs it and returns how many records
 * are bad; `cal_check_name(i, buf)` then names the i-th of them so the menu can step through the list.
 *
 * `name` is NOT a stable pointer during the scan -- cal_check_all formats it into one buffer and reuses it
 * for the next record, so a callback that keeps the pointer instead of copying the characters ends up with
 * the last name repeated. Both callers here copy.
 *
 * Only the first CAL_CHECK_MAX_NAMES bad records are retained. The COUNT is always exact; it is the list
 * that is truncated. An erased EEPROM makes every record bad, and a panel that tried to hold all of them
 * would need more RAM than the whole display does for a case where the only useful message is "all of it".
 * The menu says so when the count exceeds what it can list. */
#define CAL_CHECK_NAME_LEN 9u
#define CAL_CHECK_MAX_NAMES 8u
uint8_t cal_check_run(void);
bool cal_check_name(uint8_t index, char name[CAL_CHECK_NAME_LEN + 1u]);

/* ---- Calibration date --------------------------------------------------------------------------------
 * Exactly 8 validated characters. The original used an unbounded strcpy at 0x0700E from the token
 * buffer into the GLOBAL scratch buffer at 0x0F29, and wrote the EEPROM even after reporting a
 * validation error. An over-long :SCAL:DATE is
 * therefore a RAM overflow that overruns the globals after 0x0F29 and, past 0x0F95, reaches the
 * software stack -- not a stack overflow in the usual sense, but remotely triggerable either way. */
bool cal_date_write(const char date[8]);

/* ---- Boot defaults -----------------------------------------------------------------------------------
 * Three independently guarded blocks. The original (0x05276) has two faults this must not reproduce:
 *   - it never stamps the signature at 0x0A00, so it re-initialises the THID block -- calibration count and
 *     date -- on EVERY boot;
 *   - a bad signature at 0x894 makes it zero the DCV and OHM user-zero tables at 0x880-0x8AF.
 * It cannot touch a pair or zero record, so it is not a route to losing the coefficients themselves. There
 * is no interactive prompt here: the original blocked on a front-panel "DEBUG:" prompt during boot. */
void cal_boot_defaults_init(void);

/* NOT DECLARED ON PURPOSE: the signature-stamping routine at 0x03200, which writes the cleared signature
 * over 242 records. :SCAL:CLEAR_DATA, :SCAL:CLEAR_ALL, :SCAL:CLEAR_INIT and :SCAL:CLEAR_REL are on the
 * transport denylist and must not exist as callable operations in this firmware. If a recovery tool is ever
 * wanted it is a separate, service-locked entry point, and it is not this one. */

#endif /* BK_CAL_H */
