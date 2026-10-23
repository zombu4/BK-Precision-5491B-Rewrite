/* cal_panel.c -- panel calibration: RANGE, VERIFY, UNDO and CHECK.
 *
 * A separate file from cal_ops.c because
 * that one is already 900 lines of the :SCAL command surface, and because this is the code that lets
 * someone standing at the instrument change its calibration -- it deserves to be read on its own.
 *
 * Read cal.h's panel-calibration block first: it states the scope limit (DC V, DC A and ohms only), the sanity limit, and
 * the DEFAULT decision that a panel calibration moves the LINEAR coefficient only.
 *
 * ---- Why one point moves only the linear term -------------------------------------------------------
 * Most functions apply y = m * (f0*m + f1), a quadratic. Fitting both coefficients needs two good points
 * per range. A panel user has one reference meter and, realistically, one point. Solving a quadratic from
 * one point would mean inventing curvature out of a single measurement and its noise -- and the result
 * would look exactly as plausible as a correct one, because the coefficients cannot be read back.
 *
 * So f0 is left exactly as the vendor set it and f1 is trimmed. For DC V and AC+DC V the form is linear
 * already, so f0 IS the gain and there is nothing else to leave alone. A full two-point fit belongs to
 * BK-Calibrator, which can walk the points and show the residuals.
 *
 * ---- Every write goes through the same door ---------------------------------------------------------
 * `cal_write_pair()` in cal.c: invalidate the signature, write the data, verify it by reading back, write
 * the valid signature last. The undo copy is taken BEFORE that and verified too, because an undo slot that
 * can be half-written is worse than none -- it would restore rubbish over something that was merely wrong.
 */
#include "cal.h"

#include "../fmathf.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../hal/eeprom.h"
#include "../main.h"
#include "../serv/serv.h"
#include "../tables.h"

/* ---- Scope ------------------------------------------------------------------------------------------- */

static bool panel_function_supported(func_t f) {
    return f == FUNC_DCV || f == FUNC_DCA || f == FUNC_OHM;
}

/* ---- The undo slot ----------------------------------------------------------------------------------- */

/* Saves `len` bytes from `addr` into the undo slot, signature last. Returns false if anything did not
 * verify, in which case the caller must NOT go on to write the record: without a good undo copy, a
 * calibration becomes irreversible, and the whole point of the slot is that it is not. */
static bool undo_save(uint16_t addr, uint8_t len) {
    uint8_t i;

    if (len > EE_UNDO_LEN) {
        return false;
    }
    if (!ee_write_u16(EE_UNDO_SIG, EE_SIG_CLEARED)) {
        return false;
    }
    if (!ee_write_u16(EE_UNDO_ADDR, addr)) {
        return false;
    }
    for (i = 0u; i < len; i++) {
        if (!ee_write_u8((uint16_t)(EE_UNDO_DATA + i), ee_read_u8((uint16_t)(addr + i)))) {
            return false;
        }
    }
    /* The remaining bytes are padded so a short record cannot leave a previous, longer one's tail behind
     * and have it restored over something else later. */
    for (i = len; i < EE_UNDO_LEN; i++) {
        if (!ee_write_u8((uint16_t)(EE_UNDO_DATA + i), 0xFFu)) {
            return false;
        }
    }
    return ee_write_u16(EE_UNDO_SIG, EE_SIG_VALID);
}

bool cal_undo_available(void) {
    return ee_read_u16(EE_UNDO_SIG) == EE_SIG_VALID && ee_read_u16(EE_UNDO_ADDR) != EE_ADDR_NONE;
}

/* Is `addr` an address a calibration pair can actually live at?
 *
 * `cal_undo()` reads its target from EEPROM and then writes ten bytes there. Nothing had checked that the
 * word it read is a calibration-record address at all -- so a corrupted or never-written undo slot would
 * write ten bytes wherever that word happened to point, and the EEPROM it could land in includes the
 * :THID identity block, the calibration date and the save counter. An undo is supposed to be the SAFE
 * button; a stray one would damage data no calibration touches.
 *
 * The check asks the question directly rather than approximating it with a range test: enumerate every
 * address `cal_pair_address()` can return and look for a match. That is a few hundred iterations of
 * arithmetic, once, in response to a key press or a remote command -- it costs nothing anybody can
 * perceive, and unlike a bounds check it cannot drift out of step with the address map. */
static bool pair_address_is_real(uint16_t addr) {
    uint8_t fi;

    if (addr == EE_ADDR_NONE) {
        return false;
    }
    for (fi = 0u; fi < FUNC_COUNT; fi++) {
        const func_t f = (func_t)fi;
        uint8_t range;

        for (range = 0u; range <= RANGE_MAX_INDEX; range++) {
            uint8_t rt;

            for (rt = 0u; rt < RATE_COUNT; rt++) {
                uint8_t v;

                for (v = 0u; v <= (uint8_t)CAL_PAIR_LOW3; v++) {
                    if (cal_pair_address(f, range, (rate_t)rt, (cal_variant_t)v) == addr) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool cal_undo(void) {
    uint16_t addr;
    uint8_t i;

    if (serv_check() != SERV_ALLOWED) {
        return false;
    }
    if (!cal_undo_available()) {
        return false;
    }
    addr = ee_read_u16(EE_UNDO_ADDR);
    if (!pair_address_is_real(addr)) {
        return false; /* refuse rather than write ten bytes at an address nothing vouches for */
    }

    /* The saved bytes include the record's own signature word, so restoring them restores a record that was
     * valid when it was saved. The signature is written last here too, by clearing it first: an interrupted
     * undo must leave the record invalid rather than half-restored. */
    if (!ee_write_u16((uint16_t)(addr + 8u), EE_SIG_CLEARED)) {
        return false;
    }
    for (i = 0u; i < 8u; i++) {
        if (!ee_write_u8((uint16_t)(addr + i), ee_read_u8((uint16_t)(EE_UNDO_DATA + i)))) {
            return false;
        }
    }
    if (!ee_write_u16((uint16_t)(addr + 8u),
                      (uint16_t)(ee_read_u8((uint16_t)(EE_UNDO_DATA + 8u)) |
                                 ((uint16_t)ee_read_u8((uint16_t)(EE_UNDO_DATA + 9u)) << 8)))) {
        return false;
    }

    /* One undo, then the slot is spent. Leaving it armed would let a second press restore the same old
     * record over a newer good calibration, which is not what "undo" means to anyone. */
    return ee_write_u16(EE_UNDO_SIG, EE_SIG_CLEARED);
}

/* ---- RANGE ------------------------------------------------------------------------------------------- */

/* The trim factor the reading implies. Both arguments are in the instrument's own units, so this is just
 * how far out the meter is -- and the sanity limit is applied to exactly this number, which is why the
 * preview can show the operator a percentage that means something. */
static bool trim_factor(float measured, float reference, float *factor) {
    if (!isfinite(measured) || !isfinite(reference)) {
        return false;
    }
    if (fabsf(measured) < 1e-12f) {
        return false; /* no signal: a trim against zero is a division, not a calibration */
    }
    *factor = reference / measured;
    return isfinite(*factor);
}

static cal_range_result_t range_compute(func_t f, uint8_t range, rate_t rate, float measured,
                                        float reference, float *change_out, ee_pair_t *updated,
                                        uint16_t *addr_out) {
    /* WHICH PAIR THIS POINT BELONGS TO. `cal_apply()` chooses one of two pairs at every reading, and a
     * calibration has to trim the one it actually used -- trimming the other leaves the instrument reading
     * exactly as wrongly as before, with the stored numbers now saying otherwise.
     *
     *   LINEAR_SIGNED / QUAD_SIGNED (DC V, DC A): main for a non-negative reading, alt (CAL_PAIR_NEG) for
     *                                            a negative one -- cal.c:499-502.
     *   OHM_2SEG                   (ohms):       main below the range's segment threshold, alt
     *                                            (CAL_PAIR_OHM_Y) at or above it -- cal.c:506.
     *
     * Taking CAL_PAIR_MAIN for every ohm point would rewrite the lower segment's pair for a point in the
     * upper segment.
     *
     * The segment is chosen by the REFERENCE and not by the reading. `cal_apply()` compares the MEAN --
     * zero-subtracted and scaled, before the gain -- against the threshold, and `measured` here is the
     * value after the gain, so the two are not the same quantity. The reference is: it is the resistance
     * the operator has actually connected, in the same ohms the threshold table is written in
     * (cal.c:43 -- 25 M, 2.5 M, 250 k, 25 k, 2.5 k, 250, each half of its range's nominal full scale). It
     * is also the quantity the operator controls, so the segment a calibration lands in is a decision they
     * can make deliberately rather than one that falls out of a noisy reading near a boundary. */
    cal_set_t loaded;
    cal_variant_t variant = CAL_PAIR_MAIN;
    uint16_t addr;
    ee_pair_t rec;
    float factor;
    float change;

    cal_load(f, range, rate, &loaded);
    if ((f == FUNC_DCV || f == FUNC_DCA) && measured < 0.0f) {
        variant = CAL_PAIR_NEG;
    } else if (f == FUNC_OHM && loaded.ohm_threshold > 0.0f && reference >= loaded.ohm_threshold) {
        variant = CAL_PAIR_OHM_Y;
    }
    addr = cal_pair_address(f, range, rate, variant);

    if (!panel_function_supported(f)) {
        return CAL_RANGE_UNSUPPORTED_FUNCTION;
    }
    if (addr == EE_ADDR_NONE) {
        return CAL_RANGE_UNSUPPORTED_FUNCTION;
    }
    if (!trim_factor(measured, reference, &factor)) {
        return CAL_RANGE_NO_READING;
    }

    change = factor - 1.0f;
    if (fabsf(change) > CAL_RANGE_MAX_CHANGE) {
        return CAL_RANGE_IMPLAUSIBLE;
    }

    /* An invalid record is not a reason to refuse: an uncalibrated range is exactly what someone is here to
     * fix. But the starting point must be THE PAIR THE LOADER ACTUALLY APPLIED, not the identity.
     *
     * Falling back to the identity would be wrong for a cleared NEGATIVE DC record:
     * the loader's negative-pair fallback copies the POSITIVE pair into the alternate slot
     * (cal.c, `if (!out->alt_valid) out->alt = out->main;`). The reading is then taken through f0 = 1.0242
     * on a calibrated meter while a commit starting from 1.0 would write the point 2.4 % out -- with a
     * valid signature on it, which is the part that makes it dangerous. EVERY DC V negative record in a
     * dump of a calibrated meter is cleared (sig 0x5161), so this is not a corner case.
     *
     * `loaded` is the set `cal_load()` resolved a few lines above, fallbacks and all, so taking the pair
     * from it is by construction the pair the measurement went through. */
    if (!cal_read_pair(addr, &rec)) {
        const cal_pair_t src = (variant == CAL_PAIR_MAIN) ? loaded.main : loaded.alt;

        rec.f0 = src.f0;
        rec.f1 = src.f1;
    }

    /* SCALE THE WHOLE PAIR, not one term of it.
     *
     * The quadratic form is `y = m * (f0*m + f1)`. Scaling f1 alone gives
     * `y' = m * (f0*m + k*f1)`, whose ratio to y is `(f0*m + k*f1) / (f0*m + f1)` -- which equals k only
     * when `f0*m` is zero. On a calibrated meter it is not: OHM X R0 has f1 = -8.95 and the Y pairs reach
     * 38.8 and -6.1, so a +1 % trim moved the reading by about -9 % on R0. The operator was shown one
     * number and the instrument was given another.
     *
     * Scaling both terms makes `y' = m * (k*f0*m + k*f1) = k*y` exactly, for every m. That is what "gain"
     * means and what the preview promises, and it leaves the curve's SHAPE untouched -- the curvature
     * relative to the linear term is unchanged, because both were multiplied by the same k.
     *
     * Decision 2026-10-22: this replaces "a one-point trim adjusts the linear term only", which existed to
     * stop one noisy measurement refitting the curvature. Scaling both terms does not refit it: their RATIO
     * is unchanged, so the curve moves without bending, and the operator gets the change they confirmed. The linear form needs no special case:
     * `y = m * f0`, so scaling f0 is already scaling the whole pair. */
    rec.f0 *= factor;
    rec.f1 *= factor;
    if (!isfinite(rec.f0) || !isfinite(rec.f1)) {
        return CAL_RANGE_IMPLAUSIBLE;
    }

    *change_out = change;
    *updated = rec;
    *addr_out = addr;
    return CAL_RANGE_OK;
}

cal_range_result_t cal_range_preview(func_t f, uint8_t range, rate_t rate, float measured, float reference,
                                     bool accept_cold, float *change_fraction) {
    ee_pair_t updated;
    uint16_t addr;

    /* The warm-up check belongs in the PREVIEW as well as the commit, so the operator is told before they
     * have typed a reference reading rather than after. The 5491B's specification assumes a warmed-up
     * instrument; a calibration at two minutes is a calibration to the wrong numbers, and nothing
     * afterwards reveals it -- which is why the operator is told rather than simply allowed.
     *
     * Told, and then obeyed: `accept_cold` expresses the decision that this is a warning and
     * not a prohibition. Someone calibrating a meter that has been on all day, after a brief power cycle,
     * knows something this 30-minute timer does not. */
    if (!accept_cold && !main_warmed_up()) {
        return CAL_RANGE_NOT_WARM;
    }
    return range_compute(f, range, rate, measured, reference, change_fraction, &updated, &addr);
}

cal_range_result_t cal_range_commit(func_t f, uint8_t range, rate_t rate, float measured, float reference,
                                    bool accept_cold) {
    ee_pair_t updated;
    uint16_t addr;
    float change;
    cal_range_result_t result;

    if (serv_check() != SERV_ALLOWED) {
        return CAL_RANGE_LOCKED;
    }
    if (!accept_cold && !main_warmed_up()) {
        return CAL_RANGE_NOT_WARM;
    }

    result = range_compute(f, range, rate, measured, reference, &change, &updated, &addr);
    if (result != CAL_RANGE_OK) {
        return result;
    }

    /* The undo copy first, and its failure stops everything. A calibration that cannot be undone because
     * the slot would not write is a calibration that should not happen yet. */
    if (!undo_save(addr, EE_PAIR_SIZE)) {
        return CAL_RANGE_WRITE_FAILED;
    }
    if (!cal_write_pair(addr, updated.f0, updated.f1)) {
        return CAL_RANGE_WRITE_FAILED;
    }
    serv_note_activity();
    return CAL_RANGE_OK;
}

/* ---- VERIFY ------------------------------------------------------------------------------------------
 * Writes nothing, ever.
 *
 * The specification is "a percentage of the reading plus a number of counts", which is how the manual
 * states it and how the BK-Calibrator application's 5491B spec file transcribes it. The numbers below are the DC
 * specifications of the 5491B as published.
 *
 * UNVERIFIED: these are transcribed from the application's spec file rather than re-read from the manual
 * PDF. Check them against the manual before anyone relies on a PASS. A verify that passes
 * against a wrong limit is worse than no verify, because it manufactures confidence -- which is why the
 * limit is per function and range here rather than one number for everything. */
typedef struct {
    float pct;    /* per cent of reading */
    float counts; /* plus this many counts of the last digit */
} cal_spec_t;

static cal_spec_t spec_for(func_t f, uint8_t range) {
    cal_spec_t s;

    switch (f) {
    case FUNC_DCV:
        s.pct = 0.0003f; /* 0.03 % of reading */
        s.counts = 4.0f;
        break;
    case FUNC_DCA:
        s.pct = 0.0010f; /* 0.1 % */
        s.counts = 6.0f;
        break;
    case FUNC_OHM:
        /* The two highest resistance ranges are looser, as published. */
        s.pct = (range <= 1u) ? 0.0060f : 0.0010f;
        s.counts = 6.0f;
        break;
    default:
        s.pct = 0.0f;
        s.counts = 0.0f;
        break;
    }
    return s;
}

bool cal_verify(func_t f, uint8_t range, float measured, float reference, float *error_out, bool *pass_out) {
    cal_spec_t s;
    float allowed;
    float lsd;

    if (!panel_function_supported(f) || !isfinite(measured) || !isfinite(reference)) {
        return false;
    }
    s = spec_for(f, range);
    if (s.pct == 0.0f && s.counts == 0.0f) {
        return false; /* no published limit: report nothing rather than invent a PASS */
    }

    /* A "count" is the last displayed digit of this range. The range scale gives full scale, and the
     * display shows five digits, so one count is full scale / 100000. */
    lsd = tbl_range_scale(f, range) / 100000.0f;
    allowed = s.pct * fabsf(reference) + s.counts * lsd;

    *error_out = measured - reference;
    *pass_out = fabsf(*error_out) <= allowed;
    return true;
}

/* ---- CHECK -------------------------------------------------------------------------------------------
 * Walks every record the loaders can read and reports the ones that are not sound. This is the cheap
 * version of the thing that could not be done at all before: the coefficients cannot be read back over the
 * port on the vendor firmware, so a damaged record was invisible until a reading looked wrong.
 *
 * WHAT THIS TESTS, AND WHAT IT DELIBERATELY NO LONGER TESTS.
 *
 * It tests the two things that are unambiguous: the record's SIGNATURE, and that both coefficients are
 * FINITE. A cleared signature is what an interrupted write leaves behind, and a NaN or an infinity in a
 * coefficient makes every later reading meaningless -- both really are damage, on any range, for any
 * function, with no judgement required.
 *
 * It used to also test that the gain was "near nominal": a linear gain within a factor of two of unity, or
 * a quadratic linear term within a factor of two with a small curvature term. **That test was wrong.**
 * Run over the dump of a calibrated meter, CHECK reported twelve
 * damaged records. All twelve carry signature 0x5554 and hold the real calibration that meter measures
 * with: OHM R0 is `f0=4.0065e-07, f1=-8.9495` and OHM R1 is `f0=3.01767e-07, f1=0.211185`. The band
 * `0.5 < |f1| < 2.0` is a DC-V-shaped assumption, and the extreme ohm ranges have no reason to satisfy it.
 * Across the 88 valid records in that image the bands misfire in both directions.
 *
 * The reason to delete the test rather than widen it is what this screen is FOR. CHECK is what a user
 * looks at to decide whether their instrument is sound. A false alarm there is worse than a missed
 * detection: it tells someone who has just calibrated and verified their meter that it is broken, on the
 * one display whose whole value is being believed. A band could be restored later per function, derived
 * from the disassembly's own limits rather than guessed -- but not before it is derived.
 *
 * UNVERIFIED: whether the original firmware has any plausibility test of its own -- check: look for a
 * coefficient range test in the loader at 0x0DAA2 and the per-function loaders. The port's was invented. */
static bool pair_plausible(func_t f, const ee_pair_t *p) {
    (void)f; /* the test is the same for every function, which is the point */
    return isfinite(p->f0) && isfinite(p->f1);
}

/* Short names for the CHECK report. Deliberately NOT called func_name: tables.h already has a
 * func_name[] holding the panel keyword records, and two tables with one name is how the wrong one
 * gets used. These are for a fault message, not for the display. */
static const char *const check_func_name[FUNC_COUNT] = {"DCV", "ACV",  "DCA",  "ACA",   "OHM",  "FREQ",
                                                        "PER", "CONT", "DIOD", "ACDCV", "ACDCI"};

/* The reported name carries the RANGE as well as the function, because "DCV BAD" does not tell an operator
 * which of six DC V records to look at and the whole point of the list is to say where to look. Formatted
 * by hand rather than with snprintf: this runs on a part where every byte of flash is budgeted, and the
 * format is three letters, a space, 'R' and one digit.
 *
 * One buffer, reused per record -- see the note in cal.h. */
static char check_name_buf[CAL_CHECK_NAME_LEN + 1u];

static const char *check_name_for(uint8_t fi, uint8_t range) {
    const char *src = check_func_name[fi];
    uint8_t n = 0u;

    while (src[n] != '\0' && n < 3u) {
        check_name_buf[n] = src[n];
        n++;
    }
    check_name_buf[n++] = ' ';
    check_name_buf[n++] = 'R';
    check_name_buf[n++] = (char)('0' + (range % 10u));
    check_name_buf[n] = '\0';
    return check_name_buf;
}

/* ONE REPORT PER RECORD. Four functions do not own any records of their own -- they read
 * another function's, which `cal_pair_address()` states outright:
 *
 *     ACDCV -> the DC V records          CONT  -> the OHM X records
 *     ACDCI -> the DC A records          DIODE -> the OHM X pair of range 4
 *
 * Walking all eleven functions therefore visited the same bytes twice and reported them under two names.
 * On a calibrated meter's image six real records would come out as twelve, half of them labelled CON.
 *
 * Skipping the aliases is exact, which a deduplicating buffer would not have been: there is no bound to
 * reason about and no eviction to get wrong. The functions below are the ones that own what they read. */
static bool func_owns_its_records(func_t f) {
    /* Six aliases, not four. `cal_pair_address()` is the authority and it maps:
     *
     *     ACDCV -> the DC V records      CONT   -> the OHM X records
     *     ACDCI -> the DC A records      DIODE  -> the OHM X pair of range 4
     *     FREQ  -> the AC V MAIN pair    PERIOD -> the AC V MAIN pair
     *
     * FREQ and PERIOD borrow the AC V records because they measure the
     * shaped input's amplitude through the AC chain (cal.c, the FUNC_FREQ/FUNC_PERIOD case), so a damaged
     * AC V record was reported three times -- once as ACV, once as FRE, once as PER -- tripling the count
     * and filling the eight-name list with copies of one fault.
     *
     * It does not show on a healthy dump, where all 78 MAIN records are valid and finite so CHECK returns
     * 0 either way. A defect that only appears once something is already wrong is exactly the kind this
     * screen must not have: it would mislead at the only moment anyone reads it. */
    return f != FUNC_ACDCV && f != FUNC_ACDCI && f != FUNC_CONT && f != FUNC_DIODE && f != FUNC_FREQ &&
           f != FUNC_PERIOD;
}

uint16_t cal_check_all(cal_check_report_fn report) {
    uint16_t bad = 0u;
    uint8_t fi;

    for (fi = 0u; fi < FUNC_COUNT; fi++) {
        const func_t f = (func_t)fi;
        uint8_t range;

        if (!func_owns_its_records(f)) {
            continue;
        }
        for (range = 0u; range <= RANGE_MAX_INDEX; range++) {
            uint8_t rt;

            for (rt = 0u; rt < RATE_COUNT; rt++) {
                const uint16_t addr = cal_pair_address(f, range, (rate_t)rt, CAL_PAIR_MAIN);
                ee_pair_t rec;

                if (addr == EE_ADDR_NONE) {
                    continue; /* the combination does not exist, which is not damage */
                }
                if (!cal_read_pair(addr, &rec) || !pair_plausible(f, &rec)) {
                    bad++;
                    if (report != NULL) {
                        report(check_name_for(fi, range), addr);
                    }
                }
            }
        }
    }
    return bad;
}

/* ---- The panel's CHECK list ------------------------------------------------------------------- */
static struct {
    char name[CAL_CHECK_MAX_NAMES][CAL_CHECK_NAME_LEN + 1u];
    uint8_t held; /* how many of the names above are filled in */
} check_list;

static void check_collect(const char *name, uint16_t addr) {
    uint8_t i = 0u;

    (void)addr; /* the panel shows the name; the address is for the remote reply */
    if (check_list.held >= CAL_CHECK_MAX_NAMES) {
        return;
    }
    while (name[i] != '\0' && i < CAL_CHECK_NAME_LEN) {
        check_list.name[check_list.held][i] = name[i];
        i++;
    }
    check_list.name[check_list.held][i] = '\0';
    check_list.held++;
}

uint8_t cal_check_run(void) {
    uint16_t bad;

    check_list.held = 0u;
    bad = cal_check_all(check_collect);

    /* Saturate rather than wrap. A uint8_t return cannot express 300 bad records, and reporting 44 of them
     * because 300 does not fit would be worse than reporting the maximum: 255 and "all of it" lead to the
     * same action, 44 leads to the wrong one. */
    return bad > 255u ? 255u : (uint8_t)bad;
}

bool cal_check_name(uint8_t index, char name[CAL_CHECK_NAME_LEN + 1u]) {
    uint8_t i;

    if (index >= check_list.held) {
        return false;
    }
    for (i = 0u; i <= CAL_CHECK_NAME_LEN; i++) {
        name[i] = check_list.name[index][i];
    }
    return true;
}
