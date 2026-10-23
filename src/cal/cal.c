/* cal.c -- the calibration maths and the EEPROM record codec.
 *
 * Replaces read_gain_record 0x0CA9A, read_zero_record 0x0CAC4, the eight per-function loaders
 * (0x0CAEC DCV, 0x0CD70 ACV, 0x0CFDC ACA, 0x0D270 DCA, 0x0D4C0 OHM/CONT, 0x0D5AE DIODE, 0x0D60E ACDCV,
 * 0x0D85A ACDCI), the loader selector 0x0DAA2, and the per-sample and per-mean arithmetic of 0x0AC56 and
 * 0x0A3F2. The :SCAL operations themselves are in cal_ops.c.
 *
 * The arithmetic was recovered from the firmware and cross-checked against EEPROM dumps of a calibrated
 * meter.
 *
 * Read cal.h first; it states the four arithmetic forms and why one `cal_apply(c, raw)` was the wrong
 * shape. Two things in here are easy to get wrong and are worth naming up front:
 *
 *   1. The zero is subtracted from EVERY SAMPLE and the gain is applied to the MEAN. Under a quadratic
 *      form those two orders are not interchangeable, so `cal_sample` and `cal_apply` are separate
 *      functions on purpose and the caller must not merge them.
 *   2. An invalid record falls back to the IDENTITY OF THAT FUNCTION'S OWN FORMULA -- (1.0, 0.0) for the
 *      linear form and (0.0, 1.0) for every quadratic one. Getting that backwards does not produce a
 *      slightly wrong reading: (1.0, 0.0) in a quadratic form gives y = m*m, and (0.0, 1.0) in the linear
 *      form gives zero. An uncalibrated instrument must read approximately right, never squared.
 */
#include "cal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../fmathf.h"
#include "../hal/eeprom.h"
#include "../serv/serv.h"
#include "../tables.h"

/* ---- Constants from the image ------------------------------------------------------------------------- */

/* The converter's volts-per-count, flash 0x0B14 positive and 0x0B18 negative.
 * UNVERIFIED: this implies a reference of about 2.4 V. It is used
 * here exactly as the original used it, so an error in it is an error the original shares. */
#define CAL_VOLTS_PER_COUNT 2.86e-7f

/* DIODE divides the range-4 ohm zero by this (flash 0x0AAC, loader 0x0D5AE). */
#define CAL_DIODE_ZERO_DIVISOR 2459.3f

/* The ohm segment thresholds, flash 0x133D, copied to RAM 0x0320. Half of each range's
 * nominal full scale: below it the X pair applies, at or above it the Y pair. */
static const float ohm_threshold[EE_OHM_RANGES] = {25.0e6f, 2.5e6f, 250.0e3f, 25.0e3f, 2.5e3f, 250.0f};

/* Dead-band tables. Both are skipped while a sweep runs, and both also zero the reading rather than
 * shrinking it.
 *   DCV / ACDCV: ranges 0..3 use 0.00042 * table (flash 0x062F), range 4 and above use a flat 3e-5.
 *   OHM:         ranges 0..4 use 0.00042 * table (flash 0x06E7), range 5 uses a flat 0.02. */
#define CAL_DEADBAND_K 0.00042f
static const float deadband_v[4] = {475.0f, 47.5f, 4.75f, 0.475f};
static const float deadband_ohm[5] = {4.75e6f, 475.0e3f, 47.5e3f, 4750.0f, 475.0f};
#define CAL_DEADBAND_V_FLAT 3.0e-5f
#define CAL_DEADBAND_OHM_FLAT 0.02f

/* AC low-range brackets on the pre-gain mean, and the post-correction zero rule.
 * Voltage: flash 0x0B00, 0x0AF8, 0x0AF4 and the zero rule 0x0AFC.
 * Current: flash 0x0B08, 0x0AFC, 0x0AEC and the zero rule 0x0AF0. */
static const float ac_low_bracket_v[3] = {0.001f, 0.005f, 0.010f};
static const float ac_low_bracket_a[3] = {3.0e-5f, 1.0e-4f, 5.0e-4f};
#define CAL_AC_LOW_ZERO_V 1.0e-4f
#define CAL_AC_LOW_ZERO_A 2.0e-6f

/* The "below this fraction of full scale, read zero" rule for AC ranges 0..3 (flash 0x0B04 = 0.025, with
 * the limit tables at flash 0x066B for volts and 0x06BB for amps). */
#define CAL_AC_FLOOR_FRACTION 0.025f
static const float ac_floor_v[4] = {475.0f, 47.5f, 4.75f, 0.475f};
static const float ac_floor_a[4] = {4.75f, 0.475f, 0.0475f, 0.00475f};

/* ---- Addressing -------------------------------------------------------------------------------------- */

/* Pair-record addressing. Every family is base + rate_step*rate + 10*range; the DC
 * families use a 100-byte rate step because each rate holds a positive AND a negative block of 50.
 *
 * Returning EE_ADDR_NONE for a combination that does not exist is not defensiveness for its own sake. The
 * original computed these with no bounds check at all, and the EEPROM address register holds 12 bits, so a
 * bad index wrapped modulo 4096 and landed on another record -- the identity block, the :THID block or
 * another function's coefficients. That is the single most destructive
 * behaviour in the original firmware, and it is reachable from a malformed remote command. */
uint16_t cal_pair_address(func_t f, uint8_t range, rate_t rate, cal_variant_t variant) {
    const uint8_t rt = (uint8_t)rate;

    if (rt >= RATE_COUNT) {
        return EE_ADDR_NONE;
    }

    switch (f) {
    case FUNC_DCV:
    case FUNC_ACDCV: /* ACDCV's DC part reads the DC V records */
        if (range >= EE_DCV_RANGES) {
            return EE_ADDR_NONE;
        }
        if (variant == CAL_PAIR_MAIN) {
            return (uint16_t)(EE_DCV_POS_FAST + 100u * rt + 10u * range);
        }
        if (variant == CAL_PAIR_NEG) {
            return (uint16_t)(EE_DCV_NEG_FAST + 100u * rt + 10u * range);
        }
        return EE_ADDR_NONE;

    case FUNC_DCA:
    case FUNC_ACDCI:
        if (range >= EE_DCA_RANGES) {
            return EE_ADDR_NONE;
        }
        if (variant == CAL_PAIR_MAIN) {
            return (uint16_t)(EE_DCA_POS_FAST + 100u * rt + 10u * range);
        }
        if (variant == CAL_PAIR_NEG) {
            return (uint16_t)(EE_DCA_NEG_FAST + 100u * rt + 10u * range);
        }
        return EE_ADDR_NONE;

    case FUNC_OHM:
    case FUNC_CONT:
        if (range >= EE_OHM_RANGES) {
            return EE_ADDR_NONE;
        }
        if (variant == CAL_PAIR_MAIN) {
            return (uint16_t)(EE_OHM_X_BASE + EE_OHM_RATE_STEP * rt + 10u * range);
        }
        if (variant == CAL_PAIR_OHM_Y) {
            return (uint16_t)(EE_OHM_Y_BASE + EE_OHM_RATE_STEP * rt + 10u * range);
        }
        return EE_ADDR_NONE;

    case FUNC_DIODE:
        /* The diode function uses the X pair of ohm range 4 and nothing else (loader 0x0D5AE). */
        if (variant != CAL_PAIR_MAIN) {
            return EE_ADDR_NONE;
        }
        return (uint16_t)(EE_OHM_X_BASE + EE_OHM_RATE_STEP * rt + 10u * 4u);

    case FUNC_ACV:
        if (range >= EE_AC_RANGES) {
            return EE_ADDR_NONE;
        }
        switch (variant) {
        case CAL_PAIR_MAIN:
            return (uint16_t)(EE_ACV_BASE + EE_AC_RATE_STEP * rt + 10u * range);
        case CAL_PAIR_LOW1:
            return (uint16_t)(EE_ACLOW_V_SET1 + EE_AC_RATE_STEP * rt + 10u * range);
        case CAL_PAIR_LOW2:
            /* Set 2 lives at a HIGHER address than set 3 but covers the MIDDLE bracket. The address order
             * is not the bracket order, and reading it as such would swap two corrections silently. */
            return (uint16_t)(EE_ACLOW_V_SET2 + EE_AC_RATE_STEP * rt + 10u * range);
        case CAL_PAIR_LOW3:
            return (uint16_t)(EE_ACLOW_V_SET3 + EE_AC_RATE_STEP * rt + 10u * range);
        default:
            return EE_ADDR_NONE;
        }

    case FUNC_ACA:
        if (range >= EE_AC_RANGES) {
            return EE_ADDR_NONE;
        }
        switch (variant) {
        case CAL_PAIR_MAIN:
            return (uint16_t)(EE_ACA_BASE + EE_AC_RATE_STEP * rt + 10u * range);
        case CAL_PAIR_LOW1:
            return (uint16_t)(EE_ACLOW_A_SET1 + EE_AC_RATE_STEP * rt + 10u * range);
        case CAL_PAIR_LOW2:
            return (uint16_t)(EE_ACLOW_A_SET2 + EE_AC_RATE_STEP * rt + 10u * range);
        case CAL_PAIR_LOW3:
            return (uint16_t)(EE_ACLOW_A_SET3 + EE_AC_RATE_STEP * rt + 10u * range);
        default:
            return EE_ADDR_NONE;
        }

    case FUNC_FREQ:
    case FUNC_PERIOD:
        /* Frequency and period measure the shaped input's amplitude through the AC chain, so they borrow
         * the AC V records (or AC A when the dual display selects current: loader selector 0x0DAA2). The
         * amplitude path is the measurement subsystem's; here the AC V main pair is the right answer. */
        if (range >= EE_AC_RANGES || variant != CAL_PAIR_MAIN) {
            return EE_ADDR_NONE;
        }
        return (uint16_t)(EE_ACV_BASE + EE_AC_RATE_STEP * rt + 10u * range);

    default:
        return EE_ADDR_NONE;
    }
}

/* Zero-record addressing. Note what is NOT here: the auxiliary areas at 0x0C00 and
 * 0x0C48. The sweeps write them and no loader reads them, so they are not part of the correction path; the
 * header records that and the zero queries in cal_ops.c reach them directly. */
uint16_t cal_zero_address(func_t f, uint8_t range) {
    switch (f) {
    case FUNC_OHM:
    case FUNC_CONT:
        return (range < EE_OHM_RANGES) ? (uint16_t)(EE_ZERO_OHM + 14u * range) : EE_ADDR_NONE;
    case FUNC_DIODE:
        /* The diode zero is the ohm range-4 record, scaled (loader 0x0D5AE). */
        return (uint16_t)(EE_ZERO_OHM + 14u * 4u);
    case FUNC_DCV:
    case FUNC_ACDCV:
        return (range < EE_DCV_RANGES) ? (uint16_t)(EE_ZERO_DCV + 14u * range) : EE_ADDR_NONE;
    case FUNC_DCA:
    case FUNC_ACDCI:
        return (range < EE_DCA_RANGES) ? (uint16_t)(EE_ZERO_DCA + 14u * range) : EE_ADDR_NONE;
    case FUNC_ACA:
        return (range < EE_AC_RANGES) ? (uint16_t)(EE_ZERO_ACA + 14u * range) : EE_ADDR_NONE;
    case FUNC_ACV:
    case FUNC_FREQ:
    case FUNC_PERIOD:
        return (range < EE_AC_RANGES) ? (uint16_t)(EE_ZERO_ACV + 14u * range) : EE_ADDR_NONE;
    default:
        return EE_ADDR_NONE;
    }
}

static cal_form_t form_of(func_t f);

/* ---- Records ----------------------------------------------------------------------------------------- */

bool cal_read_pair(uint16_t addr, ee_pair_t *out) {
    if (addr == EE_ADDR_NONE) {
        return false;
    }
    out->f0 = ee_read_f32(addr);
    out->f1 = ee_read_f32((uint16_t)(addr + 4u));
    out->sig = ee_read_u16((uint16_t)(addr + 8u));
    return out->sig == EE_SIG_VALID;
}

/* The coefficient that carries the correction is the gain: f0 in the linear DC V form (y = m * f0),
 * f1 in every quadratic form (y = m * (f0 * m + f1)). A record whose gain is zero, or whose
 * terms are not finite numbers, makes every reading on its range zero or NaN, so it is as unusable as a
 * cleared signature and is refused the same way: the caller falls back to the identity of the form. A
 * dump of a calibrated meter holds two valid-signature (0, 0) pairs; no other record in it is affected. */
bool cal_pair_usable(func_t f, const ee_pair_t *p) {
    const float gain = (form_of(f) == CAL_FORM_LINEAR_SIGNED) ? p->f0 : p->f1;

    return isfinite(p->f0) && isfinite(p->f1) && gain != 0.0f;
}

bool cal_read_zero(uint16_t addr, ee_zero_t *out) {
    uint8_t i;

    if (addr == EE_ADDR_NONE) {
        return false;
    }
    for (i = 0u; i < RATE_COUNT; i++) {
        out->z[i] = ee_read_f32((uint16_t)(addr + 4u * i));
    }
    out->sig = ee_read_u16((uint16_t)(addr + 12u));
    return out->sig == EE_SIG_VALID;
}

/* The order is the whole point: invalidate, write, verify, then validate.
 *
 * The original wrote the data floats first and rewrote 0x5554 at the end while the OLD signature stayed
 * valid throughout (ORIGINAL_BUGS #48 and #110). An interruption part-way therefore left
 * a record that LOOKS valid and holds a mixture of old and new coefficients -- the worst of the three
 * possible outcomes, because nothing downstream can detect it. With the signature cleared first, an
 * interruption leaves the record invalid, and the loaders already handle that by falling back to the
 * identity: the instrument reads uncorrected, which is obvious, instead of reading plausibly and wrongly. */
bool cal_write_pair(uint16_t addr, float f0, float f1) {
    if (addr == EE_ADDR_NONE) {
        return false;
    }
    if (!ee_write_u16((uint16_t)(addr + 8u), EE_SIG_CLEARED)) {
        return false;
    }
    if (!ee_write_f32(addr, f0) || !ee_write_f32((uint16_t)(addr + 4u), f1)) {
        return false;
    }
    /* ee_write_* verify each byte by reading it back, so reaching here means the data is on the chip. */
    return ee_write_u16((uint16_t)(addr + 8u), EE_SIG_VALID);
}

bool cal_write_user_zero(func_t f, uint8_t range, float value) {
    uint16_t addr;

    if (serv_check() != SERV_ALLOWED) {
        return false;
    }
    /* A non-finite zero would be subtracted from every subsequent reading and turn the display into NaN
     * for ever, with nothing on the panel to say why. It is cheaper to refuse here than to explain later. */
    if (!isfinite(value)) {
        return false;
    }

    if ((f == FUNC_DCV || f == FUNC_ACDCV) && range < EE_DCV_RANGES) {
        addr = (uint16_t)(EE_UZERO_DCV + 4u * range);
    } else if (f == FUNC_OHM && range < EE_OHM_RANGES) {
        addr = (uint16_t)(EE_UZERO_OHM + 4u * range);
    } else {
        return false; /* the user zero exists for DC V and ohms only (cal.c's loader) */
    }
    return ee_write_f32(addr, value);
}

bool cal_write_zero(uint16_t addr, const float z[RATE_COUNT]) {
    uint8_t i;

    if (addr == EE_ADDR_NONE) {
        return false;
    }
    if (!ee_write_u16((uint16_t)(addr + 12u), EE_SIG_CLEARED)) {
        return false;
    }
    for (i = 0u; i < RATE_COUNT; i++) {
        if (!ee_write_f32((uint16_t)(addr + 4u * i), z[i])) {
            return false;
        }
    }
    return ee_write_u16((uint16_t)(addr + 12u), EE_SIG_VALID);
}

bool cal_save_counter_increment(void) {
    const uint16_t n = ee_read_u16(EE_SAVE_COUNTER);

    /* :SCAL:SAVE increments this counter and does nothing else -- it does not commit anything, because
     * nothing is staged. Saturating rather than wrapping: a counter that rolls over to 0
     * reads as "never calibrated", which is worse than reading as "many times". */
    if (n == 0xFFFFu) {
        return true;
    }
    return ee_write_u16(EE_SAVE_COUNTER, (uint16_t)(n + 1u));
}

/* ---- Loading ----------------------------------------------------------------------------------------- */

static cal_form_t form_of(func_t f) {
    switch (f) {
    case FUNC_DCV:
    case FUNC_ACDCV:
        return CAL_FORM_LINEAR_SIGNED;
    case FUNC_DCA:
    case FUNC_ACDCI:
        return CAL_FORM_QUAD_SIGNED;
    case FUNC_OHM:
        return CAL_FORM_OHM_2SEG;
    default:
        return CAL_FORM_QUAD;
    }
}

/* The identity of the form, which is what an invalid record must produce. See the file header for why
 * getting this backwards is not a small error. */
static cal_pair_t identity_of(cal_form_t form) {
    cal_pair_t p;

    if (form == CAL_FORM_LINEAR_SIGNED) {
        p.f0 = 1.0f; /* y = m * 1 */
        p.f1 = 0.0f;
    } else {
        p.f0 = 0.0f; /* y = m * (0*m + 1) */
        p.f1 = 1.0f;
    }
    return p;
}

/* Reads a pair, or the form's identity when its signature is not valid. */
static cal_pair_t load_pair(func_t f, uint8_t range, rate_t rate, cal_variant_t variant, cal_form_t form,
                            bool *valid_out) {
    ee_pair_t rec;
    cal_pair_t p;
    const uint16_t addr = cal_pair_address(f, range, rate, variant);

    if (addr != EE_ADDR_NONE && cal_read_pair(addr, &rec) && cal_pair_usable(f, &rec)) {
        p.f0 = rec.f0;
        p.f1 = rec.f1;
        if (valid_out != NULL) {
            *valid_out = true;
        }
        return p;
    }
    if (valid_out != NULL) {
        *valid_out = false;
    }
    return identity_of(form);
}

/* The per-function range scale. `range_scale` in tables.h covers the five functions that have their own
 * table in flash; the rest borrow one, which the loaders do by calling a different loader (0x0DAA2):
 * AC+DC V uses the DC V scale, AC+DC I the DC A scale, continuity the ohm scale, and the diode function
 * uses a plain 1.0. A zero scale would silently make every reading zero, so an unmapped function is an
 * explicit 1.0 rather than whatever the table holds. */
static float scale_of(func_t f, uint8_t range) {
    func_t src = f;

    switch (f) {
    case FUNC_ACDCV:
        src = FUNC_DCV;
        break;
    case FUNC_ACDCI:
        src = FUNC_DCA;
        break;
    case FUNC_CONT:
        src = FUNC_OHM;
        break;
    case FUNC_FREQ:
    case FUNC_PERIOD:
        src = FUNC_ACV;
        break;
    case FUNC_DIODE:
        return 1.0f;
    default:
        break;
    }
    if (src >= FUNC_COUNT || range > RANGE_MAX_INDEX) {
        return 1.0f;
    }
    return tbl_range_scale(src, range);
}

void cal_load(func_t f, uint8_t range, rate_t rate, cal_set_t *out) {
    ee_zero_t zrec;
    const cal_form_t form = form_of(f);
    const uint16_t zaddr = cal_zero_address(f, range);
    uint8_t i;

    out->form = form;
    out->scale = scale_of(f, range);
    out->uzero = 0.0f;
    out->ohm_threshold = 0.0f;
    out->alt_valid = false;

    /* The zero for THIS RATE. An invalid record means a zero of 0.0, never a stale value: the original
     * read the slot unconditionally and defaulted to 0.0 on a bad signature (loader 0x0CAC4). */
    out->zero = 0.0f;
    if (zaddr != EE_ADDR_NONE && cal_read_zero(zaddr, &zrec) && (uint8_t)rate < RATE_COUNT) {
        out->zero = zrec.z[(uint8_t)rate];
        if (f == FUNC_DIODE) {
            /* The diode zero is the ohm range-4 zero divided by a flash constant (loader 0x0D5AE). */
            out->zero /= CAL_DIODE_ZERO_DIVISOR;
        }
    }

    out->main = load_pair(f, range, rate, CAL_PAIR_MAIN, form, NULL);

    /* The alternate pair: the negative block for the signed forms, the upper segment for ohms. */
    if (form == CAL_FORM_LINEAR_SIGNED || form == CAL_FORM_QUAD_SIGNED) {
        out->alt = load_pair(f, range, rate, CAL_PAIR_NEG, form, &out->alt_valid);

        /* The negative-pair rule, and the one place in this file where the port deliberately differs from the original.
         * The original's DC V primary loader reads the negative record and then overwrites it with the
         * positive pair UNCONDITIONALLY at 0x0CD68, so a negative reading is always scaled by the positive
         * gain and the negative calibration table can never take effect at all. The port keeps the negative pair
         * when it is valid and falls back to the POSITIVE pair when it is not -- which on a meter
         * whose negative records are all cleared reproduces its readings exactly.
         * cal.h records the two differences. To revert to the
         * original exactly, replace the fallback below with identity_of(form) for ACDCV and with an
         * unconditional copy of `main` for DCV. */
        if (!out->alt_valid) {
            out->alt = out->main;
        }
    } else if (form == CAL_FORM_OHM_2SEG) {
        out->alt = load_pair(f, range, rate, CAL_PAIR_OHM_Y, form, &out->alt_valid);
        if (!out->alt_valid) {
            out->alt = identity_of(form);
        }
        out->ohm_threshold = (range < EE_OHM_RANGES) ? ohm_threshold[range] : 0.0f;
    } else {
        out->alt = identity_of(form);
    }

    /* The three AC low-range corrections, for AC V and AC A only. They are loaded for the current rate
     * whatever the range, as the original does, and only consulted on the lowest range. */
    for (i = 0u; i < 3u; i++) {
        out->low[i] = identity_of(CAL_FORM_QUAD);
    }
    if (f == FUNC_ACV || f == FUNC_ACA) {
        out->low[0] = load_pair(f, range, rate, CAL_PAIR_LOW1, CAL_FORM_QUAD, NULL);
        out->low[1] = load_pair(f, range, rate, CAL_PAIR_LOW2, CAL_FORM_QUAD, NULL);
        out->low[2] = load_pair(f, range, rate, CAL_PAIR_LOW3, CAL_FORM_QUAD, NULL);
    }

    /* The user zero, for DC V / AC+DC V and for ohms only, applied AFTER the gain. It has no signature of
     * its own: the word at 0x0894 is its only guard and only the boot initialiser ever checks it, so the
     * value is read unconditionally here exactly as the loaders do. */
    if (f == FUNC_DCV || f == FUNC_ACDCV) {
        if (range < EE_DCV_RANGES) {
            out->uzero = ee_read_f32((uint16_t)(EE_UZERO_DCV + 4u * range));
        }
    } else if (f == FUNC_OHM) {
        if (range < EE_OHM_RANGES) {
            out->uzero = ee_read_f32((uint16_t)(EE_UZERO_OHM + 4u * range));
        }
    }
}

/* ---- Stage 0: one sample ------------------------------------------------------------------------------ */

float cal_sample(const cal_set_t *c, uint32_t code, bool negative) {
    const float k = negative ? -CAL_VOLTS_PER_COUNT : CAL_VOLTS_PER_COUNT;

    /* (k * code) * scale - zero, in that order, matching 0x0AC8E-0x0ACDC. The zero comes off every sample
     * before any averaging, which is why it is here and not in cal_apply. */
    return (k * (float)code) * c->scale - c->zero;
}

/* ---- Stage 2: the mean ------------------------------------------------------------------------------- */

static float quad(const cal_pair_t *p, float m) {
    return m * (p->f0 * m + p->f1);
}

/* The AC low-range replacement, for the lowest range only (0x0AA5C for volts, 0x0AB82 for amps).
 * For a small enough input the main pair's result is REPLACED by one of three corrections, chosen by
 * bracket; above the third bracket the main result stands. */
static float ac_low_range(const cal_set_t *c, float m, float y, bool is_current) {
    const float *bracket = is_current ? ac_low_bracket_a : ac_low_bracket_v;
    const float zero_rule = is_current ? CAL_AC_LOW_ZERO_A : CAL_AC_LOW_ZERO_V;
    uint8_t k;

    for (k = 0u; k < 3u; k++) {
        if (m < bracket[k]) {
            const float corrected = quad(&c->low[k], m);

            /* Only the first bracket has the "and then force small results to zero" rule. */
            if (k == 0u && corrected <= zero_rule) {
                return 0.0f;
            }
            return corrected;
        }
    }
    return y;
}

float cal_apply(const cal_set_t *c, float mean, uint8_t range, uint8_t flags) {
    const bool sweep = (flags & CAL_F_SWEEP) != 0u;
    float y;

    switch (c->form) {
    case CAL_FORM_LINEAR_SIGNED:
        /* f1 is not read on this path at all. */
        y = mean * ((mean >= 0.0f) ? c->main.f0 : c->alt.f0);
        break;
    case CAL_FORM_QUAD_SIGNED:
        y = quad((mean >= 0.0f) ? &c->main : &c->alt, mean);
        break;
    case CAL_FORM_OHM_2SEG:
        /* The threshold is compared against the MEAN -- zero-subtracted and scaled, before the pair. */
        y = quad((mean < c->ohm_threshold) ? &c->main : &c->alt, mean);
        break;
    case CAL_FORM_QUAD:
    default:
        y = quad(&c->main, mean);
        break;
    }

    /* The gain stage above is NEVER skipped, not even during a sweep -- which is exactly why the sweeps
     * load the identity coefficients first. Everything that IS skipped during a sweep is
     * in cal_apply_addons, so this function has nothing left to decide. */
    (void)sweep;
    (void)range;
    return y;
}

/* The add-on stages are a separate entry point because they need the function, which cal_set_t does not
 * carry, and because the measurement path applies them in a defined order: user zero, then dead-band, then
 * the AC low-range replacement. Keeping them here rather than in meas.c means the order lives in one
 * place. */
float cal_apply_addons(const cal_set_t *c, func_t f, float mean, float y, uint8_t range, uint8_t flags) {
    if ((flags & CAL_F_SWEEP) != 0u) {
        return y;
    }

    /* User zero, after the gain. */
    if (f == FUNC_DCV || f == FUNC_ACDCV) {
        if (range < EE_DCV_RANGES) {
            y -= c->uzero;
        }
    } else if (f == FUNC_OHM) {
        if (range < EE_OHM_RANGES) {
            y -= c->uzero;
        }
    }

    /* Dead-bands. Both force the reading to exactly zero rather than scaling it. */
    if (f == FUNC_DCV || f == FUNC_ACDCV) {
        const float mag = (y < 0.0f) ? -y : y;
        const float limit = (range < 4u) ? (CAL_DEADBAND_K * deadband_v[range]) : CAL_DEADBAND_V_FLAT;

        if ((range < 4u) ? (mag < limit) : (mag <= limit)) {
            return 0.0f;
        }
    } else if (f == FUNC_OHM) {
        const float mag = (y < 0.0f) ? -y : y;
        const float limit = (range < 5u) ? (CAL_DEADBAND_K * deadband_ohm[range]) : CAL_DEADBAND_OHM_FLAT;

        if ((range < 5u) ? (mag < limit) : (mag <= limit)) {
            return 0.0f;
        }
    }

    /* The AC low-range stage, AC V and AC A only. On ranges 0..3 it is only a floor rule; on range 4 the
     * result can be replaced outright. */
    if (f == FUNC_ACV || f == FUNC_ACA) {
        const bool is_current = (f == FUNC_ACA);

        if (range < 4u) {
            const float floor_limit =
                CAL_AC_FLOOR_FRACTION * (is_current ? ac_floor_a[range] : ac_floor_v[range]);

            if (y < floor_limit) {
                return 0.0f;
            }
        } else if (range == 4u) {
            y = ac_low_range(c, mean, y, is_current);
        }
    }
    return y;
}
