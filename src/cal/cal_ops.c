/* cal_ops.c -- the :SCAL operations: COEF, the four sweeps, STD, TEST, the zero query, DATE, boot defaults.
 *
 * Replaces :SCAL:COEF 0x03F52, the sweeps 0x090BE / 0x09352 / 0x117D6 / 0x11A40, :SCAL:STD 0x11EAA and
 * 0x11F86, :SCAL:TEST 0x11D92, the zero queries 0x06A70 / 0x06C22, :SCAL:DATE 0x07000 and the boot
 * initialiser 0x05276. The record codec, the
 * addressing and the correction maths are in cal.c and are used from there; no EEPROM address is computed
 * here where cal_pair_address / cal_zero_address can supply it.
 *
 * NOT HERE, ON PURPOSE: the signature-stamping routine 0x03200 and every clear operation (cal.h). Nothing in
 * this file can write the cleared signature 0x5161 except through cal_write_pair / cal_write_zero, whose
 * first step it is, and those finish by writing the valid one.
 *
 * Every calibration WRITE entry point asks serv_check() first and refuses on anything but SERV_ALLOWED.
 * The read-only operations (STD, TEST, the zero query) are open, as serv.h lists them.
 */
#include "cal.h"

#include <avr/pgmspace.h>
#include "../fmathf.h"
#include "../main.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../fmt/fmt.h"
#include "../hal/eeprom.h"
#include "../meas/meas.h"
#include "../serv/serv.h"

/* ---- Constants --------------------------------------------------------------------------------------- */

/* The record maps address the AC low-range records and the auxiliary zero areas at range 4 only. */
#define CAL_AC_LOW_RANGE 4u

/* Conversions read and thrown away at the start of a sweep or of :SCAL:STD (three
 * calls of 0x0F2A0). The converter's register 8 and 9 setup that the original wrote just before them
 * (flash 0x15840-0x1584E) is not repeated: meas_init() and meas_set_rate() already give the converter those
 * very values, and while the converter is claimed nothing changes them. */
#define CAL_DUMMY_CONVERSIONS 3u

/* The averaging counts of a sweep cell: one conversion, then four; the four are the result (a3f2(1), 10 ms,
 * a3f2(4)). The single conversion has no other use -- its value is never looked at -- so
 * it is counted with the settling conversions instead. */
#define CAL_SWEEP_PROBE_SAMPLES 1u
#define CAL_SWEEP_RESULT_SAMPLES 4u

/* The converter free-runs at one conversion per 20 ms at every rate (meas.h). The original's settle times
 * are in milliseconds; they are converted to conversions, rounding up, so a 10 ms settle costs one. */
#define CAL_CONVERSION_MS 20u

/* A sweep settles in slices of at most this many conversions (200 ms) per cal_sweep_task() call, so the
 * longest settle of a cell (5 s) does not stall the main loop for 5 s in one piece. */
#define CAL_SETTLE_SLICE 10u

/* :SCAL:STD, AC kind: the original's 500 ms between its single conversion and its four. */
#define CAL_STD_AC_SETTLE_CONVERSIONS 25u

/* The result line is "%E,%d" (flash 0x0177 and 0x0951): upper-case exponent, six decimals, then the range. */

/* The signatures of the three guarded blocks of the boot initialiser. */
#define CAL_BLOCK_VALID EE_SIG_VALID

/* ---- Small helpers ----------------------------------------------------------------------------------- */

/* Takes and throws away `count` conversions. The converter belongs to the caller (meas_claim); the average
 * is the only way to consume conversions through the measurement module, and its value is not wanted. */
static bool discard_conversions(const cal_set_t *set, uint8_t count) {
    float unused;

    return count == 0u || meas_average_with(set, count, &unused);
}

/* The identity pair of a form: what an uncorrected reading needs (see the header of cal.c). */
static cal_pair_t identity_pair(cal_form_t form) {
    cal_pair_t p;

    if (form == CAL_FORM_LINEAR_SIGNED) {
        p.f0 = 1.0f;
        p.f1 = 0.0f;
    } else {
        p.f0 = 0.0f;
        p.f1 = 1.0f;
    }
    return p;
}

/* ---- :SCAL:COEF -------------------------------------------------------------------------------------- */

/* A write plan: every record the command will write, with its address already resolved. The plan is built
 * and checked completely before the first byte is written, so an error leaves the EEPROM untouched.
 * The largest plan is six pair records (OHM; ACV band 1; ACA band 5). */
#define COEF_MAX_PAIRS 6u

typedef struct {
    uint16_t addr;
    float f0;
    float f1;
} pair_job_t;

typedef struct {
    pair_job_t pair[COEF_MAX_PAIRS];
    uint8_t pair_count;
    bool has_zero;
    uint16_t zero_addr;
    float zero[RATE_COUNT];
} coef_plan_t;

/* The wire order inside the string is medium, fast, slow, while the records are written
 * fast, medium, slow. This maps a rate to the index of its pair in the wire order. */
static const uint8_t wire_index_of_rate[RATE_COUNT] = {1u, 0u, 2u};

static bool plan_add(coef_plan_t *plan, uint16_t addr, float f0, float f1) {
    if (addr == EE_ADDR_NONE || plan->pair_count >= COEF_MAX_PAIRS) {
        return false;
    }
    plan->pair[plan->pair_count].addr = addr;
    plan->pair[plan->pair_count].f0 = f0;
    plan->pair[plan->pair_count].f1 = f1;
    plan->pair_count++;
    return true;
}

/* The three rates of a variant, filled from the six per-range fields in the wire order medium, fast, slow:
 * rate 0 takes (v[2], v[3]), rate 1 takes (v[0], v[1]), rate 2 takes (v[4], v[5]). */
static bool plan_wire_pairs(coef_plan_t *plan, func_t f, uint8_t range, cal_variant_t variant,
                            const cal_coef_t *c) {
    uint8_t r;

    for (r = 0u; r < RATE_COUNT; r++) {
        const uint8_t w = wire_index_of_rate[r];

        if (!plan_add(plan, cal_pair_address(f, range, (rate_t)r, variant), c->v[2u * w],
                      c->v[2u * w + 1u])) {
            return false;
        }
    }
    return true;
}

/* The zero record of an AC function, written by COEF only. The zero fields Z2, Z3, Z4 arrive in wire order
 * and are stored fast = Z3, medium = Z2, slow = Z4 (original 0x43C2-0x4400 and 0x45E2-0x468C).
 * `c->z` holds Z2, Z3, Z4 in that order. */
static bool plan_ac_zero(coef_plan_t *plan, func_t f, uint8_t range, const cal_coef_t *c) {
    plan->zero_addr = cal_zero_address(f, range);
    if (plan->zero_addr == EE_ADDR_NONE) {
        return false;
    }
    plan->zero[RATE_FAST] = c->z[1];
    plan->zero[RATE_MEDIUM] = c->z[0];
    plan->zero[RATE_SLOW] = c->z[2];
    plan->has_zero = true;
    return true;
}

/* AC low set 2: rate r takes (Z[r], X[r]) -- the zero fields supply f0 and the extra fields f1, in the
 * order rate 0, 1, 2 (rt0 (Z2,X1), rt1 (Z3,X2), rt2 (Z4,X3)). */
static bool plan_low_set2(coef_plan_t *plan, func_t f, const cal_coef_t *c) {
    uint8_t r;

    for (r = 0u; r < RATE_COUNT; r++) {
        if (!plan_add(plan, cal_pair_address(f, CAL_AC_LOW_RANGE, (rate_t)r, CAL_PAIR_LOW2), c->z[r],
                      c->x[r])) {
            return false;
        }
    }
    return true;
}

/* DCV and DCA: a is the sign block (0 positive, 1 negative), b the range. */
static cal_coef_result_t plan_dc(coef_plan_t *plan, func_t f, const cal_coef_t *c) {
    if (c->count != 8u) {
        return CAL_COEF_BAD_COUNT;
    }
    if (c->a > 1u) {
        return CAL_COEF_BAD_BAND;
    }
    if (c->b >= EE_DCV_RANGES) { /* DCV and DCA both have five ranges */
        return CAL_COEF_BAD_RANGE;
    }
    if (!plan_wire_pairs(plan, f, c->b, (c->a == 0u) ? CAL_PAIR_MAIN : CAL_PAIR_NEG, c)) {
        return CAL_COEF_BAD_RANGE;
    }
    return CAL_COEF_OK;
}

/* OHM: a is the range; set X from v[] in the wire order medium, fast, slow, set Y from x[] in the order
 * fast, medium, slow (original 0x414C-0x42BE). */
static cal_coef_result_t plan_ohm(coef_plan_t *plan, const cal_coef_t *c) {
    uint8_t r;

    if (c->count != 13u) {
        return CAL_COEF_BAD_COUNT;
    }
    if (c->a >= EE_OHM_RANGES) {
        return CAL_COEF_BAD_RANGE;
    }
    if (!plan_wire_pairs(plan, FUNC_OHM, c->a, CAL_PAIR_MAIN, c)) {
        return CAL_COEF_BAD_RANGE;
    }
    for (r = 0u; r < RATE_COUNT; r++) {
        if (!plan_add(plan, cal_pair_address(FUNC_OHM, c->a, (rate_t)r, CAL_PAIR_OHM_Y), c->x[2u * r],
                      c->x[2u * r + 1u])) {
            return CAL_COEF_BAD_RANGE;
        }
    }
    return CAL_COEF_OK;
}

/* ACV: a is the band, b the range (original 0x42C4-0x4548).
 *   band 0: main pairs, and the zero record of range b
 *   band 1: low sets 1 and 2 (range 4), taking three extra fields
 *   band 2: low set 3
 * UNVERIFIED: the original takes ANY a >= 2 as set 3; only 2 is accepted here. Check that the vendor tool
 * sends 2 for set 3 (static analysis of the vendor calibration application). A tool that sends another
 * value gets CAL_COEF_BAD_BAND and nothing is written, which fails safe. */
static cal_coef_result_t plan_acv(coef_plan_t *plan, const cal_coef_t *c) {
    const uint8_t base_fields = 11u;

    if (c->count < base_fields) {
        return CAL_COEF_BAD_COUNT;
    }
    if (c->a > 2u) {
        return CAL_COEF_BAD_BAND;
    }
    if (c->count != ((c->a == 1u) ? 14u : base_fields)) {
        return CAL_COEF_BAD_COUNT;
    }
    if (c->b >= EE_AC_RANGES) {
        return CAL_COEF_BAD_RANGE;
    }

    if (c->a == 0u) {
        if (!plan_wire_pairs(plan, FUNC_ACV, c->b, CAL_PAIR_MAIN, c) ||
            !plan_ac_zero(plan, FUNC_ACV, c->b, c)) {
            return CAL_COEF_BAD_RANGE;
        }
    } else if (c->a == 1u) {
        if (!plan_wire_pairs(plan, FUNC_ACV, c->b, CAL_PAIR_LOW1, c) || !plan_low_set2(plan, FUNC_ACV, c)) {
            return CAL_COEF_BAD_RANGE;
        }
    } else if (!plan_wire_pairs(plan, FUNC_ACV, c->b, CAL_PAIR_LOW3, c)) {
        return CAL_COEF_BAD_RANGE;
    }
    return CAL_COEF_OK;
}

/* ACA: a < 5 is the range of the main pairs and the zero record; 5 selects low sets 1 and 2; 6 selects low
 * set 3; sets 1 to 3 are range 4 only (original 0x454E-0x480A).
 * UNVERIFIED: the original takes any a >= 6 as set 3; only 6 is accepted here (same check as plan_acv). */
static cal_coef_result_t plan_aca(coef_plan_t *plan, const cal_coef_t *c) {
    const uint8_t base_fields = 10u;

    if (c->count < base_fields) {
        return CAL_COEF_BAD_COUNT;
    }
    if (c->a > 6u) {
        return CAL_COEF_BAD_BAND;
    }
    if (c->count != ((c->a == 5u) ? 13u : base_fields)) {
        return CAL_COEF_BAD_COUNT;
    }

    if (c->a < EE_AC_RANGES) {
        if (!plan_wire_pairs(plan, FUNC_ACA, c->a, CAL_PAIR_MAIN, c) ||
            !plan_ac_zero(plan, FUNC_ACA, c->a, c)) {
            return CAL_COEF_BAD_RANGE;
        }
    } else if (c->a == 5u) {
        if (!plan_wire_pairs(plan, FUNC_ACA, CAL_AC_LOW_RANGE, CAL_PAIR_LOW1, c) ||
            !plan_low_set2(plan, FUNC_ACA, c)) {
            return CAL_COEF_BAD_RANGE;
        }
    } else if (!plan_wire_pairs(plan, FUNC_ACA, CAL_AC_LOW_RANGE, CAL_PAIR_LOW3, c)) {
        return CAL_COEF_BAD_RANGE;
    }
    return CAL_COEF_OK;
}

/* The service lock is asked first, and a refusal is now reported as CAL_COEF_LOCKED rather than as a
 * failed write: "you are not unlocked" and "the EEPROM did not verify" need different replies and
 * different advice to whoever is holding the screwdriver. The remote layer is meant to check the lock
 * before it gets here (cal.h), so reaching this refusal means it did not -- which is worth being able to
 * tell apart as well.
 *
 * Every coefficient is also checked for being a real number. The original validated nothing and would
 * store a NaN or an infinity, after which every reading on that range is nonsense and
 * cannot be read back to find out why. */
cal_coef_result_t cal_coef_write(func_t f, const cal_coef_t *c) {
    coef_plan_t plan = {.pair_count = 0u, .has_zero = false, .zero_addr = EE_ADDR_NONE};
    cal_coef_result_t result;
    uint8_t i;

    if (serv_check() != SERV_ALLOWED) {
        return CAL_COEF_LOCKED;
    }
    for (i = 0u; i < 6u; i++) {
        if (!isfinite(c->v[i]) || !isfinite(c->x[i])) {
            return CAL_COEF_BAD_VALUE;
        }
    }
    for (i = 0u; i < 3u; i++) {
        if (!isfinite(c->z[i])) {
            return CAL_COEF_BAD_VALUE;
        }
    }

    switch (f) {
    case FUNC_DCV:
    case FUNC_DCA:
        result = plan_dc(&plan, f, c);
        break;
    case FUNC_OHM:
        result = plan_ohm(&plan, c);
        break;
    case FUNC_ACV:
        result = plan_acv(&plan, c);
        break;
    case FUNC_ACA:
        result = plan_aca(&plan, c);
        break;
    default:
        return CAL_COEF_BAD_FUNCTION;
    }
    if (result != CAL_COEF_OK) {
        return result;
    }

    /* The plan is complete and every address is valid: only now does the EEPROM change. The records go in
     * the original's order -- rate 0, 1, 2 of each family, the zero record last -- and each is atomic
     * (cal_write_pair, cal_write_zero). A failure part-way leaves the records already written valid and the
     * failing one invalid, never half-valid. */
    for (i = 0u; i < plan.pair_count; i++) {
        if (!cal_write_pair(plan.pair[i].addr, plan.pair[i].f0, plan.pair[i].f1)) {
            return CAL_COEF_WRITE_FAILED;
        }
    }
    if (plan.has_zero && !cal_write_zero(plan.zero_addr, plan.zero)) {
        return CAL_COEF_WRITE_FAILED;
    }
    return CAL_COEF_OK;
}

/* ---- The guarded sweeps ------------------------------------------------------------------------------ */

/* SWEEP_RANGE and SWEEP_VERIFY are the panel calibration flows. They ride on this same machine
 * rather than having one of their own, because they need exactly what a sweep cell needs -- switch the
 * front end, settle, average, act -- and because the menu then watches ONE state (`cal_sweep_state()`) for
 * every long calibration operation instead of two that could disagree. The menu was written that way and
 * it is the right shape: a panel calibration must not block the loop while the meter settles and the
 * operator reads a reference meter. */
typedef enum { SWEEP_SHORT = 0, SWEEP_OPEN, SWEEP_RANGE, SWEEP_VERIFY, SWEEP_STD, SWEEP_TEST } sweep_kind_t;

/* The function and range cells each sweep covers, in the order the original visits them: SHOR is DCV 0..4, OHM 0..5, then ACV range 4; OPEN is DCA 0..4, then ACA range 4. */
typedef struct {
    uint8_t func;
    uint8_t first;
    uint8_t last;
} sweep_group_t;

static const sweep_group_t short_groups[] PROGMEM = {
    {FUNC_DCV, 0u, EE_DCV_RANGES - 1u},
    {FUNC_OHM, 0u, EE_OHM_RANGES - 1u},
    {FUNC_ACV, CAL_AC_LOW_RANGE, CAL_AC_LOW_RANGE},
};
static const sweep_group_t open_groups[] PROGMEM = {
    {FUNC_DCA, 0u, EE_DCA_RANGES - 1u},
    {FUNC_ACA, CAL_AC_LOW_RANGE, CAL_AC_LOW_RANGE},
};

/* Defect 15: the cells a sweep may write, and no others. The original's ONCE
 * forms wrote to the auxiliary area for every other function. */
static bool cell_allowed(sweep_kind_t kind, func_t f, uint8_t range) {
    if (kind == SWEEP_SHORT) {
        return (f == FUNC_DCV && range < EE_DCV_RANGES) || (f == FUNC_OHM && range < EE_OHM_RANGES) ||
               (f == FUNC_ACV && range == CAL_AC_LOW_RANGE);
    }
    if (kind == SWEEP_OPEN) {
        return (f == FUNC_DCA && range < EE_DCA_RANGES) || (f == FUNC_ACA && range == CAL_AC_LOW_RANGE);
    }

    /* The panel flows. They used to fall into the SWEEP_OPEN return above, which permits
     * only DC A and AC A -- so panel RANGE and VERIFY refused DC V and ohms, two of the three functions
     * the panel calibration scope names. The scope is DC V, DC A and ohms; AC stays with BK-Calibrator. */
    return (f == FUNC_DCV && range < EE_DCV_RANGES) || (f == FUNC_DCA && range < EE_DCA_RANGES) ||
           (f == FUNC_OHM && range < EE_OHM_RANGES);
}

/* Where a sweep result goes. DC V, ohm and DC A land in the zero record the loaders read. The two AC cells
 * go to the auxiliary areas 0x0C00 and 0x0C48, at range 4 (0x0C38 and 0x0C80); those are NOT the AC zero
 * records the loaders apply, which only :SCAL:COEF writes. */
static uint16_t sweep_zero_address(func_t f, uint8_t range) {
    if (f == FUNC_ACV) {
        return (uint16_t)(EE_ACZERO_V + 14u * range);
    }
    if (f == FUNC_ACA) {
        return (uint16_t)(EE_ACZERO_A + 14u * range);
    }
    return cal_zero_address(f, range);
}

/* The settling time before the measurement, in milliseconds, for the ALL forms and the ONCE forms:
 *   DC V   ALL: 1000 on range 3, else 10     ONCE: 1000 (0x9138)
 *   ohm    3000 (0xBB8) in both
 *   AC V   5000 (0x1388) in both
 *   DC A   10 in both
 *   AC A   ALL: 5000                          ONCE: 500 (0x93EA)
 * The ONCE figure for AC A is five seconds shorter than the ALL figure; that is what the original does and
 * it is kept. UNVERIFIED: whether 500 ms settles the AC A cell as well as 5000 ms; compare a ONCE result
 * with the ALL result on the instrument before relying on the ONCE form for AC A. */
static uint16_t sweep_settle_ms(func_t f, uint8_t range, bool once) {
    switch (f) {
    case FUNC_DCV:
        return (once || range == 3u) ? 1000u : 10u;
    case FUNC_OHM:
        return 3000u;
    case FUNC_ACV:
        return 5000u;
    case FUNC_ACA:
        return once ? 500u : 5000u;
    default:
        return 10u;
    }
}

/* The coefficient set a sweep measures with: the loader's scale and form, with every coefficient replaced by
 * the identity and the zero by 0.0. It is a LOCAL set. The running measurement state is
 * never touched, which is how a sweep restores the coefficients -- there is nothing to put back. */
static void load_sweep_set(func_t f, uint8_t range, cal_set_t *set) {
    uint8_t i;

    cal_load(f, range, RATE_MEDIUM, set);
    set->zero = 0.0f;
    set->uzero = 0.0f;
    set->main = identity_pair(set->form);
    set->alt = set->main;
    for (i = 0u; i < 3u; i++) {
        set->low[i] = identity_pair(CAL_FORM_QUAD);
    }
}

/* A sweep's per-cell result goes to whatever the caller registered, and this module neither knows nor cares
 * where that is (cal.h: the reporter). This used to format the "%E,%d" line here and write it to the host
 * link directly, which put a reply format inside the calibration module, made the sweeps impossible to
 * exercise without a serial port, and would have given a front-panel caller serial output as a side effect
 * of a key press.
 *
 * With no reporter registered a sweep still runs and still writes the EEPROM: the measurement is the point
 * and the result lines are an accessory. Formatting the line, and the trailing "!" the command ends with,
 * both belong to the remote layer. */
static cal_report_fn reporter;

void cal_set_reporter(cal_report_fn report) {
    reporter = report;
}

static void emit_result(func_t f, uint8_t range, float value) {
    if (reporter != NULL) {
        reporter(f, range, value);
    }
}

/* The settle in conversions rather than milliseconds, rounded up (see CAL_CONVERSION_MS). */
static uint16_t settle_conversions(func_t f, uint8_t range, bool once) {
    return (uint16_t)((sweep_settle_ms(f, range, once) + CAL_CONVERSION_MS - 1u) / CAL_CONVERSION_MS);
}

/* The state a sweep must leave as it found it. A sweep changes only the analog front end (meas_frontend_apply)
 * and consumes conversions; it assigns nothing in g_state and keeps its coefficients in a local set. What
 * remains to restore is therefore the front end, and the measurement pass that the discarded conversions
 * invalidated. Function and range are taken at start so the restore does not depend on g_state being
 * unchanged. The rate needs no restore: the converter is not switched by rate. */
typedef struct {
    func_t func;
    uint8_t range;
} sweep_saved_t;

static sweep_saved_t sweep_save(void) {
    sweep_saved_t s;

    s.func = g_state.func;
    s.range = g_state.range[g_state.func].index;
    return s;
}

/* One cell takes three steps, one per cal_sweep_task() call, so that no call holds the main loop for longer
 * than the slice of settling or the four result conversions: front end, settle, measure-write-report. */
typedef enum {
    CELL_FRONT_END = 0, /* switch the front end, load the identity set, work out the settle */
    CELL_SETTLE,        /* discard conversions, one slice per call */
    CELL_MEASURE        /* average, write the record, report, move to the next cell */
} cell_step_t;

static struct {
    cal_sweep_state_t state;
    sweep_kind_t kind;
    bool once;       /* a ONCE form: exactly the cell below, no walk over the groups */
    bool first_cell; /* the sweep's opening dummy conversions are still owed */
    uint8_t group;
    func_t func; /* the cell in hand */
    uint8_t range;
    cell_step_t step;
    uint16_t settle_left; /* conversions still to discard before the measurement */
    sweep_saved_t saved;
    cal_set_t set;

    /* The panel flows only. `reference` is what the better meter reads -- the operator types it before the
     * flow starts, which is why one call can do the whole thing. */
    float reference;
    float deviation;
    bool within_spec;
    bool result_valid;
    bool accept_cold; /* the operator saw the warm-up warning and chose to continue */

    /* A measured RANGE point waiting for the operator's decision: what the instrument read, at which rate,
     * and the change cal_range_preview() reported. Only cal_range_confirm() turns it into a write. */
    bool range_pending;
    rate_t range_rate;
    float range_measured;
    float range_change;

    /* :SCAL:STD and :SCAL:TEST. They are driven by the same task as the sweeps and for the same
     * reason, so they share the state, the step and the settle counter above. `value` is indexed by rate:
     * all three for STD, element 0 for TEST. */
    struct {
        float value[RATE_COUNT];
        uint8_t rate;      /* STD: the rate being measured */
        bool ac;           /* STD: the AC kind */
        rate_t saved_rate; /* TEST: the rate to put back */
    } query;
} sweep;

static const sweep_group_t *groups_of(sweep_kind_t kind) {
    return (kind == SWEEP_SHORT) ? short_groups : open_groups;
}

static uint8_t group_count_of(sweep_kind_t kind) {
    return (kind == SWEEP_SHORT) ? (uint8_t)(sizeof short_groups / sizeof short_groups[0])
                                 : (uint8_t)(sizeof open_groups / sizeof open_groups[0]);
}

static void enter_group(uint8_t group) {
    const sweep_group_t *groups = groups_of(sweep.kind);

    sweep.group = group;
    sweep.func = (func_t)pgm_read_byte(&groups[group].func);
    sweep.range = pgm_read_byte(&groups[group].first);
}

/* Moves to the cell after the one just written, in the order the original visits them. False when the sweep
 * has no more. */
static bool next_cell(void) {
    const sweep_group_t *groups = groups_of(sweep.kind);

    if (sweep.once) {
        return false;
    }
    if (sweep.range < pgm_read_byte(&groups[sweep.group].last)) {
        sweep.range++;
        return true;
    }
    if ((uint8_t)(sweep.group + 1u) < group_count_of(sweep.kind)) {
        enter_group((uint8_t)(sweep.group + 1u));
        return true;
    }
    return false;
}

/* Every exit path ends here: the success, a measurement or write failure, a guard failure and an abort. The
 * front end goes back to what the instrument is displaying, and releasing the converter invalidates the
 * measurement pass, so the next one reloads its own coefficients and settles again (meas_release). */
static void sweep_finish(cal_sweep_state_t result) {
    if (sweep.kind == SWEEP_TEST) {
        meas_set_rate(sweep.query.saved_rate); /* TEST forced the medium rate; nothing was claimed */
    } else {
        /* STD changes no front end, so there is nothing to restore for it, only the claim to give back. */
        if (sweep.kind != SWEEP_STD) {
            (void)meas_frontend_apply(sweep.saved.func, sweep.saved.range);
        }
        meas_release();
    }
    sweep.state = result;
}

static void std_front_end(void);
static void std_measure(void);
static void test_step(void);

static void cell_front_end(void) {
    uint16_t settle;

    if (sweep.kind == SWEEP_STD) {
        std_front_end();
        return;
    }
    if (!cell_allowed(sweep.kind, sweep.func, sweep.range)) {
        sweep_finish(CAL_SWEEP_FAILED); /* nothing is measured, nothing is written */
        return;
    }
    settle = meas_frontend_apply(sweep.func, sweep.range);
    settle = (uint16_t)(settle + settle_conversions(sweep.func, sweep.range, sweep.once) +
                        CAL_SWEEP_PROBE_SAMPLES);
    if (sweep.first_cell) {
        settle = (uint16_t)(settle + CAL_DUMMY_CONVERSIONS);
        sweep.first_cell = false;
    }
    /* WHICH COEFFICIENTS THE CELL MEASURES THROUGH -- and the two flows need opposite
     * answers, which is why this is a branch and not a constant.
     *
     * A ZERO SWEEP is measuring the instrument's raw offset, so it must see the converter unmodified:
     * `load_sweep_set` substitutes the identity pair and a zero of 0.0, as the original does.
     *
     * A PANEL flow is asking "what does this instrument READ right now", because `cal_range_commit` scales
     * the stored coefficient by reference over measured and `cal_verify` compares the reading against the
     * published specification. Both of those are statements about the CORRECTED reading. Measuring through
     * the identity gave an uncorrected number that the commit then treated as corrected: on the dump of a
     * calibrated meter that writes DC V range 0 as 1.049 where the right answer is 1.024, a 2.4 % error written into
     * the instrument and invisible afterwards, because the 5491B cannot read its coefficients back.
     *
     * A negative DC point therefore starts from whatever pair the loader actually applied, including the
     * negative-pair fallback, rather than from the identity. */
    if (sweep.kind == SWEEP_RANGE || sweep.kind == SWEEP_VERIFY) {
        cal_load(sweep.func, sweep.range, sweep.range_rate, &sweep.set);
    } else {
        load_sweep_set(sweep.func, sweep.range, &sweep.set);
    }
    sweep.settle_left = settle;
    sweep.step = CELL_SETTLE;
}

static void cell_settle(void) {
    const uint8_t slice =
        (sweep.settle_left < CAL_SETTLE_SLICE) ? (uint8_t)sweep.settle_left : (uint8_t)CAL_SETTLE_SLICE;

    if (!discard_conversions(&sweep.set, slice)) {
        sweep_finish(CAL_SWEEP_FAILED);
        return;
    }
    sweep.settle_left = (uint16_t)(sweep.settle_left - slice);
    if (sweep.settle_left == 0u) {
        sweep.step = CELL_MEASURE;
    }
}

/* Measure with the identity, store the same value in all three rate slots, report it. A
 * cell is written and reported only after it was measured; the record write is atomic (cal_write_zero), so
 * stopping between cells, or failing in this one, never leaves a half-valid record. The mean that
 * meas_average_with returns already has the gain stage applied, which under the identity set is a no-op. */
static void cell_measure(void) {
    float value;
    float z[RATE_COUNT];
    uint8_t i;

    if (sweep.kind == SWEEP_STD) {
        std_measure();
        return;
    }
    if (!meas_average_with(&sweep.set, CAL_SWEEP_RESULT_SAMPLES, &value)) {
        sweep_finish(CAL_SWEEP_FAILED);
        return;
    }

    /* The panel flows finish here: one cell, then done. They measure with the LOADED coefficients rather
     * than the identity the zero sweeps use, because a calibration trim and a verification are both about
     * what the instrument currently reads -- not about what it would read uncorrected. */
    if (sweep.kind == SWEEP_RANGE || sweep.kind == SWEEP_VERIFY) {
        if (sweep.kind == SWEEP_VERIFY) {
            sweep.result_valid = cal_verify(sweep.func, sweep.range, value, sweep.reference, &sweep.deviation,
                                            &sweep.within_spec);
            sweep_finish(sweep.result_valid ? CAL_SWEEP_DONE : CAL_SWEEP_FAILED);
        } else {
            /* The operator sees the change before anything is written. The sweep only measures and
             * previews; cal_range_confirm() commits the point that was previewed. */
            sweep.range_measured = value;
            sweep.range_pending =
                cal_range_preview(sweep.func, sweep.range, sweep.range_rate, value, sweep.reference,
                                  sweep.accept_cold, &sweep.range_change) == CAL_RANGE_OK;
            sweep_finish(sweep.range_pending ? CAL_SWEEP_DONE : CAL_SWEEP_FAILED);
        }
        return;
    }

    for (i = 0u; i < RATE_COUNT; i++) {
        z[i] = value;
    }
    if (!cal_write_zero(sweep_zero_address(sweep.func, sweep.range), z)) {
        sweep_finish(CAL_SWEEP_FAILED);
        return;
    }
    emit_result(sweep.func, sweep.range, value);

    if (next_cell()) {
        sweep.step = CELL_FRONT_END;
    } else {
        sweep_finish(CAL_SWEEP_DONE);
    }
}

/* The service lock first, then the function and range guard (ONCE forms), then the converter. Any refusal
 * returns false with nothing started and nothing written. A sweep already running is refused rather than
 * restarted over. */
static bool sweep_start(sweep_kind_t kind, bool once, func_t f, uint8_t range) {
    if (sweep.state == CAL_SWEEP_RUNNING) {
        return false;
    }
    if (serv_check() != SERV_ALLOWED) {
        return false;
    }
    if (once && !cell_allowed(kind, f, range)) {
        return false;
    }
    if (!meas_claim()) {
        return false;
    }

    sweep.kind = kind;
    sweep.once = once;
    sweep.range_pending = false;
    sweep.first_cell = true;
    sweep.saved = sweep_save();
    sweep.step = CELL_FRONT_END;
    if (once) {
        sweep.group = 0u;
        sweep.func = f;
        sweep.range = range;
    } else {
        enter_group(0u);
    }
    sweep.state = CAL_SWEEP_RUNNING;
    return true;
}

bool cal_short_once_start(func_t f, uint8_t range) {
    return sweep_start(SWEEP_SHORT, true, f, range);
}

bool cal_open_once_start(func_t f, uint8_t range) {
    return sweep_start(SWEEP_OPEN, true, f, range);
}

bool cal_short_all_start(void) {
    return sweep_start(SWEEP_SHORT, false, FUNC_DCV, 0u);
}

bool cal_open_all_start(void) {
    return sweep_start(SWEEP_OPEN, false, FUNC_DCA, 0u);
}

cal_sweep_state_t cal_sweep_task(void) {
    if (sweep.state != CAL_SWEEP_RUNNING) {
        return sweep.state;
    }
    if (sweep.kind == SWEEP_TEST) {
        test_step();
        return sweep.state;
    }
    switch (sweep.step) {
    case CELL_FRONT_END:
        cell_front_end();
        break;
    case CELL_SETTLE:
        cell_settle();
        break;
    case CELL_MEASURE:
    default:
        cell_measure();
        break;
    }
    return sweep.state;
}

cal_sweep_state_t cal_sweep_state(void) {
    return sweep.state;
}

/* Stops where it is. Nothing further is written; the cells already written stay, each being one atomic
 * record write. The state becomes IDLE, not FAILED: an abort is not a fault. Called with no sweep running it
 * does nothing, so a finished sweep's DONE or FAILED is still there to be read. */
void cal_sweep_abort(void) {
    if (sweep.state == CAL_SWEEP_RUNNING) {
        sweep_finish(CAL_SWEEP_IDLE);
    }
}

bool cal_sweep_busy(void) {
    return sweep.state == CAL_SWEEP_RUNNING;
}

/* ---- The panel calibration flows --------------------------------------------------------------
 * Both take the reference meter's reading, because the menu collects it before it starts anything. Both
 * are then driven by cal_sweep_task() from the main loop and watched through cal_sweep_state(), like a
 * sweep. Neither blocks.
 *
 * `cal_range_start` and `cal_verify_start` only measure; a RANGE point is written by `cal_range_confirm`. */
static bool panel_start(sweep_kind_t kind, func_t f, uint8_t range, float reference, bool accept_cold) {
    if (sweep.state == CAL_SWEEP_RUNNING) {
        return false;
    }
    if (kind == SWEEP_RANGE && serv_check() != SERV_ALLOWED) {
        return false;
    }
    /* The scope limit is checked here as well as in cal_panel.c, because this is the door the panel comes
     * in by and a guard that lives only behind the door is a guard nobody meets. */
    if (f != FUNC_DCV && f != FUNC_DCA && f != FUNC_OHM) {
        return false;
    }
    if (kind == SWEEP_RANGE && !accept_cold && !main_warmed_up()) {
        return false;
    }
    if (!meas_claim()) {
        return false;
    }

    sweep.saved = sweep_save(); /* keep the result: sweep_finish() restores the front end from it */
    sweep.kind = kind;
    sweep.once = true;
    sweep.first_cell = true;
    sweep.func = f;
    sweep.range = range;
    sweep.reference = reference;
    sweep.accept_cold = accept_cold;

    /* THE RATE IS PART OF THE REQUEST, so it is captured here and nowhere else.
     *
     * It used to be read twice: once to load the cell, and again after settling to decide which record to
     * write. Settling takes up to three seconds, and `meas_set_rate()` can change `g_state.rate` in that
     * window -- it is an ordinary remote command and the converter claim does not stop it. The point was
     * then measured through rate A's zero and coefficients and written into rate B's record: a calibration
     * write that is wrong in a way nothing afterwards can detect, on a meter whose coefficients cannot be
     * read back.
     *
     * One read, at the moment the operator asked for the point. */
    sweep.range_rate = g_state.rate;
    sweep.result_valid = false;
    sweep.range_pending = false;
    sweep.deviation = 0.0f;
    sweep.within_spec = false;
    sweep.step = CELL_FRONT_END;
    sweep.state = CAL_SWEEP_RUNNING;
    return true;
}

bool cal_range_start(func_t f, uint8_t range, float reference, bool accept_cold) {
    return panel_start(SWEEP_RANGE, f, range, reference, accept_cold);
}

bool cal_verify_start(func_t f, uint8_t range, float reference) {
    /* VERIFY writes nothing, so a cold meter is the operator's problem to interpret and not ours to
     * forbid. It has never had a warm-up gate. */
    return panel_start(SWEEP_VERIFY, f, range, reference, true);
}

bool cal_verify_result(float *deviation, bool *within_spec) {
    if (!sweep.result_valid) {
        return false;
    }
    if (deviation != NULL) {
        *deviation = sweep.deviation;
    }
    if (within_spec != NULL) {
        *within_spec = sweep.within_spec;
    }
    return true;
}

bool cal_range_pending(float *change_fraction) {
    if (!sweep.range_pending || sweep.state != CAL_SWEEP_DONE) {
        return false;
    }
    *change_fraction = sweep.range_change;
    return true;
}

/* The point that was previewed, with the value that was measured then: the commit does not take a second
 * reading, so what is written is exactly what the operator was shown. The pending point is spent whatever
 * the outcome. A calibration written here changes what the next pass must load, so the pass is invalidated. */
bool cal_range_confirm(void) {
    bool ok;

    if (!sweep.range_pending || sweep.state != CAL_SWEEP_DONE) {
        return false;
    }
    sweep.range_pending = false;
    ok = cal_range_commit(sweep.func, sweep.range, sweep.range_rate, sweep.range_measured, sweep.reference,
                          sweep.accept_cold) == CAL_RANGE_OK;
    meas_invalidate();
    return ok;
}

/* ---- :SCAL:STD and :SCAL:TEST ----------------------------------------------------------------------- */

/* Both used to be blocking calls: about a second for STD (three rates, each with its settle and four
 * conversions), and as long as a reading takes for TEST. That held the main loop, so the host link, the keys
 * and the display stopped for the duration (ORIGINAL_BUGS #1, the same defect the sweeps had).
 * They are now start/poll pairs on the sweep machine: `cal_scal_*_start()` begins, the main loop runs
 * `cal_sweep_task()`, and the remote layer sends the reply when `cal_sweep_state()` reaches DONE and has
 * collected the numbers with `cal_scal_*_result()`. */

/* The original measured once at the medium rate and printed the number three times
 * (ORIGINAL_BUGS #55). This measures the current function and range at all three rates and writes nothing.
 * It fills the array BY RATE (v[0] fast, v[1] medium, v[2] slow); the executor prints medium first, since
 * field 1 of the reply is the medium reading -- see cal.h. The converter is not switched by
 * rate, so each rate differs only in the coefficient load -- which, under the identity, matters only through
 * the zero a non-DC V function keeps.
 *
 * The identity is set exactly as the original sets it, per kind:
 *   DC: both pairs = (func == DC V ? 1.0 : 0.0, 1.0); the zero is cleared for DC V only, every other
 *       function keeps the zero of the rate being measured (0x11EAA)
 *   AC: the main pair = (0.0, 1.0); the three low sets (0.0, 1.0); the zero cleared (0x11F86); the second
 *       pair stays as loaded.
 * With AC selected on a function whose form is the linear one the main pair's f0 of 0.0 gives a zero
 * reading; that is what the original computes and it is kept.
 * Per rate the conversions discarded are the original's, in one count: 3 dummies at the first rate only,
 * the single probe conversion, and for the AC kind the 500 ms between the probe and the four (25 conversions
 * at 20 ms); the DC kind's 10 ms is less than one conversion period and is therefore not counted. Then four
 * are averaged. The converter is claimed for the duration like a sweep's, so a measurement pass cannot take
 * conversions in the middle of the average. */
static void std_front_end(void) {
    const func_t f = sweep.func;
    cal_set_t *set = &sweep.set;
    uint16_t settle = CAL_SWEEP_PROBE_SAMPLES;
    uint8_t i;

    /* A key can now be pressed while this runs. The set below belongs to the function and range STD started
     * on, so a change of either would pair coefficients with the wrong front end: stop instead. */
    if (g_state.func != f || g_state.range[f].index != sweep.range) {
        sweep_finish(CAL_SWEEP_FAILED);
        return;
    }
    cal_load(f, sweep.range, (rate_t)sweep.query.rate, set);
    set->uzero = 0.0f;
    if (!sweep.query.ac) {
        set->main.f0 = (f == FUNC_DCV) ? 1.0f : 0.0f;
        set->main.f1 = 1.0f;
        set->alt = set->main;
        if (f == FUNC_DCV) {
            set->zero = 0.0f;
        }
    } else {
        set->main.f0 = 0.0f;
        set->main.f1 = 1.0f;
        for (i = 0u; i < 3u; i++) {
            set->low[i] = identity_pair(CAL_FORM_QUAD);
        }
        set->zero = 0.0f;
        settle = (uint16_t)(settle + CAL_STD_AC_SETTLE_CONVERSIONS);
    }
    if (sweep.query.rate == 0u) {
        settle = (uint16_t)(settle + CAL_DUMMY_CONVERSIONS);
    }
    sweep.settle_left = settle;
    sweep.step = CELL_SETTLE;
}

static void std_measure(void) {
    float mean;

    if (!meas_average_with(&sweep.set, CAL_SWEEP_RESULT_SAMPLES, &mean)) {
        sweep_finish(CAL_SWEEP_FAILED);
        return;
    }
    sweep.query.value[sweep.query.rate] = mean; /* the gain stage is already applied (meas_average_with) */
    sweep.query.rate++;
    if (sweep.query.rate < RATE_COUNT) {
        sweep.step = CELL_FRONT_END;
    } else {
        sweep_finish(CAL_SWEEP_DONE);
    }
}

/* Begins a STD or TEST once the caller has made its own checks. Like a sweep start it forgets any panel point
 * that was awaiting confirmation, because the state and the cell fields are shared. The function and range
 * are taken here, once, as the request. */
static void query_begin(sweep_kind_t kind) {
    sweep.kind = kind;
    sweep.range_pending = false;
    sweep.result_valid = false;
    sweep.func = g_state.func;
    sweep.range = g_state.range[g_state.func].index;
    sweep.query.rate = 0u;
    sweep.step = CELL_FRONT_END;
    sweep.state = CAL_SWEEP_RUNNING;
}

bool cal_scal_std_start(cal_std_kind_t kind) {
    if (sweep.state == CAL_SWEEP_RUNNING || !meas_claim()) {
        return false;
    }
    query_begin(SWEEP_STD);
    sweep.query.ac = (kind == CAL_STD_AC);
    return true;
}

bool cal_scal_std_result(float out[RATE_COUNT]) {
    uint8_t r;

    if (sweep.kind != SWEEP_STD || sweep.state != CAL_SWEEP_DONE) {
        return false;
    }
    for (r = 0u; r < RATE_COUNT; r++) {
        out[r] = sweep.query.value[r];
    }
    return true;
}

/* The original forced the medium rate (leaving it changed), measured with the normal
 * coefficients, and replied the reading with the REL reference subtracted. Here the measurement module takes
 * the reading, so the coefficients are the ones in use by definition; the rate is forced to medium for the
 * reading and put back when the test ends, however it ends (sweep_finish). The original measured only
 * functions 0 to 4 (other functions replied a stale value); those other functions are refused here.
 *
 * The converter is NOT claimed: the reading comes from the normal pass. While a TEST runs the main loop calls
 * cal_sweep_task() instead of meas_task() (main.c), so test_step() runs the pass itself, one call at a time.
 * The wait is bounded by the measurement module: a conversion that never arrives moves the phase to
 * MEAS_FAULT after a bounded wait (meas.h). The one other way the original could wait for ever
 * was a trigger mode that holds the pass back (manual or bus), with the whole instrument frozen; TEST is
 * refused there instead.
 * UNVERIFIED: that g_state.reading.calibrated is the value the original replied (RAM 0x0EE5, before the
 * overload test and before dB/dBm) and that rel_t.reference is the base the original subtracted. Check both in meas/meas.c when it exists. */
bool cal_scal_test_start(void) {
    if (g_state.func > FUNC_OHM || g_state.trigger != TRIG_IMM || sweep.state == CAL_SWEEP_RUNNING) {
        return false;
    }
    query_begin(SWEEP_TEST);
    sweep.query.saved_rate = g_state.rate;
    meas_set_rate(RATE_MEDIUM);
    meas_invalidate();
    return true;
}

static void test_step(void) {
    const func_t f = sweep.func;

    /* A key can change the function or the rate while the pass runs; the reading would then be of something
     * else, so it is not reported. */
    if (g_state.func != f || g_state.rate != RATE_MEDIUM) {
        sweep_finish(CAL_SWEEP_FAILED);
        return;
    }
    meas_task();
    if (meas_reading_ready()) {
        float value = g_state.reading.calibrated;

        if (g_state.rel[f].active) {
            value -= g_state.rel[f].reference;
        }
        sweep.query.value[0] = value;
        meas_take_reading();
        sweep_finish(CAL_SWEEP_DONE);
    } else if (meas_phase() == MEAS_FAULT) {
        sweep_finish(CAL_SWEEP_FAILED);
    }
}

bool cal_scal_test_result(float *out) {
    if (sweep.kind != SWEEP_TEST || sweep.state != CAL_SWEEP_DONE) {
        return false;
    }
    *out = sweep.query.value[0];
    return true;
}

/* ---- :SCAL:DCV:ZERO and :SCAL:ACV:ZERO ---------------------------------------------------------------- */

/* The original read the zero for the current range at all three rates and printed them;
 * an invalid signature at rate 0 printed three zeros. It left the running zero changed afterwards; this
 * reads the EEPROM and changes nothing else.
 * The records read are those the original read: DC V -> 0x048C; AC V -> 0x0C00 (the auxiliary area the
 * sweep writes); AC A -> 0x0C48 (likewise), at the current range. The two commands cannot be told apart
 * from the function alone, so the union is accepted: function 0, 1 or 3. The original accepted 0 and 3 for
 * DCV:ZERO and 1 and 3 for ACV:ZERO and left the base register uninitialised for the others (U1).
 * UNVERIFIED: a DCV:ZERO issued while AC V is selected, or ACV:ZERO while DC V is selected, is answered here
 * from the selected function's record; the original's answer there is undefined. */
bool cal_zero_query(func_t f, uint8_t *range_out, float z[RATE_COUNT]) {
    ee_zero_t rec;
    uint16_t addr;
    uint8_t range;
    uint8_t i;

    switch (f) {
    case FUNC_DCV:
        addr = EE_ZERO_DCV;
        break;
    case FUNC_ACV:
        addr = EE_ACZERO_V;
        break;
    case FUNC_ACA:
        addr = EE_ACZERO_A;
        break;
    default:
        return false;
    }
    range = g_state.range[f].index;
    if (range >= EE_AC_RANGES) { /* all three families have five ranges */
        return false;
    }

    *range_out = range;
    if (cal_read_zero((uint16_t)(addr + 14u * range), &rec)) {
        for (i = 0u; i < RATE_COUNT; i++) {
            z[i] = rec.z[i];
        }
    } else {
        for (i = 0u; i < RATE_COUNT; i++) {
            z[i] = 0.0f;
        }
    }
    return true;
}

/* ---- :SCAL:DATE -------------------------------------------------------------------------------------- */

/* Each of the 8 characters must be a digit or '/', as the original tested (0x7000-0x704A),
 * but a failure now writes nothing. The original raised the error and wrote anyway. */
bool cal_date_write(const char date[8]) {
    uint8_t i;

    if (serv_check() != SERV_ALLOWED) {
        return false;
    }
    for (i = 0u; i < 8u; i++) {
        const char ch = date[i];

        if (!((ch >= '0' && ch <= '9') || ch == '/')) {
            return false;
        }
    }
    return ee_write_block(EE_CAL_DATE, date, 8u);
}

/* ---- Boot defaults ----------------------------------------------------------------------------------- */

/* The default image of one :THID field: `text` zero-padded to `len` bytes. The original wrote the literal and
 * then whatever bytes followed it in flash or in a scratch buffer (a dump of a calibrated meter holds such residue at
 * 0x0A02); padding with zeros is the only difference. An empty `text` is the image of a zero counter. */
static void field_image(uint8_t *image, const char *text, uint8_t len) {
    uint8_t i;

    for (i = 0u; i < len; i++) {
        image[i] = 0u;
    }
    for (i = 0u; i < len && text[i] != '\0'; i++) {
        image[i] = (uint8_t)text[i];
    }
}

/* Gives one :THID field its default if, and only if, the field provably holds nothing but what an
 * interrupted default write leaves. The initialiser writes a field in ascending address order and
 * each byte is verified before the next, so the only states it can leave behind are `k` bytes of the default
 * followed by erased 0xFF bytes: k = 0 is a blank field, k = len is a finished one. Any other byte means
 * somebody else wrote this field -- the vendor firmware, a :THID command, a service entry -- and it is left
 * exactly as it is. False only when a write fails. */
static bool thid_field(uint16_t addr, const char *text, uint8_t len) {
    uint8_t image[16];
    uint8_t k = 0u;
    uint8_t i;

    field_image(image, text, len);
    while (k < len && ee_read_u8((uint16_t)(addr + k)) == image[k]) {
        k++;
    }
    for (i = k; i < len; i++) {
        if (ee_read_u8((uint16_t)(addr + i)) != 0xFFu) {
            return true; /* foreign data: not ours to touch */
        }
    }
    return k == len || ee_write_block(addr, image, len);
}

/* Block 2: the user-zero tables, guarded by the word at 0x0894.
 *
 * The original wrote 0.0 into all eleven floats whenever that word was not valid, so one cleared word
 * destroyed every user zero at the next power-up (ORIGINAL_BUGS #59, which is how :SCAL:CLEAR_REL lost them).
 * Here a float is touched only if it cannot be a user zero at all -- not a finite number, which is what an
 * erased or corrupt cell reads as -- and the word is then stamped. Valid user zeros survive any state of the
 * signature. */
static bool repair_user_zero(uint16_t addr) {
    if (isfinite(ee_read_f32(addr))) {
        return true;
    }
    return ee_write_f32(addr, 0.0f);
}

static void boot_user_zeros(void) {
    uint8_t i;

    if (ee_read_u16(EE_UZERO_DCV_SIG) == CAL_BLOCK_VALID) {
        return;
    }
    for (i = 0u; i < EE_DCV_RANGES; i++) {
        if (!repair_user_zero((uint16_t)(EE_UZERO_DCV + 4u * i))) {
            return;
        }
    }
    for (i = 0u; i < EE_OHM_RANGES; i++) {
        if (!repair_user_zero((uint16_t)(EE_UZERO_OHM + 4u * i))) {
            return;
        }
    }
    (void)ee_write_u16(EE_UZERO_DCV_SIG, CAL_BLOCK_VALID);
}

/* Block 3: the :THID block. Contents as the original writes them: the serial placeholder, three dates, two
 * counters and two member numbers. The original never recorded that it had run, so the block was rewritten
 * at every boot of a unit whose production date had never been set (ORIGINAL_BUGS #108).
 *
 * The earlier UNVERIFIED here asked whether stamping 0x0A00 would close the :THID:PROD:DATE write-once
 * window. IT WOULD, and it has been settled at the instruction level: 0x048D2
 * reads that word, 0x048D8-0x048DE compares it with 0x5554 and branches away when it matches, and only the
 * writing path stamps it at 0x048F4 after storing the date. So that word is the handler's own lock, and
 * stamping it here would permanently prevent a production date from ever being set on a blank unit -- the
 * window would close before anyone reached it, with no command to reopen it. A meter that
 * already holds a valid word is unaffected; a fresh unit would have been quietly ruined, which is the
 * worst kind of ruined because it looks like it worked.
 *
 * So this block records that it has run in its OWN marker, EE_BOOT_MARK, and never WRITES 0x0A00.
 *
 * BUT IT MUST STILL READ 0x0A00 . The new
 * marker is erased -- 0xFFFF -- on every meter that has ever run the vendor firmware.
 * Deciding on the marker alone, the very first boot of this firmware would have taken the
 * uninitialised branch and written the DEFAULTS OVER REAL DATA: serial placeholder, production date
 * 00/00/00, cal count 0 where a calibrated unit's is 5, service count 0, members "00". And since 0x0A00 stays
 * stamped, the production date could then never be set again. A meter would have been wiped by the act of
 * running our firmware once.
 *
 * The rule is therefore: an EXISTING VENDOR-WRITTEN METER COUNTS AS INITIALISED. Skip the defaults when
 * either guard says so -- 0x0A00 valid (a vendor meter, which is what the vendor's own initialiser checks
 * at 0x0538E-0x053A6) or our marker valid -- and stamp our marker in both cases so later boots take the
 * fast path without re-reading. 0x0A00 is read, never written: #108 stays fixed, the write-once window
 * survives, and no existing meter is touched.
 *
 * WHEN BOTH GUARDS ARE FALSE. That is a blank unit, or one whose single guard word lost a bit. The
 * original, and this function until now, then overwrote every field with its default, so one flipped bit in
 * a two-byte word re-initialised the identity block of a unit that had been set up. A checksum over the
 * block cannot repair that here: every legitimate writer of these fields (the :THID handlers in serv/thid.c,
 * the history counters, the service entries) would have to refresh it on each write, and none of them does,
 * so a stored checksum would be stale after the first command. Instead each field proves for itself that it
 * is ours to fill -- see thid_field -- and anything else is left alone. The marker is then stamped, because
 * every field is now either default, completed, or somebody's data. */
static void boot_thid(void) {
    static const char serial_placeholder[] = "**-***-*****";
    static const char zero_date[] = "00/00/00";
    static const char zero_member[] = "00";

    /* Either guard means "already initialised", and the vendor's stamp is checked FIRST because it is the
     * one that is set on every meter in the field.
     *
     * NOTHING IS WRITTEN ON THIS PATH -- not even our own marker. Stamping the
     * marker here would let later boots skip one EEPROM word read, but the host test shows what that costs:
     * running this firmware on an existing meter changes two bytes of its EEPROM. They would be
     * harmless bytes in free space, but "our firmware never writes a single byte of an existing
     * meter's EEPROM at boot" is a far stronger property to be able to state than one saved word read per
     * power-on, and it is a property the test can check. So the marker is stamped only where it is actually
     * needed: after the defaults have been written to a blank meter, to make that idempotent. */
    if (ee_read_u16(EE_THID_PRODDATE_LOCK) == EE_SIG_VALID ||
        ee_read_u16(EE_BOOT_MARK) == EE_BOOT_MARK_VALUE) {
        return;
    }
    if (!thid_field(EE_THID_SERIAL, serial_placeholder, 16u) ||
        !thid_field(EE_THID_PROD_DATE, zero_date, 8u) || !thid_field(EE_THID_CAL_DATE, zero_date, 8u) ||
        !thid_field(EE_THID_SERV_DATE, zero_date, 8u) || !thid_field(EE_THID_CAL_COUNT, "", 2u) ||
        !thid_field(EE_THID_SERV_COUNT, "", 2u) || !thid_field(EE_THID_CAL_MEMBER, zero_member, 2u) ||
        !thid_field(EE_THID_SERV_MEMBER, zero_member, 2u)) {
        return;
    }
    (void)ee_write_u16(EE_BOOT_MARK, EE_BOOT_MARK_VALUE);
}

/* Block 1 of the original -- the identity and settings block guarded by 0x0800 -- is
 * deliberately absent: identity writes are service-locked and confirmed by the user, never boot behaviour, and the
 * original ran an interactive front-panel prompt there. This function cannot touch a pair record, a zero
 * record or the identity block. A write that fails leaves its guard unstamped, so the block is retried at the
 * next boot rather than half-trusted. */
void cal_boot_defaults_init(void) {
    boot_user_zeros();
    boot_thid();
}
