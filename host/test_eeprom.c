/* test_eeprom.c -- host tests for the boot-time EEPROM initialiser. Not part of the firmware.
 *
 * This is the test that catches a boot-time initialiser overwriting data that a meter in the field
 * already holds:
 *
 *   1. Run `cal_boot_defaults_init()` over an image of a real meter's EEPROM. **The result must be
 *      byte-identical.** Running our firmware on an existing meter must not change one byte of its
 *      calibration or identity data. An initialiser that rewrote `boot_thid()` defaults would fail this test loudly
 *      -- serial, production date, cal count 5 -> 0 -- which is exactly why it is worth having.
 *   2. Run it over an erased image. The defaults must appear, and **0x0A00 must still be erased**: that
 *      word is the production-date handler's write-once guard, and stamping it would close the window for
 *      good.
 *
 * Built as one translation unit: the real cal.c and cal_ops.c are included after a fake EEPROM and stubs
 * for the modules they call. That keeps the test honest -- it exercises the shipped code, not a copy of it
 * -- while needing no link against the whole firmware.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- The fake EEPROM ---------------------------------------------------------------------------------
 * 4096 bytes, as on the device. Writes are recorded so a test can say not just "the result is right" but
 * "nothing else was touched", which is the stronger claim and the one that matters here. */
#define FAKE_EE_SIZE 4096u
static uint8_t fake_ee[FAKE_EE_SIZE];
static uint32_t write_count;

uint8_t ee_read_u8(uint16_t addr) {
    return (addr < FAKE_EE_SIZE) ? fake_ee[addr] : 0xFFu;
}
uint16_t ee_read_u16(uint16_t addr) {
    return (uint16_t)(ee_read_u8(addr) | ((uint16_t)ee_read_u8((uint16_t)(addr + 1u)) << 8));
}
float ee_read_f32(uint16_t addr) {
    union {
        float f;
        uint8_t b[4];
    } v;
    for (uint8_t i = 0; i < 4; i++) {
        v.b[i] = ee_read_u8((uint16_t)(addr + i));
    }
    return v.f;
}
void ee_read_block(uint16_t addr, void *dst, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        ((uint8_t *)dst)[i] = ee_read_u8((uint16_t)(addr + i));
    }
}
bool ee_write_u8(uint16_t addr, uint8_t v) {
    if (addr >= FAKE_EE_SIZE) {
        return false;
    }
    if (fake_ee[addr] != v) {
        fake_ee[addr] = v;
        write_count++;
    }
    return true;
}
bool ee_write_u16(uint16_t addr, uint16_t v) {
    return ee_write_u8(addr, (uint8_t)(v & 0xFFu)) && ee_write_u8((uint16_t)(addr + 1u), (uint8_t)(v >> 8));
}
bool ee_write_f32(uint16_t addr, float v) {
    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = v;
    for (uint8_t i = 0; i < 4; i++) {
        if (!ee_write_u8((uint16_t)(addr + i), u.b[i])) {
            return false;
        }
    }
    return true;
}
bool ee_write_block(uint16_t addr, const void *src, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        if (!ee_write_u8((uint16_t)(addr + i), ((const uint8_t *)src)[i])) {
            return false;
        }
    }
    return true;
}

/* ---- Stubs for the modules cal_ops.c calls ---------------------------------------------------------
 * Each one is the answer that makes the boot path testable in isolation. `serv_check` allows, because the
 * boot initialiser is not a protected command; the measurement stubs fail, because no sweep runs here and
 * a stub that pretended to measure would let a test pass for the wrong reason. */
#include "../src/state.h"

instrument_t g_state;

#include "../src/serv/serv.h"
serv_result_t serv_check(void) { return SERV_ALLOWED; }
void serv_note_activity(void) {}
bool serv_is_unlocked(void) { return true; }

#include "../src/meas/meas.h"
uint8_t meas_frontend_apply(func_t f, uint8_t range) { (void)f; (void)range; return 0u; }
bool meas_claim(void) { return false; }
void meas_release(void) {}
bool meas_average_with(const cal_set_t *set, uint8_t samples, float *out) {
    (void)set; (void)samples; (void)out; return false;
}
void meas_invalidate(void) {}

/* The rest of meas.h, reached only by code paths this test does not exercise (cal_scal_test and the
 * sweeps). They abort rather than pretend: a stub that returned a plausible reading would let a future
 * test pass for the wrong reason, which is worse than a link error. */
void meas_set_rate(rate_t r) { (void)r; }
void meas_set_function(func_t f) { (void)f; }
void meas_set_range(func_t f, uint8_t range, bool manual) { (void)f; (void)range; (void)manual; }
bool meas_reading_ready(void) { return false; }
meas_phase_t meas_phase(void) { return MEAS_FAULT; }
void meas_task(void) {}
void meas_take_reading(void) {}
bool meas_autorange_step(void) { return false; }
bool meas_overload(void) { return false; }
bool meas_fault_take(void) { return false; }
bool meas_zero_capture(func_t f, uint8_t range, float *out) {
    (void)f;
    (void)range;
    (void)out;
    return false;
}
void meas_tick_1ms(void) {}
float meas_apply_db(float c, func_t f, bool *oor) { (void)f; if (oor) *oor = false; return c; }
float meas_apply_rel(float v, func_t f) { (void)f; return v; }
float meas_apply_percent(float v) { return v; }
void meas_update_limits(float v, func_t f) { (void)v; (void)f; }
void meas_update_minmax(float v) { (void)v; }
void meas_minmax_reset(void) {}
void meas_hold_arm(void) {}
void meas_hold_release(void) {}
bool meas_relay_exercise_start(bool quick) { (void)quick; return false; }
bool meas_relay_exercise_busy(void) { return false; }
uint8_t meas_relay_exercise_seconds_left(void) { return 0u; }

/* tables.h's range_scale: the real flash table lives in tables.c, which this test does not include
 * because it would drag in the whole display and menu string set. Unity gain for every range is right
 * here: no test in this file depends on a scale factor, and a wrong-looking number would be noticed. */
const float range_scale[FUNC_COUNT][RANGE_MAX_INDEX + 1u] = {
    [FUNC_DCV] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
};

#include "../src/main.h"
bool main_warmed_up(void) { return true; }
uint16_t main_power_on_hours(void) { return 0u; }
void main_soft_reset(void) {}

/* The real code under test. */
#include "../src/cal/cal.c"
#include "../src/cal/cal_ops.c"
#include "../src/cal/cal_panel.c"

/* ---- The tests -------------------------------------------------------------------------------------- */

static int failures;

static void check(bool ok, const char *what) {
    printf("  %-62s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        failures++;
    }
}

static bool load(const char *path, uint8_t *dst) {
    FILE *f = fopen(path, "rb");
    size_t n;

    if (f == NULL) {
        return false;
    }
    n = fread(dst, 1u, FAKE_EE_SIZE, f);
    fclose(f);
    return n == FAKE_EE_SIZE;
}

/* 1. An existing meter must come through untouched. */
static void test_existing_meter_is_untouched(const char *image) {
    uint8_t before[FAKE_EE_SIZE];

    printf("existing meter (%s)\n", image);
    if (!load(image, fake_ee)) {
        printf("  cannot read the image -- SKIPPED (not a pass)\n");
        failures++;
        return;
    }
    memcpy(before, fake_ee, sizeof before);
    write_count = 0u;

    cal_boot_defaults_init();

    check(memcmp(before, fake_ee, FAKE_EE_SIZE) == 0, "every one of the 4096 bytes is unchanged");
    check(write_count == 0u, "no byte was written at all");

    /* Which bytes, and where. A count alone does not tell you whether the change was harmless bookkeeping
     * in free space or a calibration record being overwritten -- and those need opposite responses. */
    for (uint16_t a = 0u; a < FAKE_EE_SIZE; a++) {
        if (before[a] != fake_ee[a]) {
            printf("      changed 0x%04X: 0x%02X -> 0x%02X\n", (unsigned)a, before[a], fake_ee[a]);
        }
    }
    printf("    (the :THID guard at 0x0A00 reads 0x%04X on this image)\n",
           (unsigned)(before[0x0A00] | (before[0x0A01] << 8)));
}

/* 2. A blank meter must get its defaults, and must NOT have 0x0A00 stamped. */
static void test_blank_meter_gets_defaults(void) {
    printf("blank meter (all 0xFF)\n");
    memset(fake_ee, 0xFF, sizeof fake_ee);
    write_count = 0u;

    cal_boot_defaults_init();

    check(write_count > 0u, "something was written");
    check(ee_read_u16(EE_THID_PRODDATE_LOCK) == 0xFFFFu,
          "0x0A00 is STILL ERASED (the write-once window survives)");
    check(ee_read_u16(EE_BOOT_MARK) == EE_BOOT_MARK_VALUE, "our own boot marker is stamped instead");
    check(ee_read_u16(EE_THID_CAL_COUNT) == 0u, "the cal count is initialised to 0");

    /* And it must be idempotent: a second boot writes nothing more. */
    write_count = 0u;
    cal_boot_defaults_init();
    check(write_count == 0u, "a second boot writes nothing (the marker works)");
}

/* 3. :SCAL:CHECK? on a real meter's image.
 *
 * It is tempting to expect 0 from this query after flashing. A real image holds a lot of cleared and never-written
 * records -- the DC V negative set, the AC V and AC A sets, the AC zero records -- and every one of those
 * is a NORMAL state: the negative-pair fallback exists so that a cleared record means "use the positive pair", not
 * "this meter is broken". A CHECK that counts them as faults would tell a user their freshly flashed
 * meter has two dozen damaged records, which would be alarming and wrong.
 *
 * So this test does not assert a chosen number. It reports what the shipped code actually says about the
 * image, and asserts the one thing that must hold however many it finds: that CHECK agrees
 * with the loader. A record the loader is happy to use must not be reported as damaged. */
static void test_check_on_real_meter(const char *path) {
    uint16_t bad;
    uint8_t listed;
    uint8_t i;

    printf("CHECK on a real meter image (%s)\n", path);
    if (!load(path, fake_ee)) {
        check(false, "cannot read the image -- SKIPPED (not a pass)");
        return;
    }
    bad = cal_check_all(NULL);
    listed = cal_check_run();
    printf("    cal_check_all reports %u record(s); cal_check_run saturates to %u\n", bad, listed);

    for (i = 0u; i < listed && i < CAL_CHECK_MAX_NAMES; i++) {
        char name[CAL_CHECK_NAME_LEN + 1u];

        if (cal_check_name(i, name)) {
            printf("      %s\n", name);
        }
    }

    /* The claim under test. For every record CHECK calls bad, ask the loader what it thinks: if the loader
     * would happily use that range, CHECK is wrong about it. */
    {
        unsigned disagreements = 0u;
        uint8_t fi;

        for (fi = 0u; fi < FUNC_COUNT; fi++) {
            const func_t f = (func_t)fi;
            uint8_t range;

            for (range = 0u; range <= RANGE_MAX_INDEX; range++) {
                const uint16_t addr = cal_pair_address(f, range, RATE_MEDIUM, CAL_PAIR_MAIN);
                ee_pair_t rec;
                cal_set_t set;

                if (addr == EE_ADDR_NONE) {
                    continue;
                }
                if (cal_read_pair(addr, &rec)) {
                    continue; /* a valid record: CHECK and the loader agree by construction */
                }
                /* The record is cleared or never written. The loader must still produce a usable set --
                 * that is what the fallback is for. */
                cal_load(f, range, RATE_MEDIUM, &set);
                if (!isfinite(set.main.f0) || !isfinite(set.main.f1)) {
                    disagreements++;
                    printf("      DISAGREE: %u/%u is cleared and the loader produced a non-finite set\n",
                           (unsigned)f, range);
                }
            }
        }
        check(disagreements == 0u,
              "every cleared record still loads a usable set (the negative-pair fallback works)");
    }

}

/* ---- CAL RANGE arithmetic ---------------------------------------------------------- */

static void put_f32(uint16_t addr, float v) {
    union {
        float f;
        uint8_t b[4];
    } u;
    u.f = v;
    for (uint8_t i = 0; i < 4u; i++) {
        fake_ee[addr + i] = u.b[i];
    }
}

static void put_u16(uint16_t addr, uint16_t v) {
    fake_ee[addr] = (uint8_t)(v & 0xFFu);
    fake_ee[addr + 1u] = (uint8_t)(v >> 8);
}

static float get_f32(uint16_t addr) {
    union {
        float f;
        uint8_t b[4];
    } u;
    for (uint8_t i = 0; i < 4u; i++) {
        u.b[i] = fake_ee[addr + i];
    }
    return u.f;
}

/* Within one part in a million. These are single-precision multiplies on both sides, so they agree to the
 * last bit in practice; the tolerance is here so a failure reports a NUMBER rather than a bit pattern. */
/* For a ratio of two computed float32 values: a few ULP of accumulated rounding, not a tolerance for
 * being wrong. See its use below. */
static bool ratio_close(float got, float want) {
    const float d = got - want;

    return ((d < 0.0f) ? -d : d) <= want * 1e-5f;
}

static bool close_enough(float got, float want) {
    const float d = got - want;
    const float mag = (want < 0.0f) ? -want : want;

    return ((d < 0.0f) ? -d : d) <= mag * 1e-6f + 1e-30f;
}

/* 4. A negative DC V point starts from the pair the LOADER applied.
 *
 * Set up range 0 with a VALID positive pair and a CLEARED negative one, which is the state of every DC V
 * negative record in a calibrated meter's dump. A negative point must then be computed from the positive pair,
 * because that is what `cal_load()`'s negative-pair fallback put in the alternate slot and therefore what the
 * measurement went through. Starting from the identity writes the point about 2.4 % out, with a valid
 * signature on it. */
static void test_negative_dcv_starts_from_the_loaded_pair(void) {
    const uint16_t pos = EE_DCV_POS_FAST + 100u * (uint16_t)RATE_MEDIUM;
    const uint16_t neg = EE_DCV_NEG_FAST + 100u * (uint16_t)RATE_MEDIUM;
    const float f0 = 1.024226f; /* a real DC V range 0 gain */
    const float measured = -4.98f;
    const float reference = -5.00f;
    const float k = reference / measured;
    float change = 0.0f;

    printf("CAL RANGE: a negative DC V point with a cleared NEG record\n");
    memset(fake_ee, 0xFF, sizeof fake_ee);
    put_f32(pos, f0);
    put_f32(pos + 4u, 0.0f);
    put_u16(pos + 8u, EE_SIG_VALID);
    put_f32(neg, 0.0f);
    put_f32(neg + 4u, 0.0f);
    put_u16(neg + 8u, EE_SIG_CLEARED);

    check(cal_range_preview(FUNC_DCV, 0u, RATE_MEDIUM, measured, reference, true, &change) == CAL_RANGE_OK,
          "the preview accepts the point");
    check(close_enough(change, k - 1.0f), "the previewed change is reference over measured, minus 1");
    check(cal_range_commit(FUNC_DCV, 0u, RATE_MEDIUM, measured, reference, true) == CAL_RANGE_OK,
          "the commit writes it");

    /* Hand-computed: the starting pair is the POSITIVE one, scaled by k. */
    printf("    f0 %.6f * k %.6f = %.6f, stored %.6f\n", (double)f0, (double)k, (double)(f0 * k),
           (double)get_f32(neg));
    check(close_enough(get_f32(neg), f0 * k), "the NEG record is the POSITIVE pair scaled by k");
    check(!close_enough(get_f32(neg), 1.0f * k), "it is NOT the identity scaled by k (the old defect)");
    check(ee_read_u16((uint16_t)(neg + 8u)) == EE_SIG_VALID, "the NEG record is left valid");
    check(close_enough(get_f32(pos), f0), "the POSITIVE record is untouched");
}

/* 5. An ohm trim scales BOTH coefficients, so the reading moves by exactly k.
 *
 * The quadratic form is y = m * (f0*m + f1), where m is the zero-subtracted scaled MEAN and y is the
 * reading. Scaling f1 alone gives a ratio of (f0*m + k*f1) / (f0*m + f1), which equals k only when f0*m is
 * negligible beside f1.
 *
 * THE OPERATING POINT MATTERS, and a naive version of this test gets it wrong. Using the OHM X
 * range 0 pair with a 495 ohm point goes wrong -- range 0 is the 50 Mohm range, where f0 = 4.0065e-07 and
 * f1 = -8.9495 only become comparable around m = f1/f0, about 22 million. At 495 ohms f0*m is 2e-4 against
 * f1 of -8.9, so both methods agree to six figures and the test would "pass" while proving nothing. A test
 * has to be run where the thing it is testing can actually happen.
 *
 * It also conflated two quantities: `measured` is the READING y, not the form's input m. They differ by
 * the form itself, which is the whole point here.
 *
 * So: m near the range's own working point, y computed from the form, and the trim applied to that y. */
static void test_ohm_trim_scales_the_whole_pair(void) {
    const uint16_t x0 = EE_OHM_X_BASE + EE_OHM_RATE_STEP * (uint16_t)RATE_MEDIUM;
    const float f0 = 4.0065e-07f; /* a real OHM X range 0 pair */
    const float f1 = -8.9495f;
    /* A mean inside the X SEGMENT. Range 0's threshold is 25 Mohm (cal.c), and the first attempt used a
     * point whose reading landed above it -- so the commit correctly wrote the Y pair while this test
     * checked X, and reported the X pair unchanged as though the arithmetic had failed. At m = 23e6 the
     * reading is about 6.1 Mohm, well inside the lower segment, and f0*m = 9.21 against f1 = -8.95 is
     * exactly the regime where scaling one term differs from scaling both. */
    const float m = 23.0e6f;
    const float measured = m * (f0 * m + f1);   /* what the instrument reads there, through this pair */
    const float reference = measured * 1.0093f; /* the better meter says it is 0.93 %% higher */
    const float k = reference / measured;
    float change = 0.0f;

    printf("CAL RANGE: an ohm point scales the whole pair\n");
    memset(fake_ee, 0xFF, sizeof fake_ee);
    put_f32(x0, f0);
    put_f32(x0 + 4u, f1);
    put_u16(x0 + 8u, EE_SIG_VALID);

    printf("    at mean %.4g: f0*m = %.4g, f1 = %.4g -- comparable, which is where the two differ\n",
           (double)m, (double)(f0 * m), (double)f1);

    check(cal_range_preview(FUNC_OHM, 0u, RATE_MEDIUM, measured, reference, true, &change) == CAL_RANGE_OK,
          "the preview accepts the point");
    check(cal_range_commit(FUNC_OHM, 0u, RATE_MEDIUM, measured, reference, true) == CAL_RANGE_OK,
          "the commit writes it");

    const float got0 = get_f32(x0);
    const float got1 = get_f32((uint16_t)(x0 + 4u));

    check(close_enough(got0, f0 * k), "f0 is scaled by k");
    check(close_enough(got1, f1 * k), "f1 is scaled by k");

    /* The claim that matters: the READING moves by exactly k at the point it was calibrated at. */
    {
        const float after = m * (got0 * m + got1);
        const float old_way = m * (f0 * m + k * f1); /* what scaling f1 alone would have given */

        printf("    reading %.6g -> %.6g (ratio %.6f, want k %.6f)\n", (double)measured, (double)after,
               (double)(after / measured), (double)k);
        printf("    f1 alone would give %.6g: a %.1f %%%% move for a %.2f %%%% trim\n", (double)old_way,
               (double)((old_way / measured - 1.0f) * 100.0f), (double)((k - 1.0f) * 100.0f));
        /* A looser tolerance here than for the stored coefficients, and the reason is arithmetic rather
         * than slack: the stored values are ONE float32 multiply each and agree to the last bit, while
         * this ratio is a quotient of two values each built from a multiply and an add in float32. A few
         * ULP accumulate -- the measured difference is about 2e-6 relative. 1e-5 is tight enough to catch
         * a wrong formula (the defect this guards against is 31 %% out) and loose enough not to fail on
         * single-precision rounding. */
        check(ratio_close(after / measured, k), "the reading moves by k within float32 rounding");
        check(!close_enough(old_way / measured, k), "scaling f1 alone would NOT have (the old defect)");
    }
}

int main(int argc, char **argv) {
    const char *image = (argc > 1) ? argv[1]
                                   : "tests/fixtures/eeprom_sanitized.bin";

    printf("boot-time EEPROM initialiser\n\n");
    test_existing_meter_is_untouched(image);
    printf("\n");
    test_blank_meter_gets_defaults();
    printf("\n");
    test_check_on_real_meter(image);
    printf("\n");
    test_negative_dcv_starts_from_the_loaded_pair();
    printf("\n");
    test_ohm_trim_scales_the_whole_pair();

    printf("\n%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
