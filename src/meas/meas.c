/* meas.c -- the measurement chain: front-end switching, conversion, averaging, correction, statistics.
 *
 * Replaces meas_cycle 0x0979C and everything it calls: the seven relay routines 0x06616-0x06A40 and their
 * selector 0x0BC52, the averaging loop 0x0A3F2, the secondary and AC+DC handlers 0x09CBA-0x0A270, the eight
 * autorange routines 0x0B3E4-0x0BB6E, the statistics stages 0x0AF34 / 0x0B07C / 0x0B0B8 / 0x0B2FE, the rate
 * change 0x0F036 and the ZERO CAL capture 0x01B76. Read meas.h first.
 *
 * ---- Shape ---------------------------------------------------------------------------------------------
 * The original ran one pass inside one blocking call. This is a state machine: meas_task() does at most one
 * conversion's worth of work and returns, so the host link, the keys and the watchdog keep running through a
 * reading that can last a second. There is no wait loop in the pass. The one place that waits is
 * meas_average_with() / meas_zero_capture(), which are deliberate operations with the host told to wait.
 *
 * A pass is a list of STAGES, entered in order by advance(); a stage that needs the converter sets `phase` and
 * returns, and the step that finishes it moves `stage` on and calls advance() again:
 *
 *   stage         phase while it runs        what it does                                         next
 *   ST_PROBE      PROBING                    one conversion, autorange decision   ST_LEG_OUT
 *                 SETTLING (after a change)  discards after the front end moved                   (re-probe)
 *   ST_LEG_OUT    SECONDARY_SETTLING         switch to the other quantity, discard K + 20         ST_LEG_READ
 *                                            (a Hz leg discards K only)                           or ST_LEG_GATE
 *   ST_LEG_READ   SECONDARY_READ             one conversion, corrected with the other set         ST_LEG_BACK
 *   ST_LEG_GATE   FREQ_GATING                the counter gate for a Hz secondary                  ST_LEG_BACK / ST_AMPL
 *   ST_LEG_BACK   SETTLING                   restore the primary front end, discard K + 20        ST_AMPL
 *   ST_AMPL       ACCUMULATING               FREQ / PERIOD with dual display: N amplitude samples ST_GATE
 *   ST_PRIMARY    ACCUMULATING               N conversions averaged, then the correction          ST_AC_OUT or ST_FINISH
 *   ST_GATE       FREQ_GATING                the counter gate for FREQ / PERIOD itself            ST_FINISH
 *   ST_AC_OUT     AC_SETTLING                AC+DC: AC front end, discard K + 25 (V) / 20 (I)     ST_AC_READ
 *   ST_AC_READ    AC_READ                    one conversion, combine sqrt(dc^2 + ac^2)            ST_ACDC_GATE
 *   ST_ACDC_GATE  FREQ_GATING                the Hz secondary of AC+DC                            ST_FINISH
 *   ST_FINISH     COMPLETE                   the later pipeline stages, then publish
 *
 * UNVERIFIED (timebase): the stall timer needs a 1 ms tick. meas_tick_1ms() below is that tick; it is not
 * declared in meas.h (frozen) and must be called from the main loop's 1 ms slot. Until it is, a converter that
 * stops answering is never detected. Check: main.c calls meas_tick_1ms() next to keys_tick_1ms().
 *
 * ---- Additions of 2026-10-22 ------------------------------------------------------
 * Line frequency. The converter decimation follows the LINE setting: 767 for 50 Hz, 639 for 60 Hz, so one
 * conversion lasts 20.0 ms or 16.7 ms. Every settle in this file is therefore a DURATION in milliseconds that is
 * turned into a conversion count for the line frequency in force (settle_conversions); the counts in meas.h
 * (MEAS_SETTLE_*) are the vendor's counts at 20 ms and are used only as the source of those durations. That is
 * the fix for ORIGINAL_BUGS #102.
 *
 * Self-calibration (ORIGINAL_BUGS #107). The converter is calibrated at start-up, after a LINE change and
 * whenever the PGA setting differs from the one it was last calibrated for. The calibration takes 140 or 280 ms
 * and is part of the state machine: while it runs meas_task() only watches for its completion and returns, so
 * the loop keeps servicing the host link, the keys and the watchdog. The phase reported does not change (meas.h
 * has no value for it); reading.valid is cleared for the start-up and LINE calibrations.
 * CONSEQUENCE, not solved here: the coefficients stored in an existing meter's EEPROM were captured
 * without self-calibration, so the meter must be recalibrated after the first flash of this firmware.
 *
 * Relay exercise. A guarded sequence that steps the front end through the patterns the original itself
 * writes for every function and range, to wipe the relay contacts. It is a second small state machine,
 * serviced first by meas_task(), and it holds the converter exclusively (claimed) for its whole run. The
 * entry points are meas_relay_exercise_start / _abort / _result / _seconds_left and meas_line_is_50hz /
 * meas_set_line, all declared in meas.h.
 */
#include "meas.h"

#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <float.h>
#include "../fmathf.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>

#include "../board.h"
#include "../eeprom_map.h"
#include "../hal/adcbus.h"
#include "../hal/buzzer.h"
#include "../hal/eeprom.h"
#include "../hal/freq.h"
#include "../tables.h"

/* ---- Constants --------------------------------------------------------------------------------------- */

/* Converter registers: 1 = input multiplexer, 2 = gain / ACR. */
#define ADC_REG_STATUS 0u
#define ADC_REG_MUX 1u
#define ADC_REG_ACR 2u
#define ADC_REG_DRATE_LO 8u
#define ADC_REG_DRATE_HI 9u

/* Power-up and rate-change values: RESET byte 0xFE at 0x9582, register 0 = 0x86 at 0x9588,
 * registers 8 and 9 = 0xFF / 0x32 at 0x15840-0x1584E.
 *
 * Register 8 is DEC0, the low eight bits of the decimation, register 9 holds M/DEC1 (Sinc3, bipolar, DEC[10:8]).
 * 50 Hz keeps the vendor's decimation 767 (0x2FF, DEC0 = 0xFF); 60 Hz is decimation 639 (0x27F, DEC0 = 0x7F).
 * DEC[10:8] = 010 in both, so register 9 is the same. */
#define ADC_CMD_RESET 0xFEu
#define ADC_CMD_SELFCAL 0xF0u  /* offset and gain, 14 tDATA */
#define ADC_CMD_SELFOCAL 0xF1u /* offset only, 7 tDATA */
#define ADC_STATUS_VALUE 0x86u
#define ADC_DRATE_LO_50HZ 0xFFu
#define ADC_DRATE_LO_60HZ 0x7Fu
#define ADC_DRATE_HI_VALUE 0x32u

/* The line frequency setting, EE_SET_LINE: 1 = 50 Hz, 0 = 60 Hz. Anything else, and in particular 0xFF on a
 * meter that has never run this firmware, is 60 Hz. */
#define LINE_EE_50HZ 1u
#define LINE_EE_60HZ 0u

/* tDATA in microseconds, ROUNDED DOWN so a settle computed from it is never short: 128 x 767 / 4.910 MHz is
 * 19.995 ms and 128 x 639 / 4.910 MHz is 16.658 ms. The crystal value is UNVERIFIED (ORIGINAL_BUGS #105). */
#define TDATA_US_50HZ 20000u
#define TDATA_US_60HZ 16600u

/* The vendor's conversion period, which turns the vendor's discard counts (MEAS_SETTLE_*) into durations. */
#define MEAS_ORIG_TDATA_MS 20u

/* Minimum front-end settle, from the vendor's counts at 20 ms: 5 conversions = 100 ms for DC V and AC V, 3 =
 * 60 ms for the rest. AC V's own routine discards only 3, so its floor lifts it to 100 ms. */
#define MEAS_SETTLE_FLOOR_V_MS 100u
#define MEAS_SETTLE_FLOOR_OTHER_MS 60u

/* PGA codes, the value written to the ACR register. The firmware uses 0, 1, 2, 3 and 5, that is gains 1, 2, 4,
 * 8 and 32. The datasheet optimises self-gain calibration for gains below 8, so gains 8 and 32 get the
 * offset-only calibration (ORIGINAL_BUGS #107). */
#define PGA_UNKNOWN 0xFFu
#define PGA_GAIN_8 3u
#define PGA_GAIN_32 5u

/* Self-calibration durations (14 tDATA and 7 tDATA at 50 Hz). They are used as
 * the minimum time before /DRDY is believed, and they are the 50 Hz figures: the 60 Hz calibration is shorter,
 * so these are safe for both. */
#define SELFCAL_FULL_MS 280u
#define SELFCAL_OFFSET_MS 140u

/* Conversions discarded after the front end is switched for the AC leg of AC+DC: 25 for V (0xA1BC) and 20
 * for I (0xA33E), on top of the relay routine's own discards. */
#define MEAS_AC_LEG_EXTRA_V 25u
#define MEAS_AC_LEG_EXTRA_I 20u

/* The ohm range-0 averaging counts (0x9988-0x99B4), continuity and diode (0x9A5A, 0x9AA2). */
#define MEAS_COUNT_CONT 3u
#define MEAS_COUNT_DIODE 6u
static const uint8_t ohm_range0_count[RATE_COUNT] = {10u, 20u, 40u};

/* A pass that stalls for this long with no conversion arriving is a converter fault. It is the
 * bound the bus layer already exports. A gate that never completes is not a fault: the original reads zero
 * there, so it ends when the counter's own deadline passes (hal/freq.c) as "no signal". */
#define MEAS_STALL_LIMIT_MS ADC_DRDY_TIMEOUT_MS
#define MEAS_FAULT_RETRY_MS 1000u

/* ORIGINAL_BUGS #70: autorange may move the index at most twice per range count in one pass. */
#define MEAS_AUTORANGE_CAP_FACTOR 2u

/* The ZERO CAL capture averages 6 conversions (0x1B8E). */
#define MEAS_ZERO_SAMPLES 6u

/* Plausibility limit for a captured zero: the dead-band threshold of the range, which is
 * 4.2e-4 of the lower autorange limit (0x0B10), with the flat values the dead-bands use on the last ranges
 * (DCV range 4: 3e-5, flash 0x0B08; ohm range 5: 0.02, flash 0x0B0C). */
#define MEAS_DEADBAND_K 4.2e-4f
#define MEAS_ZERO_LIMIT_DCV_FLAT 3.0e-5f
#define MEAS_ZERO_LIMIT_OHM_FLAT 0.02f

/* dB / dBm: constants at flash 0x0ADC, 0x0AD0, 0x0B00; valid windows at 0x0AD4/0x0AD8 and
 * 0x0AC8/0x0ACC. */
#define MEAS_DB_FACTOR 20.0f
#define MEAS_DBM_FACTOR 10.0f
#define MEAS_DBM_WATT_REF 0.001f
#define MEAS_DB_MIN -160.0f
#define MEAS_DB_MAX 120.0f
#define MEAS_DBM_MIN -140.0f
#define MEAS_DBM_MAX 50.0f

/* The value PERIOD shows when there is no signal; FETC? prints it as 9.9e37. */
#define MEAS_PERIOD_NO_SIGNAL 9.9e37f

/* HOLD default window when nothing has set one (the menu's "1 %" label). */
#define MEAS_HOLD_WINDOW_DEFAULT 0.01f

/* ---- Types ------------------------------------------------------------------------------------------- */

typedef enum {
    FE_DCV,             /* 0x6616 */
    FE_ACV_FULL,        /* 0x6788 */
    FE_ACV_RANGE_ONLY,  /* 0x66E2: the secondary legs and the AC leg of AC+DC V */
    FE_DCI,             /* 0x6834 */
    FE_ACI,             /* 0x68D0 */
    FE_OHM,             /* 0x6992 */
    FE_DIODE            /* 0x6A40 */
} fe_t;

typedef enum {
    SEC_NONE = 0,    /* no extra hardware work */
    SEC_CONVERSION,  /* switch to the other quantity, one conversion, switch back */
    SEC_HZ_SWITCHED, /* switch front end, counter gate, switch back */
    SEC_HZ_DIRECT,   /* counter gate on the primary front end */
    SEC_AMPLITUDE,   /* FREQ / PERIOD: average the amplitude input */
    SEC_ACDC_HZ      /* AC+DC: counter gate after the AC leg */
} sec_kind_t;

typedef struct {
    sec_kind_t kind;
    fe_t out_fe;     /* front end of the leg */
    fe_t back_fe;    /* front end restored afterwards */
    func_t leg_func; /* whose coefficients and scale correct the leg */
} sec_plan_t;

typedef enum {
    ST_PROBE = 0,
    ST_LEG_OUT,
    ST_LEG_READ,
    ST_LEG_GATE,
    ST_LEG_BACK,
    ST_AMPL,
    ST_PRIMARY,
    ST_GATE,
    ST_AC_OUT,
    ST_AC_READ,
    ST_ACDC_GATE,
    ST_FINISH
} stage_t;

/* Everything one pass carries. Rebuilt by begin_pass(). */
typedef struct {
    func_t func;
    uint8_t range;
    rate_t rate;
    stage_t stage;
    sec_plan_t plan;

    uint8_t settle_left;
    uint8_t target;
    uint8_t count;
    float sum;

    float probe_abs;
    uint8_t probe_changes;

    float calibrated; /* C: the value after the correction (the combined value for AC+DC) */
    float dc_value;
    float ac_value;
    float leg_value;
    bool leg_have;
    float hz;
    bool hz_have;
    float amplitude;
    bool amplitude_have;
    bool freq_overload;

    uint32_t adc_abs;
    bool adc_negative;
} pass_t;

/* ---- State -------------------------------------------------------------------------------------------- */

static meas_phase_t phase = MEAS_IDLE;
static pass_t pass;

static bool claimed;
static uint16_t stall_ms; /* milliseconds since the last conversion, gate start or fault */

/* The front-end state the converter and the relays are in, as the last pass left it. */
static bool hw_valid;
static func_t hw_func;
static uint8_t hw_range;
static fe_t hw_fe;
static rate_t last_rate = RATE_COUNT; /* RATE_COUNT: nothing applied yet, so the first pass settles */

/* The primary coefficient set, kept between passes and reloaded only when the key changes. */
static cal_set_t primary_set;
static cal_set_t leg_set;
static bool cal_valid;
static func_t cal_func;
static uint8_t cal_range;
static rate_t cal_rate;

static bool trigger_silenced;
static bool fault_pending;
static bool fault_active;

static bool minmax_first = true;
static bool limit_tone;

static bool hold_latched;
static uint8_t hold_run;
static float hold_first;

/* Line frequency and converter self-calibration. */
static bool line_50hz;           /* the LINE setting in force */
static uint8_t cur_pga = PGA_UNKNOWN; /* the PGA code last written to the ACR register */
static uint8_t cal_pga = PGA_UNKNOWN; /* the PGA code the converter was last self-calibrated for */
static bool selfcal_forced;      /* start-up or LINE change: calibrate whatever the PGA */
static bool selfcal_blank;       /* the forced calibration also blanks the reading */
static bool selfcal_active;      /* the command has been issued and its completion is awaited */
static uint8_t selfcal_pga;      /* the PGA the running calibration is for */
static uint16_t selfcal_min_ms;  /* the earliest time /DRDY is believed */

/* An explicit request from SELF TEST. Separate from selfcal_forced because `forced` only says "one
 * is owed" -- it is cleared the moment the command goes out, so it cannot tell a caller whether the thing
 * SUCCEEDED. This records the outcome. */
typedef enum {
    SELFCAL_REQ_NONE = 0,
    SELFCAL_REQ_RUNNING,
    SELFCAL_REQ_OK,
    SELFCAL_REQ_FAILED
} selfcal_req_t;
static selfcal_req_t selfcal_req;
static uint8_t restore_discards; /* conversions the first pass after a relay exercise must still discard */

/* The relay exercise: the result a caller reads, and the three millisecond counters meas_tick_1ms() runs
 * down. The counters are written from the tick and read in the main loop, so they are read atomically. */
/* relay_exercise_result_t now lives in meas.h, so the menu can use it. */

static bool exercising; /* the exercise owns the front end: no self-calibration decisions are taken */
static relay_exercise_result_t exercise_result = RLYEX_IDLE;
static volatile uint16_t exercise_total_ms; /* what is left of the run */
static volatile uint16_t exercise_hold_ms;  /* what is left of the current pattern's hold */
static volatile uint32_t exercise_elapsed_ms; /* milliseconds since the first pattern, while timing */
static volatile bool exercise_timing;         /* true from the first pattern until the run ends */
static uint16_t exercise_patterns_written;   /* patterns written to the port bits in this run */
static volatile uint16_t exercise_guard_ms; /* what is left before the input guard must run again */

/* ---- Small helpers ----------------------------------------------------------------------------------- */

static bool is_freq_func(func_t f) {
    return f == FUNC_FREQ || f == FUNC_PERIOD;
}

static bool is_acdc(func_t f) {
    return f == FUNC_ACDCV || f == FUNC_ACDCI;
}

static uint8_t range_of(func_t f) {
    const uint8_t index = g_state.range[f].index;
    const uint8_t max = tbl_range_max(f);

    return (index > max) ? max : index;
}

/* FREQ / PERIOD autorange and measure the AMPLITUDE input: the voltage chain, or the current chain when the
 * dual display selects current (0xBC52 switches on M012C).
 * UNVERIFIED: second_display[] is indexed by function here (state.h sizes it FUNC_COUNT). The RAM order in the
 * original is 0x0126 = DCV, 0x0127 = DCA, 0x0128 = ACV, 0x0129 = ACA, 0x012A = ACDCV, 0x012B = ACDCI,
 * 0x012C = FREQ and PERIOD together. Check: how handlers.c / menu.c store the selector, and that FREQ and
 * PERIOD share one value if the original's single byte is to be kept. */
static func_t amplitude_func(void) {
    return (g_state.dual_display && g_state.second_display[g_state.func] == 1u) ? FUNC_ACA : FUNC_ACV;
}

/* The function whose coefficients and scale correct the primary conversions of `f`. */
static func_t cal_func_of(func_t f) {
    return is_freq_func(f) ? amplitude_func() : f;
}

/* The autorange table row for `f`, or FUNC_COUNT when the function does not autorange (CONT, DIODE). */
static func_t table_row(func_t f) {
    switch (f) {
    case FUNC_DCV:
    case FUNC_ACDCV:
        return FUNC_DCV;
    case FUNC_ACV:
        return FUNC_ACV;
    case FUNC_DCA:
    case FUNC_ACDCI:
        return FUNC_DCA;
    case FUNC_ACA:
        return FUNC_ACA;
    case FUNC_OHM:
        return FUNC_OHM;
    case FUNC_FREQ:
    case FUNC_PERIOD:
        return amplitude_func();
    default:
        return FUNC_COUNT;
    }
}

static float table_up(func_t row, uint8_t r) {
    return pgm_read_float(&autorange_up[row][r]);
}

static float table_down(func_t row, uint8_t r) {
    return pgm_read_float(&autorange_down[row][r]);
}

/* ---- The overload limit --------------------------------------------------------------- */

/* Returns false when the function has no overload test: FREQ and PERIOD (0x9A0E and 0x9A34 have no compare).
 * The limit is the autorange UPPER threshold of the function's own range. `overload_limit` in tables.h has
 * no reference in the image and is not used.
 *   DC V and AC+DC V -> 0x061B row; AC V -> 0x0657; DC A and AC+DC I -> 0x067F; AC A -> 0x06A7;
 *   ohms -> 0x06CF; continuity -> the ohm row at the continuity range (0x9A7A); diode -> `diode_limit`
 *   (0x06FF, 0x9AC2-0x9AD8). tables.c has no continuity or diode row in autorange_up, so those two are
 *   handled here rather than by indexing a row that is not there. */
static bool overload_limit_of(func_t f, uint8_t range, float *limit) {
    func_t row;

    switch (f) {
    case FUNC_DIODE:
        *limit = pgm_read_float(&diode_limit);
        return true;
    case FUNC_FREQ:
    case FUNC_PERIOD:
        return false;
    case FUNC_CONT:
        row = FUNC_OHM;
        break;
    default:
        row = table_row(f);
        break;
    }
    if (row >= FUNC_COUNT) {
        return false;
    }
    if (range > tbl_range_max(row)) {
        range = tbl_range_max(row);
    }
    *limit = table_up(row, range);
    return true;
}

bool meas_overload(void) {
    const func_t f = g_state.func;
    float limit;

    if (!overload_limit_of(f, range_of(f), &limit)) {
        return false;
    }
    return fabsf(g_state.reading.calibrated) > limit;
}

/* The same test for the quantity a secondary leg measured, at the primary's range. */
static bool exceeds_upper(func_t quantity, uint8_t range, float v) {
    float limit;

    if (!overload_limit_of(quantity, range, &limit)) {
        return false;
    }
    return fabsf(v) > limit;
}

/* ---- Time-based settling (ORIGINAL_BUGS #102) --------------------------------------------------- */

static uint16_t tdata_us(void) {
    return line_50hz ? TDATA_US_50HZ : TDATA_US_60HZ;
}

/* The number of conversions that cover `ms` at the line frequency in force, rounded up. At 50 Hz this is the
 * vendor's count for a multiple of 20 ms; at 60 Hz it is more conversions for the same time. The result is
 * what a caller discards before it uses a conversion. */
static uint8_t settle_conversions(uint16_t ms) {
    const uint32_t us = (uint32_t)ms * 1000u;
    const uint32_t n = (us + tdata_us() - 1u) / tdata_us();

    return (n > UINT8_MAX) ? UINT8_MAX : (uint8_t)n;
}

/* The settle a front-end routine asks for, as a duration: its vendor count at 20 ms, but not less than the
 * floor of its function. Returns conversions at the current line frequency. */
static uint8_t settle_for_frontend(bool voltage_path, uint8_t vendor_count) {
    uint16_t ms = (uint16_t)(vendor_count * MEAS_ORIG_TDATA_MS);
    const uint16_t floor_ms = voltage_path ? MEAS_SETTLE_FLOOR_V_MS : MEAS_SETTLE_FLOOR_OTHER_MS;

    if (ms < floor_ms) {
        ms = floor_ms;
    }
    return settle_conversions(ms);
}

/* A vendor count of extra conversions (MEAS_SETTLE_SECONDARY and the AC leg's 25 / 20) as a duration. */
static uint8_t settle_extra(uint8_t vendor_count) {
    return settle_conversions((uint16_t)(vendor_count * MEAS_ORIG_TDATA_MS));
}

/* ---- Converter PGA and self-calibration ---------------------------------------------------------------- */

/* Every write of the ACR register goes through here, so the PGA the converter is calibrated for is known. */
static void set_pga(uint8_t code) {
    adc_write_register(ADC_REG_ACR, code);
    cur_pga = code;
}

/* True when a calibration is owed: the converter has never been calibrated for this PGA, or start-up / LINE
 * asked for one. Not while the relay exercise owns the front end; it restores the PGA it found. */
static bool selfcal_wanted(void) {
    return !exercising && (selfcal_forced || cur_pga != cal_pga);
}

static uint8_t selfcal_command(uint8_t pga) {
    return (pga == PGA_GAIN_8 || pga == PGA_GAIN_32) ? ADC_CMD_SELFOCAL : ADC_CMD_SELFCAL;
}

/* The forced request is consumed when the command goes out, not when it completes: a LINE change that lands
 * while a calibration is running sets it again, and the calibration that was running used the old rate. */
static void selfcal_issue(void) {
    selfcal_pga = cur_pga;
    selfcal_forced = false;
    adc_command(selfcal_command(cur_pga));
}

static void selfcal_finished(void) {
    cal_pga = selfcal_pga;
    selfcal_active = false;
    if (selfcal_req == SELFCAL_REQ_RUNNING) {
        selfcal_req = SELFCAL_REQ_OK;
    }
}

/* ---- Front end, in the original's order ---------------------------------------------------------------- */

#define BIT_SET(port, bit) ((port) |= (uint8_t)(1u << (bit)))
#define BIT_CLR(port, bit) ((port) &= (uint8_t) ~(1u << (bit)))
#define BIT_PUT(port, bit, on)                                                                             \
    do {                                                                                                   \
        if (on) {                                                                                          \
            BIT_SET(port, bit);                                                                            \
        } else {                                                                                           \
            BIT_CLR(port, bit);                                                                            \
        }                                                                                                  \
    } while (0)

/* The per-range block shared by 0x66E2 and 0x6788. */
static void acv_range_block(uint8_t r) {
    switch (r) {
    case 0:
        BIT_SET(PORTB, FE_B_ACV_ATTEN);
        BIT_SET(PORTG, FE_G_ACV_750);
        set_pga(2u);
        break;
    case 1:
        BIT_SET(PORTB, FE_B_ACV_ATTEN);
        BIT_CLR(PORTG, FE_G_ACV_750);
        set_pga(2u);
        break;
    case 2:
        BIT_CLR(PORTB, FE_B_ACV_ATTEN);
        BIT_CLR(PORTG, FE_G_ACV_750);
        set_pga(2u);
        break;
    case 3:
        BIT_SET(PORTC, FE_C_ACV_RANGE_5V);
        BIT_CLR(PORTB, FE_B_ACV_ATTEN);
        BIT_CLR(PORTG, FE_G_ACV_750);
        set_pga(2u);
        break;
    default:
        BIT_CLR(PORTC, FE_C_ACV_RANGE_5V);
        BIT_CLR(PORTB, FE_B_ACV_ATTEN);
        BIT_CLR(PORTG, FE_G_ACV_750);
        set_pga(3u);
        break;
    }
}

/* 0x6616. PA5 is not written by the original here; the port drives PA5 = 0, the
 * power-up state, so DCV no longer inherits whatever the previous function left. The write sits after PA6,
 * before the first of the original's remaining bits. */
static uint8_t fe_dcv(uint8_t r) {
    BIT_CLR(PORTA, FE_A_50M_RANGE);
    BIT_SET(PORTA, FE_A_VI_SELECT);
    BIT_CLR(PORTA, FE_A_MODE_I_OHM);
    BIT_CLR(PORTC, FE_C_SHUNT_5MA);
    BIT_CLR(PORTB, FE_B_AC_PATH);
    BIT_CLR(PORTA, FE_A_OHM_SRC_EN);
    BIT_CLR(PORTD, FE_D_OHM_PATH);
    BIT_CLR(PORTE, FE_E_50M_PATH);
    if (r < 4u) {
        adc_write_register(ADC_REG_MUX, 0x03u);
        BIT_SET(PORTA, FE_A_ATTEN);
        BIT_SET(PORTA, FE_A_HV_PATH);
    }
    switch (r) {
    case 0:
        BIT_SET(PORTF, FE_F_LADDER1);
        BIT_CLR(PORTA, FE_A_LADDER0);
        set_pga(1u);
        break;
    case 1:
        BIT_SET(PORTF, FE_F_LADDER1);
        BIT_CLR(PORTA, FE_A_LADDER0);
        set_pga(2u);
        break;
    case 2:
        BIT_CLR(PORTF, FE_F_LADDER1);
        BIT_SET(PORTA, FE_A_LADDER0);
        set_pga(2u);
        break;
    case 3:
        adc_write_register(ADC_REG_MUX, 0x07u);
        BIT_SET(PORTA, FE_A_LADDER0);
        BIT_SET(PORTF, FE_F_LADDER1);
        set_pga(2u);
        break;
    default:
        adc_write_register(ADC_REG_MUX, 0x23u);
        BIT_CLR(PORTA, FE_A_ATTEN);
        BIT_CLR(PORTF, FE_F_LADDER1);
        BIT_CLR(PORTA, FE_A_LADDER0);
        set_pga(2u);
        break;
    }
    return MEAS_SETTLE_DCV;
}

/* 0x66E2: used only by the secondary legs and the AC leg of AC+DC V. */
static uint8_t fe_acv_range_only(uint8_t r) {
    adc_write_register(ADC_REG_MUX, 0x68u);
    BIT_CLR(PORTA, FE_A_MODE_I_OHM);
    BIT_SET(PORTB, FE_B_AC_PATH);
    BIT_PUT(PORTC, FE_C_ACV_RANGE_HI, r < 3u);
    BIT_CLR(PORTG, FE_G_ACI_GAIN);
    acv_range_block(r);
    return MEAS_SETTLE_RANGE_ONLY;
}

/* 0x6788: the AC V function. Three discards, not five. */
static uint8_t fe_acv_full(uint8_t r) {
    adc_write_register(ADC_REG_MUX, 0x68u);
    BIT_CLR(PORTA, FE_A_50M_RANGE);
    BIT_SET(PORTF, FE_F_LADDER1);
    BIT_CLR(PORTA, FE_A_LADDER0);
    BIT_SET(PORTA, FE_A_ATTEN);
    BIT_CLR(PORTA, FE_A_MODE_I_OHM);
    BIT_SET(PORTA, FE_A_VI_SELECT);
    BIT_SET(PORTB, FE_B_AC_PATH);
    BIT_PUT(PORTC, FE_C_ACV_RANGE_HI, r < 3u);
    BIT_CLR(PORTG, FE_G_ACI_GAIN);
    acv_range_block(r);
    return MEAS_SETTLE_DEFAULT;
}

/* 0x6834: DC current. */
static uint8_t fe_dci(uint8_t r) {
    adc_write_register(ADC_REG_MUX, 0x45u);
    BIT_CLR(PORTA, FE_A_50M_RANGE);
    BIT_SET(PORTF, FE_F_LADDER1);
    BIT_CLR(PORTA, FE_A_LADDER0);
    BIT_SET(PORTA, FE_A_ATTEN);
    BIT_SET(PORTA, FE_A_MODE_I_OHM);
    BIT_SET(PORTA, FE_A_VI_SELECT);
    BIT_CLR(PORTB, FE_B_AC_PATH);
    BIT_SET(PORTF, FE_F_I_DC);
    switch (r) {
    case 0:
    case 1:
        BIT_CLR(PORTC, FE_C_SHUNT_5MA);
        BIT_CLR(PORTE, FE_E_SHUNT_5MA);
        BIT_CLR(PORTE, FE_E_SHUNT_MA);
        BIT_SET(PORTF, FE_F_SHUNT_20A);
        set_pga((r == 0u) ? 3u : 5u);
        break;
    case 2:
    case 3:
        BIT_CLR(PORTC, FE_C_SHUNT_5MA);
        BIT_CLR(PORTE, FE_E_SHUNT_5MA);
        BIT_SET(PORTE, FE_E_SHUNT_MA);
        BIT_CLR(PORTF, FE_F_SHUNT_20A);
        set_pga((r == 2u) ? 2u : 5u);
        break;
    default:
        BIT_SET(PORTC, FE_C_SHUNT_5MA);
        BIT_SET(PORTE, FE_E_SHUNT_5MA);
        BIT_CLR(PORTE, FE_E_SHUNT_MA);
        BIT_CLR(PORTF, FE_F_SHUNT_20A);
        set_pga(2u);
        break;
    }
    return MEAS_SETTLE_DEFAULT;
}

/* 0x68D0: AC current, and the amplitude input of FREQ / PERIOD in the current mode. */
static uint8_t fe_aci(uint8_t r) {
    adc_write_register(ADC_REG_MUX, 0x68u);
    BIT_CLR(PORTA, FE_A_50M_RANGE);
    BIT_SET(PORTF, FE_F_LADDER1);
    BIT_CLR(PORTA, FE_A_LADDER0);
    BIT_SET(PORTA, FE_A_ATTEN);
    BIT_SET(PORTA, FE_A_MODE_I_OHM);
    BIT_SET(PORTA, FE_A_VI_SELECT);
    BIT_CLR(PORTC, FE_C_ACV_RANGE_HI);
    BIT_CLR(PORTC, FE_C_ACV_RANGE_5V);
    BIT_SET(PORTB, FE_B_AC_PATH);
    BIT_CLR(PORTF, FE_F_I_DC);
    BIT_CLR(PORTB, FE_B_ACV_ATTEN);
    BIT_CLR(PORTG, FE_G_ACV_750);
    switch (r) {
    case 0:
    case 1:
        BIT_CLR(PORTC, FE_C_SHUNT_5MA);
        BIT_CLR(PORTE, FE_E_SHUNT_5MA);
        BIT_CLR(PORTE, FE_E_SHUNT_MA);
        BIT_SET(PORTF, FE_F_SHUNT_20A);
        BIT_PUT(PORTG, FE_G_ACI_GAIN, r == 1u);
        set_pga((r == 0u) ? 3u : 2u);
        break;
    case 2:
    case 3:
        BIT_CLR(PORTC, FE_C_SHUNT_5MA);
        BIT_CLR(PORTE, FE_E_SHUNT_5MA);
        BIT_SET(PORTE, FE_E_SHUNT_MA);
        BIT_CLR(PORTF, FE_F_SHUNT_20A);
        BIT_PUT(PORTG, FE_G_ACI_GAIN, r == 3u);
        set_pga((r == 2u) ? 3u : 2u);
        break;
    default:
        BIT_SET(PORTC, FE_C_SHUNT_5MA);
        BIT_SET(PORTE, FE_E_SHUNT_5MA);
        BIT_CLR(PORTE, FE_E_SHUNT_MA);
        BIT_CLR(PORTF, FE_F_SHUNT_20A);
        BIT_CLR(PORTG, FE_G_ACI_GAIN);
        set_pga(3u);
        break;
    }
    return MEAS_SETTLE_DEFAULT;
}

/* 0x6992: ohms and continuity. PB6 is set and never cleared by the original (ORIGINAL_BUGS #103). */
static uint8_t fe_ohm(uint8_t r) {
    adc_write_register(ADC_REG_MUX, 0x23u);
    BIT_CLR(PORTA, FE_A_ATTEN);
    BIT_SET(PORTA, FE_A_MODE_I_OHM);
    BIT_CLR(PORTA, FE_A_VI_SELECT);
    BIT_CLR(PORTC, FE_C_SHUNT_5MA);
    BIT_SET(PORTB, FE_B_AC_PATH);
    BIT_SET(PORTA, FE_A_OHM_SRC_EN);
    BIT_CLR(PORTA, FE_A_HV_PATH);
    BIT_CLR(PORTG, PG2);
    BIT_SET(PORTB, FE_B_OHM_SRC);
    if (r == 0u) {
        BIT_SET(PORTE, FE_E_50M_PATH);
        BIT_CLR(PORTD, FE_D_OHM_PATH);
        BIT_SET(PORTA, FE_A_50M_RANGE);
    } else {
        BIT_CLR(PORTE, FE_E_50M_PATH);
        BIT_SET(PORTD, FE_D_OHM_PATH);
        BIT_CLR(PORTA, FE_A_50M_RANGE);
    }
    switch (r) {
    case 0:
        set_pga(0u);
        break;
    case 1:
        BIT_SET(PORTF, FE_F_LADDER1);
        BIT_SET(PORTA, FE_A_LADDER0);
        set_pga(0u);
        break;
    case 2:
        BIT_CLR(PORTF, FE_F_LADDER1);
        BIT_SET(PORTA, FE_A_LADDER0);
        set_pga(0u);
        break;
    case 3:
        BIT_SET(PORTF, FE_F_LADDER1);
        BIT_CLR(PORTA, FE_A_LADDER0);
        set_pga(0u);
        break;
    case 4:
        BIT_CLR(PORTF, FE_F_LADDER1);
        BIT_CLR(PORTA, FE_A_LADDER0);
        set_pga(0u);
        break;
    default:
        BIT_CLR(PORTF, FE_F_LADDER1);
        BIT_CLR(PORTA, FE_A_LADDER0);
        BIT_CLR(PORTB, FE_B_AC_PATH);
        set_pga(3u);
        break;
    }
    return MEAS_SETTLE_DEFAULT;
}

/* 0x6A40: the diode function, a single range. */
static uint8_t fe_diode(void) {
    adc_write_register(ADC_REG_MUX, 0x23u);
    BIT_CLR(PORTA, FE_A_ATTEN);
    BIT_SET(PORTA, FE_A_MODE_I_OHM);
    BIT_CLR(PORTA, FE_A_VI_SELECT);
    BIT_CLR(PORTC, FE_C_SHUNT_5MA);
    BIT_SET(PORTA, FE_A_OHM_SRC_EN);
    BIT_SET(PORTB, FE_B_AC_PATH);
    BIT_CLR(PORTA, FE_A_HV_PATH);
    BIT_CLR(PORTG, PG2);
    BIT_SET(PORTB, FE_B_OHM_SRC);
    BIT_CLR(PORTE, FE_E_50M_PATH);
    BIT_SET(PORTD, FE_D_OHM_PATH);
    BIT_CLR(PORTA, FE_A_50M_RANGE);
    BIT_CLR(PORTF, FE_F_LADDER1);
    BIT_CLR(PORTA, FE_A_LADDER0);
    set_pga(0u);
    return MEAS_SETTLE_DEFAULT;
}

/* Writes the front end and returns how many conversions to discard afterwards AT THE LINE FREQUENCY IN FORCE:
 * the routines above return the vendor's counts, which are durations at 20 ms, and the floor of 100 ms for
 * the two voltage paths and 60 ms for the rest is applied here (ORIGINAL_BUGS #102). */
static uint8_t fe_apply(fe_t fe, uint8_t range) {
    uint8_t vendor_count;
    bool voltage_path = false;

    switch (fe) {
    case FE_DCV:
        vendor_count = fe_dcv(range);
        voltage_path = true;
        break;
    case FE_ACV_FULL:
        vendor_count = fe_acv_full(range);
        voltage_path = true;
        break;
    case FE_ACV_RANGE_ONLY:
        vendor_count = fe_acv_range_only(range);
        voltage_path = true;
        break;
    case FE_DCI:
        vendor_count = fe_dci(range);
        break;
    case FE_ACI:
        vendor_count = fe_aci(range);
        break;
    case FE_OHM:
        vendor_count = fe_ohm(range);
        break;
    case FE_DIODE:
    default:
        vendor_count = fe_diode();
        break;
    }
    return settle_for_frontend(voltage_path, vendor_count);
}

/* The selector 0xBC52: which relay routine serves a function. */
static fe_t primary_fe(func_t f) {
    switch (f) {
    case FUNC_DCV:
    case FUNC_ACDCV:
        return FE_DCV;
    case FUNC_ACV:
        return FE_ACV_FULL;
    case FUNC_DCA:
    case FUNC_ACDCI:
        return FE_DCI;
    case FUNC_ACA:
        return FE_ACI;
    case FUNC_OHM:
    case FUNC_CONT:
        return FE_OHM;
    case FUNC_DIODE:
        return FE_DIODE;
    case FUNC_FREQ:
    case FUNC_PERIOD:
    default:
        return (amplitude_func() == FUNC_ACA) ? FE_ACI : FE_ACV_FULL;
    }
}

/* The primary front end for a pass, with the bookkeeping that lets the next pass skip it. */
static uint8_t apply_primary(func_t f, uint8_t range) {
    const fe_t fe = primary_fe(f);

    hw_valid = true;
    hw_func = f;
    hw_range = range;
    hw_fe = fe;
    return fe_apply(fe, range);
}

/* ---- The converter ----------------------------------------------------------------------------------- */

/* Called from the main loop's 1 ms slot; see the file header. Saturates so an idle instrument cannot wrap. */
void meas_tick_1ms(void) {
    if (stall_ms < UINT16_MAX) {
        stall_ms++;
    }
    if (exercise_total_ms != 0u) {
        exercise_total_ms--;
    }
    if (exercise_hold_ms != 0u) {
        exercise_hold_ms--;
    }
    if (exercise_timing) {
        exercise_elapsed_ms++;
    }
    if (exercise_guard_ms != 0u) {
        exercise_guard_ms--;
    }
}

/* One conversion if one is ready RIGHT NOW. adc_read_result() with a zero timeout is a single look at /DRDY
 * (adcbus.c polls zero times and then tests the line once), so this never waits. */
static bool take_conversion(uint32_t *magnitude, bool *negative) {
    int32_t code;

    if (!adc_read_result(&code, 0u)) {
        return false;
    }
    stall_ms = 0u;
    fault_active = false;
    if (code < 0) {
        *negative = true;
        *magnitude = (uint32_t)(-code);
    } else {
        *negative = false;
        *magnitude = (uint32_t)code;
    }
    return true;
}

/* The blocking read the two deliberate operations use. It is bounded by ADC_DRDY_TIMEOUT_MS, taken in slices
 * so the watchdog is fed between them. A normal conversion arrives within 20 ms. */
#define MEAS_BLOCK_SLICE_MS 50u

static bool read_blocking(uint32_t *magnitude, bool *negative) {
    uint16_t waited;
    int32_t code;

    for (waited = 0u; waited < ADC_DRDY_TIMEOUT_MS; waited = (uint16_t)(waited + MEAS_BLOCK_SLICE_MS)) {
        wdt_reset();
        if (adc_read_result(&code, MEAS_BLOCK_SLICE_MS)) {
            if (code < 0) {
                *negative = true;
                *magnitude = (uint32_t)(-code);
            } else {
                *negative = false;
                *magnitude = (uint32_t)code;
            }
            return true;
        }
    }
    return false;
}

static void enter_fault(void) {
    phase = MEAS_FAULT;
    g_state.reading.valid = false;
    hw_valid = false;
    cal_valid = false;
    selfcal_active = false;
    selfcal_forced = true; /* the converter state is unknown after a fault: calibrate again on recovery */
    if (selfcal_req == SELFCAL_REQ_RUNNING) {
        selfcal_req = SELFCAL_REQ_FAILED;
    }
    stall_ms = 0u;
    if (!fault_active) {
        fault_pending = true;
    }
    fault_active = true;
}

bool meas_fault_take(void) {
    const bool was = fault_pending;

    fault_pending = false;
    return was;
}

/* ---- Self-calibration, the two ways it is run ---------------------------------------------------------- */

/* The state-machine way, called by meas_task() before it looks at the phase. Returns true while it owns the
 * converter, which makes meas_task() return: the pass that asked for the calibration simply waits where it
 * is, with its phase unchanged, and the loop keeps running.
 *
 * Issue: the command goes out on the first call after the front end was written. Wait: /DRDY is not looked at
 * until the datasheet time has passed, so a conversion that was already waiting cannot end the calibration
 * early. After that the first conversion that appears is the calibration's completion; it is discarded.
 * A converter that never answers is a fault, bounded by the same limit as a stalled pass. */
static bool selfcal_step(void) {
    uint32_t magnitude;
    bool negative;

    if (!selfcal_active) {
        if (!selfcal_wanted()) {
            return false;
        }
        selfcal_min_ms = (selfcal_command(cur_pga) == ADC_CMD_SELFCAL) ? SELFCAL_FULL_MS : SELFCAL_OFFSET_MS;
        selfcal_issue();
        selfcal_active = true;
        stall_ms = 0u;
        if (selfcal_blank) {
            g_state.reading.valid = false;
            selfcal_blank = false;
        }
        return true;
    }
    if (stall_ms < selfcal_min_ms) {
        return true;
    }
    if (take_conversion(&magnitude, &negative)) {
        selfcal_finished();
    } else if (stall_ms >= (uint16_t)(selfcal_min_ms + MEAS_STALL_LIMIT_MS)) {
        enter_fault();
    }
    return true;
}

/* The waiting way, for the deliberate operations that already wait (the calibration sweeps and ZERO CAL,
 * through meas_frontend_apply). The watchdog is fed inside read_blocking(). Returns false on a timeout.
 * UNVERIFIED: that /DRDY is high from the command until the calibration completes. If it can read low
 * straight after the command this returns early, and the first conversions of the caller are then taken
 * during the calibration. Check: time this call on the bench; it should take about 140 or 280 ms. */
static bool selfcal_blocking(void) {
    uint32_t magnitude;
    bool negative;

    selfcal_issue();
    selfcal_blank = false;
    if (!read_blocking(&magnitude, &negative)) {
        enter_fault();
        return false;
    }
    selfcal_finished();
    return true;
}

uint8_t meas_frontend_apply(func_t f, uint8_t range) {
    uint8_t discards;

    if (f >= FUNC_COUNT) {
        f = FUNC_DCV;
    }
    if (range > tbl_range_max(f)) {
        range = tbl_range_max(f);
    }
    /* Whoever calls this has moved the front end outside a pass, so the pass bookkeeping no longer
     * describes it: the next pass applies its own state again. */
    hw_valid = false;
    discards = fe_apply(primary_fe(f), range);
    if (selfcal_wanted() && !selfcal_active) {
        (void)selfcal_blocking();
    }
    return discards;
}

/* ---- Coefficients ------------------------------------------------------------------------------------ */

static void load_primary(func_t f, uint8_t range, rate_t rate) {
    cal_func = cal_func_of(f);
    cal_range = range;
    cal_rate = rate;
    cal_load(cal_func, range, rate, &primary_set);
    cal_valid = true;
}

/* The two stages after the per-sample zero, on the MEAN, in the one order the instrument uses. Returns the
 * corrected value and, through `forced_zero`, whether the add-ons forced a non-zero value to exactly zero
 * (the dead-bands and the AC V low-range rule then clear the sign flag, at 0xA974 / 0xA9F2). */
static float correct_mean(const cal_set_t *set, func_t calfunc, float mean, uint8_t range, bool *forced_zero) {
    const float y = cal_apply(set, mean, range, 0u);
    const float out = cal_apply_addons(set, calfunc, mean, y, range, 0u);

    if (forced_zero != NULL) {
        *forced_zero = (out == 0.0f && y != 0.0f &&
                        (calfunc == FUNC_DCV || calfunc == FUNC_ACDCV || calfunc == FUNC_OHM ||
                         calfunc == FUNC_ACV));
    }
    return out;
}

/* ---- Secondary plan ---------------------------------------------------------------------------------- */

static sec_plan_t plan_secondary(func_t f) {
    sec_plan_t p = {SEC_NONE, FE_DCV, FE_DCV, FUNC_DCV};
    uint8_t sel;

    if (!g_state.dual_display) {
        return p;
    }
    sel = g_state.second_display[f];
    switch (f) {
    case FUNC_DCV:
        if (sel <= 1u) {
            p.kind = (sel == 0u) ? SEC_CONVERSION : SEC_HZ_SWITCHED;
            p.out_fe = FE_ACV_RANGE_ONLY;
            p.back_fe = FE_DCV;
            p.leg_func = FUNC_ACV;
        }
        break;
    case FUNC_ACV:
        if (sel == 0u) {
            p.kind = SEC_CONVERSION;
            p.out_fe = FE_DCV;
            p.back_fe = FE_ACV_RANGE_ONLY;
            p.leg_func = FUNC_DCV;
        } else if (sel == 1u) {
            p.kind = SEC_HZ_DIRECT;
        }
        break;
    case FUNC_DCA:
        if (sel <= 1u) {
            p.kind = (sel == 0u) ? SEC_CONVERSION : SEC_HZ_SWITCHED;
            p.out_fe = FE_ACI;
            p.back_fe = FE_DCI;
            p.leg_func = FUNC_ACA;
        }
        break;
    case FUNC_ACA:
        if (sel == 0u) {
            p.kind = SEC_CONVERSION;
            p.out_fe = FE_DCI;
            p.back_fe = FE_ACI;
            p.leg_func = FUNC_DCA;
        } else if (sel == 1u) {
            p.kind = SEC_HZ_DIRECT;
        }
        break;
    case FUNC_FREQ:
    case FUNC_PERIOD:
        if (sel <= 1u) {
            p.kind = SEC_AMPLITUDE;
            p.leg_func = amplitude_func();
        }
        break;
    case FUNC_ACDCV:
    case FUNC_ACDCI:
        if (sel == 0u) {
            p.kind = SEC_ACDC_HZ;
        }
        break;
    default:
        break;
    }
    return p;
}

/* ---- Counts ------------------------------------------------------------------------------------------ */

/* The number of conversions averaged for the primary reading. */
static uint8_t primary_count(func_t f, uint8_t range, rate_t rate) {
    switch (f) {
    case FUNC_OHM:
        return (range == 0u) ? ohm_range0_count[rate] : rate_sample_count[rate];
    case FUNC_CONT:
        return MEAS_COUNT_CONT;
    case FUNC_DIODE:
        return MEAS_COUNT_DIODE;
    default:
        return rate_sample_count[rate];
    }
}

/* ---- Autorange ------------------------------------------------------------------------- */

/* Decides from the probe the pass is carrying. One probe, no agreement count: step to a smaller range while
 * |x| < lower[R], and only if that did not move, to a larger range while |x| > upper[R] (0xB430-0xB4BC).
 * Both loops may move several steps without a new conversion. The compares are strict. Returns true when the
 * index moved; the new index is already in pass.range and g_state. */
bool meas_autorange_step(void) {
    const func_t f = pass.func;
    const func_t row = table_row(f);
    const uint8_t max = tbl_range_max(f);
    const uint8_t start = pass.range;
    uint8_t r = start;

    if (row >= FUNC_COUNT) {
        return false;
    }
    while (r < max && pass.probe_abs < table_down(row, r)) {
        r++;
    }
    if (r == start) {
        while (r > 0u && pass.probe_abs > table_up(row, r)) {
            r--;
        }
    }
    if (r == start) {
        return false;
    }
    pass.range = r;
    g_state.range[f].index = r;
    return true;
}

static bool autorange_applies(func_t f) {
    return table_row(f) < FUNC_COUNT && !g_state.range[f].manual;
}

/* ---- Statistics stages ------------------------------------------------------------------------------- */

static bool db_applicable(func_t f) {
    return f == FUNC_DCV || f == FUNC_ACV || f == FUNC_ACDCV;
}

static float db_value(float c, bool *oor) {
    const float ref = g_state.math.db_ref;
    float v;

    if (ref == 0.0f) {
        v = INFINITY; /* the original does not guard this; the menu cannot set 0, :UNIT can */
    } else {
        v = MEAS_DB_FACTOR * log10f(fabsf(c / ref));
    }
    *oor = !(v >= MEAS_DB_MIN && v <= MEAS_DB_MAX);
    return v;
}

static float dbm_value(float c, bool *oor) {
    const int16_t ref = g_state.math.dbm_ref;
    float v;

    if (ref <= 0) {
        v = INFINITY;
    } else {
        v = MEAS_DBM_FACTOR * log10f(((c * c) / (float)ref) / MEAS_DBM_WATT_REF);
    }
    *oor = !(v >= MEAS_DBM_MIN && v <= MEAS_DBM_MAX);
    return v;
}

/* P8, the primary branch of 0xB0B8 (0xB236 for dB, 0xB28A for dBm). The number is kept even when it is out
 * of range; only the flag records it. The caller skips an overloaded reading. */
float meas_apply_db(float corrected, func_t f, bool *out_of_range) {
    bool oor = false;
    float v = corrected;

    if (db_applicable(f)) {
        if (g_state.math.db_on) {
            v = db_value(corrected, &oor);
        } else if (g_state.math.dbm_on) {
            v = dbm_value(corrected, &oor);
        }
    }
    if (out_of_range != NULL) {
        *out_of_range = oor;
    }
    return v;
}

/* P10 (0x9BC2-0x9C00). Skipped for continuity and diode (0x9BD8-0x9BE0); applied to an overloaded reading. */
float meas_apply_rel(float value, func_t f) {
    if (f >= FUNC_COUNT || f == FUNC_CONT || f == FUNC_DIODE || !g_state.rel[f].active) {
        return value;
    }
    return value - g_state.rel[f].reference;
}

/* P11 (0xB07C). The original divides by the reference unguarded and gets +FLT_MAX for a zero one
 * (ORIGINAL_BUGS #78); the same number comes out here without the divide. The menu refuses a zero reference. */
float meas_apply_percent(float value) {
    const float ref = g_state.math.percent_ref;

    if (!g_state.math.percent_on) {
        return value;
    }
    if (ref == 0.0f) {
        return FLT_MAX;
    }
    return 100.0f * (value - ref) / ref;
}

/* P12 (0xB2FE). The lamps and the HI / IN / LO code are derived by display.c from the same value; what is
 * left for this stage is the tone. A failed limit sounds the beeper when it is enabled, an in-limit or
 * overloaded reading silences it (0xB308-0xB33C). Only a tone this stage started is ever stopped, so it
 * cannot cut a key click or the continuity tone short. Strict compares: a value equal to a limit is IN. */
void meas_update_limits(float value, func_t f) {
    bool want = false;

    if (g_state.math.limit_on && f != FUNC_CONT && !g_state.reading.overload) {
        want = g_state.settings.beeper && (value > g_state.math.limit_hi || value < g_state.math.limit_lo);
    }
    if (want != limit_tone) {
        buzzer_set(want);
        limit_tone = want;
    }
}

/* Starts the extremes afresh. Both show the current reading until the next completed pass replaces them: the
 * original only raises a flag here and the tracker (0xAF34) seeds both extremes from the next pass. */
void meas_minmax_reset(void) {
    g_state.minmax.max = g_state.reading.primary;
    g_state.minmax.min = g_state.reading.primary;
    minmax_first = true;
}

/* P13 (0xAF34). Tracking is on while the MAX/MIN view is not LIVE, or `minmax.active` is set; the original
 * tests only RAM 0x0130. The first reading after switching on sets both extremes; an overloaded reading
 * updates nothing (0xAF5A, 0xAFD6, 0xB036). */
void meas_update_minmax(float value) {
    if (!g_state.minmax.active && g_state.minmax_view == MINMAX_VIEW_LIVE) {
        return;
    }
    if (g_state.reading.overload) {
        return;
    }
    if (minmax_first) {
        g_state.minmax.max = value;
        g_state.minmax.min = value;
        minmax_first = false;
        return;
    }
    if (value > g_state.minmax.max) {
        g_state.minmax.max = value;
    }
    if (value < g_state.minmax.min) {
        g_state.minmax.min = value;
    }
}

/* ---- HOLD ------------------------------------------------------------------------------- */

void meas_hold_arm(void) {
    g_state.hold_active = true;
    hold_latched = false;
    hold_run = 0u;
}

void meas_hold_release(void) {
    g_state.hold_active = false;
    hold_latched = false;
    hold_run = 0u;
}

/* Called with each published reading while HOLD is armed.
 * UNVERIFIED: the value compared is `reading.primary`, what the operator sees. This
 * is a design decision; the alternative is `reading.raw`.
 * UNVERIFIED (item 2): when the first reading of a run is exactly zero the window is zero wide and the run
 * needs exact equality. The proposal's max(|first|, tiny) is not applied because `tiny` is not decided.
 * UNVERIFIED (item 3): the latched state keeps `reading` frozen because meas_task() stops measuring; it is
 * cleared by meas_hold_release(), which the key handler and :HOLD:STATe OFF must call, and a setting change
 * un-latches it so a function or range change cannot leave the instrument frozen on the old reading.
 * UNVERIFIED (item 4): the beep is buzzer_error(), a 20 ms pulse that honours settings.beeper. */
static void hold_observe(float primary, bool overload) {
    const uint8_t need = (g_state.hold_count < 1u) ? 1u : g_state.hold_count;

    if (!g_state.hold_active || hold_latched) {
        return;
    }
    if (overload) {
        hold_run = 0u;
        return;
    }
    if (hold_run == 0u) {
        hold_first = primary;
        hold_run = 1u;
    } else if (fabsf(primary - hold_first) <= g_state.hold_window * fabsf(hold_first)) {
        hold_run++;
    } else {
        hold_first = primary;
        hold_run = 1u;
    }
    if (hold_run >= need) {
        hold_latched = true;
        buzzer_error();
    }
}

/* ---- Publishing a pass (P6-P14) ---------------------------------------------------------------------- */

/* The secondary number. Returns true when the secondary dB / dBm range check failed, because the original
 * then returns from 0xB0B8 without running the primary branch (0xB162, 0xB224): with the unit in dB and the
 * secondary out of range the primary stays in volts for that pass. Reproduced as found. */
static bool publish_secondary(func_t f, float c, bool overload) {
    reading_t *rd = &g_state.reading;
    const uint8_t sel = g_state.second_display[f];
    const uint8_t range = pass.range;
    bool oor = false;
    bool derived = false;
    bool dbm = false;

    rd->secondary_valid = false;
    rd->secondary_overload = false;
    if (!g_state.dual_display) {
        return false;
    }

    switch (f) {
    case FUNC_DCV:
    case FUNC_ACV:
        if (sel == 0u && pass.leg_have) {
            rd->secondary = pass.leg_value;
            rd->secondary_overload = exceeds_upper(pass.plan.leg_func, range, pass.leg_value);
            rd->secondary_valid = true;
        } else if (sel == 1u && pass.hz_have) {
            rd->secondary = pass.hz;
            rd->secondary_valid = true;
        } else if (sel == 2u || sel == 3u) {
            derived = true;
            dbm = (sel == 3u);
        }
        break;
    case FUNC_DCA:
    case FUNC_ACA:
        if (sel == 0u && pass.leg_have) {
            rd->secondary = pass.leg_value;
            rd->secondary_overload = exceeds_upper(pass.plan.leg_func, range, pass.leg_value);
            rd->secondary_valid = true;
        } else if (sel == 1u && pass.hz_have) {
            rd->secondary = pass.hz;
            rd->secondary_valid = true;
        }
        break;
    case FUNC_FREQ:
    case FUNC_PERIOD:
        if (pass.amplitude_have) {
            rd->secondary = pass.amplitude;
            rd->secondary_overload = exceeds_upper(amplitude_func(), range, pass.amplitude);
            rd->secondary_valid = true;
        }
        break;
    case FUNC_ACDCV:
        if (sel == 0u && pass.hz_have) {
            rd->secondary = pass.hz;
            rd->secondary_valid = true;
        } else if (sel == 1u || sel == 2u) {
            derived = true;
            dbm = (sel == 2u);
        } else if (sel == 3u) {
            rd->secondary = pass.dc_value;
            rd->secondary_overload = exceeds_upper(FUNC_DCV, range, pass.dc_value);
            rd->secondary_valid = true;
        } else if (sel == 4u) {
            rd->secondary = pass.ac_value;
            rd->secondary_overload = exceeds_upper(FUNC_ACV, range, pass.ac_value);
            rd->secondary_valid = true;
        }
        break;
    case FUNC_ACDCI:
        if (sel == 0u && pass.hz_have) {
            rd->secondary = pass.hz;
            rd->secondary_valid = true;
        } else if (sel == 1u) {
            rd->secondary = pass.ac_value;
            rd->secondary_overload = exceeds_upper(FUNC_ACA, range, pass.ac_value);
            rd->secondary_valid = true;
        } else if (sel == 2u) {
            rd->secondary = pass.dc_value;
            rd->secondary_overload = exceeds_upper(FUNC_DCA, range, pass.dc_value);
            rd->secondary_valid = true;
        }
        break;
    default:
        break;
    }

    if (derived && !overload) {
        rd->secondary = dbm ? dbm_value(c, &oor) : db_value(c, &oor);
        rd->secondary_overload = oor;
        rd->secondary_valid = true;
    }
    return derived && !overload && oor;
}

static void publish(void) {
    reading_t *rd = &g_state.reading;
    const func_t f = pass.func;
    const float c = pass.calibrated;
    bool overload;
    bool skip_primary_db;
    bool oor = false;
    float v = c;

    rd->calibrated = c;
    rd->adc_abs = (int32_t)pass.adc_abs;
    rd->adc_negative = pass.adc_negative;
    overload = is_freq_func(f) ? pass.freq_overload : meas_overload();
    rd->overload = overload;

    skip_primary_db = publish_secondary(f, c, overload);

    /* P8: dB / dBm, never on an overloaded reading (0xB0C2). */
    if (!overload && !skip_primary_db) {
        v = meas_apply_db(c, f, &oor);
    }
    rd->db_out_of_range = oor;

    /* P9: the value ACQuire and the REL key capture, after dB / dBm and before REL (0x9BB6). */
    rd->raw = v;

    /* P10, P11: REL still applies to an overloaded reading, percent does not (0x9BC2, 0xB084). */
    v = meas_apply_rel(v, f);
    if (!overload) {
        v = meas_apply_percent(v);
    }
    rd->primary = v;
    rd->valid = true;

    /* P12, P13 see the final value. */
    meas_update_limits(v, f);
    meas_update_minmax(v);
    hold_observe(v, overload);

    phase = MEAS_COMPLETE;
}

/* ---- The pass ---------------------------------------------------------------------------------------- */

static void start_gate(void) {
    freq_gate_start();
    stall_ms = 0u;
    phase = MEAS_FREQ_GATING;
}

static void start_accumulating(uint8_t target) {
    pass.sum = 0.0f;
    pass.count = 0u;
    pass.target = target;
    phase = MEAS_ACCUMULATING;
}

/* Enters the stage `pass.stage` and every following stage that has nothing to wait for, stopping at the first
 * one that needs the converter or the counter. A loop, not a recursion: stages that do not apply move
 * `stage` on and continue. */
static void advance(void) {
    const uint8_t range = pass.range;
    const func_t f = pass.func;
    uint8_t discards;

    for (;;) {
        switch (pass.stage) {
        case ST_PROBE:
            if (!autorange_applies(f)) {
                pass.stage = ST_LEG_OUT;
                continue;
            }
            phase = MEAS_PROBING;
            return;

        case ST_LEG_OUT:
            if (pass.plan.kind == SEC_CONVERSION || pass.plan.kind == SEC_HZ_SWITCHED) {
                cal_load(pass.plan.leg_func, range, pass.rate, &leg_set);
                discards = fe_apply(pass.plan.out_fe, range);
                hw_valid = false; /* the front end is not the primary one until ST_LEG_BACK restores it */
                if (pass.plan.kind == SEC_CONVERSION) {
                    pass.settle_left = (uint8_t)(discards + settle_extra(MEAS_SETTLE_SECONDARY));
                    pass.stage = ST_LEG_READ;
                } else {
                    pass.settle_left = discards;
                    pass.stage = ST_LEG_GATE;
                }
                phase = MEAS_SECONDARY_SETTLING;
                return;
            }
            pass.stage = (pass.plan.kind == SEC_HZ_DIRECT) ? ST_LEG_GATE : ST_AMPL;
            continue;

        case ST_LEG_READ:
            phase = MEAS_SECONDARY_READ;
            return;

        case ST_LEG_GATE:
            start_gate();
            return;

        case ST_LEG_BACK:
            if (pass.plan.kind == SEC_CONVERSION || pass.plan.kind == SEC_HZ_SWITCHED) {
                /* The restore, then the 20 conversions the original discards after it (0x9D3C, 0x9D82,
                 * 0x9E1A...). The bookkeeping records the restored state as the primary one, as the
                 * original does: the ACV restore uses the range-only routine, whose bit state differs
                 * from the full routine's. UNVERIFIED: that this is intended and not a latent fault of the
                 * original; check on the bench by comparing an ACV reading with and without the leg. */
                discards = fe_apply(pass.plan.back_fe, range);
                hw_valid = true;
                hw_func = f;
                hw_range = range;
                hw_fe = primary_fe(f);
                pass.settle_left = (uint8_t)(discards + settle_extra(MEAS_SETTLE_SECONDARY));
                pass.stage = ST_AMPL;
                phase = MEAS_SETTLING;
                return;
            }
            pass.stage = ST_AMPL;
            continue;

        case ST_AMPL:
            if (is_freq_func(f)) {
                if (pass.plan.kind == SEC_AMPLITUDE) {
                    start_accumulating(rate_sample_count[pass.rate]);
                    return;
                }
                pass.stage = ST_GATE;
            } else {
                pass.stage = ST_PRIMARY;
            }
            continue;

        case ST_PRIMARY:
            start_accumulating(primary_count(f, range, pass.rate));
            return;

        case ST_GATE:
            start_gate();
            return;

        case ST_AC_OUT:
            cal_load((f == FUNC_ACDCV) ? FUNC_ACV : FUNC_ACA, range, pass.rate, &leg_set);
            discards = fe_apply((f == FUNC_ACDCV) ? FE_ACV_RANGE_ONLY : FE_ACI, range);
            /* The DC front end is restored by the NEXT pass applying it again, which costs the same
             * discards the original spends at the end of this one (0xA25A, 0xA3E4). */
            hw_valid = false;
            pass.settle_left = (uint8_t)(discards + settle_extra((f == FUNC_ACDCV) ? MEAS_AC_LEG_EXTRA_V
                                                                                   : MEAS_AC_LEG_EXTRA_I));
            pass.stage = ST_AC_READ;
            phase = MEAS_AC_SETTLING;
            return;

        case ST_AC_READ:
            phase = MEAS_AC_READ;
            return;

        case ST_ACDC_GATE:
            if (pass.plan.kind == SEC_ACDC_HZ) {
                start_gate();
                return;
            }
            pass.stage = ST_FINISH;
            continue;

        case ST_FINISH:
        default:
            publish();
            return;
        }
    }
}

/* P1: the trigger wait (0x96EE). IMM starts at once; MAN waits for the TRIG key, BUS for a bus trigger, and
 * either request is consumed. Entering a wait silences the beeper for continuity and when the beeper is on
 * (0x9700-0x970E). */
static bool trigger_ready(void) {
    bool go = false;

    if (g_state.trigger == TRIG_IMM) {
        trigger_silenced = false;
        return true;
    }
    if (!trigger_silenced) {
        trigger_silenced = true;
        if (g_state.func == FUNC_CONT || g_state.settings.beeper) {
            buzzer_set(false);
            limit_tone = false;
        }
    }
    if (g_state.trigger == TRIG_MAN) {
        go = g_state.manual_trigger_req;
        g_state.manual_trigger_req = false;
    } else {
        go = g_state.bus_trigger_pending;
        g_state.bus_trigger_pending = false;
    }
    if (go) {
        trigger_silenced = false;
    }
    return go;
}

static void begin_pass(void) {
    const func_t f = g_state.func;
    const uint8_t range = range_of(f);
    uint8_t discards = 0u;

    pass.func = f;
    pass.range = range;
    pass.rate = g_state.rate;
    pass.stage = ST_PROBE;
    pass.plan = plan_secondary(f);
    pass.settle_left = 0u;
    pass.sum = 0.0f;
    pass.count = 0u;
    pass.probe_changes = 0u;
    pass.calibrated = 0.0f;
    pass.dc_value = 0.0f;
    pass.ac_value = 0.0f;
    pass.leg_value = 0.0f;
    pass.leg_have = false;
    pass.hz = 0.0f;
    pass.hz_have = false;
    pass.amplitude = 0.0f;
    pass.amplitude_have = false;
    pass.freq_overload = false;
    pass.adc_abs = 0u;
    pass.adc_negative = false;
    stall_ms = 0u;

    if (!cal_valid || cal_func != cal_func_of(f) || cal_range != range || cal_rate != pass.rate) {
        load_primary(f, range, pass.rate);
    }
    if (!hw_valid || hw_func != f || hw_range != range || hw_fe != primary_fe(f)) {
        discards = apply_primary(f, range);
    }
    if (last_rate != pass.rate) {
        const uint8_t rate_discards = settle_extra(MEAS_SETTLE_RATE);

        if (discards < rate_discards) {
            discards = rate_discards;
        }
        last_rate = pass.rate;
    }
    /* After a relay exercise the front end was put back at once, so the pass finds it in place; the time it
     * needs to settle was recorded then and is owed here. */
    if (discards < restore_discards) {
        discards = restore_discards;
    }
    restore_discards = 0u;
    if (discards != 0u) {
        pass.settle_left = discards;
        phase = MEAS_SETTLING;
        return;
    }
    advance();
}

/* A pass is stale when the settings it was built for are no longer the settings in force: the original
 * flagged this with M0F60.4 and showed dashes. */
static bool pass_is_stale(void) {
    return g_state.func != pass.func || g_state.rate != pass.rate || range_of(pass.func) != pass.range;
}

/* One discarded conversion; the stage that asked for the settling carries on when the count runs out. */
static void step_discard(void) {
    uint32_t magnitude;
    bool negative;

    if (!take_conversion(&magnitude, &negative)) {
        if (stall_ms >= MEAS_STALL_LIMIT_MS) {
            enter_fault();
        }
        return;
    }
    if (pass.settle_left > 0u) {
        pass.settle_left--;
    }
    if (pass.settle_left == 0u) {
        advance();
    }
}

/* The probe conversion goes through the whole correction chain, user zero and dead-band included, exactly as
 * the original's a3f2(1) does (0x15EE0). */
static void step_probe(uint32_t magnitude, bool negative) {
    const func_t calfunc = cal_func_of(pass.func);
    const float sample = cal_sample(&primary_set, magnitude, negative);
    const float v = correct_mean(&primary_set, calfunc, sample, pass.range, NULL);
    const uint8_t cap = (uint8_t)(MEAS_AUTORANGE_CAP_FACTOR * (tbl_range_max(pass.func) + 1u));
    uint8_t discards;

    pass.probe_abs = fabsf(v);
    if (pass.probe_changes < cap && meas_autorange_step()) {
        pass.probe_changes++;
        load_primary(pass.func, pass.range, pass.rate);
        discards = apply_primary(pass.func, pass.range);
        pass.settle_left = discards;
        phase = MEAS_SETTLING; /* the stage stays ST_PROBE: the next probe follows the settling */
        return;
    }
    pass.stage = ST_LEG_OUT;
    advance();
}

static void step_accumulate(uint32_t magnitude, bool negative) {
    const func_t f = pass.func;
    const func_t calfunc = cal_func_of(f);
    float mean;
    float c;
    bool forced;

    pass.sum += cal_sample(&primary_set, magnitude, negative);
    pass.count++;
    pass.adc_abs = magnitude;
    pass.adc_negative = negative;
    if (pass.count < pass.target) {
        return;
    }

    mean = pass.sum / (float)pass.count;
    c = correct_mean(&primary_set, calfunc, mean, pass.range, &forced);
    if (forced) {
        pass.adc_negative = false;
    }

    if (pass.stage == ST_AMPL) {
        pass.amplitude = c;
        pass.amplitude_have = true;
        pass.stage = ST_GATE;
    } else {
        pass.calibrated = c;
        pass.dc_value = c;
        pass.stage = is_acdc(f) ? ST_AC_OUT : ST_FINISH;
    }
    advance();
}

static void step_leg_read(uint32_t magnitude, bool negative) {
    const float sample = cal_sample(&leg_set, magnitude, negative);

    pass.leg_value = correct_mean(&leg_set, pass.plan.leg_func, sample, pass.range, NULL);
    pass.leg_have = true;
    pass.stage = ST_LEG_BACK;
    advance();
}

/* The AC leg of AC+DC: the combination is the RMS sum of the two parts (0xA234-0xA256, 0xA3B6-0xA3D4). The
 * sign of the result is lost, so the flag reports the sign of the last AC conversion. */
static void step_ac_read(uint32_t magnitude, bool negative) {
    const func_t leg = (pass.func == FUNC_ACDCV) ? FUNC_ACV : FUNC_ACA;
    const float sample = cal_sample(&leg_set, magnitude, negative);
    bool forced;

    pass.ac_value = correct_mean(&leg_set, leg, sample, pass.range, &forced);
    pass.calibrated = sqrtf(pass.dc_value * pass.dc_value + pass.ac_value * pass.ac_value);
    pass.adc_abs = magnitude;
    pass.adc_negative = negative && !forced;
    pass.stage = ST_ACDC_GATE;
    advance();
}

static void step_read(void) {
    uint32_t magnitude;
    bool negative;

    if (!take_conversion(&magnitude, &negative)) {
        if (stall_ms >= MEAS_STALL_LIMIT_MS) {
            enter_fault();
        }
        return;
    }
    switch (phase) {
    case MEAS_PROBING:
        step_probe(magnitude, negative);
        break;
    case MEAS_ACCUMULATING:
        step_accumulate(magnitude, negative);
        break;
    case MEAS_SECONDARY_READ:
        step_leg_read(magnitude, negative);
        break;
    case MEAS_AC_READ:
        step_ac_read(magnitude, negative);
        break;
    default:
        break;
    }
}

/* The counter. A gate that ends without a result is "no signal", not a fault: the original reads 0.0 there.
 * The deadline is the counter's own: gate time plus two signal periods, enforced by the Timer1
 * overflow interrupt for the whole measurement, so this only polls. The ended gate is stopped with
 * freq_gate_abort(): left armed it would keep its state until the next freq_gate_start(). */
static void step_gate(void) {
    float hz = 0.0f;
    bool ok = false;

    switch (freq_gate_poll()) {
    case FREQ_GATE_RUNNING:
        return;
    case FREQ_GATE_DONE:
        if (pass.stage == ST_GATE && pass.func == FUNC_PERIOD) {
            ok = freq_period_read(&hz);
        } else {
            ok = freq_read(&hz);
        }
        break;
    case FREQ_GATE_NO_SIGNAL:
    default:
        freq_gate_abort();
        break;
    }
    if (!ok) {
        hz = 0.0f;
    }

    switch (pass.stage) {
    case ST_LEG_GATE:
        pass.hz = hz;
        pass.hz_have = true;
        pass.stage = (pass.plan.kind == SEC_HZ_SWITCHED) ? ST_LEG_BACK : ST_AMPL;
        break;
    case ST_ACDC_GATE:
        pass.hz = hz;
        pass.hz_have = true;
        pass.stage = ST_FINISH;
        break;
    case ST_GATE:
    default:
        /* FREQ with no signal reads 0.0 and is valid; PERIOD with no signal is an overload
         * carrying the number FETC? prints as 9.9e37, so the display shows OVL rather than a believable
         * huge number. A deliberate change from the original's +FLT_MAX. */
        if (pass.func == FUNC_PERIOD && !ok) {
            pass.calibrated = MEAS_PERIOD_NO_SIGNAL;
            pass.freq_overload = true;
        } else {
            pass.calibrated = hz;
        }
        pass.stage = ST_FINISH;
        break;
    }
    advance();
}

/* ---- The relay exercise ------------------------------------------------------------------- */

/* How long one relay pattern is held before the next is written: the STEP PERIOD, and the one number that sets
 * how fast the exercise cycles. It may be retuned: a smaller value cycles faster, 25 ms being the least a
 * mechanical relay needs to operate and release, and a larger one works each relay less often.
 *
 * 75 ms was chosen from the simulator (sim test 19, converter at 16.658 ms per conversion, 60 Hz). Before the
 * guard was moved off the per-pattern path, one pattern took the guard (7 discards + 2 reads = 9 conversions,
 * 150 ms) plus the 25 ms hold, and the measured rate was 5.50 patterns/s. With a 75 ms step and the guard once
 * per MEAS_GUARD_PERIOD_MS the measured rate is 9.25 patterns/s, 1.68 times the old one; the aim was 1.5 times.
 *
 * Measured on the meter with :SYST:REL:STAT?: the power-on pass wrote 16 patterns in 1345 ms with the
 * 75 ms step, 84 ms per pattern, so 9 ms per pattern is spent outside the hold. No speed change was noticeable
 * and about 40 % faster was wanted. With the guard (about 167 ms) once per 800 ms, a 75 ms step gives about
 * 9.8 patterns/s on the meter; 50 ms gives 800 / 59 = 13.6 patterns per 967 ms, about 14 patterns/s, 1.43 times. */
#define MEAS_RELAY_STEP_MS 50u
#define MEAS_RELAY_FULL_MS 60000u /* the menu's exercise */
#define MEAS_RELAY_BOOT_MS 1500u  /* the quick one at power-on */

/* The input guard: DC V on its highest range, in the state the instrument powers up in. Above this the
 * exercise refuses to start and stops if it is running. */
#define MEAS_GUARD_VOLTS 2.0f
#define MEAS_GUARD_RANGE 0u
#define MEAS_GUARD_READS 2u

/* The guard is run before the first pattern and then again whenever this much time has passed since the last
 * guard began, checked after each pattern. It is not run between patterns: a live input is still found within
 * one second, which is the property that matters. The guard is found due at the end of a pattern's
 * hold, so two guards begin at most MEAS_GUARD_PERIOD_MS + MEAS_RELAY_STEP_MS apart, and the compared conversion
 * is at the same place in every guard; an input that arrives just after one is therefore seen at the next
 * within that time, which the assertion keeps under 1000 ms. */
#define MEAS_GUARD_PERIOD_MS 800u
_Static_assert(MEAS_GUARD_PERIOD_MS + MEAS_RELAY_STEP_MS < 1000u,
               "the input guard must run at least once a second");

/* The functions whose patterns are stepped through, in the order the original numbers them. Each is applied
 * with the original's own routine for every range it has, in that routine's own write order.
 * Nothing here toggles a single bit and no state the original does not itself produce is entered. */
static const uint8_t exercise_funcs[] PROGMEM = {FUNC_DCV, FUNC_ACV, FUNC_DCA, FUNC_ACA, FUNC_OHM, FUNC_DIODE};
#define EXERCISE_FUNC_COUNT ((uint8_t)sizeof exercise_funcs)

typedef enum {
    EX_GUARD_APPLY,  /* DC V highest range; the guard clock restarts here */
    EX_GUARD_SETTLE, /* discard conversions while the front end settles */
    EX_GUARD_READ,   /* the conversions that are compared with the limit */
    EX_HOLD          /* a pattern is applied; wait out its step */
} exercise_step_t;

static struct {
    bool active;
    exercise_step_t step;
    uint8_t pattern;  /* index into the list of (function, range) patterns */
    uint8_t discards; /* conversions left to discard in EX_GUARD_SETTLE */
    uint8_t reads;    /* guard conversions taken so far */
    func_t saved_func;
    uint8_t saved_range;
} exercise;

static uint16_t timer_ms(const volatile uint16_t *counter) {
    uint16_t v;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        v = *counter;
    }
    return v;
}

static void timer_set(volatile uint16_t *counter, uint16_t ms) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        *counter = ms;
    }
}

static uint8_t exercise_pattern_count(void) {
    uint8_t total = 0u;
    uint8_t i;

    for (i = 0u; i < EXERCISE_FUNC_COUNT; i++) {
        total = (uint8_t)(total + tbl_range_max((func_t)pgm_read_byte(&exercise_funcs[i])) + 1);
    }
    return total;
}

static void exercise_pattern(uint8_t index, func_t *f, uint8_t *range) {
    uint8_t i;

    for (i = 0u; i < EXERCISE_FUNC_COUNT; i++) {
        const func_t fn = (func_t)pgm_read_byte(&exercise_funcs[i]);
        const uint8_t n = (uint8_t)(tbl_range_max(fn) + 1u);

        if (index < n) {
            *f = fn;
            *range = index;
            return;
        }
        index = (uint8_t)(index - n);
    }
    *f = FUNC_DCV;
    *range = 0u;
}

/* The one exit. Every way out ends here, the guard trip and the abort included, so the front end is always
 * put back. It follows the calibration sweeps: the function and range are those saved at the start, the front
 * end is applied at once so the relays are not left in an exercise pattern, and the coefficients are dropped
 * so the next pass reloads its own. The time the front end needs to settle is carried to that pass in
 * restore_discards. A PGA that differs from the calibrated one is picked up by selfcal_wanted(). */
static void exercise_finish(relay_exercise_result_t result) {
    exercise_timing = false;
    exercise.active = false;
    exercising = false;
    exercise_result = result;
    exercise_total_ms = 0u;
    timer_set(&exercise_hold_ms, 0u);
    timer_set(&exercise_guard_ms, 0u);
    claimed = false;
    meas_invalidate();
    restore_discards = apply_primary(exercise.saved_func, exercise.saved_range);
}

static void exercise_converter_stalled(void) {
    if (stall_ms >= MEAS_STALL_LIMIT_MS) {
        exercise_finish(RLYEX_FAULT);
        enter_fault();
    }
}

/* What one guard takes: the discards of the DC V front end, the conversions that are compared, and one more for
 * the wait until the first of them is ready. Used only to decide that a guard still fits in the time left. */
static uint16_t exercise_guard_cost_ms(void) {
    const uint32_t conversions = (uint32_t)settle_for_frontend(true, MEAS_SETTLE_DCV) + MEAS_GUARD_READS + 1u;

    return (uint16_t)((conversions * tdata_us() + 999u) / 1000u);
}

/* Decides what follows the end of a pattern's hold, or a guard that passed. The guard comes first when it is due
 * and there is time for it and a pattern after it; otherwise the next pattern is written if a whole step still
 * fits in the time left; otherwise the run is over. Nothing is started that the time cannot finish, so the
 * run ends within MEAS_RELAY_FULL_MS or MEAS_RELAY_BOOT_MS and never after. */
static void exercise_next(void) {
    const uint16_t left = timer_ms(&exercise_total_ms);
    func_t f;
    uint8_t range;

    if (timer_ms(&exercise_guard_ms) == 0u && left >= exercise_guard_cost_ms() + MEAS_RELAY_STEP_MS) {
        exercise.step = EX_GUARD_APPLY;
        return;
    }
    if (left < MEAS_RELAY_STEP_MS) {
        exercise_finish(RLYEX_DONE);
        return;
    }
    exercise_pattern(exercise.pattern, &f, &range);
    (void)fe_apply(primary_fe(f), range);
    exercise_patterns_written++;
    exercise_timing = true;
    timer_set(&exercise_hold_ms, MEAS_RELAY_STEP_MS);
    exercise.step = EX_HOLD;
}

/* One call does at most one conversion's worth of work or one relay write. The guard runs before the first
 * pattern and then once every MEAS_GUARD_PERIOD_MS, so a source connected while the exercise runs is noticed
 * within a second. */
static void exercise_task(void) {
    uint32_t magnitude;
    bool negative;

    wdt_reset();
    switch (exercise.step) {
    case EX_GUARD_APPLY:
        cal_load(FUNC_DCV, MEAS_GUARD_RANGE, g_state.rate, &leg_set);
        exercise.discards = fe_apply(FE_DCV, MEAS_GUARD_RANGE);
        exercise.reads = 0u;
        stall_ms = 0u;
        timer_set(&exercise_guard_ms, MEAS_GUARD_PERIOD_MS);
        exercise.step = EX_GUARD_SETTLE;
        return;

    case EX_GUARD_SETTLE:
        if (!take_conversion(&magnitude, &negative)) {
            exercise_converter_stalled();
            return;
        }
        if (exercise.discards > 0u) {
            exercise.discards--;
        }
        if (exercise.discards == 0u) {
            exercise.step = EX_GUARD_READ;
        }
        return;

    case EX_GUARD_READ:
        if (!take_conversion(&magnitude, &negative)) {
            exercise_converter_stalled();
            return;
        }
        if (fabsf(correct_mean(&leg_set, FUNC_DCV, cal_sample(&leg_set, magnitude, negative), MEAS_GUARD_RANGE,
                               NULL)) > MEAS_GUARD_VOLTS) {
            exercise_finish(RLYEX_INPUT_ON);
            return;
        }
        exercise.reads++;
        if (exercise.reads < MEAS_GUARD_READS) {
            return;
        }
        exercise_next();
        return;

    case EX_HOLD:
    default:
        if (timer_ms(&exercise_hold_ms) != 0u) {
            return;
        }
        exercise.pattern = (uint8_t)((exercise.pattern + 1u) % exercise_pattern_count());
        exercise_next();
        return;
    }
}

/* Starts the exercise: `full` is the 60 s menu run, otherwise the 1.5 s power-on run. Returns false when it
 * cannot start: the converter is in use (a calibration sweep, ZERO CAL or an exercise already running). It
 * does not look at the input itself; the guard is the first state of the machine, so the caller learns of a
 * live input from meas_relay_exercise_result() == RLYEX_INPUT_ON a moment later, with the front end already
 * restored. The reading is invalid for as long as it runs.
 *
 * BLOCKED-ON-HARDWARE: the patterns are the original's, in the original's order, and nothing break-before-make
 * is added. Which bits drive relays and which drive solid-state switches is
 * unknown until the board's continuity has been checked. */
bool meas_relay_exercise_start(bool full) {
    if (exercise.active || !meas_claim()) {
        return false;
    }
    exercise.saved_func = g_state.func;
    exercise.saved_range = range_of(g_state.func);
    exercise.pattern = 0u;
    exercise.step = EX_GUARD_APPLY;
    exercise.active = true;
    exercising = true;
    exercise_result = RLYEX_RUNNING;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        exercise_total_ms = full ? MEAS_RELAY_FULL_MS : MEAS_RELAY_BOOT_MS;
        exercise_hold_ms = 0u;
        exercise_guard_ms = 0u;
        exercise_elapsed_ms = 0u;
        exercise_timing = false;
        exercise_patterns_written = 0u;
    }
    g_state.reading.valid = false;
    return true;
}

/* Any key. Safe to call when nothing is running. */
void meas_relay_exercise_abort(void) {
    if (exercise.active) {
        exercise_finish(RLYEX_ABORTED);
    }
}

relay_exercise_result_t meas_relay_exercise_result(void) {
    return exercise_result;
}

void meas_relay_exercise_stats(uint16_t *patterns, uint32_t *elapsed_ms) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        *patterns = exercise_patterns_written;
        *elapsed_ms = exercise_elapsed_ms;
    }
}

/* Whole seconds left, rounded up, for the countdown on the display. 0 when nothing is running. */
uint8_t meas_relay_exercise_seconds_left(void) {
    if (!exercise.active) {
        return 0u;
    }
    return (uint8_t)((timer_ms(&exercise_total_ms) + 999u) / 1000u);
}

/* ---- Line frequency ----------------------------------------------------------------------- */

static void abort_pass(void);

/* Registers 8 and 9, written wherever the original writes them: start-up, a rate change and before a
 * calibration measurement. */
static void program_data_rate(void) {
    adc_write_register(ADC_REG_DRATE_LO, line_50hz ? ADC_DRATE_LO_50HZ : ADC_DRATE_LO_60HZ);
    adc_write_register(ADC_REG_DRATE_HI, ADC_DRATE_HI_VALUE);
}

bool meas_line_is_50hz(void) {
    return line_50hz;
}

/* Sets the line frequency, stores it, reprograms the converter and owes a self-calibration before the next
 * reading. Returns false when the converter is in use or the byte could not be stored; nothing has changed in
 * that case. Not protected by the service lock. The next pass discards the settle for a rate
 * change, which at the new tDATA is the right time for the digital filter to refill. */
bool meas_set_line(bool is_50hz) {
    if (claimed) {
        return false;
    }
    if (is_50hz == line_50hz) {
        return true;
    }
    if (!ee_write_u8(EE_SET_LINE, is_50hz ? LINE_EE_50HZ : LINE_EE_60HZ)) {
        return false;
    }
    line_50hz = is_50hz;
    program_data_rate();
    selfcal_forced = true;
    selfcal_blank = true;
    last_rate = RATE_COUNT;
    cal_valid = false;
    abort_pass();
    return true;
}

/* ---- Self-calibration on demand ---------------------------------------------------------------- */

bool meas_selfcal_start(void) {
    if (claimed || exercising || selfcal_req == SELFCAL_REQ_RUNNING) {
        return false;
    }
    selfcal_req = SELFCAL_REQ_RUNNING;
    selfcal_forced = true;
    selfcal_blank = true; /* the reading on screen was taken with the old calibration: drop it */
    return true;
}

bool meas_selfcal_done(bool *ok) {
    if (selfcal_req != SELFCAL_REQ_OK && selfcal_req != SELFCAL_REQ_FAILED) {
        return false;
    }
    if (ok != NULL) {
        *ok = (selfcal_req == SELFCAL_REQ_OK);
    }
    selfcal_req = SELFCAL_REQ_NONE; /* reported once, so a second poll does not re-announce it */
    return true;
}

void meas_task(void) {
    if (exercise.active) {
        exercise_task();
        return;
    }
    if (claimed || hold_latched) {
        return;
    }
    if (phase != MEAS_COMPLETE && phase != MEAS_FAULT && selfcal_step()) {
        return;
    }
    switch (phase) {
    case MEAS_IDLE:
        if (trigger_ready()) {
            begin_pass();
        }
        return;
    case MEAS_COMPLETE:
        return;
    case MEAS_FAULT:
        if (stall_ms >= MEAS_FAULT_RETRY_MS) {
            phase = MEAS_IDLE;
        }
        return;
    default:
        break;
    }

    if (pass_is_stale()) {
        phase = MEAS_IDLE;
        g_state.reading.valid = false;
        return;
    }
    switch (phase) {
    case MEAS_SETTLING:
    case MEAS_SECONDARY_SETTLING:
    case MEAS_AC_SETTLING:
        step_discard();
        break;
    case MEAS_PROBING:
    case MEAS_ACCUMULATING:
    case MEAS_SECONDARY_READ:
    case MEAS_AC_READ:
        step_read();
        break;
    case MEAS_FREQ_GATING:
        step_gate();
        break;
    default:
        break;
    }
}

meas_phase_t meas_phase(void) {
    return phase;
}

bool meas_reading_ready(void) {
    return phase == MEAS_COMPLETE;
}

void meas_take_reading(void) {
    if (phase == MEAS_COMPLETE) {
        phase = MEAS_IDLE;
    }
}

/* ---- Settings changes -------------------------------------------------------------------------------- */

/* The reading that was being taken belongs to the old setting and is dropped. A latched HOLD is released so
 * a function, range or rate change cannot leave the instrument frozen (hold_observe, item 3). */
static void abort_pass(void) {
    if (!claimed) {
        phase = MEAS_IDLE;
    }
    g_state.reading.valid = false;
    hold_latched = false;
    hold_run = 0u;
}

void meas_invalidate(void) {
    hw_valid = false;
    cal_valid = false;
    abort_pass();
}

/* 0xF000 and its siblings: a function key clears the dual display and MAX/MIN, then applies the front end and
 * reloads the coefficients. */
void meas_set_function(func_t f) {
    if (f >= FUNC_COUNT) {
        return;
    }
    g_state.func = f;
    g_state.dual_display = false;
    g_state.minmax.active = false;
    g_state.minmax_view = MINMAX_VIEW_LIVE;
    meas_invalidate();
}

void meas_set_range(func_t f, uint8_t range, bool manual) {
    if (f >= FUNC_COUNT) {
        return;
    }
    if (range > tbl_range_max(f)) {
        range = tbl_range_max(f);
    }
    g_state.range[f].manual = manual;
    g_state.range[f].index = range;
    if (f == g_state.func) {
        meas_invalidate();
    }
}

/* 0xF036: the rate registers are written again, the coefficients reload (the zero is per rate) and the next
 * pass discards 3 conversions (begin_pass sees the rate differ from the one last applied). */
void meas_set_rate(rate_t r) {
    if (r >= RATE_COUNT) {
        return;
    }
    g_state.rate = r;
    if (!claimed) {
        program_data_rate();
    }
    cal_valid = false;
    abort_pass();
}

/* 0x9582: RESET, the status register, then the rate registers. The first pass applies the front end and
 * loads the coefficients (the original does both here, 0xBC52 and 0xDAA2). */
void meas_init(void) {
    adc_command(ADC_CMD_RESET);
    adc_write_register(ADC_REG_STATUS, ADC_STATUS_VALUE);
    line_50hz = (ee_read_u8(EE_SET_LINE) == LINE_EE_50HZ);
    program_data_rate();

    /* The converter has just been reset and has never been calibrated: owe a full calibration, which the
     * state machine runs before the first reading and which blanks the display meanwhile. */
    cur_pga = PGA_UNKNOWN;
    cal_pga = PGA_UNKNOWN;
    selfcal_forced = true;
    selfcal_blank = true;
    selfcal_active = false;
    exercise.active = false;
    exercising = false;
    exercise_result = RLYEX_IDLE;
    restore_discards = 0u;

    phase = MEAS_IDLE;
    claimed = false;
    hw_valid = false;
    cal_valid = false;
    last_rate = RATE_COUNT;
    trigger_silenced = false;
    fault_pending = false;
    fault_active = false;
    limit_tone = false;
    minmax_first = true;
    hold_latched = false;
    hold_run = 0u;
    stall_ms = 0u;

    /* The original leaves the window zero at power-up while the menu offers "1 %". */
    if (!(g_state.hold_window > 0.0f)) {
        g_state.hold_window = MEAS_HOLD_WINDOW_DEFAULT;
    }
}

/* ---- Exclusive use of the converter ------------------------------------------------------------------- */

bool meas_claim(void) {
    if (claimed) {
        return false;
    }
    if (selfcal_active) {
        /* A calibration is running: the sweep's conversions would otherwise end it by accident. */
        uint32_t magnitude;
        bool negative;

        if (!read_blocking(&magnitude, &negative)) {
            enter_fault();
            return false;
        }
        selfcal_finished();
    }
    program_data_rate();
    claimed = true;
    phase = MEAS_IDLE;
    hw_valid = false;
    cal_valid = false;
    return true;
}

void meas_release(void) {
    claimed = false;
    meas_invalidate();
}

/* The mean of `samples` conversions corrected with the caller's set: the zero off every sample, the gain on
 * the mean, and nothing else -- no user zero, no dead-band, no low-range replacement, which
 * is what CAL_F_SWEEP means. The caller needs no further cal_apply; applying the identity sets a sweep uses
 * a second time changes nothing.
 *
 * This waits. A sweep is a deliberate operation with the host told to wait. Each conversion takes about 20 ms
 * and the watchdog is fed every 50 ms, so `samples` conversions take about samples x 20 ms (80 ms for a
 * sweep's 4, at most 5.1 s for 255). A converter that stops answering ends it after ADC_DRDY_TIMEOUT_MS with
 * a fault. */
bool meas_average_with(const cal_set_t *set, uint8_t samples, float *out) {
    float sum = 0.0f;
    uint8_t i;

    if (!claimed || samples == 0u) {
        return false;
    }
    for (i = 0u; i < samples; i++) {
        uint32_t magnitude;
        bool negative;

        if (!read_blocking(&magnitude, &negative)) {
            enter_fault();
            return false;
        }
        sum += cal_sample(set, magnitude, negative);
    }
    *out = cal_apply(set, sum / (float)samples, 0u, CAL_F_SWEEP);
    return true;
}

/* ---- ZERO CAL (0x1B76) ------------------------------------------------------------------------------- */

/* The captured zero is refused when it exceeds the dead-band threshold of the range. */
static bool zero_plausible(func_t f, uint8_t range, float z) {
    float limit;

    if (f == FUNC_DCV) {
        limit = (range < 4u) ? MEAS_DEADBAND_K * table_down(FUNC_DCV, range) : MEAS_ZERO_LIMIT_DCV_FLAT;
    } else {
        limit = (range < 5u) ? MEAS_DEADBAND_K * table_down(FUNC_OHM, range) : MEAS_ZERO_LIMIT_OHM_FLAT;
    }
    return fabsf(z) <= limit;
}

/* The capture is the same average the original takes (6 conversions, zero per sample, gain on the mean:
 * DC V mean * gain, ohms mean * (mean * f0 + f1)); the caller stores it as the user zero. It is a separate
 * sequence from the pass, as the original's is, and it claims the converter for its duration.
 *
 * Differences from 0x1B76 (ORIGINAL_BUGS #65): the 20 ms delay, which the first conversion can predate, is
 * replaced by discarding the conversions the front-end change needs plus one more, so every conversion
 * averaged started after the front end settled. Only DC V and ohms have a user zero. The function and range
 * are the caller's, and the displayed ones are not touched: releasing the converter makes the next pass apply
 * its own front end again. */
bool meas_zero_capture(func_t f, uint8_t range, float *out) {
    uint8_t discards;
    cal_set_t set;
    float z = 0.0f;
    bool ok = true;

    if (f != FUNC_DCV && f != FUNC_OHM) {
        return false;
    }
    if (range > tbl_range_max(f)) {
        return false;
    }
    if (!meas_claim()) {
        return false;
    }

    discards = (uint8_t)(meas_frontend_apply(f, range) + 1u);
    while (ok && discards != 0u) {
        uint32_t magnitude;
        bool negative;

        ok = read_blocking(&magnitude, &negative);
        discards--;
    }
    if (ok) {
        cal_load(f, range, g_state.rate, &set);
        ok = meas_average_with(&set, MEAS_ZERO_SAMPLES, &z);
    } else {
        enter_fault();
    }

    /* PUT THE FRONT END BACK. `meas_frontend_apply()` above moved the relays to the
     * function and range being captured, and nothing else was putting them back: `meas_release()` hands the
     * converter over and invalidates the pass, but it does not touch the front end.
     *
     * The visible consequence is not a wrong reading, it is a live instrument. ZERO CAL sweeps the ohm
     * ranges last, so the relays were left on ohm range 5 with the OHM SOURCE CONNECTED TO THE INPUT JACKS.
     * The next measurement pass would correct it -- except that in MAN or BUS trigger mode no pass starts
     * until a trigger arrives (`trigger_ready`), so the source stays on the terminals indefinitely, after a
     * menu operation the operator has already finished and walked away from.
     *
     * Restored before the release, while this function still owns the converter. */
    (void)meas_frontend_apply(g_state.func, range_of(g_state.func));

    meas_release();

    if (!ok || !zero_plausible(f, range, z)) {
        return false;
    }
    *out = z;
    return true;
}
