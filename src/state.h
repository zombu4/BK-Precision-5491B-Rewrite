/* state.h -- global instrument state: functions, ranges, rates, settings, flags.
 *
 * FROZEN FOUNDATION HEADER. Modules code against this; changing it changes every module.
 *
 * Source: the original's RAM variables and front-panel state.
 * The original kept these as scattered RAM bytes; here they are one struct. The *values* (function
 * numbering, rate numbering, range index order) are protocol- and EEPROM-visible and are reproduced exactly.
 */
#ifndef BK_STATE_H
#define BK_STATE_H

#include <stdbool.h>
#include <stdint.h>

/* Function codes. Order is EEPROM- and protocol-visible: do not renumber. (RAM 0x0B12 in the original.) */
typedef enum {
    FUNC_DCV = 0,
    FUNC_ACV = 1,
    FUNC_DCA = 2,
    FUNC_ACA = 3,
    FUNC_OHM = 4, /* 2-wire */
    FUNC_FREQ = 5,
    FUNC_PERIOD = 6,
    FUNC_CONT = 7,
    FUNC_DIODE = 8,
    FUNC_ACDCV = 9,
    FUNC_ACDCI = 10,
    FUNC_COUNT = 11
} func_t;

/* Integration rate. Index order is EEPROM-visible (tables are stored fast, medium, slow). */
typedef enum { RATE_FAST = 0, RATE_MEDIUM = 1, RATE_SLOW = 2, RATE_COUNT = 3 } rate_t;

/* Averaging sample counts per rate: 3 / 6 / 11 (original RAM 0x0106). */
extern const uint8_t rate_sample_count[RATE_COUNT];

typedef enum { TRIG_IMM = 0, TRIG_MAN = 1, TRIG_BUS = 2 } trig_t;

/* Range index 0 = highest range. Max index per function comes from tables.h (flash 0x5B9). */
#define RANGE_MAX_INDEX 5u

/* Per-function range selection (original RAM 0x0B25/0x0B26 + 2f). */
typedef struct {
    bool manual;      /* false = autorange */
    uint8_t index;    /* 0 = highest range */
} range_sel_t;

/* REL (relative) record per function (original RAM 0x0E32 + 9f).
 * `raw_reference` exists because changing dB REF or dBm REF recomputes an active REL reference in the
 * original, and the menu code cannot repeat that step without somewhere to keep the raw second float. The
 * remote code also reconstructs the ACQuire source from it. */
typedef struct {
    bool active;
    float reference;
    float raw_reference;
} rel_t;

/* Operator settings mirrored from EEPROM (original RAM 0x0E1B-0x0E1F). */
typedef struct {
    bool echo;            /* EEPROM 0x865 */
    uint8_t baud_index;   /* EEPROM 0x854 */
    uint8_t tx_terminator;/* EEPROM 0x863: 0 = LF, else CR */
    bool beeper;          /* EEPROM 0x856 */
    bool key_click;       /* EEPROM 0x855 */
} settings_t;

/* :UNIT:VOLTage dB / dBm state.
 *
 * This is NOT "per display channel, [0] primary and [1] secondary". The instrument has no primary/secondary unit pair at all:
 * the two bytes are PER FUNCTION. RAM 0x0133 is the DC voltage unit and 0x0134 is the AC voltage unit, so
 * the index here is 0 = DC voltage, 1 = AC voltage.
 *
 * There are two levels, and they are different storage:
 *  - this BACKING STORE, one entry per function: the mode byte, and the references the :UNIT queries read
 *    back -- AC at RAM 0x010E (dB) and 0x0116 (dBm), DC at 0x0112 and 0x0118. Stored at 0x07864 and
 *    0x0791E, read back at 0x07892 and 0x0789A.
 *  - the WORKING pair that the MATH menu edits and the dB/dBm conversion actually uses: RAM 0x010A (dB)
 *    and 0x0108 (dBm), which are math_state_t.db_ref and .dbm_ref. Read at 0x05B62, 0x060C0 and 0x0B1C2;
 *    written by the menu at 0x062C6.
 * The original copies a backing value into the working pair ONLY when that function is the active one:
 * 0x07870-0x07878 tests the function byte at RAM 0x0B12 and then runs the copy at 0x15F06. So a MATH menu
 * edit does NOT reach the backing store, and both levels have to exist.
 *
 * The active unit itself is a third thing: a shadow at RAM 0x0135, copied from 0x0133 or 0x0134 by 0x166F4
 * and 0x16BB6. Everything the display reads is that shadow (0x9C6C, 0xAED4, 0xF3D4, 0xF880, 0xFA38), and
 * it is already modelled by math_state_t.db_on / .dbm_on. display.c reads those, not this array. */
typedef enum { UNIT_NATIVE = 0, UNIT_DB = 1, UNIT_DBM = 2 } unit_mode_t;

typedef struct {
    unit_mode_t mode;
    float db_ref;
    int16_t dbm_ref; /* ohms, 1..9999 -- an int16 in the instrument, as in math_state_t below */
} unit_channel_t;

/* Math/limit state (MENU A).
 *
 * `dbm_ref` is not a float: in the instrument RAM 0x0108 is an
 * INT16 holding 1..9999 -- the dBm reference impedance in ohms, which the menu edits as an integer. A float
 * would accept 75.5 ohm from the remote port and then disagree with what the menu can display and store. */
typedef struct {
    bool percent_on;
    bool db_on;
    bool dbm_on;
    bool limit_on;
    float percent_ref;
    float db_ref;
    int16_t dbm_ref; /* ohms, 1..9999 (RAM 0x0108) */
    float limit_hi;
    float limit_lo;
} math_state_t;

/* Measurement results. */
typedef struct {
    float primary;    /* original RAM 0x0EE5 */
    float secondary;  /* original RAM 0x0EE1 */
    /* This is not "pre-REL, pre-math", and the
     * difference matters because ACQuire is how a client captures a reference.
     *
     * The original's pipeline is: overload test, then dB/dBm, then a COPY to RAM 0x0EAD (0x09BB6), then
     * REL, then percent, then the limit test, then MIN/MAX, then the FETCh print. Every ACQuire path
     * (0x07DBE, 0x10222, 0x106D2, 0x10A0E, 0x10D88) and the front-panel REL key (0x0E474) store RAM
     * 0x0EAD -- so the captured value is AFTER the dB or dBm conversion and BEFORE REL and percent. REL
     * then subtracts it from the dB value, which only works because both are in the same unit.
     *
     * So `raw` is the value after dB/dBm and before REL and percent. Calling it "pre-math" would make a
     * dB REL subtract a linear reference from a logarithmic one. For an overloaded reading it holds the
     * calibrated value. */
    float raw;

    /* The calibrated value, before the overload test and before dB/dBm: what `rel_t.raw_reference` needs.
     * The original never writes the REL record's bytes +5..+8 at all, so this has no counterpart in the
     * firmware -- it exists because the port recomputes an active REL reference when the dB or dBm
     * reference changes, which the original did by a route that is not reproduced. */
    float calibrated;
    int32_t adc_abs;  /* |24-bit ADC code|, original RAM 0x0E9D */
    bool adc_negative;/* sign, original RAM 0x0F5F bit 6 */
    bool overload;
    bool valid;

    /* Added by this port. The original has no flag for any of these: the
     * secondary display's overload is text only, and a dB or dBm value outside the representable range just
     * printed whatever the arithmetic produced. The port needs them so that a reply and the panel can agree
     * about a reading being absent rather than each inventing a number. */
    bool secondary_valid;
    bool secondary_overload;
    bool db_out_of_range;
} reading_t;

/* Which value the secondary display is showing (original RAM 0x0130). */
typedef enum { MINMAX_VIEW_LIVE = 0, MINMAX_VIEW_MAX = 1, MINMAX_VIEW_MIN = 2 } minmax_view_t;

/* MAX/MIN tracking. */
typedef struct {
    bool active;
    float max;
    float min;
} minmax_t;

/* Whole-instrument state. One instance, g_state, defined in main.c. */
typedef struct {
    func_t func;
    rate_t rate;
    trig_t trigger;
    range_sel_t range[FUNC_COUNT];
    uint8_t second_display[FUNC_COUNT]; /* original RAM 0x0126-0x012C */
    rel_t rel[FUNC_COUNT];
    settings_t settings;
    math_state_t math;
    reading_t reading;
    minmax_t minmax;
    bool remote;        /* RMT annunciator; cleared by the LOCAL key */
    bool hold_active;
    bool shift_active;
    uint8_t hold_count;
    float hold_window;  /* stored as a FRACTION, not the menu code */

    /* --- UI and measurement flags. The original packed these into RAM flag bytes; here they are named
     * fields. --- */
    minmax_view_t minmax_view;  /* RAM 0x0130 */
    bool dual_display;          /* RAM 0x0F60 bit 2: the secondary display is on (E-3) */
    bool manual_trigger_req;    /* RAM 0x0F61 bit 3: a manual trigger is pending */
    bool local_lockout;         /* RAM 0x0F5F bit 3 */
    bool menu_mode;             /* RAM 0x0F61 bits 1/6/7, collapsed: a menu owns the display */
    bool error_indicated;       /* drives the ERR lamp; set by the error queue, cleared on read */
    bool fetch_pending;         /* a FETC?/READ? is waiting for a completed measurement */
    bool bus_trigger_pending;   /* a *TRG / BUS trigger arrived */

    /* These live here, not as private statics in handlers.c, so that :DISP OFF reaches display.c and
     * :UNIT:VOLT DB reaches the display and measurement path. display.c and meas/ read them from here. */
    /* :DISPlay OFF/ON -- original RAM 0x0F61 bit 2, set at 0x10DF6 and 0xFAE2, cleared at 0x10E10. When
     * false the original blanks nothing: it skips the whole draw, so the panel freezes on its last
     * content while the formatting (and therefore the continuity beeper) still runs. */
    bool display_enabled;
    unit_channel_t unit_channel[2];   /* [0] DC voltage (RAM 0x0133), [1] AC voltage (RAM 0x0134) */
} instrument_t;

extern instrument_t g_state;

#endif /* BK_STATE_H */
