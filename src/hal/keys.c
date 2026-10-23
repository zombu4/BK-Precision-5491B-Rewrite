/* keys.c -- front-panel keyboard, read through VFD controller 1 (command 0x42, three bytes), and the
 * key dispatcher.
 *
 * Original: key_scan_read 0x3774, key_decode_scan_byte 0x3630, key_wait_press 0x37E2, key_sub_next 0x8E86,
 * key_sub_prev 0x8FA2, key_main_handler 0xDB4E, key_dispatch_shifted 0xDF00, key_dispatch_unshifted 0xE380.
 *
 * The original blocked in key_wait_press for 100 ms plus 50 ms; here the same debounce rule (the same code on
 * two scans at least 50 ms apart) runs as a state machine on a 1 ms tick and never waits. There is no
 * auto-repeat: a held key is reported once and reports again only after it was released.
 *
 * The dispatcher is the three original handlers merged. Order, all from 0xDB4E:
 *   1. a menu that is open owns the key (the original menu loops were blocking and never reached 0xDB4E;
 *      they called 0xDF00 / 0xE380 themselves, at 0xE50C / 0xE510 and 0xE9FC / 0xEA00, for a function key);
 *   2. any key clears HOLD (0xDB86-0xDB9A);
 *   3. in remote only SHIFT acts, as LOCAL (0xDBA2-0xDBC0);
 *   4. SHIFT latches, TRIG in manual-trigger mode, RATE, AUTO, RANGE UP / DOWN, PREV / NEXT;
 *   5. what is left goes to the SHIFT layer or the plain layer (0xDEEC-0xDEF8).
 * SHIFT is a latch with no timeout: it is toggled at 0xDBD8 (a second press clears it) and cleared by the first
 * key that reaches the SHIFT layer (0xDF06) or the RATE branch (0xDC3E). The only writers of RAM 0x0F5F bit 0
 * are the toggle 0x15870 and the clear 0x16012, and neither is on a timer.
 */
#include "keys.h"

#include <avr/interrupt.h>
#include <float.h>
#include "../fmathf.h"
#include <util/atomic.h>

#include "../board.h"
#include "../meas/meas.h"
#include "../remote/remote.h"
#include "../state.h"
#include "../tables.h"
#include "../ui/display.h"
#include "../ui/menu.h"
#include "buzzer.h"
#include "vfd.h"

#define KEY_SCAN_BYTES 3u

/* Scan code for each bit of scan bytes 0 and 1 (bit 0 first). Byte 2 only carries KEY_DCV as value 1. */
static const uint8_t key_code_by_bit[2][8] = {
    {KEY_PREV, KEY_NEXT, KEY_RANGE_DOWN, KEY_RANGE_UP, KEY_AUTO, KEY_MENU, KEY_SHIFT, KEY_RATE},
    {KEY_MAXMIN, KEY_REL, KEY_TRIG, KEY_DIODE, KEY_ACDC, KEY_FREQ, KEY_OHM, KEY_ACV},
};

static volatile uint16_t now_ms;
static uint8_t pending_code;
static uint16_t pending_since_ms;
static uint8_t held_code;

/* One scan byte carries a key only when exactly one bit is set; anything else decodes to no key. */
static uint8_t decode_scan_byte(uint8_t group, uint8_t value) {
    if (group == 2u) {
        return value == 0x01u ? KEY_DCV : KEY_NONE;
    }
    for (uint8_t bit = 0; bit < 8u; bit++) {
        if (value == (uint8_t)(1u << bit)) {
            return key_code_by_bit[group][bit];
        }
    }
    return KEY_NONE;
}

/* The first scan byte that is neither 0x00 nor 0xFF decides; later bytes are not consulted. */
static uint8_t scan_keys(void) {
    uint8_t raw[KEY_SCAN_BYTES];

    vfd_read_keyscan(raw);
    for (uint8_t group = 0; group < KEY_SCAN_BYTES; group++) {
        if (raw[group] != 0x00u && raw[group] != 0xFFu) {
            return decode_scan_byte(group, raw[group]);
        }
    }
    return KEY_NONE;
}

static uint16_t millis_now(void) {
    uint16_t t;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = now_ms; }
    return t;
}

/* Called from the main loop's 1 ms slot. */
void keys_tick_1ms(void) {
    now_ms++;
}

void keys_init(void) {
    pending_code = KEY_NONE;
    held_code = KEY_NONE;
    pending_since_ms = millis_now();
    g_state.shift_active = false;
}

/* How often the keyboard is actually read from the controller.
 *
 * The scan is not free and it is not private: reading the keys is a PT6311 command on the same serial bus
 * the display text goes over (`vfd_read_keyscan`, command 0x42). `keys_poll()` is
 * called once per main-loop pass, and the loop runs far faster than any finger -- measured in the
 * simulator with the SYS page open, the main controller was receiving **7,614 frames per second, every one
 * of them the read-key command**, and nothing else at all.
 *
 * 8 ms is chosen from the debounce rule, which is the only thing that defines how fast a scan needs to be:
 * a key must be seen twice KEY_DEBOUNCE_MS apart, so sampling several times inside that window is already
 * more than the algorithm can use. It cuts the bus traffic by a factor of about sixty and cannot change
 * which key presses are reported -- a press lasts tens of milliseconds at least.
 *
 * The original's cadence is NOT ESTABLISHED as a number, but it cannot have been anything like 7,614 Hz:
 * its main loop does a full measurement pass per iteration and its
 * debounce is the same 50 ms two-sample rule. */
#define KEY_SCAN_INTERVAL_MS 8u

uint8_t keys_poll(void) {
    static uint16_t last_scan_ms;
    static bool scanned_once;
    static uint8_t last_code;
    const uint16_t scan_now = millis_now();
    uint8_t code;

    if (scanned_once && (uint16_t)(scan_now - last_scan_ms) < KEY_SCAN_INTERVAL_MS) {
        /* Too soon to ask the hardware again. Re-use the last answer so the debounce below still sees a
         * continuous view of what is held -- skipping the scan must not look like the key being released. */
        code = last_code;
    } else {
        code = scan_keys();
        last_code = code;
        last_scan_ms = scan_now;
        scanned_once = true;
    }

    if (code == KEY_NONE) {
        pending_code = KEY_NONE;
        held_code = KEY_NONE;
        return KEY_NONE;
    }
    if (code == held_code) {
        return KEY_NONE;
    }
    const uint16_t now = millis_now();
    if (code != pending_code) {
        pending_code = code;
        pending_since_ms = now;
        return KEY_NONE;
    }
    if ((uint16_t)(now - pending_since_ms) < KEY_DEBOUNCE_MS) {
        return KEY_NONE;
    }

    held_code = code;
    if (g_state.func == FUNC_CONT) {
        buzzer_set(false);
    }
    if (!g_state.remote) {
        buzzer_click();
    }
    return code;
}

/* ---- Second-display parameter cycling (SHIFT+RATE selected display, then PREV / NEXT) ---------------------- */

/* Highest selector value per function; functions not listed have no second-display parameter. FREQ and PERIOD
 * share one selector (the original used a single byte, RAM 0x012C, for both). */
static bool second_display_slot(func_t f, uint8_t *slot, uint8_t *max_value) {
    switch (f) {
    case FUNC_DCV:
    case FUNC_ACV:
        *max_value = 3u;
        break;
    case FUNC_DCA:
    case FUNC_ACA:
        *max_value = 1u;
        break;
    case FUNC_FREQ:
    case FUNC_PERIOD:
        *max_value = 1u;
        f = FUNC_FREQ;
        break;
    case FUNC_ACDCV:
        *max_value = 4u;
        break;
    case FUNC_ACDCI:
        *max_value = 2u;
        break;
    default:
        return false;
    }
    *slot = (uint8_t)f;
    return true;
}

/* Frequency and period take their input path from the selector, so the front end and the loaded coefficients
 * must follow it (original 0x15F7C). */
static void second_display_changed(void) {
    if (g_state.func == FUNC_FREQ || g_state.func == FUNC_PERIOD) {
        const range_sel_t sel = g_state.range[g_state.func];
        meas_set_range(g_state.func, sel.index, sel.manual);
    }
}

void keys_sub_next(void) {
    uint8_t slot;
    uint8_t max_value;

    if (second_display_slot(g_state.func, &slot, &max_value)) {
        uint8_t *value = &g_state.second_display[slot];
        *value = (*value >= max_value) ? 0u : (uint8_t)(*value + 1u);
        second_display_changed();
    }
}

void keys_sub_prev(void) {
    uint8_t slot;
    uint8_t max_value;

    if (second_display_slot(g_state.func, &slot, &max_value)) {
        uint8_t *value = &g_state.second_display[slot];
        *value = (*value == 0u) ? max_value : (uint8_t)(*value - 1u);
        second_display_changed();
    }
}

/* ---- Dispatcher helpers ------------------------------------------------------------------------------------ */

/* Range index 5 of continuity, which SHIFT+OHM selects as a manual range (0xDF9A). */
#define CONTINUITY_KEY_RANGE 5u

/* The reference dB and dBm measure against: 1 mW (flash 0x0B00). */
#define DBM_REFERENCE_WATT 0.001f

typedef enum { REF_NONE, REF_PERCENT, REF_DB, REF_DBM } ref_edit_t;

/* Which reference editor a SHIFT key opened, until the editor reports a confirmed ENTER. */
static ref_edit_t pending_ref = REF_NONE;

/* Original 0xF744 + 0xF322 after a clear (0x15D62): the annunciators and the function label follow g_state. */
static void refresh_display(void) {
    display_annunciators();
    display_update();
}

/* The secondary views end whenever the function is applied again (0xF000: 0x166FE clears the dual-display bit,
 * 0x16D80 clears the MAX/MIN view). */
static void drop_secondary_views(void) {
    g_state.dual_display = false;
    g_state.minmax_view = MINMAX_VIEW_LIVE;
    g_state.minmax.active = false;
}

/* Original disp_refresh_function_mode 0xF000, as the key handlers use it after every mode change. */
static void reapply_function(void) {
    drop_secondary_views();
    meas_set_function(g_state.func);
    refresh_display();
}

static void select_function(func_t f) {
    g_state.func = f;
    reapply_function();
}

/* log10 as the original library computes it: a value that is not positive gives -FLT_MAX (0x14590). */
static float log10_or_floor(float x) {
    return (x > 0.0f) ? log10f(x) : -FLT_MAX;
}

/* The REL reference expressed in the new unit. dB (0xE072-0xE094): 20 log10(|reference / dB REF|). dBm
 * (0xE176-0xE1B4): 10 log10(reference^2 / dBm REF / 1 mW).
 * UNVERIFIED: for a zero reference the original multiplies -FLT_MAX by 20 or 10 in its soft-float multiply
 * (0x13610); the overflow result is not known. Check by running 0x14590 then 0x13610 on 0.0 in the AVR
 * simulator. */
static float reference_in_unit(unit_mode_t mode, float reference) {
    if (mode == UNIT_DB) {
        return 20.0f * log10_or_floor(fabsf(reference / g_state.math.db_ref));
    }
    return 10.0f * log10_or_floor(reference * reference / g_state.math.dbm_ref / DBM_REFERENCE_WATT);
}

/* Switches the voltage unit on (0xE030-0xE098 for dB, 0xE140-0xE1BE for dBm). An active REL reference is kept
 * in raw_reference and carried into the unit. Entering dB while dBm is on converts the already converted
 * reference once more, exactly as the original does (it saves the current reference without checking which
 * unit it is in). */
static void enter_unit(unit_mode_t mode) {
    rel_t *rel = &g_state.rel[g_state.func];

    g_state.math.db_on = (mode == UNIT_DB);
    g_state.math.dbm_on = (mode == UNIT_DBM);
    if (rel->active) {
        rel->raw_reference = rel->reference;
        rel->reference = reference_in_unit(mode, rel->reference);
    }
}

/* ---- Dispatcher: keys that act in every layer -------------------------------------------------------------- */

/* SHIFT+RATE (0xDC34-0xDCC0) switches the dual display; RATE alone (0xDCC2-0xDCE4) steps the integration
 * rate. Neither is available in continuity or diode, and the dual display is not available in ohms. The key
 * ends here: the layers have nothing on RATE, and the SHIFT latch is already cleared when this was SHIFT+RATE. */
static void key_rate(void) {
    const func_t f = g_state.func;

    if (g_state.shift_active) {
        g_state.shift_active = false;
        display_annunciators();
        if (f == FUNC_OHM || f == FUNC_CONT || f == FUNC_DIODE) {
            return;
        }
        g_state.dual_display = !g_state.dual_display;
        g_state.minmax_view = MINMAX_VIEW_LIVE;
        g_state.minmax.active = false;
        /* The original runs the front end (0xBC52) and, when the dual display turned on, the coefficient loader
         * (0xDAA2). meas_set_range does both for the current range. */
        meas_set_range(f, g_state.range[f].index, g_state.range[f].manual);
        refresh_display();
        return;
    }
    if (f == FUNC_CONT || f == FUNC_DIODE) {
        return;
    }
    g_state.rate = (rate_t)(((uint8_t)g_state.rate + 1u) % (uint8_t)RATE_COUNT);
    meas_set_rate(g_state.rate);
    display_annunciators();
}

/* AUTO (0xDCE8-0xDD32) toggles between autorange and the manual range now selected. Not in continuity or
 * diode. Returns true when the key goes on to the layer, which clears a latched SHIFT. */
static bool key_auto(void) {
    const func_t f = g_state.func;
    range_sel_t *sel = &g_state.range[f];

    if (f == FUNC_CONT || f == FUNC_DIODE) {
        return false;
    }
    sel->manual = !sel->manual;
    meas_set_range(f, sel->index, sel->manual);
    display_annunciators();
    return true;
}

/* RANGE UP (0xDD36-0xDD78): a higher range is a lower index. The key always makes the range manual; at the top
 * range that is all it does and the key ends there, without reaching the layer. Not in diode. */
static bool key_range_up(void) {
    const func_t f = g_state.func;
    range_sel_t *sel = &g_state.range[f];

    if (f == FUNC_DIODE) {
        return false;
    }
    sel->manual = true;
    if (sel->index == 0u) {
        display_annunciators();
        return false;
    }
    sel->index--;
    meas_set_range(f, sel->index, true);
    display_annunciators();
    return true;
}

/* RANGE DOWN (0xDD7A-0xDDD8): as RANGE UP towards the last index of the function's range table (flash
 * 0x05B9). */
static bool key_range_down(void) {
    const func_t f = g_state.func;
    range_sel_t *sel = &g_state.range[f];

    if (f == FUNC_DIODE) {
        return false;
    }
    sel->manual = true;
    if (sel->index >= tbl_range_max(f)) {
        display_annunciators();
        return false;
    }
    sel->index++;
    meas_set_range(f, sel->index, true);
    display_annunciators();
    return true;
}

/* PREV / NEXT (0xDDE4-0xDEE8, 0xDE64-0xDEE8). With the dual display on they pick its parameter and the key
 * ends; otherwise they swap between the MAX and MIN views while one is shown (both keys do the same: the
 * original swaps 1 and 2 in either direction). Returns true when the key goes on to the layer. */
static bool key_sub_select(bool forward) {
    if (g_state.dual_display) {
        if (forward) {
            keys_sub_next();
        } else {
            keys_sub_prev();
        }
        refresh_display();
        g_state.reading.valid = false; /* 0x15E52 sets the restart bit 0x0F60.4 */
        return false;
    }
    if (g_state.minmax_view == MINMAX_VIEW_LIVE) {
        return false;
    }
    g_state.minmax_view = (g_state.minmax_view == MINMAX_VIEW_MAX) ? MINMAX_VIEW_MIN : MINMAX_VIEW_MAX;
    display_annunciators();
    return true;
}

/* ---- Dispatcher: plain layer (0xE380) ---------------------------------------------------------------------- */

/* ACDC (0xE3F0-0xE41C): AC+DC volts from either voltage function, AC+DC current from either current
 * function; nothing from any other. */
static void plain_acdc(void) {
    switch (g_state.func) {
    case FUNC_DCV:
    case FUNC_ACV:
        select_function(FUNC_ACDCV);
        break;
    case FUNC_DCA:
    case FUNC_ACA:
        select_function(FUNC_ACDCI);
        break;
    default:
        break;
    }
}

/* MAX/MIN (0xE492-0xE4CA). From the live view it starts the extremes and shows MAX; from MAX or MIN it returns
 * to the live view. The dual display ends either way. The original raises RAM 0x0F60 bit 7 here and the
 * tracker (0xAF34) seeds both extremes from the next pass; meas_minmax_reset() is that flag. */
static void plain_maxmin(void) {
    if (g_state.minmax_view == MINMAX_VIEW_LIVE) {
        g_state.minmax_view = MINMAX_VIEW_MAX;
        meas_minmax_reset();
        g_state.minmax.active = true;
    } else {
        g_state.minmax_view = MINMAX_VIEW_LIVE;
        g_state.minmax.active = false;
    }
    g_state.dual_display = false;
    refresh_display();
}

/* REL (0xE434-0xE490), not in continuity or diode: the first press takes the last pre-REL reading (RAM
 * 0x0EAD, reading.raw) as the reference of the function, the next releases it. The original then waits 200 ms
 * with the keyboard dead; that busy-wait is not reproduced (ORIGINAL_BUGS 93). */
static void plain_rel(void) {
    rel_t *rel = &g_state.rel[g_state.func];

    if (g_state.func == FUNC_CONT || g_state.func == FUNC_DIODE) {
        return;
    }
    if (rel->active) {
        rel->active = false;
    } else {
        rel->active = true;
        rel->reference = g_state.reading.raw;
    }
    display_annunciators();
}

static void dispatch_plain(uint8_t key) {
    switch (key) {
    case KEY_DCV:
        select_function(FUNC_DCV);
        break;
    case KEY_ACV:
        select_function(FUNC_ACV);
        break;
    case KEY_OHM:
        select_function(FUNC_OHM);
        break;
    case KEY_FREQ:
        select_function(FUNC_FREQ);
        break;
    case KEY_ACDC:
        plain_acdc();
        break;
    case KEY_DIODE:
        g_state.range[FUNC_DIODE].manual = true;
        select_function(FUNC_DIODE);
        break;
    case KEY_MAXMIN:
        plain_maxmin();
        break;
    case KEY_REL:
        plain_rel();
        break;
    default:
        break;
    }
}

/* ---- Dispatcher: SHIFT layer (0xDF00) ---------------------------------------------------------------------- */

/* SHIFT+ACDC is dB and SHIFT+DIODE is dBm (0xDFAC, 0xE0BC), for the voltage functions and AC+DC volts only,
 * and not while a menu is open (RAM 0x0F61 bits 6 and 7). Pressing it while the unit is on turns it off and
 * restores the REL reference. Otherwise it opens the reference editor; the unit turns on when the editor
 * reports a confirmed ENTER (keys_ref_confirmed), and a cancelled editor leaves the unit as it was. */
static void shifted_unit(unit_mode_t mode) {
    const func_t f = g_state.func;
    const bool on = (mode == UNIT_DB) ? g_state.math.db_on : g_state.math.dbm_on;

    if (g_state.menu_mode || !(f == FUNC_DCV || f == FUNC_ACV || f == FUNC_ACDCV)) {
        return;
    }
    if (on) {
        rel_t *rel = &g_state.rel[f];

        g_state.math.db_on = false;
        g_state.math.dbm_on = false;
        if (rel->active) {
            rel->reference = rel->raw_reference;
        }
        reapply_function();
        return;
    }
    drop_secondary_views();
    if (mode == UNIT_DB) {
        pending_ref = REF_DB;
        menu_edit_db_ref(); /* original 0x5B5C */
    } else {
        pending_ref = REF_DBM;
        menu_edit_dbm_ref(); /* original 0x60BA */
    }
}

/* SHIFT+REL (0xE1E2-0xE246) is percent: off if on, otherwise the PERC REF editor, and on when it is confirmed.
 * The original also presets the MATH menu cursor (RAM 0x0131, 0x0E24); menu.h has no way to set it. */
static void shifted_percent(void) {
    if (g_state.math.percent_on) {
        g_state.math.percent_on = false;
        reapply_function();
        return;
    }
    drop_secondary_views();
    pending_ref = REF_PERCENT;
    menu_edit_percent_ref(); /* original 0x63A4 */
}

/* SHIFT+MAXMIN (0xE248-0xE2AA) is the limit test. Turning it off silences the tone a HI or LO result started
 * (0xE26E). The original skips that when the result was IN (RAM 0x0132 = 1) because the test itself turns the
 * tone off for IN (0xB3C2), so there is nothing to silence. */
static void shifted_limit(void) {
    g_state.math.limit_on = !g_state.math.limit_on;
    if (!g_state.math.limit_on) {
        buzzer_set(false);
    }
    reapply_function();
}

/* SHIFT+TRIG (0xE2AC-0xE2E0) is HOLD. Every key clears HOLD on entry (0xDB86), so the toggle that the original
 * performs here always ends with HOLD on. In manual-trigger mode TRIG never gets here: 0xDC0A takes it first
 * (ORIGINAL_BUGS 81: confirmed by that trace, HOLD cannot be set from the panel in that mode). */
static void shifted_hold(void) {
    meas_hold_arm();
    display_annunciators();
}

static void dispatch_shifted(uint8_t key) {
    g_state.shift_active = false;
    display_annunciators();

    switch (key) {
    case KEY_DCV:
        select_function(FUNC_DCA);
        break;
    case KEY_ACV:
        select_function(FUNC_ACA);
        break;
    case KEY_OHM:
        g_state.range[FUNC_CONT].manual = true;
        g_state.range[FUNC_CONT].index = CONTINUITY_KEY_RANGE;
        select_function(FUNC_CONT);
        break;
    case KEY_FREQ:
        select_function(FUNC_PERIOD);
        break;
    case KEY_ACDC:
        shifted_unit(UNIT_DB);
        break;
    case KEY_DIODE:
        shifted_unit(UNIT_DBM);
        break;
    case KEY_MENU:
        menu_enter();
        break;
    case KEY_MAXMIN:
        shifted_limit();
        break;
    case KEY_TRIG:
        shifted_hold();
        break;
    case KEY_REL:
        shifted_percent();
        break;
    default:
        break;
    }
}

/* The reference editor finished with a confirmed ENTER. The menu calls this; a cancelled editor does not. */
void keys_ref_confirmed(void) {
    const ref_edit_t which = pending_ref;

    pending_ref = REF_NONE;
    switch (which) {
    case REF_PERCENT:
        g_state.math.percent_on = true;
        break;
    case REF_DB:
        enter_unit(UNIT_DB);
        break;
    case REF_DBM:
        enter_unit(UNIT_DBM);
        break;
    case REF_NONE:
        return;
    }
    reapply_function();
}

/* ---- Dispatcher entry (0xDB4E) ----------------------------------------------------------------------------- */

/* The one key that works in remote is SHIFT, as LOCAL (0xDBA6-0xDBBC), unless the local lockout is set. It
 * leaves remote (0xFAD0) and also turns the display back on (0xFAE2 sets RAM 0x0F61 bit 2), which is how the
 * panel comes back after :DISPlay OFF. */
static void key_in_remote(uint8_t key) {
    if (!g_state.local_lockout && key == KEY_SHIFT) {
        g_state.display_enabled = true;
        remote_go_local();
    }
}

void keys_dispatch(uint8_t key) {
    if (key == KEY_NONE) {
        return;
    }
    if (menu_active()) {
        menu_key(key);
        return;
    }
    if (g_state.hold_active) {
        meas_hold_release();
        display_annunciators(); /* ORIGINAL_BUGS 80: the original left the HOLD lamp lit */
    }
    if (g_state.remote) {
        key_in_remote(key);
        return;
    }
    if (key == KEY_SHIFT) {
        g_state.shift_active = !g_state.shift_active;
        display_annunciators();
        return;
    }
    if (key == KEY_TRIG && g_state.trigger == TRIG_MAN) {
        g_state.manual_trigger_req = true;
        return;
    }

    switch (key) {
    case KEY_RATE:
        key_rate();
        return;
    case KEY_AUTO:
        if (!key_auto()) {
            return;
        }
        break;
    case KEY_RANGE_UP:
        if (!key_range_up()) {
            return;
        }
        break;
    case KEY_RANGE_DOWN:
        if (!key_range_down()) {
            return;
        }
        break;
    case KEY_PREV:
        if (!key_sub_select(false)) {
            return;
        }
        break;
    case KEY_NEXT:
        if (!key_sub_select(true)) {
            return;
        }
        break;
    default:
        break;
    }

    if (g_state.shift_active) {
        dispatch_shifted(key);
    } else {
        dispatch_plain(key);
    }
}
