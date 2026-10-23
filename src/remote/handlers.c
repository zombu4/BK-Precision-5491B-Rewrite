/* handlers.c -- the leaf command handlers: FUNCtion, FUNC2, CALCulate, HOLD, CURRent, VOLTage, RESistance,
 * FREQuency/PERiod, DISPlay, UNIT, TRIGger, FETCh and SYSTem.
 *
 * As in the original, a handler runs once per ':' level of the header; scpi_depth() says
 * which level it is at and the handler reads the keyword and argument of that level itself. The
 * ';'-without-colon rule (a sibling statement at the same level) falls out of that.
 *
 * Every handler validates first and changes state last. A bad argument reports an error and
 * returns; the original stored a default (top range, 0.1 PLC, out-of-range hold values) after the error.
 *
 * The :SCAL tree and the executor are in exec.c; the :THID and * commands are in serv/ and the menus in ui/.
 * :SYSTem:INFO? is the one command here with no original.
 */
#include <avr/pgmspace.h>
#include <float.h>
#include "../fmathf.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../cal/cal.h"
#include "../eeprom_map.h"
#include "../fmt/fmt.h"
#include "../hal/eeprom.h"
#include "../main.h"
#include "../meas/meas.h"
#include "../serv/serv.h"
#include "../state.h"
#include "../tables.h"
#include "../ui/display.h"
#include "../ui/menu.h"
#include "../ui/settings.h"
#include "../version.h"
#include "scpi.h"

#define KW(text) scpi_keyword_match(PSTR(text))

/* Argument limits of the original. */
#define REF_LIMIT_DCV 1010.0f
#define REF_LIMIT_ACV 757.5f
#define REF_LIMIT_CURRENT 20.0f
#define REF_LIMIT_OHM 5.0e7f
#define REF_MAX_FREQUENCY 1.5e7f
#define MATH_LIMIT 1.0e8f
#define HOLD_WINDOW_MIN_PERCENT 0.009999f
#define HOLD_WINDOW_MAX_PERCENT 20.0f
#define HOLD_COUNT_MIN 2
#define HOLD_COUNT_MAX 100
#define DB_REF_MIN 1.0e-4f
#define DB_REF_MAX 1000.0f
#define IMPEDANCE_MIN 1
#define IMPEDANCE_MAX 9999
#define DBM_REFERENCE_WATT 0.001f

/* ---- :UNIT channels ---------------------------------------------------------------------------------------
 *
 * The unit and the dB / dBm references of a voltage channel live in g_state.unit_channel[]. Index 0 is the DC
 * voltage channel (original unit byte RAM 0x0133, references 0x0112 and 0x0118) and index 1 the AC voltage
 * channel (0x0134, 0x010E and 0x0116), the order of the :UNIT:VOLTage:DC / :AC nodes.
 *
 * These references are NOT the storage of the MATH menu's dB REF and dBm REF. The menu edits and the dB/dBm
 * conversion read the working pair g_state.math.db_ref / .dbm_ref (RAM 0x010A and 0x0108, flash 0x05B5C /
 * 0x060BA and 0x0B0B8); the per-channel values are a backing store that is copied into the working pair when
 * the channel is edited while its function is active (0x07864-0x07878, 0x0791E) and when the function
 * changes. A MATH menu edit therefore does not reach the channel store, as in the original. */
typedef enum { CHANNEL_DC = 0, CHANNEL_AC = 1 } channel_t;

static unit_channel_t *channel_state(channel_t channel) {
    return &g_state.unit_channel[channel];
}

static bool token_is_colon(void) {
    return scpi_token_type() == SCPI_TOK_SEP && scpi_token()[0] == ':';
}

/* ---- FUNCtion ------------------------------------------------------------------------------------------ */

static func_t unit_function(channel_t channel) {
    return channel == CHANNEL_AC ? FUNC_ACV : FUNC_DCV;
}

/* Makes the unit of the function's channel the working dB/dBm state. Only voltage functions have units. */
static void unit_apply_for_function(func_t f) {
    unit_mode_t mode = UNIT_NATIVE;
    const unit_channel_t *c = NULL;

    if (f == FUNC_DCV) {
        c = channel_state(CHANNEL_DC);
    } else if (f == FUNC_ACV) {
        c = channel_state(CHANNEL_AC);
    }
    if (c != NULL) {
        mode = c->mode;
        if (mode == UNIT_DB) {
            g_state.math.db_ref = c->db_ref;
        } else if (mode == UNIT_DBM) {
            g_state.math.dbm_ref = c->dbm_ref;
        }
    }
    g_state.math.db_on = (mode == UNIT_DB);
    g_state.math.dbm_on = (mode == UNIT_DBM);
}

static void function_refresh(void) {
    display_annunciators();
    display_update();
}

/* Re-applies the active function. The second display is switched off, as a function change always did. */
void scpi_function_apply(void) {
    g_state.dual_display = false;
    meas_set_function(g_state.func);
    function_refresh();
}

/* Re-applies the active function after a second-display change; only meaningful while it is on. */
void scpi_func2_apply(void) {
    if (g_state.dual_display) {
        meas_set_function(g_state.func);
        function_refresh();
    }
}

static void function_select(func_t f) {
    meas_set_function(f);
    g_state.func = f;
    unit_apply_for_function(f);
    g_state.dual_display = false;
    function_refresh();
}

/* The two functions the original's name table does not hold. Its lookup runs past the nine records and
 * answers an empty line for AC+DC volts (the sentinel record at flash 0x07EA, 0x0787 + 9 * 11 bytes) and
 * "volt:ac" for AC+DC current (the first record of the next list, 0x07F5); PROTOCOL_CHANGES G.2 #30 plans the
 * correct name for all eleven functions. The spellings are the ones :FUNCtion accepts, so a client can send back what it read. */
static const char name_acdc_volt[] PROGMEM = "volt:acdc";
static const char name_acdc_curr[] PROGMEM = "curr:acdc";

static void reply_function_name(uint8_t index) {
    char text[FUNC_NAME_LEN + 1u];

    text[0] = '\0';
    if (index < func_name_count) {
        name_record_t record;

        memcpy_P(&record, &func_name[index], sizeof record);
        memcpy(text, record.name, FUNC_NAME_LEN);
        text[FUNC_NAME_LEN] = '\0';
    } else if (index == (uint8_t)FUNC_ACDCV) {
        strcpy_P(text, name_acdc_volt);
    } else if (index == (uint8_t)FUNC_ACDCI) {
        strcpy_P(text, name_acdc_curr);
    }
    remote_reply(text);
}

/* After VOLTage or CURRent: ":DC", ":AC", ":ACDC", or nothing at all (DC). */
static bool function_variant(func_t dc, func_t ac, func_t acdc, func_t *out) {
    if (scpi_next_is_colon()) {
        if (scpi_next_token() != SCPI_TOK_WORD) {
            scpi_fail_bad_command();
        } else if (KW("dc")) {
            *out = dc;
            return true;
        } else if (KW("ac")) {
            *out = ac;
            return true;
        } else if (KW("acdc")) {
            *out = acdc;
            return true;
        } else {
            scpi_fail_parameter();
        }
        return false;
    }
    if (scpi_token_is_end() || scpi_token()[0] == ';') {
        scpi_unget_token();
        *out = dc;
        return true;
    }
    scpi_fail_bad_command();
    return false;
}

/* :FUNCtion <VOLTage[:DC|:AC|:ACDC]|CURRent[:DC|:AC|:ACDC]|RESistance|FREQuency|PERiod|DIODe|CONTinuity>
 * :FUNCtion? */
void scpi_cmd_function(void) {
    func_t f;

    if (scpi_depth() > 1u) {
        scpi_fail_bad_command();
        return;
    }
    if (scpi_next_token() != SCPI_TOK_WORD) {
        if (scpi_token_is_query()) {
            reply_function_name((uint8_t)g_state.func);
        } else {
            scpi_fail_parameter();
        }
        return;
    }
    if (KW("volt") || KW("voltage")) {
        if (!function_variant(FUNC_DCV, FUNC_ACV, FUNC_ACDCV, &f)) {
            return;
        }
    } else if (KW("curr") || KW("current")) {
        if (!function_variant(FUNC_DCA, FUNC_ACA, FUNC_ACDCI, &f)) {
            return;
        }
    } else if (KW("res") || KW("resistance")) {
        f = FUNC_OHM;
    } else if (KW("per") || KW("period")) {
        f = FUNC_PERIOD;
    } else if (KW("freq") || KW("frequency")) {
        f = FUNC_FREQ;
    } else if (KW("diod") || KW("diode")) {
        f = FUNC_DIODE;
    } else if (KW("cont") || KW("continuity")) {
        f = FUNC_CONT;
    } else {
        scpi_fail_parameter();
        return;
    }
    function_select(f);
}

/* ---- FUNC2 --------------------------------------------------------------------------------------------- */

/* What the second display can show. The names are the spellings the query returns. */
typedef enum { SD_VOLT_DC, SD_VOLT_AC, SD_CURR_DC, SD_CURR_AC, SD_FREQ, SD_DB, SD_DBM } second_t;

/* Per primary function: the second-display choices in selector order (original RAM 0x0126-0x012C). */
static const uint8_t second_dcv[] PROGMEM = {SD_VOLT_AC, SD_FREQ, SD_DB, SD_DBM};
static const uint8_t second_dca[] PROGMEM = {SD_CURR_AC, SD_FREQ};
static const uint8_t second_acv[] PROGMEM = {SD_VOLT_DC, SD_FREQ, SD_DB, SD_DBM};
static const uint8_t second_aca[] PROGMEM = {SD_CURR_DC, SD_FREQ};
static const uint8_t second_acdcv[] PROGMEM = {SD_FREQ, SD_DB, SD_DBM, SD_VOLT_DC, SD_VOLT_AC};
static const uint8_t second_acdci[] PROGMEM = {SD_FREQ, SD_CURR_AC, SD_CURR_DC};
static const uint8_t second_freq[] PROGMEM = {SD_VOLT_AC, SD_CURR_AC};

static const uint8_t *second_choices(func_t f, uint8_t *count) {
    switch (f) {
    case FUNC_DCV:
        *count = sizeof second_dcv;
        return second_dcv;
    case FUNC_DCA:
        *count = sizeof second_dca;
        return second_dca;
    case FUNC_ACV:
        *count = sizeof second_acv;
        return second_acv;
    case FUNC_ACA:
        *count = sizeof second_aca;
        return second_aca;
    case FUNC_ACDCV:
        *count = sizeof second_acdcv;
        return second_acdcv;
    case FUNC_ACDCI:
        *count = sizeof second_acdci;
        return second_acdci;
    case FUNC_FREQ:
    case FUNC_PERIOD:
        *count = sizeof second_freq;
        return second_freq;
    default:
        *count = 0u;
        return NULL;
    }
}

/* The selector value that shows `wanted` while the primary function is f, or false when it cannot. */
static bool second_selector(second_t wanted, func_t f, uint8_t *selector) {
    uint8_t count;
    const uint8_t *choices = second_choices(f, &count);

    for (uint8_t i = 0u; choices != NULL && i < count; i++) {
        if (pgm_read_byte(&choices[i]) == (uint8_t)wanted) {
            *selector = i;
            return true;
        }
    }
    return false;
}

static void reply_second_name(second_t name) {
    switch (name) {
    case SD_VOLT_DC:
        reply_function_name((uint8_t)FUNC_DCV);
        break;
    case SD_VOLT_AC:
        reply_function_name((uint8_t)FUNC_ACV);
        break;
    case SD_CURR_DC:
        reply_function_name((uint8_t)FUNC_DCA);
        break;
    case SD_CURR_AC:
        reply_function_name((uint8_t)FUNC_ACA);
        break;
    case SD_FREQ:
        reply_function_name((uint8_t)FUNC_FREQ);
        break;
    case SD_DB:
        remote_reply_P(PSTR("db"));
        break;
    default:
        remote_reply_P(PSTR("dbm"));
        break;
    }
}

/* Reads the part after VOLTage or CURRent of a FUNC2 request: ":DC", ":AC", or nothing (DC). */
static bool second_variant(second_t dc, second_t ac, second_t *out) {
    if (scpi_next_is_colon()) {
        if (scpi_next_token() != SCPI_TOK_WORD) {
            scpi_fail_bad_command();
        } else if (KW("dc")) {
            *out = dc;
            return true;
        } else if (KW("ac")) {
            *out = ac;
            return true;
        } else {
            scpi_fail_parameter();
        }
        return false;
    }
    if (scpi_token_is_end() || scpi_token()[0] == ';') {
        scpi_unget_token();
        *out = dc;
        return true;
    }
    scpi_fail_bad_command();
    return false;
}

/* :FUNC2 <VOLTage[:DC|:AC]|CURRent[:DC|:AC]|FREQuency|DB|DBM>, :FUNC2:STATe ON|OFF, :FUNC2? */
void scpi_cmd_function2(void) {
    second_t wanted;
    uint8_t selector;
    func_t f = g_state.func;

    if (scpi_depth() > 1u) {
        scpi_fail_bad_command();
        return;
    }
    if (scpi_next_token() == SCPI_TOK_WORD) {
        if (KW("volt") || KW("voltage")) {
            if (!second_variant(SD_VOLT_DC, SD_VOLT_AC, &wanted)) {
                return;
            }
        } else if (KW("curr") || KW("current")) {
            if (!second_variant(SD_CURR_DC, SD_CURR_AC, &wanted)) {
                return;
            }
        } else if (KW("freq") || KW("frequency")) {
            wanted = SD_FREQ;
        } else if (KW("db")) {
            wanted = SD_DB;
        } else if (KW("dbm")) {
            wanted = SD_DBM;
        } else {
            scpi_fail_parameter();
            return;
        }
        /* The original mapped the DC request on AC+DC current to the AC entry; the list holds both
         * (second_acdci), and PROTOCOL_CHANGES section C plans that CURR:DC selects the DC one. */
        if (!second_selector(wanted, f, &selector)) {
            scpi_fail_parameter();
            return;
        }
        g_state.second_display[f] = selector;
        if (f == FUNC_FREQ || f == FUNC_PERIOD) {
            g_state.second_display[FUNC_FREQ] = selector;
            g_state.second_display[FUNC_PERIOD] = selector;
        }
        scpi_func2_apply();
        return;
    }
    if (scpi_token_is_query()) {
        uint8_t count;
        const uint8_t *choices = second_choices(f, &count);
        uint8_t sel = g_state.second_display[f];

        if (choices == NULL) {
            scpi_fail_param_missing();
        } else if (sel < count) {
            reply_second_name((second_t)pgm_read_byte(&choices[sel]));
        } else {
            remote_reply_P(PSTR(""));
        }
        return;
    }
    if (token_is_colon()) {
        scpi_next_token();
        if (!KW("stat") && !KW("state")) {
            scpi_fail_bad_command();
            return;
        }
        switch (scpi_arg_bool()) {
        case SCPI_BOOL_ON:
            g_state.dual_display = true;
            meas_set_function(g_state.func);
            function_refresh();
            break;
        case SCPI_BOOL_OFF:
            scpi_function_apply();
            break;
        default:
            scpi_fail_parameter();
            break;
        }
        return;
    }
    scpi_fail_bad_command();
}

/* ---- Reference, state and acquire shared by the measurement families ----------------------------------- */

typedef enum { FAMILY_VOLT, FAMILY_CURR, FAMILY_RES } family_t;

/* Which keyword a leaf word was reached under. The original accepted AUTO, UPPer, STATe and ACQuire under
 * either parent (`VOLT:DC:RANGe:STATe ON` switched REL on); PROTOCOL_CHANGES G.2 #26 plans that AUTO and
 * UPPer belong to RANGe, STATe and ACQuire to REFerence, and FREQuency and PERiod likewise (VOLTage to
 * THReshold, STATe and ACQuire to REFerence). The handler runs once per ':' level, so the parent and the
 * function chosen at the node level are carried from one call to the next in these two statics. They are
 * rewritten by every node, RANGe and REFerence word and are only read below a level that wrote them, so a
 * stale value from an earlier statement or line is never used: the node level is mandatory for VOLT and CURR
 * (family_node reports anything else) and the parent level for the leaves. */
typedef enum { PARENT_RANGE, PARENT_REFERENCE, PARENT_THRESHOLD } parent_t;

static func_t node_function; /* the function the VOLT/CURR node selected */
static parent_t leaf_parent;

/* The bound on |reference| for function f, if it has one. */
static bool family_ref_limit(family_t family, func_t f, float *limit) {
    switch (family) {
    case FAMILY_VOLT:
        if (f == FUNC_DCV) {
            *limit = REF_LIMIT_DCV;
            return true;
        }
        if (f == FUNC_ACV) {
            *limit = REF_LIMIT_ACV;
            return true;
        }
        return false;
    case FAMILY_CURR:
        if (f == FUNC_DCA || f == FUNC_ACA) {
            *limit = REF_LIMIT_CURRENT;
            return true;
        }
        return false;
    default:
        *limit = REF_LIMIT_OHM;
        return true;
    }
}

static void ref_value_set(func_t f, float value) {
    g_state.rel[f].reference = value;
}

/* :REFerence <value> for the V, I and R families. */
static void ref_value_arg(family_t family, func_t f) {
    float value;
    float limit;

    if (!scpi_arg_float(&value)) {
        return;
    }
    if (family_ref_limit(family, f, &limit) && fabsf(value) > limit) {
        scpi_fail_data();
        return;
    }
    ref_value_set(f, value);
}

/* :REFerence:STATe ON|OFF|? -- REL on or off for function f. */
static void ref_state_arg(func_t f) {
    rel_t *rel = &g_state.rel[f];

    switch (scpi_arg_bool()) {
    case SCPI_BOOL_ON:
        rel->active = true;
        display_annunciators();
        break;
    case SCPI_BOOL_OFF:
        rel->active = false;
        display_annunciators();
        break;
    case SCPI_BOOL_QUERY:
        scpi_reply_bool(rel->active);
        break;
    default:
        scpi_fail_parameter();
        break;
    }
}

/* :REFerence:ACQuire -- the last reading, before REL and the percent mode were applied (RAM 0x0EAD in the
 * original), becomes the reference. The last reading is a reading of the ACTIVE function, so naming any
 * other function here is refused: it would store a quantity of the wrong kind. */
static void ref_acquire(func_t f) {
    if (f != g_state.func) {
        scpi_fail_cannot_execute();
        return;
    }
    ref_value_set(f, g_state.reading.raw);
}

/* ---- VOLTage, CURRent, RESistance ---------------------------------------------------------------------- */

/* The node level of VOLT and CURR (:DC, :AC, :ACDC) selects the function the commands below it edit
 * (PROTOCOL_CHANGES G.2 #25). The original validated the word and then ignored it, so `VOLT:DC:RANGe 5` in a
 * current function changed the current range. RES has no node and always means the ohms function. */
static void family_node(family_t family) {
    const bool volt = (family == FAMILY_VOLT);

    if (scpi_next_token() != SCPI_TOK_WORD) {
        scpi_fail_bad_command();
    } else if (KW("dc")) {
        node_function = volt ? FUNC_DCV : FUNC_DCA;
    } else if (KW("ac")) {
        node_function = volt ? FUNC_ACV : FUNC_ACA;
    } else if (KW("acdc")) {
        node_function = volt ? FUNC_ACDCV : FUNC_ACDCI;
    } else {
        scpi_fail_bad_command();
    }
}

/* :RANGe <range> | ? | :... */
static void family_range_arg(func_t f) {
    uint8_t type = scpi_next_token();

    if (type == SCPI_TOK_SEP) {
        if (scpi_token()[0] == ':') {
            scpi_unget_token();
        } else if (scpi_token_is_query()) {
            scpi_range_reply(f);
        } else {
            scpi_fail_bad_command();
        }
    } else if (type == SCPI_TOK_NUMBER || type == SCPI_TOK_WORD) {
        scpi_range_set_from_token(f);
    } else {
        scpi_fail_bad_command();
    }
}

/* :REFerence <value> | ? | :... */
static void family_ref_arg(family_t family, func_t f) {
    uint8_t type = scpi_next_token();

    if (type == SCPI_TOK_NUMBER) {
        ref_value_arg(family, f);
    } else if (token_is_colon()) {
        scpi_unget_token();
    } else if (scpi_token_is_query()) {
        scpi_reply_reference(f, SCPI_DECIMALS_READING);
    } else {
        scpi_fail_bad_command();
    }
}

static void family_main_level(family_t family, func_t f) {
    scpi_next_token();
    if (KW("nplc") || KW("nplcycles")) {
        scpi_set_nplc();
    } else if (KW("rang") || KW("range")) {
        leaf_parent = PARENT_RANGE;
        family_range_arg(f);
    } else if (KW("ref") || KW("reference")) {
        leaf_parent = PARENT_REFERENCE;
        family_ref_arg(family, f);
    } else {
        scpi_fail_bad_command();
    }
}

/* The words below RANGe (UPPer, AUTO) and REFerence (STATe, ACQuire). A word that exists but sits under the
 * other parent is a syntax error; a word that exists nowhere is a bad command. */
static void leaf_level(func_t f) {
    const bool under_range = (leaf_parent == PARENT_RANGE);

    scpi_next_token();
    if (KW("upp") || KW("upper")) {
        if (under_range) {
            scpi_range_upper_arg(f);
        } else {
            scpi_fail_syntax();
        }
    } else if (KW("auto")) {
        if (under_range) {
            scpi_range_auto_arg(f);
        } else {
            scpi_fail_syntax();
        }
    } else if (KW("stat") || KW("state")) {
        if (under_range) {
            scpi_fail_syntax();
        } else {
            ref_state_arg(f);
        }
    } else if (KW("acq") || KW("acquire")) {
        if (under_range) {
            scpi_fail_syntax();
        } else {
            ref_acquire(f);
        }
    } else {
        scpi_fail_bad_command();
    }
}

static void family_command(family_t family) {
    uint8_t depth = scpi_depth();
    uint8_t node = (family == FAMILY_RES) ? (uint8_t)0u : (uint8_t)1u;
    func_t f = (family == FAMILY_RES) ? FUNC_OHM : node_function;

    if (depth == 1u) {
        return;
    }
    if (node != 0u && depth == 2u) {
        family_node(family);
    } else if (depth == 2u + node) {
        family_main_level(family, f);
    } else if (depth == 3u + node) {
        leaf_level(f);
    } else {
        scpi_fail_bad_command();
    }
}

void scpi_cmd_voltage(void) {
    family_command(FAMILY_VOLT);
}
void scpi_cmd_current(void) {
    family_command(FAMILY_CURR);
}
void scpi_cmd_resistance(void) {
    family_command(FAMILY_RES);
}

/* ---- FREQuency and PERiod ------------------------------------------------------------------------------ */

/* Depth 2: THReshold (a ':' must follow) or REFerence. The root selects the function: FREQuency edits the
 * frequency function's selections and PERiod the period function's (PROTOCOL_CHANGES G.2 #25). */
static void frequency_level2(func_t f) {
    scpi_next_token();
    if (KW("thr") || KW("threshold")) {
        leaf_parent = PARENT_THRESHOLD;
        if (scpi_next_is_colon()) {
            scpi_unget_token();
        } else {
            scpi_fail_syntax();
        }
    } else if (KW("ref") || KW("reference")) {
        leaf_parent = PARENT_REFERENCE;
        if (scpi_next_is_colon()) {
            scpi_unget_token();
        } else if (scpi_token_type() == SCPI_TOK_NUMBER) {
            float value;

            if (!scpi_arg_float(&value)) {
                return;
            }
            if (value > REF_MAX_FREQUENCY || value < 0.0f) {
                scpi_fail_data();
                return;
            }
            ref_value_set(f, value);
        } else if (scpi_token_is_query()) {
            scpi_reply_reference(f, SCPI_DECIMALS_STANDARD);
        } else {
            scpi_fail_bad_command();
        }
    } else {
        scpi_fail_bad_command();
    }
}

/* THReshold:VOLTage:RANGe <number>|DEF|MIN|MAX|?|:AUTO ON|OFF|?  -- the same spellings as the other
 * families (PROTOCOL_CHANGES G.2 #28; the original took numbers only here). */
static void frequency_threshold_range(func_t f) {
    uint8_t type;

    scpi_next_token();
    if (!KW("rang") && !KW("range")) {
        scpi_fail_bad_command();
        return;
    }
    type = scpi_next_token();
    if (type == SCPI_TOK_NUMBER || type == SCPI_TOK_WORD) {
        scpi_range_set_from_token(f);
    } else if (type != SCPI_TOK_SEP) {
        scpi_fail_bad_command();
    } else if (scpi_token_is_query()) {
        scpi_range_reply(f);
    } else if (token_is_colon()) {
        scpi_next_token();
        if (KW("auto")) {
            scpi_range_auto_arg(f);
        } else {
            scpi_fail_bad_command();
        }
    } else {
        scpi_fail_parameter();
    }
}

/* Depth 3: VOLTage:RANGe... below THReshold, STATe and ACQuire below REFerence. */
static void frequency_level3(func_t f) {
    const bool under_threshold = (leaf_parent == PARENT_THRESHOLD);

    scpi_next_token();
    if (KW("volt") || KW("voltage")) {
        if (!under_threshold) {
            scpi_fail_syntax();
        } else if (!scpi_next_is_colon()) {
            scpi_fail_syntax();
        } else {
            frequency_threshold_range(f);
        }
    } else if (KW("stat") || KW("state")) {
        if (under_threshold) {
            scpi_fail_syntax();
        } else {
            ref_state_arg(f);
        }
    } else if (KW("acq") || KW("acquire")) {
        if (under_threshold) {
            scpi_fail_syntax();
        } else {
            ref_acquire(f);
        }
    } else {
        scpi_fail_bad_command();
    }
}

void scpi_cmd_frequency_period(func_t f) {
    switch (scpi_depth()) {
    case 1u:
        break;
    case 2u:
        frequency_level2(f);
        break;
    case 3u:
        frequency_level3(f);
        break;
    default:
        scpi_fail_bad_command();
        break;
    }
}

/* ---- HOLD ---------------------------------------------------------------------------------------------- */

/* :HOLD:WINDow <percent 0.01..20>|? -- g_state.hold_window is the window as a fraction. */
static void hold_window_arg(void) {
    if (scpi_next_token() == SCPI_TOK_NUMBER) {
        float percent;

        if (!scpi_arg_float(&percent)) {
            return;
        }
        if (percent < HOLD_WINDOW_MIN_PERCENT || percent > HOLD_WINDOW_MAX_PERCENT) {
            scpi_fail_data();
            return;
        }
        g_state.hold_window = percent / 100.0f;
    } else if (scpi_token_is_query()) {
        scpi_reply_hold_window();
    } else {
        scpi_fail_bad_command();
    }
}

/* :HOLD:COUNt <2..100>|? */
static void hold_count_arg(void) {
    if (scpi_next_token() == SCPI_TOK_NUMBER) {
        int32_t count;

        if (!scpi_arg_long(&count)) {
            return;
        }
        if (count < HOLD_COUNT_MIN || count > HOLD_COUNT_MAX) {
            scpi_fail_data();
            return;
        }
        g_state.hold_count = (uint8_t)count;
    } else if (scpi_token_is_query()) {
        scpi_reply_hold_count();
    } else {
        scpi_fail_bad_command();
    }
}

/* :HOLD:STATe ON|OFF|? */
static void hold_state_arg(void) {
    switch (scpi_arg_bool()) {
    case SCPI_BOOL_ON:
        meas_hold_arm();
        display_annunciators();
        break;
    case SCPI_BOOL_OFF:
        meas_hold_release();
        display_annunciators();
        break;
    case SCPI_BOOL_QUERY:
        scpi_reply_bool(g_state.hold_active);
        break;
    default:
        scpi_fail_bad_command();
        break;
    }
}

void scpi_cmd_hold(void) {
    uint8_t depth = scpi_depth();

    if (depth == 1u) {
        return;
    }
    if (depth != 2u || scpi_next_token() != SCPI_TOK_WORD) {
        scpi_fail_bad_command();
    } else if (KW("wind") || KW("window")) {
        hold_window_arg();
    } else if (KW("coun") || KW("count")) {
        hold_count_arg();
    } else if (KW("stat") || KW("state")) {
        hold_state_arg();
    } else {
        scpi_fail_bad_command();
    }
}

/* ---- DISPlay, TRIGger, FETCh, SYSTem ------------------------------------------------------------------- */

/* :DISPlay:ENABle ON|OFF|? */
void scpi_cmd_display(void) {
    uint8_t depth = scpi_depth();

    if (depth == 1u) {
        return;
    }
    if (depth != 2u) {
        scpi_fail_bad_command();
        return;
    }
    scpi_next_token();
    if (KW("boot")) {
        /* NEW. The power-up splash text. Open, not protected: it changes nothing that
         * affects a measurement, and it is the one setting whose effect the user can see for themselves.
         *
         * A text with a character the display cannot show, or one longer than the main display's cells,
         * is a parameter error rather than a quiet substitution -- see ui_boot_text_set(). */
        char text[EE_BOOT_MSG_LEN + 1u];

        if (scpi_next_token() == SCPI_TOK_STRING) {
            if (!ui_boot_text_set(scpi_token())) {
                scpi_fail_parameter();
            }
        } else if (scpi_token_is_query()) {
            ui_boot_text(text);
            scpi_reply_ram_string(text);
        } else {
            scpi_fail_bad_command();
        }
        return;
    }
    if (!KW("enab") && !KW("enable")) {
        scpi_fail_bad_command();
        return;
    }
    switch (scpi_arg_bool()) {
    case SCPI_BOOL_ON:
        g_state.display_enabled = true;
        display_update();
        break;
    case SCPI_BOOL_OFF:
        g_state.display_enabled = false;
        break;
    case SCPI_BOOL_QUERY:
        scpi_reply_bool(g_state.display_enabled);
        break;
    default:
        scpi_fail_bad_command();
        break;
    }
}

static void scpi_set_trigger_source(trig_t source) {
    g_state.trigger = source;
    display_annunciators();
}

/* :TRIGger:SOURce IMMediate|MANual|BUS, :TRIGger:SOURce? */
void scpi_cmd_trigger(void) {
    uint8_t depth = scpi_depth();

    if (depth == 1u) {
        return;
    }
    if (depth != 2u) {
        scpi_fail_bad_command();
        return;
    }
    scpi_next_token();
    if (!KW("sour") && !KW("source")) {
        scpi_fail_bad_command();
        return;
    }
    scpi_next_token();
    if (scpi_token_is_query()) {
        switch (g_state.trigger) {
        case TRIG_IMM:
            remote_reply_P(PSTR("IMM"));
            break;
        case TRIG_MAN:
            remote_reply_P(PSTR("MAN"));
            break;
        default:
            remote_reply_P(PSTR("BUS"));
            break;
        }
    } else if (KW("imm") || KW("immediate")) {
        scpi_set_trigger_source(TRIG_IMM);
    } else if (KW("bus")) {
        scpi_set_trigger_source(TRIG_BUS);
    } else if (KW("man") || KW("manual")) {
        scpi_set_trigger_source(TRIG_MAN);
    } else {
        scpi_fail_parameter();
    }
}

/* :FETCh? -- answered at the end of the next completed measurement (scpi_reply_reading). The original also
 * stepped to the next separator here, so the executor's own step then read a second one and `FETC?;<cmd>`
 * was a syntax error. This handler leaves the separator to the executor, which makes the chained form work
 * (PROTOCOL_CHANGES section C, planned row "FETCh?;<cmd>"). Every request gets its own reply (G.2 #40); a
 * request that finds the pending count full is refused instead of being dropped. */
void scpi_cmd_fetch(void) {
    scpi_next_token();
    if (!scpi_token_is_query()) {
        scpi_fail_bad_command();
    } else if (!scpi_fetch_request()) {
        scpi_fail_cannot_execute();
    }
}

/* ORIGINAL_BUGS #42, FIXED here, and the port had been reproducing the bug: in the vendor firmware
 * `:SYSTem:BEEPer` writes the KEY-CLICK setting (EEPROM 0x855), not the beeper that the SYS menu's BEEP STA
 * item controls -- so a client that turned "the beeper" off silenced the key click and left the error,
 * limit and hold beeps sounding, with no command to reach them at all.
 *
 * `:SYSTem:BEEPer` controls the BEEP STA beeper, which is what its name says
 * and what a client expects, and the new `:SYSTem:KCLick` controls the key click. Both are in
 * PROTOCOL_CHANGES section A. A failed store is reported and the setting rolled back, so a refused write
 * never leaves the RAM copy disagreeing with the EEPROM. */
static void setting_set(bool *field, bool on) {
    const bool previous = *field;

    *field = on;
    if (!settings_store()) {
        *field = previous;
        scpi_fail_cannot_execute();
    }
}

/* ---- :SYSTem:INFO? ---------------------------------------------------------------------------------------
 *
 * One line, deliberately not an extension of *IDN?, which clients parse on its four
 * fields. Seven comma separated fields:
 *
 *   <firmware version>,<serial>,<cal date>,<cal count>,<production date>,<power-on hours>,<power-up count>
 *
 * Sources. The version is the firmware constant that *IDN? reports. The serial is the identity serial at
 * EE_ID_SERIAL, the one *IDN? reports. The calibration date, calibration count and production date are the
 * :THID block (EE_THID_CAL_DATE, EE_THID_CAL_COUNT, EE_THID_PROD_DATE). The hours come from main.c. Text read
 * from the EEPROM stops at the first zero or erased byte, and a character outside printable ASCII is replaced
 * by '?' and a comma by '.', so a damaged field can never add or remove a field of the reply. A count that
 * reads as erased (0xFFFF) is 0.
 *
 * UNVERIFIED: that "cal date and count" means the :THID calibration pair rather than the :SCAL:DATE field
 * at EE_CAL_DATE and the :SCAL:SAVE counter at EE_SAVE_COUNTER. The :THID pair is the one the front-panel CAL
 * menu counts and dates; check which pair the app expects before relying on this field.
 * UNVERIFIED: the power-up count is read straight from EE_PWRUP_COUNT because main.h has no accessor for it;
 * main.c keeps it with the same erased-is-zero rule (powerup_count_bump). */
#define INFO_LINE_MAX 112u
#define INFO_FIELD_MAX 20u
#define INFO_ERASED_COUNT 0xFFFFu

typedef struct {
    char text[INFO_LINE_MAX];
    size_t length;
} info_line_t;

static void info_char(info_line_t *line, char c) {
    if (line->length < INFO_LINE_MAX - 1u) {
        line->text[line->length++] = c;
    }
}

static void info_text(info_line_t *line, const char *raw, uint8_t count) {
    for (uint8_t i = 0u; i < count && raw[i] != '\0' && (uint8_t)raw[i] != 0xFFu; i++) {
        char c = raw[i];

        if (c == ',') {
            c = '.';
        } else if (c < ' ' || c > '~') {
            c = '?';
        }
        info_char(line, c);
    }
}

static void info_eeprom_text(info_line_t *line, uint16_t address, uint8_t count) {
    char raw[INFO_FIELD_MAX];

    ee_read_block(address, raw, count);
    info_text(line, raw, count);
}

static void info_count(info_line_t *line, uint16_t value) {
    char digits[8];
    size_t length = fmt_int(digits, sizeof digits, (int32_t)value);

    if (length > sizeof digits) {
        length = sizeof digits;
    }
    for (size_t i = 0u; i < length; i++) {
        info_char(line, digits[i]);
    }
}

static uint16_t info_stored_count(uint16_t address) {
    const uint16_t value = ee_read_u16(address);

    return (value == INFO_ERASED_COUNT) ? 0u : value;
}

static void reply_info(void) {
    static const char version_flash[] PROGMEM = FW_VERSION;
    char version[INFO_FIELD_MAX];
    info_line_t line = {.length = 0u};

    strncpy_P(version, version_flash, sizeof version - 1u);
    version[sizeof version - 1u] = '\0';

    info_text(&line, version, (uint8_t)sizeof version);
    info_char(&line, ',');
    info_eeprom_text(&line, EE_ID_SERIAL, INFO_FIELD_MAX);
    info_char(&line, ',');
    info_eeprom_text(&line, EE_THID_CAL_DATE, 8u);
    info_char(&line, ',');
    info_count(&line, info_stored_count(EE_THID_CAL_COUNT));
    info_char(&line, ',');
    info_eeprom_text(&line, EE_THID_PROD_DATE, 8u);
    info_char(&line, ',');
    info_count(&line, main_power_on_hours());
    info_char(&line, ',');
    info_count(&line, info_stored_count(EE_PWRUP_COUNT));
    line.text[line.length] = '\0';
    remote_reply(line.text);
}

/* :SYSTem:ERRor?, :SYSTem:BEEPer ON|OFF|?, :SYSTem:INFO? */
void scpi_cmd_system(void) {
    if (scpi_depth() > 1u || !scpi_next_is_colon()) {
        scpi_fail_bad_command();
        return;
    }
    if (scpi_next_token() != SCPI_TOK_WORD) {
        scpi_fail_parameter();
        return;
    }
    if (KW("error") || KW("err")) {
        scpi_next_token();
        if (scpi_token_is_query()) {
            scpi_reply_last_error();
        } else {
            scpi_fail_bad_command();
        }
    } else if (KW("info")) {
        scpi_next_token();
        if (scpi_token_is_query()) {
            reply_info();
        } else {
            scpi_fail_bad_command();
        }
    } else if (KW("beeper") || KW("beep")) {
        switch (scpi_arg_bool()) {
        case SCPI_BOOL_ON:
            setting_set(&g_state.settings.beeper, true);
            break;
        case SCPI_BOOL_OFF:
            setting_set(&g_state.settings.beeper, false);
            break;
        case SCPI_BOOL_QUERY:
            scpi_reply_bool(g_state.settings.beeper);
            break;
        default:
            scpi_fail_parameter();
            break;
        }
    } else if (KW("kclick") || KW("kcl")) {
        /* NEW. The key click had a menu item and no remote command, so a client
         * could read and set every other SYS setting but not this one. Not protected. */
        switch (scpi_arg_bool()) {
        case SCPI_BOOL_ON:
            setting_set(&g_state.settings.key_click, true);
            break;
        case SCPI_BOOL_OFF:
            setting_set(&g_state.settings.key_click, false);
            break;
        case SCPI_BOOL_QUERY:
            scpi_reply_bool(g_state.settings.key_click);
            break;
        default:
            scpi_fail_parameter();
            break;
        }
    } else if (KW("lfrequency") || KW("lfr")) {
        /* NEW. The line frequency sets the converter's decimation so that mains
         * hum is notched; getting it wrong costs noise rejection, not accuracy, which is why it is an open
         * command and not a protected one.
         *
         * A change triggers a converter self-calibration, so it is refused while a sweep owns the
         * converter rather than being queued -- a calibration that moved the conversion rate underneath
         * itself would produce a reading nobody could account for afterwards. */
        int32_t hz;

        scpi_next_token();
        if (scpi_token_is_query()) {
            remote_reply_int(meas_line_is_50hz() ? 50 : 60);
        } else if (!scpi_arg_long(&hz)) {
            /* scpi_arg_long has already reported the error. */
        } else if (hz != 50 && hz != 60) {
            scpi_fail_parameter();
        } else if (!meas_set_line(hz == 50)) {
            scpi_fail_cannot_execute();
        }
    } else {
        scpi_fail_bad_command();
    }
}

/* ---- UNIT ---------------------------------------------------------------------------------------------- */

/* A REL reference expressed in dB: 20 log10(|V / dB REF|), the same formula the measurement applies to the
 * reading (meas.c db_value), so the subtraction in REL compares like with like. A zero or negative argument
 * has no logarithm and gives the lowest value the float holds, as the front-panel path does (keys.c). */
static float volts_to_db(float volts, float db_ref) {
    const float ratio = fabsf(volts / db_ref);

    return (ratio > 0.0f) ? 20.0f * log10f(ratio) : -FLT_MAX;
}

/* ... and in dBm: 10 log10(V^2 / Z / 1 mW) (meas.c dbm_value). */
static float volts_to_dbm(float volts, int16_t ohms) {
    const float power = volts * volts / (float)ohms / DBM_REFERENCE_WATT;

    return (power > 0.0f) ? 10.0f * log10f(power) : -FLT_MAX;
}

/* Carries the REL reference of the channel's function from one unit into another when the unit changes.
 *
 * Every conversion starts from VOLTS. The original applied the dBm formula for dB as well as for dBm, and when
 * the unit went from dB to dBm it took the reference it had just converted into dB, called that volts, and
 * converted it again; PROTOCOL_CHANGES section C plans 20 log10(V/Vref) for dB and no second conversion. The
 * volts value is the reference itself while the unit is V, and the copy kept in raw_reference (state.h)
 * while it is dB or dBm; raw_reference is written on the way out of volts and read on the way
 * back, which is also what the panel's unit keys do. Selecting the unit that is already selected changes
 * nothing. A reference that is not in use (REL off) is not touched, as in the original. */
static void convert_reference(channel_t channel, unit_mode_t from, unit_mode_t to) {
    const unit_channel_t *c = channel_state(channel);
    rel_t *rel = &g_state.rel[unit_function(channel)];
    float volts;

    if (!rel->active || from == to) {
        return;
    }
    volts = (from == UNIT_NATIVE) ? rel->reference : rel->raw_reference;
    if (to == UNIT_NATIVE) {
        rel->reference = volts;
        return;
    }
    rel->raw_reference = volts;
    rel->reference = (to == UNIT_DB) ? volts_to_db(volts, c->db_ref) : volts_to_dbm(volts, c->dbm_ref);
}

/* A dB or dBm conversion divides by its reference, so the unit is not switched while the reference it would
 * use is zero or less (meas.c guards the conversion itself as well). The references are
 * range-checked on entry, so this only refuses a store that was never set. */
static bool unit_reference_usable(channel_t channel, unit_mode_t mode) {
    const unit_channel_t *c = channel_state(channel);

    if (mode == UNIT_DB) {
        return c->db_ref > 0.0f;
    }
    if (mode == UNIT_DBM) {
        return c->dbm_ref > 0;
    }
    return true;
}

static void unit_set(channel_t channel, unit_mode_t mode) {
    func_t f = unit_function(channel);
    unit_mode_t previous;

    if (!unit_reference_usable(channel, mode)) {
        scpi_fail_data();
        return;
    }
    previous = channel_state(channel)->mode;
    channel_state(channel)->mode = mode;
    if (g_state.func == f) {
        convert_reference(channel, previous, mode);
        g_state.math.db_on = (mode == UNIT_DB);
        g_state.math.dbm_on = (mode == UNIT_DBM);
    }
    function_refresh();
}

static void reply_unit(channel_t channel) {
    switch (channel_state(channel)->mode) {
    case UNIT_NATIVE:
        remote_reply_P(PSTR("V"));
        break;
    case UNIT_DB:
        remote_reply_P(PSTR("DB"));
        break;
    default:
        remote_reply_P(PSTR("DBM"));
        break;
    }
}

/* :UNIT:VOLTage:<ac|dc>:DB:REFerence <volts 1e-4..1000>|? */
static void unit_db_reference(channel_t channel) {
    float value;

    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    scpi_next_token();
    if (!KW("ref") && !KW("reference")) {
        scpi_fail_bad_command();
        return;
    }
    if (scpi_next_token() == SCPI_TOK_NUMBER) {
        if (!scpi_arg_float(&value)) {
            return;
        }
        if (value < DB_REF_MIN || value > DB_REF_MAX) {
            scpi_fail_data();
            return;
        }
        channel_state(channel)->db_ref = value;
        if (g_state.func == unit_function(channel)) {
            g_state.math.db_ref = value;
        }
    } else if (scpi_token_is_query()) {
        remote_reply_float(channel_state(channel)->db_ref);
    } else {
        scpi_fail_param_error();
    }
}

/* :UNIT:VOLTage:<ac|dc>:DBM:IMPedance <ohm 1..9999>|? */
static void unit_dbm_impedance(channel_t channel) {
    int32_t value;

    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    scpi_next_token();
    if (!KW("imp") && !KW("impedance")) {
        scpi_fail_bad_command();
        return;
    }
    if (scpi_next_token() == SCPI_TOK_NUMBER) {
        if (!scpi_arg_long(&value)) {
            return;
        }
        if (value < IMPEDANCE_MIN || value > IMPEDANCE_MAX) {
            scpi_fail_data();
            return;
        }
        channel_state(channel)->dbm_ref = (int16_t)value;
        if (g_state.func == unit_function(channel)) {
            g_state.math.dbm_ref = (int16_t)value;
        }
    } else if (scpi_token_is_query()) {
        remote_reply_int((int32_t)channel_state(channel)->dbm_ref);
    } else {
        scpi_fail_param_error();
    }
}

/* The token after :VOLTage[:AC|:DC] -- V, DB, DBM, a deeper node, or ?. */
static void unit_channel_token(channel_t channel) {
    if (scpi_token_type() == SCPI_TOK_WORD) {
        if (KW("v")) {
            unit_set(channel, UNIT_NATIVE);
        } else if (KW("db")) {
            unit_set(channel, UNIT_DB);
        } else if (KW("dbm")) {
            unit_set(channel, UNIT_DBM);
        } else {
            scpi_fail_param_error();
        }
    } else if (token_is_colon()) {
        scpi_next_token();
        if (KW("db")) {
            unit_db_reference(channel);
        } else if (KW("dbm")) {
            unit_dbm_impedance(channel);
        } else {
            scpi_fail_bad_command();
        }
    } else if (scpi_token_is_query()) {
        reply_unit(channel);
    } else {
        scpi_fail_bad_command();
    }
}

/* :UNIT:VOLTage[:AC|:DC] <V|DB|DBM>|?, :UNIT:VOLTage:<AC|DC>:DB:REFerence,
 * :UNIT:VOLTage:<AC|DC>:DBM:IMPedance. Without :AC or :DC the DC channel is meant. */
void scpi_cmd_unit(void) {
    uint8_t depth = scpi_depth();

    if (depth == 1u) {
        return;
    }
    if (depth != 2u) {
        scpi_fail_bad_command();
        return;
    }
    scpi_next_token();
    if (!KW("volt") && !KW("voltage")) {
        scpi_fail_bad_command();
        return;
    }
    if (!scpi_next_is_colon()) {
        unit_channel_token(CHANNEL_DC);
        return;
    }
    scpi_next_token();
    if (KW("ac")) {
        scpi_next_token();
        unit_channel_token(CHANNEL_AC);
    } else if (KW("dc")) {
        scpi_next_token();
        unit_channel_token(CHANNEL_DC);
    } else if (KW("db")) {
        unit_db_reference(CHANNEL_DC);
    } else if (KW("dbm")) {
        unit_dbm_impedance(CHANNEL_DC);
    } else {
        scpi_fail_bad_command();
    }
}

/* ---- CALCulate ----------------------------------------------------------------------------------------- */

/* The comparison result of the limit test: 1 in limits, 2 above or overload, 3 below, 0 when the test is off.
 * The original keeps it in a variable the measurement cycle updates; it is derived from the reading here. */
static uint8_t limit_result(void) {
    const math_state_t *m = &g_state.math;

    if (!m->limit_on || !g_state.reading.valid) {
        return 0u;
    }
    if (g_state.reading.overload || g_state.reading.primary > m->limit_hi) {
        return 2u;
    }
    return (g_state.reading.primary < m->limit_lo) ? 3u : 1u;
}

/* A value of -1e8..1e8 for a percent reference or a limit. The original reaches the same inclusive test
 * through scpi_parse_float_checked 0x07D10 for the percent reference (call at 0x07D82) and through an inline
 * copy of it for the upper and lower limit (0x07F38-0x07F52 and 0x08016-0x0802E); all three report (5,4). */
static bool math_value_arg(float *value) {
    return scpi_arg_float_limited(value, -MATH_LIMIT, MATH_LIMIT);
}

/* :CALCulate:KMATh:PERCent <value>|?, :PERCent:ACQuire */
static void calc_percent(void) {
    float value;

    if (scpi_next_token() == SCPI_TOK_NUMBER) {
        if (math_value_arg(&value)) {
            /* The percent mode divides by this reference, so a zero reference is refused here exactly as
             * the ACQuire form below refuses a zero reading, with nothing stored (ORIGINAL_BUGS #20, #78). */
            if (value == 0.0f) {
                scpi_fail_data();
            } else {
                g_state.math.percent_ref = value;
            }
        }
    } else if (token_is_colon()) {
        scpi_next_token();
        if (KW("acq") || KW("acquire")) {
            /* The percent mode divides by this reference, so a zero reading is refused with nothing stored
             * (ORIGINAL_BUGS #78, PROTOCOL_CHANGES section F). */
            if (g_state.reading.raw == 0.0f) {
                scpi_fail_data();
            } else {
                g_state.math.percent_ref = g_state.reading.raw;
            }
        } else {
            scpi_fail_bad_command();
        }
    } else if (scpi_token_is_query()) {
        scpi_reply_float_clamped(g_state.math.percent_ref, SCPI_DECIMALS_STANDARD);
    } else {
        scpi_fail_parameter();
    }
}

/* :CALCulate:KMATh:STATe ON|OFF|? -- percent mode. A change re-applies the active function. */
static void calc_percent_state(void) {
    switch (scpi_arg_bool()) {
    case SCPI_BOOL_ON:
        g_state.math.percent_on = true;
        scpi_function_apply();
        break;
    case SCPI_BOOL_OFF:
        g_state.math.percent_on = false;
        scpi_function_apply();
        break;
    case SCPI_BOOL_QUERY:
        scpi_reply_bool(g_state.math.percent_on);
        break;
    default:
        scpi_fail_parameter();
        break;
    }
}

static void calc_kmath(void) {
    if (!token_is_colon()) {
        scpi_fail_bad_command();
        return;
    }
    scpi_next_token();
    if (KW("perc") || KW("percent")) {
        calc_percent();
    } else if (KW("stat") || KW("state")) {
        calc_percent_state();
    } else {
        scpi_fail_bad_command();
    }
}

/* :CALCulate:LIMit:UPPer|LOWer <value>|?. The original treated every argument that was not a number as a
 * query and replied the stored value; a mistyped word therefore looked like a successful read. Now only `?`
 * is a query and anything else is a parameter error (PROTOCOL_CHANGES section C, planned row). */
static void calc_limit_value(float *limit) {
    if (scpi_next_token() == SCPI_TOK_NUMBER) {
        float value;

        if (math_value_arg(&value)) {
            *limit = value;
        }
    } else if (scpi_token_is_query()) {
        scpi_reply_float_clamped(*limit, SCPI_DECIMALS_STANDARD);
    } else {
        scpi_fail_parameter();
    }
}

/* :CALCulate:LIMit:STATe ON|OFF|? */
static void calc_limit_state(void) {
    switch (scpi_arg_bool()) {
    case SCPI_BOOL_ON:
        g_state.math.limit_on = true;
        display_annunciators();
        break;
    case SCPI_BOOL_OFF:
        g_state.math.limit_on = false;
        display_annunciators();
        break;
    case SCPI_BOOL_QUERY:
        scpi_reply_bool(g_state.math.limit_on);
        break;
    default:
        scpi_fail_bad_command();
        break;
    }
}

static void calc_limit(void) {
    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    scpi_next_token();
    if (KW("upp") || KW("upper")) {
        calc_limit_value(&g_state.math.limit_hi);
    } else if (KW("low") || KW("lower")) {
        calc_limit_value(&g_state.math.limit_lo);
    } else if (KW("stat") || KW("state")) {
        calc_limit_state();
    } else if (KW("fail")) {
        scpi_next_token();
        if (scpi_token_is_query()) {
            /* The original answered 1 for "in limits", the opposite of the name. FAIL is 1 when the reading
             * is above the upper limit or overloaded (result 2) or below the lower limit (result 3), and 0
             * when it is inside them or the test is off (PROTOCOL_CHANGES section C, planned row). */
            const uint8_t result = limit_result();

            scpi_reply_bool(result == 2u || result == 3u);
        } else {
            scpi_fail_syntax();
        }
    } else {
        scpi_fail_bad_command();
    }
}

void scpi_cmd_calculate(void) {
    uint8_t depth = scpi_depth();

    if (depth == 1u) {
        return;
    }
    if (depth != 2u) {
        scpi_fail_bad_command();
        return;
    }
    if (scpi_next_token() != SCPI_TOK_WORD) {
        scpi_fail_parameter();
    } else if (KW("kmat") || KW("kmath")) {
        scpi_next_token();
        calc_kmath();
    } else if (KW("lim") || KW("limit")) {
        calc_limit();
    } else {
        scpi_fail_parameter();
    }
}

/* ---- SERVice: the service lock ----------------------------------------------------- */

/* The quoted string argument of a service command; the password keeps its case. */
static bool string_arg(void) {
    if (scpi_next_token() == SCPI_TOK_STRING) {
        return true;
    }
    scpi_fail_parameter();
    return false;
}

/* :SERVice:UNLOCK "<password>", :SERVice:LOCK, :SERVice:STATe?, :SERVice:PASSword:SET "<new>".
 * The lock decides everything else; a refusal is reported as the lock error and changes nothing. STATe? is
 * always answered. */
void scpi_cmd_serv(void) {
    uint8_t depth = scpi_depth();

    if (depth == 1u) {
        return;
    }
    if (depth != 2u) {
        scpi_fail_bad_command();
        return;
    }
    scpi_next_token();
    if (KW("unlock")) {
        if (string_arg() && !serv_unlock(scpi_token(), SERV_FROM_REMOTE)) {
            scpi_fail_locked();
        }
    } else if (KW("lock")) {
        serv_lock();
    } else if (KW("stat") || KW("state")) {
        scpi_next_token();
        if (!scpi_token_is_query()) {
            scpi_fail_bad_command();
        } else if (serv_is_unlocked()) {
            remote_reply_P(PSTR("UNLOCKED"));
        } else {
            remote_reply_P(PSTR("LOCKED"));
        }
    } else if (KW("pass") || KW("password")) {
        if (!scpi_next_is_colon()) {
            scpi_fail_syntax();
            return;
        }
        scpi_next_token();
        if (!KW("set")) {
            scpi_fail_bad_command();
        } else if (string_arg() && !serv_set_password(scpi_token())) {
            scpi_fail_locked();
        }
    } else {
        scpi_fail_bad_command();
    }
}

/* ---- :SCAL:COEF? -----------------------------------------------------------------------------------------
 */

/* Eight decimals: nine significant digits are what it takes to round-trip a binary32, so a
 * client that writes a coefficient and reads it back to check it gets exactly the number it wrote. The
 * writer never overruns what it is given, and the longest value (sign, mantissa, "e-38") is 15 characters,
 * so six values and five commas fit the 112-byte line. */
static size_t append_float(char *line, size_t size, size_t length, float value) {
    if (length >= size) {
        return length;
    }
    return length + fmt_float_e_prec(line + length, size - length, value, SCPI_DECIMALS_READING);
}

/* :SCAL:COEF? -- the coefficients in use for the active function and range, in the order the
 * :SCAL:COEF write takes them: the main pair (f0, f1) at medium, then fast, then slow, as eight-decimal
 * numbers. Protected; the :SCAL dispatcher in exec.c calls this for "coef" followed by "?",
 * and the lock is asked here as well because the function is public. Only the five measurement functions with
 * a coefficient set (DCV, ACV, DCA, ACA, OHM) answer. */
void scpi_scal_coef_query(void) {
    static const uint8_t wire_order[] PROGMEM = {RATE_MEDIUM, RATE_FAST, RATE_SLOW};
    func_t f = g_state.func;
    char line[112];
    size_t length = 0u;

    if (serv_check() != SERV_ALLOWED) {
        scpi_fail_locked();
        return;
    }
    if (f > FUNC_OHM) {
        scpi_fail_parameter();
        return;
    }
    for (uint8_t i = 0u; i < sizeof wire_order; i++) {
        cal_set_t set;

        cal_load(f, g_state.range[f].index, (rate_t)pgm_read_byte(&wire_order[i]), &set);
        length = append_float(line, sizeof line, length, set.main.f0);
        if (length < sizeof line - 1u) {
            line[length++] = ',';
        }
        length = append_float(line, sizeof line, length, set.main.f1);
        if (i + 1u < sizeof wire_order && length < sizeof line - 1u) {
            line[length++] = ',';
        }
    }
    line[length < sizeof line ? length : sizeof line - 1u] = '\0';
    remote_reply(line);
}
