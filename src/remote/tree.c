/* tree.c -- the command tree below the parser: root keywords, range and NPLC arguments, and the shared
 * range sub-tree.
 *
 * Only the 18 root keywords are table driven (tables.h root_keyword[]); every deeper
 * keyword is compared in the handler that owns it, as in the original.
 *
 * The original raised an error for an unknown range or NPLC token and then stored index 0 (the
 * top range, 0.1 PLC) anyway. Here a bad argument reports and changes nothing.
 */
#include <avr/pgmspace.h>
#include <float.h>
#include "../fmathf.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../meas/meas.h"
#include "../state.h"
#include "../tables.h"
#include "../ui/display.h"
#include "scpi.h"

/* An NPLC argument matches a table entry when it is within two units of the last place of it. fmt_parse_float
 * is good to 1.5 ulp (PROTOCOL_CHANGES section H), so "0.1", "1.0" and "1e0" land on the table value, and
 * "10.009" does not. The tolerance is the table entry times 2 FLT_EPSILON, which is between two and four
 * ulp of that entry. */
#define NPLC_TOLERANCE (2.0f * FLT_EPSILON)

/* ---- Root keywords ------------------------------------------------------------------------------------- */

/* The original stores root names in 12-byte fields. "scalibration" has 12 characters, fills its field with
 * no terminator (flash 0x1505, the id byte 0x10 follows directly) and so could never be matched by the
 * original's string compare, which made it a dead spelling. PROTOCOL_CHANGES section C plans it as accepted,
 * and a port that compares against a flash string has no field to overflow, so it is. It
 * reaches the same :SCAL dispatcher as "scal" and so the same service-lock checks. */
static bool spelling_matches(const char *token, const char *spelling_P) {
    return spelling_P != NULL && strcmp_P(token, spelling_P) == 0;
}

/* The root code (1..18) of a lower-cased token, or 0 when it is no root keyword. Long and short forms are
 * separate spellings; there is no abbreviation logic. */
uint8_t scpi_root_keyword_lookup(const char *token) {
    for (uint8_t i = 0u; i < root_keyword_count; i++) {
        keyword_t k;

        memcpy_P(&k, &root_keyword[i], sizeof k);
        if (spelling_matches(token, k.long_form) || spelling_matches(token, k.short_form)) {
            return k.code;
        }
    }
    return 0u;
}

/* ---- Range spellings ----------------------------------------------------------------------------------- */

/* Functions that share one spelling table in the original: the range names of the AC volts table serve
 * frequency, period and AC+DC volts too, the current tables are identical, and continuity uses ohms. */
static uint8_t range_group(func_t f) {
    switch (f) {
    case FUNC_DCV:
        return 0u;
    case FUNC_ACV:
    case FUNC_FREQ:
    case FUNC_PERIOD:
    case FUNC_ACDCV:
        return 1u;
    case FUNC_DCA:
    case FUNC_ACA:
    case FUNC_ACDCI:
        return 2u;
    case FUNC_OHM:
    case FUNC_CONT:
        return 3u;
    default:
        return 4u;
    }
}

/* Looks the current token up in the spelling table of function f (numeric names, names with a unit, DEF, MIN
 * and MAX). An unknown spelling is a parameter error. */
bool scpi_parse_range_token(func_t f, uint8_t *index) {
    uint8_t group = range_group(f);

    for (uint8_t i = 0u; i < range_word_count; i++) {
        range_word_t w;

        memcpy_P(&w, &range_word[i], sizeof w);
        if (range_group(w.func) == group && strcmp_P(scpi_token(), w.text) == 0) {
            *index = w.range_index;
            return true;
        }
    }
    scpi_fail_parameter();
    return false;
}

/* True for the plain numeric spelling ("1000", "0.5"), as opposed to "500mv", "def" or "max". */
static bool is_numeric_spelling(const char *text_P) {
    char c = (char)pgm_read_byte(text_P);

    if (c == '\0') {
        return false;
    }
    for (; c != '\0'; c = (char)pgm_read_byte(++text_P)) {
        if (!((c >= '0' && c <= '9') || c == '.')) {
            return false;
        }
    }
    return true;
}

/* The numeric spelling of a range index in the table of table_func, as used by the range queries. */
bool scpi_range_text(func_t table_func, uint8_t index, char *dst, size_t dst_size) {
    uint8_t group = range_group(table_func);

    for (uint8_t i = 0u; i < range_word_count; i++) {
        range_word_t w;

        memcpy_P(&w, &range_word[i], sizeof w);
        if (range_group(w.func) == group && w.range_index == index && is_numeric_spelling(w.text)) {
            strncpy_P(dst, w.text, dst_size);
            dst[dst_size - 1u] = '\0';
            return true;
        }
    }
    dst[0] = '\0';
    return false;
}

/* ---- Range commands shared by every function family ---------------------------------------------------- */

/* Pushes function f's range selection to the front end and refreshes the Auto annunciator. The converter is
 * only touched when f is the active function: PROTOCOL_CHANGES G.2 #25 lets a command name another function
 * (`:CURR:DC:RANGe 1` while measuring volts), which edits that function's stored selection and leaves the
 * hardware alone until it is selected. */
void scpi_range_apply_refresh(func_t f) {
    const range_sel_t *sel = &g_state.range[f];

    if (f == g_state.func) {
        meas_set_range(f, sel->index, sel->manual);
    }
    display_annunciators();
}

/* The current token names a range of function f: select it manually. */
void scpi_range_set_from_token(func_t f) {
    uint8_t index;

    if (!scpi_parse_range_token(f, &index)) {
        return;
    }
    g_state.range[f].index = index;
    g_state.range[f].manual = true;
    scpi_range_apply_refresh(f);
}

/* Replies the range name of function f, taken from f's own spelling table. */
void scpi_range_reply(func_t f) {
    char text[16];

    if (!scpi_range_text(f, g_state.range[f].index, text, sizeof text)) {
        text[0] = '\0';
    }
    remote_reply(text);
}

/* :RANGe:UPPer <range>|? -- the same as :RANGe. */
void scpi_range_upper_arg(func_t f) {
    uint8_t type = scpi_next_token();

    if (type == SCPI_TOK_NUMBER || type == SCPI_TOK_WORD) {
        scpi_range_set_from_token(f);
    } else if (scpi_token_is_query()) {
        scpi_range_reply(f);
    } else {
        scpi_fail_parameter();
    }
}

/* :RANGe:AUTO ON|OFF|1|0|? */
void scpi_range_auto_arg(func_t f) {
    range_sel_t *sel = &g_state.range[f];

    switch (scpi_arg_bool()) {
    case SCPI_BOOL_ON:
        sel->manual = false;
        scpi_range_apply_refresh(f);
        break;
    case SCPI_BOOL_OFF:
        sel->manual = true;
        scpi_range_apply_refresh(f);
        break;
    case SCPI_BOOL_QUERY:
        scpi_reply_bool(!sel->manual);
        break;
    default:
        scpi_fail_parameter();
        break;
    }
}

/* Continuity (root 11) owns a range sub-tree on the ohms spelling table (range_group() puts FUNC_CONT with
 * FUNC_OHM) and edits the continuity function's own selection whatever function is active:
 *   :RANGe <range>|?       and  :RANGe:UPPer <range>|?   and  :RANGe:AUTO ON|OFF|?
 * Depth 2 is the RANGe keyword, depth 3 the upper/auto node. */
void scpi_node_range_handler(void) {
    uint8_t depth = scpi_depth();
    uint8_t type;

    if (depth == 1u) {
        return;
    }
    if (depth == 2u) {
        scpi_next_token();
        if (!scpi_keyword_match(PSTR("rang")) && !scpi_keyword_match(PSTR("range"))) {
            scpi_fail_bad_command();
            return;
        }
        type = scpi_next_token();
        if (type == SCPI_TOK_SEP) {
            if (scpi_token()[0] == ':') {
                scpi_unget_token();
            } else if (scpi_token_is_query()) {
                scpi_range_reply(FUNC_CONT);
            } else {
                scpi_fail_bad_command();
            }
        } else if (type == SCPI_TOK_NUMBER || type == SCPI_TOK_WORD) {
            scpi_range_set_from_token(FUNC_CONT);
        } else {
            scpi_fail_bad_command();
        }
        return;
    }
    if (depth == 3u) {
        scpi_next_token();
        if (scpi_keyword_match(PSTR("upp")) || scpi_keyword_match(PSTR("upper"))) {
            scpi_range_upper_arg(FUNC_CONT);
        } else if (scpi_keyword_match(PSTR("auto"))) {
            scpi_range_auto_arg(FUNC_CONT);
        } else {
            scpi_fail_bad_command();
        }
        return;
    }
    scpi_fail_bad_command();
}

/* ---- NPLC ---------------------------------------------------------------------------------------------- */

/* The current number token as an integration rate: 0.1, 1 or 10 PLC are rate 0, 1 and 2. */
bool scpi_nplc_lookup(rate_t *rate) {
    float value;

    if (!scpi_arg_float(&value)) {
        return false;
    }
    for (uint8_t i = 0u; i < NPLC_COUNT; i++) {
        float plc;

        memcpy_P(&plc, &nplc_value[i], sizeof plc);
        if (fabsf(value - plc) <= plc * NPLC_TOLERANCE) {
            *rate = (rate_t)i;
            return true;
        }
    }
    scpi_fail_data();
    return false;
}

/* The argument of :NPLCycles: DEF, MIN, MAX, 0.1, 1, 10 or ?. MIN is the smallest number of power line cycles
 * (0.1, the fast rate) and MAX the largest (10, the slow rate), as in SCPI. The original had them the other
 * way round (PROTOCOL_CHANGES section C, planned row "NPLC MIN/MAX"), which made `NPLC MAX` the least
 * accurate rate there is. */
void scpi_set_nplc(void) {
    rate_t rate;
    uint8_t type = scpi_next_token();

    if (scpi_keyword_match(PSTR("def")) || scpi_keyword_match(PSTR("default"))) {
        rate = RATE_MEDIUM;
    } else if (scpi_keyword_match(PSTR("min")) || scpi_keyword_match(PSTR("minimum"))) {
        rate = RATE_FAST;
    } else if (scpi_keyword_match(PSTR("max")) || scpi_keyword_match(PSTR("maximum"))) {
        rate = RATE_SLOW;
    } else if (type == SCPI_TOK_NUMBER) {
        if (!scpi_nplc_lookup(&rate)) {
            return;
        }
    } else if (scpi_token_is_query()) {
        switch (g_state.rate) {
        case RATE_FAST:
            remote_reply_P(PSTR("0.1"));
            break;
        case RATE_MEDIUM:
            remote_reply_P(PSTR("1"));
            break;
        default:
            remote_reply_P(PSTR("10"));
            break;
        }
        return;
    } else {
        scpi_fail_cannot_execute();
        return;
    }
    g_state.rate = rate;
    meas_set_rate(rate);
    display_annunciators();
}
