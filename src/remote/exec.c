/* exec.c -- the remote command executor: takes one completed line, walks it statement by statement,
 * dispatches each header to its handler and stops at the first error. Also the :SCAL command tree, because it
 * is routing rather than calibration, and the host-link service that the main loop calls once per pass.
 *
 * Replaces scpi_execute_line 0x0FBEA, its 18-way jump table (0x0FCF0-0x0FD4C, table at
 * flash 0x008C), the default error tail 0x0FD4C, and the :SCAL dispatcher scpi_cmd_scal 0x06DD4.
 *
 * ---- No interrupts disabled, no waiting ---------------------------------------------------------------------
 * The original executor ran `cli` on entry (0x0FBEE) and `sei` on exit (0x0FD80), and an error inside it
 * busy-waited a second with interrupts still off, so a host lost every byte it sent during that second. Nothing
 * here touches the interrupt flag and nothing here waits: the receive ring (uart1.c) keeps filling while a line
 * runs, an error is queued and returned from, and the one slow operation a line can start, a calibration
 * sweep, is handed to the main loop as a start/poll pair.
 *
 * ---- An error aborts the line --------------------------------------------------------------------------------
 * An error aborts the rest of the line (scpi_error_report ends it). The original reported and then carried on
 * to the next statement, and its handlers went on to store defaults.
 *
 * ---- What the line walk is ----------------------------------------------------------------------------------
 * The ':' depth is the parser's (scpi_depth). At depth 0 the next token is a root keyword (or a '*' command);
 * from then on a handler runs once per ':' level and scpi_parse_separator() moves between levels. :SCAL and
 * :THID are the two roots that consume their whole header path in one call, as the originals do.
 *
 * ---- The service lock ----------------------------------------------------------------------------------------
 * serv_check() is asked before every protected :SCAL command and the command is refused on anything but
 * SERV_ALLOWED. serv_note_activity() follows only a SUCCESS, never an attempt, so a client cannot hold a
 * session open by failing at a command.
 */
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../cal/cal.h"
#include "../fmt/fmt.h"
#include "../hal/uart1.h"
#include "../meas/meas.h"
#include "../serv/serv.h"
#include "../state.h"
#include "scpi.h"

#define KW(text) scpi_keyword_match(PSTR(text))

/* The root codes of tables.c root_keyword[], which are the original's jump-table indexes plus one. */
enum {
    ROOT_FUNCTION = 1,
    ROOT_FUNCTION2 = 2,
    ROOT_CALCULATE = 3,
    ROOT_HOLD = 4,
    ROOT_CURRENT = 5,
    ROOT_VOLTAGE = 6,
    ROOT_RESISTANCE = 7,
    ROOT_FREQUENCY = 8,
    ROOT_PERIOD = 9,
    ROOT_DIODE = 10,
    ROOT_CONTINUITY = 11,
    ROOT_FETCH = 12,
    ROOT_DISPLAY = 13,
    ROOT_UNIT = 14,
    ROOT_TRIGGER = 15,
    ROOT_SCAL = 16,
    ROOT_THID = 17,
    ROOT_SYSTEM = 18,
    /* Not in root_keyword[] (which has 18 rows and is frozen), so it never collides with a table code; see
     * root_step(). */
    ROOT_SERV = 20
};

/* ---- Reply line builder ------------------------------------------------------------------------------------ */

/* The fmt.h writers return a length and do not terminate, so a reply is built in a bounded buffer and
 * terminated once, at the end. One byte is always kept back for that terminator. */
typedef struct {
    char *text;
    size_t size;
    size_t length;
} line_t;

static size_t line_room(const line_t *line) {
    return line->size - 1u - line->length;
}

static void line_char(line_t *line, char c) {
    if (line_room(line) > 0u) {
        line->text[line->length++] = c;
    }
}

static void line_advance(line_t *line, size_t written) {
    const size_t room = line_room(line);

    line->length += (written < room) ? written : room;
}

static void line_float(line_t *line, float value, uint8_t decimals, bool upper_case) {
    char *dst = line->text + line->length;
    const size_t room = line_room(line);

    line_advance(line, upper_case ? fmt_float_E_prec(dst, room, value, decimals)
                                  : fmt_float_e_prec(dst, room, value, decimals));
}

static void line_int(line_t *line, int32_t value) {
    line_advance(line, fmt_int(line->text + line->length, line_room(line), value));
}

static void line_send(line_t *line) {
    line->text[line->length] = '\0';
    remote_reply(line->text);
}

/* ---- Errors reported between lines ------------------------------------------------------------------------- */

/* An error that belongs to no line (a sweep that failed after its command had returned). The parser is
 * restarted on an empty line first, because scpi_error_report() drops an error when the previous line has
 * already failed. */
static void report_cannot_execute_between_lines(void) {
    scpi_parser_begin("", 0u);
    scpi_fail_cannot_execute();
}

/* ---- The service lock -------------------------------------------------------------------------------------- */

static bool protected_begin(void) {
    if (serv_check() != SERV_ALLOWED) {
        scpi_fail_locked();
        return false;
    }
    return true;
}

static void protected_done(void) {
    if (!scpi_line_failed()) {
        serv_note_activity();
    }
}

/* ---- Calibration sweeps: start, poll, terminate ------------------------------------------------------------
 *
 * A :SCAL sweep is up to about thirty seconds of settling and conversions, so the command only STARTS it
 * (cal_*_start) and returns; the main loop advances it with cal_sweep_task(). From here:
 *   - each finished cell arrives through report_cell(), registered before the start, and is sent as it
 *     happens, as "%E,%d" (value in upper-case exponent form with six decimals, then the range index);
 *   - remote_poll() watches the state. CAL_SWEEP_DONE sends the closing "!" (kept from the original) and counts as a
 *     successful protected command; CAL_SWEEP_FAILED reports "cannot execute" with no closing "!"; IDLE after
 *     a start means the sweep was aborted and nothing more is sent.
 * While a sweep runs, a command that would drive the front end is refused with the same error (root_conflicts).
 */
static bool sweep_pending; /* a sweep this module started has not yet been terminated */

/* ORIGINAL_BUGS #1 / #93: :SCAL:STD and :SCAL:TEST used to run their conversions in a loop inside the
 * command, so the instrument was deaf for the whole measurement -- the last two blocking commands in the
 * executor. They are now kinds on the same sweep machine the short/open sweeps use, and their reply is sent
 * from the poll routine when the machine reaches DONE.
 *
 * This marker is deliberately NOT sweep_pending. A sweep's DONE branch sends the closing "!" and calls
 * serv_note_activity(); a query's DONE branch sends a reply line and no "!". Sharing one flag would send
 * both, which would corrupt the reply a client is parsing. */
typedef enum { QUERY_NONE, QUERY_STD, QUERY_TEST } query_pending_t;

static query_pending_t query_pending;

static void report_cell(func_t f, uint8_t range, float value) {
    char text[32];
    line_t line = {.text = text, .size = sizeof text, .length = 0u};

    (void)f;
    line_float(&line, value, SCPI_DECIMALS_STANDARD, true);
    line_char(&line, ',');
    line_int(&line, (int32_t)range);
    line_send(&line);
}

typedef enum { SWEEP_SHORT_ONCE, SWEEP_OPEN_ONCE, SWEEP_SHORT_ALL, SWEEP_OPEN_ALL } sweep_request_t;

/* The ONCE forms sweep the active function and range; the guard that refuses any other cell lives in cal.h,
 * so a false return is a refusal with nothing started and nothing written. */
static void sweep_start(sweep_request_t request) {
    const func_t f = g_state.func;
    const uint8_t range = g_state.range[f].index;
    bool started;

    cal_set_reporter(report_cell);
    switch (request) {
    case SWEEP_SHORT_ONCE:
        started = cal_short_once_start(f, range);
        break;
    case SWEEP_OPEN_ONCE:
        started = cal_open_once_start(f, range);
        break;
    case SWEEP_SHORT_ALL:
        started = cal_short_all_start();
        break;
    default:
        started = cal_open_all_start();
        break;
    }
    if (started) {
        sweep_pending = true;
    } else {
        scpi_fail_cannot_execute();
    }
}

static void sweep_service(void) {
    if (!sweep_pending) {
        return;
    }
    switch (cal_sweep_state()) {
    case CAL_SWEEP_DONE:
        sweep_pending = false;
        scpi_reply_bang();
        serv_note_activity();
        break;
    case CAL_SWEEP_FAILED:
        sweep_pending = false;
        report_cannot_execute_between_lines();
        break;
    case CAL_SWEEP_IDLE:
        sweep_pending = false;
        break;
    default:
        break;
    }
}

/* The reply for a :SCAL:STD or :SCAL:TEST that is still running. Both reply shapes are unchanged from when
 * the command blocked: STD sends medium, fast, slow in that order (ORIGINAL_BUGS #39 -- the order is the
 * rate index, not the order the rates were measured), TEST sends the value and a trailing colon. */
static void query_service(void) {
    if (query_pending == QUERY_NONE) {
        return;
    }

    const cal_sweep_state_t state = cal_sweep_state();

    if (state == CAL_SWEEP_DONE) {
        char text[32];
        line_t line = {.text = text, .size = sizeof text, .length = 0u};

        if (query_pending == QUERY_STD) {
            float value[RATE_COUNT];

            if (cal_scal_std_result(value)) {
                line_float(&line, value[RATE_MEDIUM], SCPI_DECIMALS_READING, false);
                line_char(&line, ':');
                line_float(&line, value[RATE_FAST], SCPI_DECIMALS_READING, true);
                line_char(&line, ',');
                line_float(&line, value[RATE_SLOW], SCPI_DECIMALS_READING, true);
                line_send(&line);
            } else {
                report_cannot_execute_between_lines();
            }
        } else {
            float value;

            if (cal_scal_test_result(&value)) {
                line_float(&line, value, SCPI_DECIMALS_STANDARD, false);
                line_char(&line, ':');
                line_send(&line);
            } else {
                report_cannot_execute_between_lines();
            }
        }
        query_pending = QUERY_NONE;
    } else if (state == CAL_SWEEP_FAILED) {
        /* The guards cal.h added for the non-blocking form: a function change, a range change during STD, or
         * a rate change during TEST ends the run. The client gets no reply line, so the reason is queued. */
        query_pending = QUERY_NONE;
        report_cannot_execute_between_lines();
    } else if (state == CAL_SWEEP_IDLE) {
        query_pending = QUERY_NONE;
    }
}

/* The roots that would change what the converter is doing. Everything else (queries, the display, HOLD,
 * UNIT, TRIGger, FETCh, SYSTem, THID, SERVice) leaves the front end alone and is allowed during a sweep.
 * CALCulate is in the list because KMATh:STATe re-applies the active function. */
static bool root_conflicts(uint8_t root) {
    switch (root) {
    case ROOT_FUNCTION:
    case ROOT_FUNCTION2:
    case ROOT_CALCULATE:
    case ROOT_CURRENT:
    case ROOT_VOLTAGE:
    case ROOT_RESISTANCE:
    case ROOT_FREQUENCY:
    case ROOT_PERIOD:
    case ROOT_DIODE:
    case ROOT_CONTINUITY:
    case ROOT_SCAL:
        return true;
    default:
        return false;
    }
}

/* ---- :SCAL --------------------------------------------------------------------------------------------------
 *
 * Grammar as the original (0x06DD4), every path starting with ':' after SCAL:
 *   :OPEN[:ALL|:ONCE]   :SHORt[:ALL|:ONCE]    a bare OPEN or SHOR means ALL
 *   :STD[:DC|:AC]       a bare STD means DC
 *   :ACV:ZERO  :DCV:ZERO  :TEST  :COEF "<csv>"  :COEF?  :DATE:<date>  :SAVE
 * :CLEAR_DATA, :CLEAR_ALL, :CLEAR_INIT and :CLEAR_REL are NOT here and are removed on purpose, not left
 * unfinished: they stamp the cleared signature over every calibration record (0x03200), a memory wipe that
 * project rule 1.3 forbids outright, and they are on the transport denylist. They fall into the
 * unknown-keyword error like any other word. Protected: COEF, COEF?, DATE, SAVE, OPEN and SHOR (serv.h). Open: STD, TEST and the two
 * zero queries.
 *
 * Differences from the original that are not new features, all because an error must not be followed by an
 * action or by silence: a word after STD that is neither ":DC" nor ":AC" nor the end of the
 * statement is a bad-command error (the original returned without a reply and without an error), and a
 * function with no coefficient set for :COEF is an error (the original ignored it silently). */

/* After a keyword that may stand alone: true when the statement ends here, either at the end of the line or at
 * a ';'. The token is already read; a ';' is given back so that scpi_parse_separator() sees it. */
static bool statement_ends(void) {
    if (scpi_token_type() != SCPI_TOK_SEP) {
        return false;
    }
    if (scpi_token_is_end() || scpi_token()[0] == '\0') {
        return true;
    }
    if (scpi_token()[0] == ';') {
        scpi_unget_token();
        return true;
    }
    return false;
}

/* Reads the token after a complete command and requires the statement to end there. Every writer calls this
 * BEFORE it acts, so that `:SCAL:SAVE?`, `:SCAL:UNDO?` or `:SCAL:SHOR:ALL x` is a syntax error with no state
 * change. The original acted first and checked the separator afterwards (0x07052 increments the counter, and
 * the check follows), which made the query form of a calibration command a write; the contract of remote.h
 * is that an error comes before any state change. */
static bool require_statement_end(void) {
    scpi_next_token();
    if (statement_ends()) {
        return true;
    }
    scpi_fail_syntax();
    return false;
}

static void scal_sweep(bool is_short) {
    sweep_request_t request = is_short ? SWEEP_SHORT_ALL : SWEEP_OPEN_ALL;

    if (!protected_begin()) {
        return;
    }
    if (scpi_next_is_colon()) {
        if (scpi_next_token() != SCPI_TOK_WORD) {
            scpi_fail_bad_command();
            return;
        }
        if (KW("once")) {
            request = is_short ? SWEEP_SHORT_ONCE : SWEEP_OPEN_ONCE;
        } else if (!KW("all")) {
            scpi_fail_bad_command();
            return;
        }
        if (!require_statement_end()) {
            return;
        }
    } else if (!statement_ends()) {
        scpi_fail_bad_command();
        return;
    }
    sweep_start(request);
}

/* :SCAL:STD[:DC|:AC] -- the reading with identity coefficients at the medium, fast and slow rates, in that
 * field order: "%.8e:%.8E,%.8E" (the first number in lower case, as the original's format string at flash
 * 0x093F has it). ORIGINAL_BUGS #55 made the three numbers different; the layout is unchanged.
 *
 * Field 1, before the colon, is the MEDIUM reading. The vendor utility's ReadStd (@51e36c) parses
 * "<text>:<fast>,<slow>" and takes the text before the colon as medium, and the 5492B source does the same;
 * the original firmware printed one medium reading in all three fields, so it never contradicted that.
 * The order is medium, fast, slow. */
static void scal_std(void) {
    cal_std_kind_t kind = CAL_STD_DC;

    if (scpi_next_is_colon()) {
        if (scpi_next_token() != SCPI_TOK_WORD) {
            scpi_fail_bad_command();
            return;
        }
        if (KW("ac")) {
            kind = CAL_STD_AC;
        } else if (!KW("dc")) {
            scpi_fail_bad_command();
            return;
        }
    } else if (!statement_ends()) {
        scpi_fail_bad_command();
        return;
    }
    if (!cal_scal_std_start(kind)) {
        scpi_fail_cannot_execute();
        return;
    }
    /* The reply is sent by query_service() when the sweep machine finishes. */
    query_pending = QUERY_STD;
}

/* :SCAL:TEST -- the corrected reading with REL applied. The original's reply is "%e:" followed by the text of
 * the main display (RAM 0x0F19). No module offers that text, so the part after the colon is empty. The vendor
 * utility's ReadTest (@51e798) parses "<value>:<text>" and uses the value only, so nothing that is known to read this reply misses the text. PROTOCOL_CHANGES section A records
 * the omission. A trailing `?` is refused before the measurement, like `:SCAL:STD?` (PROTOCOL_CHANGES G.2
 * #43): the measurement leaves the rate and the filter state changed. */
static void scal_test(void) {
    if (!require_statement_end()) {
        return;
    }
    if (!cal_scal_test_start()) {
        scpi_fail_cannot_execute();
        return;
    }
    /* The reply is sent by query_service() when the sweep machine finishes. */
    query_pending = QUERY_TEST;
}

#define ZERO_PRINT_THRESHOLD 1.0e-7f

static void line_zero_value(line_t *line, float value) {
    if (value > -ZERO_PRINT_THRESHOLD && value < ZERO_PRINT_THRESHOLD) {
        line_char(line, '0');
    } else {
        line_float(line, value, SCPI_DECIMALS_STANDARD, true);
    }
}

/* :SCAL:ACV:ZERO and :SCAL:DCV:ZERO -- "<range>,<z>,<z>,<z>" for the active function, a value of magnitude
 * under 1e-7 printed as "0". The original read a base address chosen by the active function and left it
 * undefined for any other: ACV:ZERO answers for AC V and AC A (0x06A70), DCV:ZERO for DC V and AC A (0x06C22),
 * and every other function is refused. */
static void scal_zero(bool ac_command) {
    const func_t f = g_state.func;
    const bool allowed = ac_command ? (f == FUNC_ACV || f == FUNC_ACA) : (f == FUNC_DCV || f == FUNC_ACA);
    float z[RATE_COUNT];
    uint8_t range;
    char text[64];
    line_t line = {.text = text, .size = sizeof text, .length = 0u};

    if (!scpi_next_is_colon() || scpi_next_token() != SCPI_TOK_WORD || !KW("zero")) {
        scpi_fail_bad_command();
        return;
    }
    if (!require_statement_end()) {
        return;
    }
    if (!allowed || !cal_zero_query(f, &range, z)) {
        scpi_fail_cannot_execute();
        return;
    }
    line_int(&line, (int32_t)range);
    for (uint8_t i = 0u; i < RATE_COUNT; i++) {
        line_char(&line, ',');
        line_zero_value(&line, z[i]);
    }
    line_send(&line);
}

/* ---- :SCAL:COEF -------------------------------------------------------------------------------------------- */

/* The longest field list is ACV with the low-range extras: 14 fields. A 15th makes the count wrong for every
 * function, which cal_coef_write() refuses. */
#define COEF_FIELDS_MAX 14u
#define COEF_FIELD_CHARS 40u

typedef enum { SLOT_V, SLOT_Z, SLOT_X, SLOT_A, SLOT_B, SLOT_NONE } slot_kind_t;

typedef struct {
    slot_kind_t kind;
    uint8_t index;
} slot_t;

/* Where field number `position` (0 based) of the string goes for function f:
 *   DC V, DC A   v[0..5], a = sign block, b = range
 *   ohm          v[0..5], a = range, x[0..5] = the Y set
 *   AC V         v[0..5], z[0..2], a = band, b = range, x[0..2] (band 1 only)
 *   AC A         v[0..5], z[0..2], a = range or band, x[0..2] (selector 5 only)
 * A position beyond the function's list has no slot; it still counts, so cal_coef_write() sees the wrong count. */
static slot_t coef_slot(func_t f, uint8_t position) {
    slot_t slot = {SLOT_NONE, 0u};

    if (position < 6u) {
        slot.kind = SLOT_V;
        slot.index = position;
        return slot;
    }
    switch (f) {
    case FUNC_DCV:
    case FUNC_DCA:
        if (position == 6u) {
            slot.kind = SLOT_A;
        } else if (position == 7u) {
            slot.kind = SLOT_B;
        }
        break;
    case FUNC_OHM:
        if (position == 6u) {
            slot.kind = SLOT_A;
        } else if (position <= 12u) {
            slot.kind = SLOT_X;
            slot.index = (uint8_t)(position - 7u);
        }
        break;
    case FUNC_ACV:
    case FUNC_ACA:
        if (position <= 8u) {
            slot.kind = SLOT_Z;
            slot.index = (uint8_t)(position - 6u);
        } else if (position == 9u) {
            slot.kind = SLOT_A;
        } else if (f == FUNC_ACV && position == 10u) {
            slot.kind = SLOT_B;
        } else if (f == FUNC_ACV && position >= 11u && position <= 13u) {
            slot.kind = SLOT_X;
            slot.index = (uint8_t)(position - 11u);
        } else if (f == FUNC_ACA && position >= 10u && position <= 12u) {
            slot.kind = SLOT_X;
            slot.index = (uint8_t)(position - 10u);
        }
        break;
    default:
        break;
    }
    return slot;
}

static bool only_blanks(const char *s) {
    while (scpi_char_is_blank(*s)) {
        s++;
    }
    return *s == '\0';
}

static bool field_float(const char *text, float *out) {
    const char *end;

    return fmt_parse_float(text, &end, out) == FMT_OK && only_blanks(end);
}

/* The original read an integer field from its FIRST CHARACTER only ("12" was 1, "x" was garbage). The whole
 * field must be a number here, and it must fit the byte it goes into. */
static bool field_byte(const char *text, uint8_t *out) {
    const char *end;
    int32_t value;

    if (fmt_parse_long(text, &end, &value) != FMT_OK || !only_blanks(end) || value < 0 || value > 255) {
        return false;
    }
    *out = (uint8_t)value;
    return true;
}

static bool coef_store(cal_coef_t *coef, slot_t slot, const char *text) {
    uint8_t byte;
    float number;

    switch (slot.kind) {
    case SLOT_A:
    case SLOT_B:
        if (!field_byte(text, &byte)) {
            return false;
        }
        if (slot.kind == SLOT_A) {
            coef->a = byte;
        } else {
            coef->b = byte;
        }
        return true;
    case SLOT_NONE:
        return true;
    default:
        break;
    }
    if (!field_float(text, &number)) {
        return false;
    }
    if (slot.kind == SLOT_V) {
        coef->v[slot.index] = number;
    } else if (slot.kind == SLOT_Z) {
        coef->z[slot.index] = number;
    } else {
        coef->x[slot.index] = number;
    }
    return true;
}

/* Splits and converts the quoted list into typed fields. Reports and returns false on a field that is too long
 * or is not the number its place needs; a wrong COUNT is left to cal_coef_write(), which owns the layouts. */
static bool coef_parse(func_t f, const char *list, cal_coef_t *coef) {
    const char *cursor = list;
    char field[COEF_FIELD_CHARS];

    for (;;) {
        const scpi_csv_t status = scpi_csv_next_field(&cursor, field, sizeof field);

        if (status == SCPI_CSV_END) {
            return true;
        }
        if (status == SCPI_CSV_TOO_LONG) {
            scpi_fail_too_long();
            return false;
        }
        if (coef->count >= COEF_FIELDS_MAX) {
            coef->count = COEF_FIELDS_MAX + 1u;
            return true;
        }
        if (!coef_store(coef, coef_slot(f, coef->count), field)) {
            scpi_fail_data();
            return false;
        }
        coef->count++;
    }
}

/* :SCAL:COEF "<csv>" for the active function. On success the reply is "!" (0x03F52 replies it for functions
 * 0 to 4). The wire field order and the validation of values, bands, ranges and count are cal_coef_write()'s. */
static void scal_coef_write(void) {
    cal_coef_t coef = {.count = 0u};
    const func_t f = g_state.func;

    /* The quoted list is converted into the typed fields first, which frees the token buffer, and the end of
     * the statement is checked before anything is written. */
    if (!protected_begin() || !coef_parse(f, scpi_token(), &coef) || !require_statement_end()) {
        return;
    }
    switch (cal_coef_write(f, &coef)) {
    case CAL_COEF_OK:
        scpi_reply_bang();
        protected_done();
        break;
    case CAL_COEF_BAD_COUNT:
        scpi_fail_param_missing();
        break;
    case CAL_COEF_BAD_FUNCTION:
    case CAL_COEF_WRITE_FAILED:
        scpi_fail_cannot_execute();
        break;
    case CAL_COEF_LOCKED:
        scpi_fail_locked();
        break;
    default:
        scpi_fail_data();
        break;
    }
}

static void scal_coef(void) {
    if (scpi_next_token() == SCPI_TOK_STRING) {
        scal_coef_write();
    } else if (scpi_token_is_query()) {
        if (require_statement_end()) {
            scpi_scal_coef_query();
            protected_done();
        }
    } else {
        scpi_fail_parameter();
    }
}

/* ---- :SCAL:DATE and :SCAL:SAVE ------------------------------------------------------------------------------ */

#define CAL_DATE_CHARS 8u

static bool date_char_valid(char c) {
    return (c >= '0' && c <= '9') || c == '/';
}

/* :SCAL:DATE:<date> -- the colon form, as the original (0x07000). Exactly eight characters, each a digit or a
 * slash; the original wrote the first eight whatever the length and wrote even after a validation error. No
 * reply on success, as the original. The text is copied out of the token buffer before the end of the
 * statement is checked, because reading that token reuses the buffer. */
static void scal_date(void) {
    char date[CAL_DATE_CHARS + 1u];
    const char *token;

    if (!protected_begin()) {
        return;
    }
    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    scpi_next_token();
    if (scpi_line_failed()) {
        return;
    }
    token = scpi_token();
    if (strlen(token) != CAL_DATE_CHARS) {
        scpi_fail_data();
        return;
    }
    for (uint8_t i = 0u; i < CAL_DATE_CHARS; i++) {
        if (!date_char_valid(token[i])) {
            scpi_fail_bad_command();
            return;
        }
        date[i] = token[i];
    }
    date[CAL_DATE_CHARS] = '\0';
    if (!require_statement_end()) {
        return;
    }
    if (!cal_date_write(date)) {
        scpi_fail_cannot_execute();
        return;
    }
    protected_done();
}

/* :SCAL:SAVE -- increments the save counter and does nothing else (cal.h). No reply, as the original. */
static void scal_save(void) {
    if (!protected_begin() || !require_statement_end()) {
        return;
    }
    if (!cal_save_counter_increment()) {
        scpi_fail_cannot_execute();
        return;
    }
    protected_done();
}

/* `:SCAL:UNDO` (protected). Restores the record the last calibration write saved, panel or remote.
 * Refused rather than silently doing nothing when the slot is empty: "nothing to undo" and "undone" must
 * not look the same to a caller, because the coefficients cannot be read back to find out which happened. */
static void scal_undo(void) {
    if (!protected_begin() || !require_statement_end()) {
        return;
    }
    if (!cal_undo()) {
        scpi_fail_cannot_execute();
        return;
    }
    scpi_reply_bang();
    protected_done();
}

/* `:SCAL:CHECK?` (open). The number of calibration records that fail the signature or plausibility
 * test, then each one's name. Open rather than protected because it only reads, and because a user deciding
 * whether their meter needs attention should not have to unlock it first.
 *
 * The reply is the count, and then one line per bad record. A sound meter answers "0" and nothing else.
 *
 * The lines are streamed, one per main-loop pass (check_service). A cleared meter has every record bad,
 * which is up to 78 lines of about 12 characters, and uart1 paces each character (a 1 ms pause plus its
 * time on the wire): at 600 baud that is about 0.2 s a line and some 16 s for the list, against the 2 s
 * watchdog of main.c and a main loop that must keep measuring. A line is therefore the unit of blocking, as
 * it is for every other reply. cal_check_all() cannot be resumed, so each pass runs the scan again and sends
 * the record whose position is next; the scan reads the EEPROM only. While the list is going out no new
 * line is taken from the receive ring (remote_poll), so nothing can be sent between two of its lines, and
 * CHECK? must be the last statement of its line for the same reason: a statement after it would be answered
 * before the list it follows. */
static bool check_streaming;
static uint16_t check_total; /* bad records counted when the command ran */
static uint16_t check_sent;  /* lines already sent */
static uint16_t check_seen;  /* reports passed by the scan under way */

static void check_report(const char *name, uint16_t addr) {
    char text[24];
    line_t line = {.text = text, .size = sizeof text, .length = 0u};
    uint8_t i;

    if (check_seen++ != check_sent) {
        return;
    }
    for (i = 0u; name[i] != '\0'; i++) {
        line_char(&line, name[i]);
    }
    line_char(&line, ',');
    line_int(&line, (int32_t)addr);
    line_send(&line);
}

static void check_service(void) {
    const uint16_t index = check_sent;

    check_seen = 0u;
    (void)cal_check_all(check_report);
    check_sent++;
    /* The scan ran out of records before the count did: the EEPROM changed under it, so stop. */
    if (check_sent >= check_total || check_seen <= index) {
        check_streaming = false;
    }
}

static void scal_check(void) {
    scpi_next_token();
    if (!scpi_token_is_query()) {
        scpi_fail_bad_command();
        return;
    }
    scpi_next_token();
    if (scpi_token_type() != SCPI_TOK_SEP || !scpi_token_is_end()) {
        scpi_fail_syntax();
        return;
    }
    /* Counted first and reported as a number, then listed. A caller that only wants "is it sound?" reads
     * the first line and stops. */
    check_total = cal_check_all(NULL);
    remote_reply_int((int32_t)check_total);
    check_sent = 0u;
    check_streaming = (check_total != 0u);
}

static void scal_command(void) {
    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    if (scpi_next_token() != SCPI_TOK_WORD) {
        scpi_fail_bad_command();
    } else if (KW("open")) {
        scal_sweep(false);
    } else if (KW("shor")) {
        scal_sweep(true);
    } else if (KW("acv")) {
        scal_zero(true);
    } else if (KW("dcv")) {
        scal_zero(false);
    } else if (KW("std")) {
        scal_std();
    } else if (KW("test")) {
        scal_test();
    } else if (KW("coef")) {
        scal_coef();
    } else if (KW("date")) {
        scal_date();
    } else if (KW("save")) {
        scal_save();
    } else if (KW("undo")) {
        scal_undo();
    } else if (KW("check")) {
        scal_check();
    } else {
        scpi_fail_bad_command();
    }
}

/* ---- The line walk ----------------------------------------------------------------------------------------- */

static void dispatch(uint8_t root) {
    switch (root) {
    case ROOT_FUNCTION:
        scpi_cmd_function();
        break;
    case ROOT_FUNCTION2:
        scpi_cmd_function2();
        break;
    case ROOT_CALCULATE:
        scpi_cmd_calculate();
        break;
    case ROOT_HOLD:
        scpi_cmd_hold();
        break;
    case ROOT_CURRENT:
        scpi_cmd_current();
        break;
    case ROOT_VOLTAGE:
        scpi_cmd_voltage();
        break;
    case ROOT_RESISTANCE:
        scpi_cmd_resistance();
        break;
    case ROOT_FREQUENCY:
        scpi_cmd_frequency_period(FUNC_FREQ);
        break;
    case ROOT_PERIOD:
        scpi_cmd_frequency_period(FUNC_PERIOD);
        break;
    case ROOT_CONTINUITY:
        scpi_node_range_handler();
        break;
    case ROOT_FETCH:
        scpi_cmd_fetch();
        break;
    case ROOT_DISPLAY:
        scpi_cmd_display();
        break;
    case ROOT_UNIT:
        scpi_cmd_unit();
        break;
    case ROOT_TRIGGER:
        scpi_cmd_trigger();
        break;
    case ROOT_SCAL:
        scal_command();
        break;
    case ROOT_THID:
        scpi_cmd_thid();
        break;
    case ROOT_SYSTEM:
        scpi_cmd_system();
        break;
    case ROOT_SERV:
        scpi_cmd_serv();
        break;
    default:
        /* ROOT_DIODE is a valid keyword with no command behind it, as in the original (0x0FD4C). */
        scpi_fail_bad_command();
        break;
    }
}

typedef enum { ROOT_STEP_DISPATCH, ROOT_STEP_AGAIN, ROOT_STEP_END } root_step_t;

/* Reads what stands at depth 0: a root keyword, a '*' command, a leading ':' or the end. */
static root_step_t root_step(uint8_t *root) {
    const uint8_t type = scpi_next_token();
    uint8_t code;

    if (scpi_line_failed()) {
        return ROOT_STEP_END;
    }
    if (type == SCPI_TOK_SEP) {
        const char first = scpi_token()[0];

        if (first == ':') {
            return ROOT_STEP_AGAIN;
        }
        if (first == '*') {
            scpi_cmd_common_star();
            scpi_parse_separator();
            if (!scpi_line_stopped() && scpi_depth() != 0u) {
                scpi_fail_syntax();
            }
            return ROOT_STEP_AGAIN;
        }
        if (first != '\0') {
            scpi_fail_bad_command();
        }
        return ROOT_STEP_END;
    }
    if (type != SCPI_TOK_WORD) {
        scpi_fail_bad_command();
        return ROOT_STEP_END;
    }
    code = scpi_root_keyword_lookup(scpi_token());
    if (code == 0u && (KW("serv") || KW("service"))) {
        code = ROOT_SERV;
    }
    if (code == 0u) {
        scpi_fail_bad_command();
        return ROOT_STEP_END;
    }
    *root = code;
    scpi_set_depth(1u);
    return ROOT_STEP_DISPATCH;
}

/* Runs one line. It never disables interrupts and never waits: see the header. */
void remote_execute_line(char *line, uint16_t len) {
    uint8_t root = 0u;

    scpi_parser_begin(line, len);
    while (!scpi_line_stopped()) {
        if (scpi_depth() == 0u) {
            const root_step_t step = root_step(&root);

            if (step == ROOT_STEP_END) {
                break;
            }
            if (step == ROOT_STEP_AGAIN) {
                continue;
            }
        }
        if (cal_sweep_busy() && root_conflicts(root)) {
            scpi_fail_cannot_execute();
            break;
        }
        dispatch(root);
        if (scpi_line_stopped()) {
            break;
        }
        scpi_parse_separator();
    }
}

/* ---- The main-loop side ------------------------------------------------------------------------------------ */

static char line_buffer[UART1_LINE_MAX];

/* A reading that has just completed goes to ONE FETCh? that is waiting for it. main.c calls remote_poll()
 * after meas_task() and before meas_take_reading(), and MEAS_COMPLETE lasts from the first to the
 * second, so this sees each completed reading exactly once. remote_poll() answers before it runs the next
 * line, which means a request is never answered by a reading that completed before it was parsed: the reply
 * comes at the end of the NEXT measurement, as the original's does (0x115E2, called from meas_cycle). */
static void fetch_service(void) {
    if (meas_reading_ready() && scpi_fetch_pending()) {
        scpi_reply_reading();
    }
}

/* A byte lost or corrupted on the wire is reported as a data error. DEFAULT: the original appended a corrupt
 * byte to the line as data and so reported a syntax error with no explanation; a data error says what
 * happened. */
static void host_link_faults(void) {
    if (uart1_rx_overrun_take()) {
        remote_error_push(SCPI_ERR_DATA);
    }
    if (uart1_rx_frame_error_take()) {
        remote_error_push(SCPI_ERR_DATA);
    }

    /* The converter stopped answering. Without this call the fault would be latched and then thrown away,
     * and a dead converter would be indistinguishable from a meter that had not taken its first reading
     * yet: both show the restart dashes, because `enter_fault()` clears `reading.valid` and that is the
     * only thing the panel looks at.
     *
     * A host gets an error it can see.
     *
     * SCPI_ERR_DATA, and not a new code of its own. The original firmware has no converter-fault error at
     * all -- it had no timeout -- so there is no original code to match and a new enum
     * member would mean inventing the display text that goes with it. The two UART hardware faults above
     * are in the same position and already use SCPI_ERR_DATA, so this follows them. */
    if (meas_fault_take()) {
        remote_error_push(SCPI_ERR_DATA);
    }
}

void remote_init(void) {
    sweep_pending = false;
    query_pending = QUERY_NONE;
    check_streaming = false;
    scpi_reply_init();
}

/* Called once per main-loop pass, after meas_task() and before meas_take_reading(). Runs at most one line, and
 * none while a CHECK? list is still being sent. An over-long line (ORIGINAL_BUGS #7) is dropped whole and reported; it is never executed
 * truncated. */
void remote_poll(void) {
    host_link_faults();
    sweep_service();
    query_service();
    fetch_service();
    if (check_streaming) {
        check_service();
        return;
    }
    if (uart1_line_ready()) {
        if (uart1_line_overlong()) {
            (void)uart1_line_take(line_buffer, UART1_LINE_MAX);
            scpi_parser_begin("", 0u);
            scpi_fail_too_long();
        } else {
            const uint16_t length = uart1_line_take(line_buffer, UART1_LINE_MAX);

            remote_execute_line(line_buffer, length);
        }
    }
}

/* The front-panel LOCAL key (0x0FAD0). It also ends a running calibration sweep: control goes back to the
 * panel, and a sweep must not keep driving the front end underneath it. DEFAULT: nothing further is written
 * after the abort (cal.h); cells already written stay, each being one atomic record write. */
void remote_go_local(void) {
    check_streaming = false;
    if (cal_sweep_busy()) {
        cal_sweep_abort();
    }
    if (sweep_pending) {
        /* A client waits for the closing "!" of a sweep it started. The abort sends none, so the reason is
         * queued for :SYST:ERR? instead of leaving the client to find out by its own timeout. */
        sweep_pending = false;
        report_cannot_execute_between_lines();
    }
    if (query_pending != QUERY_NONE) {
        /* The same reasoning for a :SCAL:STD or :SCAL:TEST the LOCAL key cut short: the client is waiting
         * for a reply line that will now never come, so the reason is queued rather than left to a timeout. */
        query_pending = QUERY_NONE;
        report_cannot_execute_between_lines();
    }
    scpi_enter_local();
}
