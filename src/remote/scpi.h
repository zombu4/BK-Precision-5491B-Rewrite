/* scpi.h -- internal interface of the remote module, shared by parser.c, tree.c, handlers.c, reply.c and the
 * executor (remote_execute_line). It is not part of the public remote.h.
 *
 * The executor starts a line with scpi_parser_begin(), reads the root keyword with scpi_next_token(), looks it
 * up with scpi_root_keyword_lookup() and calls the root handler once per ':' level; scpi_parse_separator()
 * steps from one level to the next and scpi_line_stopped() says when the line is finished.
 */
#ifndef BK_SCPI_H
#define BK_SCPI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../state.h"
#include "remote.h"

/* ---- Token types --------------------------------------------------------------------------------------- */
#define SCPI_TOK_NONE 0u   /* invalid first character, or an error was reported while reading */
#define SCPI_TOK_SEP 1u    /* one delimiter character, or the end of the line (empty text) */
#define SCPI_TOK_NUMBER 2u /* starts with a digit or '-' */
#define SCPI_TOK_WORD 4u   /* starts with a letter */
#define SCPI_TOK_STRING 5u /* double quoted, case kept */

/* ---- Errors -------------------------------------------------------------------------------------------- */

/* An error is a pair of indexes into error_fragment[] (tables.h); the message is the first fragment followed
 * by the second. Index SCPI_FRAGMENT_LOCKED is not in the original table: it is the fragment of the service
 * lock refusal and lives in reply.c. */
#define SCPI_FRAGMENT_LOCKED 14u

void scpi_error_report(uint8_t first, uint8_t second);
#define scpi_fail_bad_command() scpi_error_report(1u, 2u) /* " BUS: BAD  COMMAND. " */
#define scpi_fail_syntax() scpi_error_report(3u, 4u)      /* "BUS:SYNTAX ERROR   " */
#define scpi_fail_data() scpi_error_report(5u, 4u)        /* "BUS: DATA  ERROR   " */
#define scpi_fail_too_long() scpi_error_report(5u, 6u)    /* "BUS: DATA TOO LONG " */
#define scpi_fail_cannot_execute() scpi_error_report(7u, 8u)
#define scpi_fail_suffix() scpi_error_report(9u, 10u)    /* unparsable number */
#define scpi_fail_parameter() scpi_error_report(9u, 11u) /* "BUS:ERROR PARAMETER." */
#define scpi_fail_param_error() scpi_error_report(11u, 4u)
#define scpi_fail_param_missing() scpi_error_report(11u, 13u)
#define scpi_fail_locked() scpi_error_report(SCPI_FRAGMENT_LOCKED, 2u) /* the service lock refused it */

typedef enum { SCPI_BOOL_OFF, SCPI_BOOL_ON, SCPI_BOOL_QUERY, SCPI_BOOL_OTHER } scpi_bool_arg_t;
typedef enum { SCPI_CSV_OK, SCPI_CSV_END, SCPI_CSV_TOO_LONG } scpi_csv_t;

/* ---- parser.c ------------------------------------------------------------------------------------------ */
void scpi_parser_begin(const char *line, uint16_t len);
uint8_t scpi_next_token(void);
void scpi_unget_token(void);
void scpi_parse_separator(void);
const char *scpi_token(void);
uint8_t scpi_token_type(void);
bool scpi_token_is_end(void);
bool scpi_token_is_query(void);
bool scpi_next_is_colon(void);
uint8_t scpi_depth(void);
void scpi_set_depth(uint8_t depth);
bool scpi_line_stopped(void);
bool scpi_line_failed(void);
void scpi_line_abort(void);
bool scpi_keyword_match(const char *keyword_P);
bool scpi_char_is_terminator(char c);
bool scpi_char_is_blank(char c);
bool scpi_arg_float(float *out);
bool scpi_arg_float_limited(float *out, float lo, float hi);
bool scpi_arg_long(int32_t *out);
scpi_bool_arg_t scpi_arg_bool(void);
scpi_csv_t scpi_csv_next_field(const char **cursor, char *dst, size_t dst_size);
void scpi_bus_trigger(void);
void scpi_cmd_common_star(void); /* serv/ */

/* ---- tree.c -------------------------------------------------------------------------------------------- */
uint8_t scpi_root_keyword_lookup(const char *token);
bool scpi_parse_range_token(func_t f, uint8_t *index);
bool scpi_range_text(func_t table_func, uint8_t index, char *dst, size_t dst_size);
void scpi_range_apply_refresh(func_t f);
void scpi_range_set_from_token(func_t f);
void scpi_range_reply(func_t f);
void scpi_range_upper_arg(func_t f);
void scpi_range_auto_arg(func_t f);
bool scpi_nplc_lookup(rate_t *rate);
void scpi_set_nplc(void);
void scpi_node_range_handler(void);

/* ---- handlers.c ---------------------------------------------------------------------------------------- */
void scpi_cmd_system(void);
void scpi_cmd_unit(void);
void scpi_cmd_calculate(void);
void scpi_cmd_function(void);
void scpi_cmd_function2(void);
void scpi_cmd_voltage(void);
void scpi_cmd_current(void);
void scpi_cmd_resistance(void);
void scpi_cmd_frequency_period(func_t f); /* FUNC_FREQ or FUNC_PERIOD, whichever root was typed */
void scpi_cmd_display(void);
void scpi_cmd_hold(void);
void scpi_cmd_fetch(void);
void scpi_cmd_trigger(void);
void scpi_cmd_serv(void);
void scpi_scal_coef_query(void);
void scpi_function_apply(void);
void scpi_func2_apply(void);

/* Decimals of the %e reply layouts: 6 for the CALC, UNIT, FREQuency:REFerence and HOLD
 * replies, 8 for FETCh? and the V/I/R REFerence? replies. */
#define SCPI_DECIMALS_STANDARD 6u
#define SCPI_DECIMALS_READING 8u

/* ---- reply.c ------------------------------------------------------------------------------------------- */
void scpi_reply_bool(bool value);
void scpi_reply_1(void);
void scpi_reply_0(void);
void scpi_reply_bang(void);
void scpi_reply_ram_string(const char *text);
void scpi_reply_float_clamped(float value, uint8_t decimals);
void scpi_reply_reference(func_t f, uint8_t decimals);
void scpi_reply_reading(void);
void scpi_reply_hold_window(void);
void scpi_reply_hold_count(void);
void scpi_reply_last_error(void);
bool scpi_fetch_request(void); /* false when the pending count is full: nothing was queued */
bool scpi_fetch_pending(void);
void scpi_reply_init(void); /* empties the error queue and the FETCh? count */
void scpi_enter_local(void);

/* Added 2026-10-22. The :THID subsystem is handled in serv/thid.c, not in handlers.c, because every
 * write in it is a protected operation and belongs with the lock that guards it. Like the original
 * (0x04DBC) it consumes the WHOLE header path, so the executor calls it once, straight after the root
 * keyword, and does not walk the colons itself. */
void scpi_cmd_thid(void);

#endif /* BK_SCPI_H */
