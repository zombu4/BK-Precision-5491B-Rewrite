/* parser.c -- SCPI tokenizer, header separators and argument scanning.
 *
 * The executor starts a line with scpi_parser_begin(), reads the root keyword
 * with scpi_next_token(), looks it up with scpi_root_keyword_lookup() and calls the root handler once per
 * ':' level; scpi_parse_separator() steps from one level to the next.
 *
 * The original accept/reject set is kept: delimiters ; : , ? * blank and the single
 * quote; numbers start with a digit or '-'; words start with a letter; everything but quoted strings is
 * lower-cased; there is no keyword abbreviation. Differences, all memory-safety fixes:
 *   - tokens are bounded by SCPI_TOKEN_MAX; the line is bounded by the length the executor passes;
 *   - an unterminated quoted string is a syntax error and yields no token, never stale buffer bytes;
 *   - ";*cmd;*cmd" chains are a loop, not recursion.
 */
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../fmt/fmt.h"
#include "../state.h"
#include "scpi.h"

typedef struct {
    const char *cursor;    /* next unread character of the line */
    const char *end;       /* one past the last character of the line */
    const char *tok_begin; /* where the current token started, after skipped blanks */
    uint8_t depth;         /* ':' levels consumed in the current header */
    uint8_t tok_type;
    bool tok_end; /* the current token is the end-of-line marker */
    bool stop;    /* nothing more to execute on this line */
    bool failed;  /* an error was reported on this line */
    char tok[SCPI_TOKEN_MAX];
} parser_t;

static parser_t p;

/* ---- Character classes --------------------------------------------------------------------------------- */

bool scpi_char_is_blank(char c) {
    return c == ' ' || c == '\t';
}

/* The delimiter set ; : , ? * blank ' plus the end of the text. */
bool scpi_char_is_terminator(char c) {
    switch (c) {
    case '\0':
    case ';':
    case ':':
    case ',':
    case '?':
    case '*':
    case ' ':
    case '\'':
        return true;
    default:
        return false;
    }
}

static bool is_digit(unsigned char c) {
    return c >= '0' && c <= '9';
}

static bool is_alpha(unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

static char to_lower(unsigned char c) {
    return (char)((c >= 'A' && c <= 'Z') ? c + ('a' - 'A') : c);
}

static unsigned char peek(void) {
    return p.cursor < p.end ? (unsigned char)*p.cursor : 0u;
}

static void advance(void) {
    if (p.cursor < p.end) {
        p.cursor++;
    }
}

/* ---- Line state ---------------------------------------------------------------------------------------- */

void scpi_parser_begin(const char *line, uint16_t len) {
    p.cursor = line;
    p.end = line + len;
    p.tok_begin = line;
    p.depth = 0u;
    p.tok_type = SCPI_TOK_NONE;
    p.tok_end = false;
    p.stop = (len == 0u);
    p.failed = false;
    p.tok[0] = '\0';
}

const char *scpi_token(void) {
    return p.tok;
}
uint8_t scpi_token_type(void) {
    return p.tok_type;
}
bool scpi_token_is_end(void) {
    return p.tok_end;
}
bool scpi_token_is_query(void) {
    return p.tok_type == SCPI_TOK_SEP && p.tok[0] == '?';
}
uint8_t scpi_depth(void) {
    return p.depth;
}
void scpi_set_depth(uint8_t depth) {
    p.depth = depth;
}
bool scpi_line_stopped(void) {
    return p.stop;
}
bool scpi_line_failed(void) {
    return p.failed;
}

/* An error ends the line: nothing after it runs. */
void scpi_line_abort(void) {
    p.failed = true;
    p.stop = true;
}

bool scpi_keyword_match(const char *keyword_P) {
    return strcmp_P(p.tok, keyword_P) == 0;
}

/* ---- Tokenizer ----------------------------------------------------------------------------------------- */

static void token_reset(void) {
    p.tok_type = SCPI_TOK_NONE;
    p.tok_end = false;
    p.tok[0] = '\0';
    p.tok_begin = p.cursor;
}

/* Appends one character; the flag records an overflow instead of writing past the buffer. */
static void token_put(size_t *length, bool *overflow, char c) {
    if (*length < SCPI_TOKEN_MAX - 1u) {
        p.tok[(*length)++] = c;
    } else {
        *overflow = true;
    }
}

static uint8_t token_finish(size_t length, bool overflow, uint8_t type) {
    p.tok[length] = '\0';
    if (overflow) {
        p.tok[0] = '\0';
        scpi_fail_too_long();
        return p.tok_type;
    }
    p.tok_type = type;
    return type;
}

static uint8_t read_string(void) {
    size_t length = 0u;
    bool overflow = false;
    unsigned char c;

    advance(); /* the opening quote */
    for (c = peek(); c != '"' && c != '\r' && c != '\n' && c != '\0'; c = peek()) {
        token_put(&length, &overflow, (char)c);
        advance();
    }
    if (c != '"') {
        p.tok[0] = '\0';
        scpi_fail_syntax();
        return p.tok_type;
    }
    advance(); /* the closing quote */
    return token_finish(length, overflow, SCPI_TOK_STRING);
}

static uint8_t read_bare(uint8_t type) {
    size_t length = 0u;
    bool overflow = false;

    while (!scpi_char_is_terminator((char)peek())) {
        token_put(&length, &overflow, to_lower(peek()));
        advance();
    }
    return token_finish(length, overflow, type);
}

uint8_t scpi_next_token(void) {
    unsigned char c;

    token_reset();
    c = peek();
    while (scpi_char_is_blank((char)c)) {
        advance();
        c = peek();
    }
    p.tok_begin = p.cursor;
    if (c == '\0') {
        /* The end of the line, with or without blanks in front of it. The original made blanks that run into
         * the end a separator token with empty text that is not the end marker (0x0FC56-0x0FC78), which every
         * separator test then rejected: `:FUNC? ` and `:HOLD:STAT ON ` were syntax errors. A trailing blank
         * is harmless to a client and the planned row of PROTOCOL_CHANGES section C accepts it. */
        p.tok_end = true;
        p.tok_type = SCPI_TOK_SEP;
        return p.tok_type;
    }
    if (scpi_char_is_terminator((char)c)) {
        /* One delimiter is a token of its own. */
        p.tok[0] = (char)c;
        p.tok[1] = '\0';
        advance();
        p.tok_type = SCPI_TOK_SEP;
        return p.tok_type;
    }
    if (c == '"') {
        return read_string();
    }
    if (is_digit(c) || c == '-') {
        return read_bare(SCPI_TOK_NUMBER);
    }
    if (is_alpha(c)) {
        return read_bare(SCPI_TOK_WORD);
    }
    return p.tok_type; /* SCPI_TOK_NONE: the cursor stays, the caller reports the error */
}

/* Gives the current token back, so that the next read returns it again. */
void scpi_unget_token(void) {
    p.cursor = p.tok_begin;
}

bool scpi_next_is_colon(void) {
    scpi_next_token();
    return p.tok_type == SCPI_TOK_SEP && p.tok[0] == ':';
}

/* ---- Header separators --------------------------------------------------------------------------------- */

/* Runs after a root handler: reads the separator that follows and moves to the next level or statement.
 *   end of line         stop
 *   ':'                 one level deeper
 *   ';' then ':'        a new header from the root
 *   ';' then '*'        a common command, then the next separator
 *   ';' then a word     the next statement at the same level (the word is given back)
 *   anything else       syntax error
 *
 * Nothing is read once the line has stopped. The original went on to the next statement after an error
 * (0xFD50 loops to the separator), so `*BAD;*RST` reported a bad command and then restarted the meter; an
 * error ends the line here, and that has to hold for a `*` command reached through this function
 * as much as for a handler reached through the executor. */
void scpi_parse_separator(void) {
    while (!p.stop) {
        scpi_next_token();
        if (p.tok_type != SCPI_TOK_SEP) {
            scpi_fail_syntax();
            return;
        }
        if (p.tok_end) {
            p.stop = true;
            return;
        }
        if (p.tok[0] == ';') {
            scpi_next_token();
            if (p.tok[0] == ':') {
                p.depth = 0u;
                return;
            }
            if (p.tok[0] == '*') {
                scpi_cmd_common_star();
                continue;
            }
            if (p.tok_end) {
                p.stop = true;
                return;
            }
            scpi_unget_token();
            return;
        }
        if (p.tok[0] == ':') {
            p.depth++;
            return;
        }
        scpi_fail_syntax();
        return;
    }
}

/* ---- Argument scanning --------------------------------------------------------------------------------- */

/* True when nothing but blanks follows the number.
 *
 * The original ignored whatever came after a number, so "12abc" was 12, "1,5" was 1 and "1 5" was 1
 * (a10 sec.4.2 item 6). ORIGINAL_BUGS #21: that silently misreads a mistyped parameter, so trailing text
 * is now a suffix error. The original had no unit-suffix table either, so nothing legitimate is lost. */
static bool only_blanks(const char *s) {
    while (*s == ' ' || (*s >= '\t' && *s <= '\r')) {
        s++;
    }
    return *s == '\0';
}

/* The current number token as a float. Unparsable text is "ERROR SUFFIX" as in the original; an
 * out-of-range exponent is a data error instead of a silently wrong value; a parameter of zero is
 * accepted, which the original refused. Reports and returns false. */
bool scpi_arg_float(float *out) {
    const char *end;

    switch (fmt_parse_float(p.tok, &end, out)) {
    case FMT_OK:
        if (!only_blanks(end)) {
            break;
        }
        return true;
    case FMT_ERR_RANGE:
        scpi_fail_data();
        break;
    case FMT_ERR_OVERFLOW:
        scpi_fail_too_long();
        break;
    default:
        scpi_fail_suffix();
        break;
    }
    return false;
}

/* A float parameter that must lie in lo..hi, both inclusive; outside it is a data error and *out is not
 * written. This is the original's scpi_parse_float_checked (0x07D10), which wraps the float parser 0x12498
 * with the test -1e8 <= x <= 1e8 and reports error (5,4) (data error). The original reports and then RETURNS
 * THE VALUE: the compressed call at 0x07D40 is `ldi r18,4 ; ldi r16,5 ; call 0x123E4 ; ret`, 0x123E4 comes
 * back, and 0x07D44 reloads the float from Y+0..3 (in the original listing), so the caller stored an
 * out-of-range number after the error. Here the error is fatal to the command and *out is
 * left alone. */
bool scpi_arg_float_limited(float *out, float lo, float hi) {
    float value;

    if (!scpi_arg_float(&value)) {
        return false;
    }
    if (value < lo || value > hi) {
        scpi_fail_data();
        return false;
    }
    *out = value;
    return true;
}

bool scpi_arg_long(int32_t *out) {
    const char *end;

    /* ORIGINAL_BUGS #22: the original's 16-bit atoi stopped at the first non-digit, so "5.7" became 5 for
     * HOLD:COUNt. fmt_parse_long stops there too, and the trailing-text check turns it into an error. */
    if (fmt_parse_long(p.tok, &end, out) == FMT_OK && only_blanks(end)) {
        return true;
    }
    scpi_fail_data();
    return false;
}

/* Reads the next token as an ON/OFF argument. "1" and "0" are accepted as words, whatever their type. */
scpi_bool_arg_t scpi_arg_bool(void) {
    scpi_next_token();
    if (scpi_keyword_match(PSTR("on")) || scpi_keyword_match(PSTR("1"))) {
        return SCPI_BOOL_ON;
    }
    if (scpi_keyword_match(PSTR("off")) || scpi_keyword_match(PSTR("0"))) {
        return SCPI_BOOL_OFF;
    }
    if (scpi_token_is_query()) {
        return SCPI_BOOL_QUERY;
    }
    return SCPI_BOOL_OTHER;
}

/* Next comma separated field of a quoted list. Leading blanks are skipped and the comma is consumed. Unlike
 * the original, a list that ends early is reported (SCPI_CSV_END) instead of repeating the previous field,
 * and a field that does not fit the buffer is SCPI_CSV_TOO_LONG instead of overflowing it. */
scpi_csv_t scpi_csv_next_field(const char **cursor, char *dst, size_t dst_size) {
    const char *s = *cursor;
    size_t length = 0u;

    while (scpi_char_is_blank(*s)) {
        s++;
    }
    if (*s == '\0') {
        *cursor = s;
        dst[0] = '\0';
        return SCPI_CSV_END;
    }
    while (*s != ',' && *s != '\0') {
        if (length + 1u >= dst_size) {
            dst[0] = '\0';
            return SCPI_CSV_TOO_LONG;
        }
        dst[length++] = *s++;
    }
    dst[length] = '\0';
    if (*s == ',') {
        s++;
    }
    *cursor = s;
    return SCPI_CSV_OK;
}

/* ---- Bus trigger --------------------------------------------------------------------------------------- */

/* *TRG: honoured only while the trigger source is BUS. */
void scpi_bus_trigger(void) {
    if (g_state.trigger == TRIG_BUS) {
        g_state.bus_trigger_pending = true;
    }
}
