/* remote.h -- SCPI parser, command tree, leaf handlers and reply formatting.
 * FROZEN. Parser, tree, handlers and replies.
 * The EXECUTOR remote_execute_line() is where the ring buffer meets the parser and is not to be
 * reimplemented elsewhere.
 *
 * The parser drains the uart1 ring buffer from the main loop. It must NEVER disable
 * interrupts for a whole command line and must never busy-wait on an error (the original stalled 1 s
 * and lost every byte that arrived meanwhile).
 * An error aborts the handler BEFORE any state change or EEPROM write.
 * No fixed-size overflow; an unterminated quoted string is an error, never stale bytes;
 * the dead "scalibration" root keyword is not reproduced.
 * Protected commands call serv_check() first (serv.h).
 *
 * Reply terminator: LF, or CR when g_state.settings.tx_terminator is non-zero.
 * The :SCAL:SHOR:ONCE / :SCAL:OPEN:ONCE replies end with '!'.
 * Number formatting and scanning go through fmt.h -- do not call avr-libc printf/strtod directly,
 * because the reply format is byte-exact and the input accept/reject set is deliberate.
 */
#ifndef BK_REMOTE_H
#define BK_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

#define SCPI_TOKEN_MAX 256u

typedef enum {
    SCPI_ERR_NONE = 0,
    SCPI_ERR_BAD_COMMAND, /* "BUS: BAD  COMMAND." */
    SCPI_ERR_SYNTAX,
    SCPI_ERR_PARAMETER,
    SCPI_ERR_DATA,
    SCPI_ERR_LOCKED /* the service lock refused it */
} scpi_err_t;

void remote_init(void);
void remote_poll(void); /* called from the main loop; drains uart1 */
/* remote_poll() must run once per pass, AFTER meas_task() and BEFORE meas_take_reading(): FETCh? replies are
 * sent from the pass in which a reading completes, and that phase exists only between those two calls. */

void remote_execute_line(char *line, uint16_t len); /* the executor lives in exec.c */

void remote_error_push(scpi_err_t e);

/* Puts the ERR lamp out. The queued errors stay readable with `:SYST:ERR?` -- this acknowledges the
 * indication, it does not discard the evidence. Called from the front panel on any key. */
void remote_error_acknowledge(void);

void remote_reply(const char *s);
void remote_reply_P(const char *s_flash);
void remote_reply_float(float v);
void remote_reply_int(int32_t v);

/* Amendment 2026-10-22: the front-panel LOCAL action (0xFAD0) -- the only key honoured
 * while remote -- had no home. It belongs here and aliases scpi_enter_local(). */
void remote_go_local(void);

#endif /* BK_REMOTE_H */
