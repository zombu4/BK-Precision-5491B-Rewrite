/* uart1.h -- host serial link (USART1) with an ISR-fed receive ring buffer.
 * FROZEN. The remote code consumes this; it must not poll the UART directly.
 *
 * The original ran a whole command line with interrupts disabled and stalled 1 s on an
 * error, losing bytes. Here the RX ISR only enqueues; the parser drains the buffer in the main loop.
 * Echo (settings.echo) is applied by the ISR, exactly as the original did, so timing is unchanged.
 */
#ifndef BK_UART1_H
#define BK_UART1_H
#include <stdbool.h>
#include <stdint.h>
#define UART1_RX_RING 256u   /* >= 128 needed to keep a whole line */
#define UART1_LINE_MAX 256u  /* original line buffer was 256 B (RAM 0x0CC3) */
void uart1_init(uint8_t baud_index);
void uart1_set_baud(uint8_t baud_index);   /* writes the full 12-bit divisor (UBRR1H+L) */
void uart1_putc(char c);
void uart1_write(const char *s);
void uart1_write_P(const char *s_flash);
void uart1_write_terminator(void);         /* LF, or CR when settings.tx_terminator != 0 */
/* A terminator has arrived and the line before it is whole. A line with a byte lost on the wire (framing or
 * parity error, USART overrun, full ring) is never reported here: it is dropped and the loss is reported
 * through uart1_rx_overrun_take() / uart1_rx_frame_error_take(). */
bool uart1_line_ready(void);

/* Amendment 2026-10-22, additive. ORIGINAL_BUGS #7: a line of 255 characters or more had its extra
 * bytes dropped and the TRUNCATED LINE WAS THEN EXECUTED, so a cut-off `:SCAL:COEF` or `:SCAL:DATE`
 * would run and write whichever fields survived into calibration memory. The line is now flagged
 * instead. Call this after uart1_line_ready() returns true and BEFORE uart1_line_take(), which
 * clears it: when it is true, report a data error and execute nothing. */
bool uart1_line_overlong(void);
uint16_t uart1_line_take(char *dst, uint16_t dst_size); /* copies and clears; returns length */
bool uart1_rx_overrun_take(void);          /* true once per overrun, for the error queue */

/* The original's receive ISR read UDR1 without looking at the error flags
 * (0x0FAF6), so a byte received with a framing or parity error was appended to the command line as data --
 * a line corrupted on the wire became a command, or a syntax error with no explanation. Such a byte is now
 * discarded and flagged here, so the error queue can say what happened instead of reporting a bad command.
 * The remote executor drains this alongside the overrun flag. */
bool uart1_rx_frame_error_take(void);
#endif
