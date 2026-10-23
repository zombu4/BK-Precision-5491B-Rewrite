/* uart1.c -- host serial link (USART1).
 *
 * Replaces uart1_init_baud 0x01DBA, uart1_putc 0x01E3A, uart1_send_string 0x01E48 and the receive
 * interrupt isr_usart1_rx 0x0FAF6.
 *
 * USART1 on PD2/PD3, 8 data bits, 1 stop bit, no parity
 * (UCSR1C = 0x06), receive interrupt enabled, transmit polled.
 *
 * ---- The reason this file is shaped the way it is -------------------------------------------------
 * The original ran a whole command line with interrupts DISABLED: the executor at 0x0FBEA does `cli` on
 * entry and `sei` on exit, and the error routine 0x123E4 then busy-waits 1000 ms to show its message with
 * interrupts still off. The UART's two-byte FIFO overflows long before that second is up, so every byte a
 * host sends during a command error is lost. That is the ~1 s input lockout measured on the real
 * instrument. The same happens during the calibration sweeps, which take tens of seconds.
 *
 * The fix is structural, not a tuning change: the interrupt handler does the minimum -- echo, then put the
 * byte in a ring -- and the line is assembled and executed in the main loop, which may take as long as it
 * likes without losing anything. 256 bytes of ring is 26 character times at 9600 baud.
 *
 * The original also DISCARDED every byte that arrived while a completed line was still waiting to run
 * (RAM 0x0F62 bit 1), so a host that pipelined two commands lost the second. Here a second line simply
 * queues in the ring. That is a deliberate, client-visible improvement and is recorded in
 * PROTOCOL_CHANGES.md.
 *
 * ---- What is reproduced exactly ------------------------------------------------------------------------
 *  - Echo happens in the ISR, immediately, including for CR and LF, when EEPROM 0x865 says so. Keeping it
 *    there keeps the echo interleaved with the input the way a host sees it today.
 *  - Line rules: any byte except LF or CR is appended while the length is under 255; LF or CR ends the
 *    line; a full buffer ends the line too, dropping the byte that would not fit.
 *  - A completed line turns the remote annunciator on.
 *
 * Two of the original's line rules are deliberately NOT reproduced, both by design decision:
 * an over-length line is refused instead of being executed truncated (ORIGINAL_BUGS #7), and an
 * empty line is ignored instead of running an empty command and lighting REM (#6, which also makes CR LF
 * one terminator).
 *  - One terminator byte per reply, LF by default and CR when EEPROM 0x863 is non-zero, never both.
 *  - A 1 ms pause after each transmitted character.
 *
 * ---- Other defects fixed here ---------------------------------------------------------------------------
 *  - ORIGINAL_BUGS #89: the original wrote only the low byte of the baud divisor (UBRR1H forced to 0), so
 *    2400, 1200 and 600 baud actually ran at about 13393, 6637 and 3318 baud -- three of the seven
 *    selectable rates simply did not work. The full 12-bit divisor is written.
 *  - ORIGINAL_BUGS #90: 38400 used divisor 18, truncated from 18.53, giving 39474 baud -- +2.8 %, outside
 *    the usual 2 % tolerance, so 38400 was unreliable rather than merely imprecise. It now uses the
 *    double-speed mode, where 12 MHz divides cleanly: +0.16 %, the same accuracy as every other rate.
 *  - The original's ISR read UDR1 without looking at the error flags, so a byte received with a framing or
 *    parity error entered the command line as data. Such a byte is now discarded and reported.
 */
#include "uart1.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "../board.h"
#include "../state.h"

/* ---- Baud table ---------------------------------------------------------------------------------------
 * Rate = 600 * 2^(6 - index), with the index at EEPROM 0x854: 0 = 38400 ... 6 = 600.
 * A typical unit is index 2, 9600 baud.
 *
 * The divisor is truncated, as the original's was, NOT rounded. That is deliberate: truncation is what
 * produced the +0.16 % rates of 19200, 9600, 4800 and 2400 that work today and that the vendor software is
 * known to talk to. Rounding would shift 2400 by one count for no gain, and changing a rate that works is
 * a risk with no upside. The defect was never the truncation; it was the missing high byte.
 *
 * Arithmetic at 12 MHz, which reproduces ORIGINAL_BUGS #89's measured figures exactly and is the reason I
 * am confident this model is right:
 *   index 1..6 -> UBRR 38, 77, 155, 311, 624, 1249 -> 19230.8, 9615.4, 4807.7, 2403.8, 1200.0, 600.0
 *   the original kept only the low byte, so 311 became 55, 624 became 112 and 1249 became 225, which give
 *   13393, 6637 and 3319 baud -- the three broken rates, to the digit.
 *   index 0 uses U2X: UBRR 38 at 8x -> 38461.5 baud, +0.16 %.
 */
#define UART1_BAUD_INDEX_MAX 6u
#define UART1_BAUD_38400_INDEX 0u

static uint32_t baud_of_index(uint8_t index) {
    if (index > UART1_BAUD_INDEX_MAX) {
        index = 2u; /* an out-of-range EEPROM byte falls back to 9600, not to a wild divisor */
    }
    return 600UL << (UART1_BAUD_INDEX_MAX - index);
}

/* ---- Receive ring ------------------------------------------------------------------------------------- */

/* The ring is exactly 256 bytes, so uint8_t indices wrap on their own and no masking or bounds test is
 * needed in the interrupt handler. The assert keeps that true if the size is ever edited. */
_Static_assert(UART1_RX_RING == 256u, "the uint8_t ring indices below depend on a 256-byte ring");

static volatile uint8_t ring[UART1_RX_RING];
static volatile uint8_t ring_head;
static volatile uint8_t ring_tail;
static volatile bool rx_overrun;
static volatile bool rx_frame_error;

/* One bit per ring slot: set when a byte was lost on the wire immediately before that slot's
 * byte (framing or parity error, USART overrun, or a full ring). The line assembler turns it into "the line
 * being built has a hole" and refuses to hand that line out, because a hole in a :SCAL:COEF string is a
 * different number that still parses. `lost_pending` carries the loss from the handler call that saw it to
 * the next byte that is actually stored. Both belong to the receive handler; the main loop only reads a
 * slot's bit after the head index has passed it. */
static volatile uint8_t ring_gap[UART1_RX_RING / 8u];
static volatile bool lost_pending;

/* ---- Line assembly (main loop, never the ISR) --------------------------------------------------------- */

static char line[UART1_LINE_MAX];
static uint16_t line_len;
static bool line_complete;
static bool line_overlong;
static bool line_damaged;  /* a byte was lost somewhere in the line being built */
static bool line_skipping; /* dropping the tail of an over-long line, up to its terminator */

/* ---- Echo ---------------------------------------------------------------------------------------------
 * ORIGINAL_BUGS #5: the original echoed from inside the receive interrupt with a BLOCKING wait on UDRE1
 * (0x0FAF6). Normally that wait is nothing, because an echo runs at the same rate as the input. But if the
 * transmitter happens to be in the middle of a reply when a byte arrives, the handler stalls for a whole
 * character time with interrupts off, and the receive FIFO is two bytes deep -- so a host that talks while
 * a reply is going out can lose input.
 *
 * The echo still happens in the interrupt, in order, and in the common case it is still written straight to
 * UDR1, so what a host sees is unchanged. The difference is that the handler never waits: a byte that
 * cannot go out immediately is parked in a small ring and sent by the next interrupt or by the main loop,
 * whichever reaches it first. Order is preserved because every writer drains the ring before writing a new
 * byte directly.
 *
 * Echo bytes are deliberately NOT paced at 1 ms. That pacing belongs to replies; the
 * original's echo went straight out, and pacing it would make typing feel broken at 600 baud. */
#define UART1_ECHO_RING 32u
_Static_assert(UART1_ECHO_RING == 32u, "the index masking below depends on a power-of-two ring");

static volatile uint8_t echo_ring[UART1_ECHO_RING];
static volatile uint8_t echo_head;
static volatile uint8_t echo_tail;

/* Sends as much parked echo as the transmitter will take, without ever waiting. Safe to call from the ISR
 * and from the main loop: it only ever advances the tail, and both callers hold interrupts off. */
static void echo_drain(void) {
    while (echo_head != echo_tail && (UCSR1A & (1u << UDRE1)) != 0u) {
        UDR1 = echo_ring[echo_tail];
        echo_tail = (uint8_t)((echo_tail + 1u) & (UART1_ECHO_RING - 1u));
    }
}

static void echo_byte(uint8_t c) {
    uint8_t next;

    echo_drain(); /* keep the order: anything parked goes first */
    if (echo_head == echo_tail && (UCSR1A & (1u << UDRE1)) != 0u) {
        UDR1 = c; /* the common case, identical to the original */
        return;
    }
    next = (uint8_t)((echo_head + 1u) & (UART1_ECHO_RING - 1u));
    if (next == echo_tail) {
        return; /* 32 characters behind with echo on: drop it rather than stall the receiver */
    }
    echo_ring[echo_head] = c;
    echo_head = next;
}

/* ---- Transmit ----------------------------------------------------------------------------------------- */

/* Longest wait for the transmitter to take one byte. The worst legitimate case is a full echo ring (31
 * parked bytes) plus the byte in the shift register and the one in UDR1 at 600 baud, 10 bits / 600 =
 * 16.67 ms each: 33 * 16.67 ms = 550 ms. 20000 steps of 50 us is 1.0 s plus loop overhead, so a transmitter
 * that is merely slow never trips it. Only a transmitter that is not running at all does. */
#define UART1_TX_WAIT_STEP_US 50u
#define UART1_TX_WAIT_STEPS 20000u

void uart1_putc(char c) {
    /* Any echo still parked goes out first, so a reply can never overtake the echo of the command that
     * asked for it. The original's echo blocked, which gave that ordering for free; here it has to be
     * stated.
     *
     * The "is the data register free" test and the write to it are one atomic step. With them
     * apart, a receive interrupt between the two could echo into UDR1 and the write that follows would
     * overwrite the echo byte (ATmega128 USART: UDR1 is a single buffer, writing it while UDRE1 is 0
     * replaces the pending byte).
     *
     * The wait is bounded and feeds the watchdog. The watchdog period is 1.8 s at 5 V (armed as
     * WDTO_2S in main.c); a 113 character reply at 600 baud takes 113 * (16.67 + 1) ms = 2.0 s. */
    for (uint16_t step = 0u; step < UART1_TX_WAIT_STEPS; step++) {
        bool sent = false;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            echo_drain();
            if (echo_head == echo_tail && (UCSR1A & (1u << UDRE1)) != 0u) {
                UDR1 = (uint8_t)c;
                sent = true;
            }
        }
        if (sent) {
            return;
        }
        wdt_reset();
        _delay_us(UART1_TX_WAIT_STEP_US);
    }
}

/* The original paused 1 ms after each transmitted character (0x01E48). That pacing is
 * visible to a host -- it is why a long reply arrives at a steady rate rather than at line speed -- so it
 * is reproduced rather than optimised away. */
static void putc_paced(char c) {
    uart1_putc(c);
    wdt_reset();
    _delay_ms(1);
}

void uart1_write(const char *s) {
    while (*s != '\0') {
        putc_paced(*s++);
    }
}

void uart1_write_P(const char *s_flash) {
    char c;

    while ((c = (char)pgm_read_byte(s_flash++)) != '\0') {
        putc_paced(c);
    }
}

/* One byte, never CR+LF: LF when EEPROM 0x863 is 0 (the default), CR otherwise. */
void uart1_write_terminator(void) {
    putc_paced(g_state.settings.tx_terminator == 0u ? '\n' : '\r');
}

/* ---- Receive interrupt -------------------------------------------------------------------------------- */

ISR(USART1_RX_vect) {
    /* UCSR1A must be read BEFORE UDR1: reading the data register clears the error flags that belong to the
     * byte being read. The original read UDR1 first and never looked at them. */
    const uint8_t status = UCSR1A;
    const uint8_t c = UDR1; /* always read, so that RXC clears even for a byte we drop */
    bool gap = lost_pending;
    uint8_t next;

    lost_pending = false;
    if ((status & ((1u << FE1) | (1u << UPE1))) != 0u) {
        rx_frame_error = true; /* a corrupt byte must not enter the command line as data */
        lost_pending = true;   /* it is a hole in whatever line it belonged to */
        return;
    }
    if ((status & (1u << DOR1)) != 0u) {
        /* The hardware FIFO already lost at least one byte. ATmega128 doc2467 does not say on which side of
         * this byte the lost one falls, so both this byte and the next stored one are marked. */
        rx_overrun = true;
        gap = true;
        lost_pending = true;
    }

    if (g_state.settings.echo) {
        echo_byte(c);
    }

    next = (uint8_t)(ring_head + 1u);
    if (next == ring_tail) {
        rx_overrun = true; /* ring full: drop this byte and tell the error queue */
        lost_pending = true;
        return;
    }
    if (gap) {
        ring_gap[ring_head >> 3] |= (uint8_t)(1u << (ring_head & 7u));
    } else {
        ring_gap[ring_head >> 3] &= (uint8_t)~(1u << (ring_head & 7u));
    }
    ring[ring_head] = c;
    ring_head = next;
}

/* ---- Ring access -------------------------------------------------------------------------------------- */

/* Takes the oldest byte and tells whether a byte was lost on the wire just before it. */
static bool ring_pop(char *out, bool *gap) {
    const uint8_t slot = ring_tail;

    if (ring_head == slot) {
        return false;
    }
    *out = (char)ring[slot];
    *gap = (ring_gap[slot >> 3] & (1u << (slot & 7u))) != 0u;
    ring_tail = (uint8_t)(slot + 1u);
    return true;
}

bool uart1_rx_overrun_take(void) {
    bool flagged;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        flagged = rx_overrun;
        rx_overrun = false;
    }
    return flagged;
}

bool uart1_rx_frame_error_take(void) {
    bool flagged;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        flagged = rx_frame_error;
        rx_frame_error = false;
    }
    return flagged;
}

/* ---- Lines -------------------------------------------------------------------------------------------- */

static bool is_terminator(char c) {
    return c == '\n' || c == '\r';
}

/* Drains the ring into the line buffer until a line ends. The original's rules:
 * append anything that is not LF or CR while the length is under 255; LF or CR ends the line. Differences,
 * all deliberate:
 *  - an empty line is ignored (ORIGINAL_BUGS #6);
 *  - a line over 255 characters is reported, not executed truncated (ORIGINAL_BUGS #7), and the rest of it is
 *    dropped up to its terminator whenever that rest arrives;
 *  - a line with a byte lost on the wire is dropped whole and never handed out. The fault
 *    itself is reported through the overrun and frame-error flags. */
bool uart1_line_ready(void) {
    char c;
    bool gap;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { echo_drain(); }

    while (!line_complete && ring_pop(&c, &gap)) {
        if (gap) {
            line_damaged = true;
        }
        if (line_skipping) {
            /* The tail of an over-long line. This state lives here, not in the line flags,
             * because uart1_line_take() clears those as soon as the over-long report has been read, and the
             * rest of the line may not have arrived yet at that moment (a byte takes 1.04 ms at 9600 baud,
             * far longer than a main-loop pass). */
            if (is_terminator(c)) {
                line_skipping = false;
                line_damaged = false;
            }
            continue;
        }
        if (is_terminator(c)) {
            if (line_damaged) {
                /* Bytes of this line were lost, so what is in the buffer is not what the
                 * host sent. A digit lost from :SCAL:COEF would still parse, as a different number. */
                line_len = 0u;
                line_damaged = false;
                continue;
            }
            /* Design decision (ORIGINAL_BUGS #6): an EMPTY line is ignored -- no command is run
             * and the remote annunciator is not lit. The original treated it as a line, so a bare CR or LF
             * ran an empty command and lit REM.
             *
             * This one rule also gives the second half of the decision, that CR LF counts as ONE
             * terminator: the CR ends the command, and the LF that follows it produces an empty line, which
             * is now ignored. No separate CR-LF special case is needed, and a deliberate blank line is
             * ignored too, which is the same decision applied consistently. */
            if (line_len == 0u) {
                continue;
            }
            line_complete = true;
        } else if (line_len < (UART1_LINE_MAX - 1u)) {
            line[line_len++] = c;
        } else {
            /* ORIGINAL_BUGS #7: the original dropped the byte that would not fit and then EXECUTED the
             * truncated line. A truncated :SCAL:COEF or :SCAL:DATE would therefore run, writing
             * whichever fields happened to survive the cut -- into calibration memory. The line is now
             * marked over-length, and the caller must report an error and run nothing. */
            line_overlong = true;
            line_complete = true;
            line_skipping = true;
            line_damaged = false;
        }
    }
    if (line_complete) {
        /* The original set the remote flag and lit the annunciator at the moment the line completed, even
         * for an empty line (0x0FAF6). Reproduced here, at the same moment. */
        g_state.remote = true;
    }
    return line_complete;
}

bool uart1_line_overlong(void) {
    return line_overlong;
}

uint16_t uart1_line_take(char *dst, uint16_t dst_size) {
    uint16_t n = line_len;

    if (dst_size == 0u) {
        return 0u;
    }
    if (n > (uint16_t)(dst_size - 1u)) {
        n = (uint16_t)(dst_size - 1u);
    }
    for (uint16_t i = 0u; i < n; i++) {
        dst[i] = line[i];
    }
    dst[n] = '\0';

    line_len = 0u;
    line_complete = false;
    line_overlong = false;
    return n;
}

/* ---- Setup -------------------------------------------------------------------------------------------- */

void uart1_set_baud(uint8_t baud_index) {
    const uint32_t baud = baud_of_index(baud_index);
    uint16_t ubrr;

    if (baud_index == UART1_BAUD_38400_INDEX) {
        UCSR1A |= (1u << U2X1); /* ORIGINAL_BUGS #90 */
        ubrr = (uint16_t)((F_CPU_HZ / (8UL * baud)) - 1UL);
    } else {
        UCSR1A &= (uint8_t)~(1u << U2X1);
        ubrr = (uint16_t)((F_CPU_HZ / (16UL * baud)) - 1UL);
    }

    /* Both halves, high first, which is what ORIGINAL_BUGS #89 was about. */
    UBRR1H = (uint8_t)(ubrr >> 8);
    UBRR1L = (uint8_t)(ubrr & 0xFFu);
}

void uart1_init(uint8_t baud_index) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ring_head = 0u;
        ring_tail = 0u;
        rx_overrun = false;
        rx_frame_error = false;
        lost_pending = false;
        for (uint8_t i = 0u; i < (uint8_t)(UART1_RX_RING / 8u); i++) {
            ring_gap[i] = 0u;
        }
    }
    line_len = 0u;
    line_complete = false;
    line_overlong = false;
    line_damaged = false;
    line_skipping = false;
    echo_head = 0u;
    echo_tail = 0u;

    UCSR1B = 0u; /* quiet while the divisor changes */
    uart1_set_baud(baud_index);
    UCSR1C = (1u << UCSZ11) | (1u << UCSZ10); /* 0x06: 8 data bits, 1 stop bit, no parity */
    UCSR1B = (1u << RXEN1) | (1u << TXEN1) | (1u << RXCIE1);
}
