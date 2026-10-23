/* uart1.c -- host serial link (USART1).
 *
 * Replaces uart1_init_baud 0x01DBA, uart1_putc 0x01E3A, uart1_send_string 0x01E48 and the receive
 * interrupt isr_usart1_rx 0x0FAF6.
 *
 * USART1 on PD2/PD3, 8 data bits, 1 stop bit, no parity
 * (UCSR1C = 0x06), receive interrupt enabled. Transmit goes through a ring emptied by the UDRE interrupt.
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
 * The original's 1 ms pause after each transmitted character is NOT reproduced (ORIGINAL_BUGS #8, #93; see
 * the transmit section).
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

/* ---- Transmit ------------------------------------------------------------------------------------------
 * ORIGINAL_BUGS #8 and #93. The original paused 1 ms after every reply character with a busy wait (0x01E48),
 * so a `%.8e` reply blocked the meter for 15 ms or more and the pause throttled every rate above 9600 baud.
 * This firmware has no such pause.
 *
 * Everything that goes out, reply or echo, passes through one ring that the UDRE interrupt empties, so a
 * reply never waits for the wire. Echo and reply share the ring on purpose: a reply can never overtake the
 * echo of the command that asked for it, and only one piece of code, tx_service(), ever writes UDR1.
 *
 * ORIGINAL_BUGS #5: the original echoed with a blocking wait on UDRE1 inside the receive interrupt. Here the
 * echo is a ring append; with the ring full the echo is dropped, never waited for, so the receiver is never
 * stalled.
 *
 * Atomicity: the ring indices and UCSR1B are touched from the main loop and from two interrupts. The receive
 * interrupt runs with interrupts off already (AVR hardware clears I on entry and nothing here sets it), the
 * UDRE interrupt likewise, and the main loop takes ATOMIC_BLOCK around every access. The UDRE interrupt is
 * enabled by the code that appends a byte and disabled by tx_service() when the ring runs empty. */
#define UART1_TX_RING 128u /* a reply is at most 113 characters, so one fits */
#define UART1_TX_MASK (UART1_TX_RING - 1u)
_Static_assert((UART1_TX_RING & UART1_TX_MASK) == 0u && UART1_TX_RING <= 256u,
               "the masking below needs a power-of-two ring that a uint8_t index can address");

static volatile uint8_t tx_ring[UART1_TX_RING];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile bool tx_in_flight; /* a byte has been written to UDR1 since the last flush saw TXC1 */

/* The callers below hold interrupts off, or are interrupt handlers. */
static bool tx_full(void) {
    return ((uint8_t)(tx_head + 1u) & UART1_TX_MASK) == tx_tail;
}

static void tx_push(uint8_t c) {
    tx_ring[tx_head] = c;
    tx_head = (uint8_t)((tx_head + 1u) & UART1_TX_MASK);
    UCSR1B |= (1u << UDRIE1);
}

/* The only writer of UDR1. Sends one byte if the data register is free, and switches the UDRE interrupt
 * off when nothing is left to send. Safe to call with interrupts off and the interrupt disabled (a polled
 * drain), and from the UDRE interrupt itself. */
static void tx_service(void) {
    if (tx_head != tx_tail && (UCSR1A & (1u << UDRE1)) != 0u) {
        UDR1 = tx_ring[tx_tail];
        UCSR1A |= (1u << TXC1); /* writing 1 clears TXC1: the flush below waits for it to come back */
        tx_in_flight = true;
        tx_tail = (uint8_t)((tx_tail + 1u) & UART1_TX_MASK);
    }
    if (tx_head == tx_tail) {
        UCSR1B &= (uint8_t)~(1u << UDRIE1);
    }
}

ISR(USART1_UDRE_vect) {
    tx_service();
}

/* Longest wait for one free ring slot: one character time at the slowest rate, 10 bits / 600 baud =
 * 16.67 ms. 20000 steps of 50 us is 1.0 s plus loop overhead, so a transmitter that is merely slow never
 * trips it. Only a transmitter that is not running at all does, and the byte is then dropped. */
#define UART1_TX_WAIT_STEP_US 50u
#define UART1_TX_WAIT_STEPS 20000u

/* Longest wait for the whole ring to empty, for a baud change: a full ring is 127 bytes plus one in the
 * data register and one in the shift register, 129 * 16.67 ms = 2.15 s at 600 baud. 60000 steps of 50 us
 * is 3.0 s plus loop overhead. */
#define UART1_TX_FLUSH_STEPS 60000u

void uart1_putc(char c) {
    /* The wait is bounded and feeds the watchdog (armed as WDTO_2S in main.c, 1.8 s at 5 V). It
     * only happens when the ring is full, which a reply of up to 113 characters never makes it. When the
     * caller has interrupts off, the wait drains the ring by polling instead of waiting for an interrupt that
     * cannot come. */
    for (uint16_t step = 0u; step < UART1_TX_WAIT_STEPS; step++) {
        bool queued = false;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (!tx_full()) {
                tx_push((uint8_t)c);
                queued = true;
            } else {
                tx_service();
            }
        }
        if (queued) {
            return;
        }
        wdt_reset();
        _delay_us(UART1_TX_WAIT_STEP_US);
    }
}

/* Waits until every queued byte has left the shift register. A baud change before that would garble the
 * tail of a reply. */
static void tx_flush(void) {
    for (uint16_t step = 0u; step < UART1_TX_FLUSH_STEPS; step++) {
        bool done = false;

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            tx_service();
            if (tx_head == tx_tail && (UCSR1A & (1u << UDRE1)) != 0u &&
                (!tx_in_flight || (UCSR1A & (1u << TXC1)) != 0u)) {
                tx_in_flight = false;
                done = true;
            }
        }
        if (done) {
            return;
        }
        wdt_reset();
        _delay_us(UART1_TX_WAIT_STEP_US);
    }
}

void uart1_write(const char *s) {
    while (*s != '\0') {
        uart1_putc(*s++);
    }
}

void uart1_write_P(const char *s_flash) {
    char c;

    while ((c = (char)pgm_read_byte(s_flash++)) != '\0') {
        uart1_putc(c);
    }
}

/* One byte, never CR+LF: LF when EEPROM 0x863 is 0 (the default), CR otherwise. */
void uart1_write_terminator(void) {
    uart1_putc(g_state.settings.tx_terminator == 0u ? '\n' : '\r');
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

    if (g_state.settings.echo && !tx_full()) {
        tx_push(c); /* a full ring drops the echo rather than stalling the receiver */
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
            /* ORIGINAL_BUGS #6: an EMPTY line is ignored -- no command is run
             * and the remote annunciator is not lit. The original treated it as a line, so a bare CR or LF
             * ran an empty command and lit REM.
             *
             * This one rule also gives the second half of the rule, that CR LF counts as ONE
             * terminator: the CR ends the command, and the LF that follows it produces an empty line, which
             * is now ignored. No separate CR-LF special case is needed, and a deliberate blank line is
             * ignored too, which is the same rule applied consistently. */
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

static void apply_baud(uint8_t baud_index) {
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

void uart1_set_baud(uint8_t baud_index) {
    tx_flush();
    apply_baud(baud_index);
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

    UCSR1B = 0u; /* quiet while the divisor changes */
    tx_head = 0u;
    tx_tail = 0u;
    tx_in_flight = false;
    apply_baud(baud_index);
    UCSR1C = (1u << UCSZ11) | (1u << UCSZ10); /* 0x06: 8 data bits, 1 stop bit, no parity */
    UCSR1B = (1u << RXEN1) | (1u << TXEN1) | (1u << RXCIE1);
}
