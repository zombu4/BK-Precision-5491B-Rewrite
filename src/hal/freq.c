/* freq.c -- frequency and period by reciprocal counting.
 *
 * Replaces timer1_init_capture 0x01D62, timer3_init_stopped 0x01D86, meas_freq_gate_start 0x08D22,
 * isr_timer1_capt 0x08D60, isr_timer1_ovf 0x08DE2, isr_timer3_ovf 0x08E02 and the measurement wrapper
 * meas_freq_period_capture 0x0ACEA.
 *
 * The capture sequence below was read from the original firmware.
 *
 * ---- How the instrument measures frequency -------------------------------------------------------------
 * It is a RECIPROCAL counter, not a gated counter, which is why it reads low frequencies well:
 *   Timer1 runs from the system clock at 12 MHz and timestamps rising edges of the shaped input through
 *     the input-capture pin ICP1 (PD4), with the noise canceller on.
 *   Timer3 counts those same rising edges as an external clock on T3 (PE6).
 * The sequence is:
 *   - the second capture after arming starts Timer3 and is remembered as T2;
 *   - the gate then runs for 46, 92 or 184 Timer1 overflows -- 0.251, 0.502 or 1.005 s -- for fast,
 *     medium and slow;
 *   - once the gate expires the next two captures are discarded, and the one after that is remembered as
 *     T5 and stops both timers.
 * Because both ends of the window are input edges, the window holds a whole number of input cycles:
 *     f = N3 * 12 000 000 / (T5 - T2)
 * and the resolution does not collapse at low frequencies the way a fixed-gate counter's does.
 *
 * ---- The three defects fixed here (ORIGINAL_BUGS) ------------------------------------------------------
 * 1. The original weighted each timer overflow as 65535 instead of 65536. A 16-bit timer counts 65536
 *    values per overflow, so every overflow lost one count. Over the 1.005 s slow gate that is 183
 *    overflows in the DENOMINATOR, which makes the measured time 15.25 ppm short and therefore every
 *    reading 15.25 ppm high. The numerator is only affected above about 65 kHz, where Timer3 itself starts
 *    to overflow, so the two errors do not cancel at the frequencies this meter is usually used at.
 *    FREQ_OVERFLOW_WEIGHT is 65536. **This changes every frequency and period reading by about 15 ppm.**
 *    It is a deliberate change and it moves the readings towards the truth, not away.
 * 2. The wait for the final capture was bounded by a count of 3000 software loop iterations, so how long
 *    the instrument waited depended on what the compiler emitted and on nothing physical. The timeout here
 *    is real time, measured in Timer1 overflows, which are 5.4613 ms each and keep running regardless.
 * 3. Period was computed as 1/f with no guard, so an absent or too-slow input divided by zero. In the
 *    original's arithmetic that returned FLT_MAX rather than trapping, which then printed as a
 *    plausible-looking huge number. A zero or missing frequency is now simply "no reading".
 */
#include "freq.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include "../board.h"
#include "../state.h"

/* Gate length in Timer1 overflows, per rate: 46 / 92 / 184 for fast / medium / slow. */
static const uint16_t gate_overflows[RATE_COUNT] = {46u, 92u, 184u};

/* One Timer1 overflow is 65536 / 12 MHz = 5.4613 ms. Used for the real-time timeout, so that the timeout
 * is a duration rather than a loop count. */
#define FREQ_OVF_US 5461UL

/* Where the capture state machine is. The names follow the original's edge numbering so the code can
 * be compared with it directly. */
typedef enum {
    FREQ_IDLE = 0,
    FREQ_WAIT_EDGE2,  /* armed; the first capture is ignored, the second starts the measurement */
    FREQ_COUNTING,    /* Timer3 running, waiting for the gate to expire */
    FREQ_DISCARDING,  /* gate expired; two captures are discarded */
    FREQ_DONE,        /* T5 taken, both timers stopped */
    FREQ_TIMED_OUT
} freq_state_t;

static volatile freq_state_t state;
static volatile uint16_t ovf1;        /* Timer1 overflows since arming */
static volatile uint16_t ovf1_at_t2;  /* the overflow count when T2 was captured */
static volatile uint16_t ovf1_at_t5;  /* and when T5 was, both corrected for a pending overflow */
static volatile uint16_t ovf3;        /* Timer3 overflows, i.e. input edges / 65536 */
static volatile uint16_t icr_t2;
static volatile uint16_t icr_t5;
static volatile uint16_t tcnt3_final;
static volatile uint8_t discards;
static volatile uint16_t gate_limit;  /* copied from gate_overflows at arm time */
static volatile uint16_t timeout_ovf; /* 0 = no limit */

/* ---- Capture interrupt mask ---------------------------------------------------------------------------
 * The capture interrupt is live only while an edge is wanted: edge 1 and edge 2 after arming,
 * and the three edges after the gate expires. The original masked it from edge 2 until the gate ended
 * (TIMSK 0x24 -> 0x04 at 0x8DAC) and wrote TIMSK = 0 at the final edge (0x8DC8), so a fast input cost it
 * nothing during the gate. The compiled capture handler costs about 66 cycles (vector 3
 * + response 4 + prologue 20 + body 14 + epilogue 19 + reti 4 + the rest), so left live it would load the
 * CPU by f * 66 / 12 MHz: 100 % at 182 kHz, while the frequency range reaches 500 kHz (an edge every
 * 12e6 / 5e5 = 24 cycles). The overflow handler ranks below the capture handler (vector 14 against 11), so
 * it would starve, the gate would never end and the watchdog would reset the meter.
 *
 * TIMSK is shared with the 1 ms tick (OCIE0), so only the TICIE1 bit is touched. Every caller is either an
 * interrupt handler or inside an ATOMIC_BLOCK, because TIMSK (I/O 0x37) is read-modify-written with in/out,
 * which is not atomic against an interrupt. */
static inline void capture_irq_enable(void) {
    TIMSK |= (1u << TICIE1);
}

static inline void capture_irq_disable(void) {
    TIMSK &= (uint8_t)~(1u << TICIE1);
}

/* ---- Setup -------------------------------------------------------------------------------------------- */

void freq_init(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state = FREQ_IDLE;

        /* Timer1: normal mode, input capture on the rising edge of ICP1 with the noise canceller, clock
         * stopped until a gate is armed. */
        TCCR1A = 0u;
        TCCR1B = (1u << ICNC1) | (1u << ICES1);
        TCNT1 = 0u;

        /* Timer3: normal mode, stopped. Its clock source is the T3 pin, selected when the gate starts. */
        TCCR3A = 0u;
        TCCR3B = 0u;
        TCNT3 = 0u;

        /* Clear any pending flags from before, then enable both overflow interrupts. The capture interrupt
         * stays masked until freq_gate_start() arms a gate. */
        TIFR = (1u << ICF1) | (1u << TOV1);
        ETIFR = (1u << TOV3);
        capture_irq_disable();
        TIMSK |= (1u << TOIE1);
        ETIMSK |= (1u << TOIE3);
    }
}

void freq_gate_start(void) {
    const uint8_t rate = (g_state.rate < RATE_COUNT) ? (uint8_t)g_state.rate : (uint8_t)RATE_SLOW;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR1B = (1u << ICNC1) | (1u << ICES1); /* stop Timer1 while it is reset */
        TCCR3B = 0u;                            /* stop Timer3 */
        TCNT1 = 0u;
        TCNT3 = 0u;

        ovf1 = 0u;
        ovf3 = 0u;
        ovf1_at_t2 = 0u;
        ovf1_at_t5 = 0u;
        icr_t2 = 0u;
        icr_t5 = 0u;
        tcnt3_final = 0u;
        discards = 0u;
        gate_limit = gate_overflows[rate];
        timeout_ovf = 0u;
        state = FREQ_WAIT_EDGE2;

        TIFR = (1u << ICF1) | (1u << TOV1);
        ETIFR = (1u << TOV3);
        capture_irq_enable();
        TCCR1B = (1u << ICNC1) | (1u << ICES1) | (1u << CS10); /* clk/1, 12 MHz */
    }
}

void freq_gate_abort(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        TCCR1B = (1u << ICNC1) | (1u << ICES1); /* stop Timer1, keep the capture configuration */
        TCCR3B = 0u;                            /* stop Timer3 */
        capture_irq_disable();
        state = FREQ_IDLE;
        timeout_ovf = 0u;
        discards = 0u;
    }
}

bool freq_result_ready(void) {
    freq_state_t s;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { s = state; }
    return s == FREQ_DONE;
}

/* ---- Interrupt handlers ------------------------------------------------------------------------------- */

/* A rising edge of the shaped input. The capture register holds the Timer1 value at the edge itself, which
 * is what makes the measurement free of interrupt latency: the latency affects when this code runs, not
 * what it reads. */
ISR(TIMER1_CAPT_vect) {
    const uint16_t icr = ICR1;
    uint16_t ovf_now = ovf1;

    /* ORIGINAL_BUGS #76: the original had no protection here, and this is a genuine race, not a
     * theoretical one. Timer1 wraps every 5.4613 ms. If an input edge arrives just after the wrap but
     * before the overflow handler has run -- the two interrupts share one priority ladder, and either can
     * be waiting on the other -- then ICR1 already holds a small post-wrap value while the overflow counter
     * has not yet been incremented. The timestamp is then 65536 ticks short, and a reading that happens to
     * land in that window is wrong by about 0.5 % of a 1 s gate while every neighbouring reading is right.
     * Intermittent, plausible-looking, and very hard to attribute later.
     *
     * The test: if the overflow flag is still pending AND the captured count is in the low half of the
     * range, the wrap happened before this capture, so count it. A capture in the high half with the flag
     * pending means the capture came first and the flag belongs to a wrap that has not happened yet from
     * this timestamp's point of view. */
    if ((TIFR & (1u << TOV1)) != 0u && icr < 0x8000u) {
        ovf_now++;
    }

    switch (state) {
    case FREQ_WAIT_EDGE2:
        /* The first capture after arming is only there to synchronise; the second one starts Timer3 and is
         * T2. */
        if (discards == 0u) {
            discards = 1u;
            break;
        }
        icr_t2 = icr;
        ovf1_at_t2 = ovf_now;
        discards = 0u;
        TCNT3 = 0u;
        ovf3 = 0u;
        /* Timer3's clock is the T3 pin, rising edge: external clock source 7. */
        TCCR3B = (1u << CS32) | (1u << CS31) | (1u << CS30);
        capture_irq_disable(); /* no more edges are wanted until the gate ends; see the mask section */
        state = FREQ_COUNTING;
        break;

    case FREQ_DISCARDING:
        /* The gate has expired; the original lets two captures go by before taking the last one. */
        if (discards < 2u) {
            discards++;
            break;
        }
        icr_t5 = icr;
        ovf1_at_t5 = ovf_now;
        tcnt3_final = TCNT3;
        TCCR1B = (1u << ICNC1) | (1u << ICES1); /* stop both timers, in this order */
        TCCR3B = 0u;
        capture_irq_disable();
        state = FREQ_DONE;
        break;

    case FREQ_COUNTING:
    case FREQ_IDLE:
    case FREQ_DONE:
    case FREQ_TIMED_OUT:
    default:
        break;
    }
}

ISR(TIMER1_OVF_vect) {
    ovf1++;

    /* The gate ends gate_limit overflows after T2. The original's wait (0xADAE-0xADBC, then 0xADC2-0xADCC)
     * ends one overflow earlier, so this window is 5.4613 ms longer. The result is a ratio of edges to
     * ticks over a window bounded by two input edges, so it is unaffected; only the update time grows. */
    if (state == FREQ_COUNTING && (uint16_t)(ovf1 - ovf1_at_t2) >= gate_limit) {
        discards = 0u;
        state = FREQ_DISCARDING;
        /* The original unmasks here (0xADCE) and leaves ICF1 as it is, so an edge that arrived while the
         * handler was masked fires at once and counts as the first discarded edge. Kept: it is what the
         * original's "two discards, then T5" sequence is made of. */
        capture_irq_enable();
    }
    if (timeout_ovf != 0u && ovf1 >= timeout_ovf &&
        (state == FREQ_WAIT_EDGE2 || state == FREQ_COUNTING || state == FREQ_DISCARDING)) {
        TCCR1B = (1u << ICNC1) | (1u << ICES1);
        TCCR3B = 0u;
        capture_irq_disable();
        state = FREQ_TIMED_OUT;
    }
}

ISR(TIMER3_OVF_vect) {
    ovf3++;
}

/* ---- Results ------------------------------------------------------------------------------------------ */

/* Blocks until the measurement finishes or the timeout expires. The timeout is a real duration counted in
 * Timer1 overflows, which is the second half of the frequency-counter fix: the original counted 3000 software iterations, so its
 * timeout was whatever the compiler happened to produce.
 *
 * NOTE for callers: this blocks the main loop for up to a second of gate plus the timeout, exactly as the
 * original did. Nothing is LOST while it blocks, because the receive ring keeps filling (see uart1.c), but no command executes either. The measurement loop should therefore arm with freq_gate_start()
 * and poll freq_result_ready() between its other work, and use this blocking form only where the original
 * also blocked. freq_result_ready() exists for that reason. */
bool freq_read(float *hz, uint16_t timeout_ms) {
    uint32_t edges;
    uint32_t ticks;
    freq_state_t s;

    if (timeout_ms == 0u) {
        timeout_ms = FREQ_GATE_TIMEOUT_MS;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        const uint32_t limit = ((uint32_t)timeout_ms * 1000UL) / FREQ_OVF_US;

        timeout_ovf = (limit > 0xFFFFUL) ? 0xFFFFu : (uint16_t)limit;
        if (timeout_ovf == 0u) {
            timeout_ovf = 1u;
        }
    }

    for (;;) {
        /* The default timeout (3000 ms) is longer than the watchdog period (1.8 s at 5 V, set to
         * WDTO_2S in main.c), so the wait must feed it. The wait itself is bounded by timeout_ovf above, which
         * the Timer1 overflow handler enforces. */
        wdt_reset();
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { s = state; }
        if (s == FREQ_DONE || s == FREQ_TIMED_OUT || s == FREQ_IDLE) {
            break;
        }
    }
    if (s != FREQ_DONE) {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        /* FREQ_OVERFLOW_WEIGHT is 65536, not the original's 65535: a 16-bit counter passes 65536 values
         * between overflows, and counting 65535 lost one per overflow. */
        edges = (uint32_t)ovf3 * FREQ_OVERFLOW_WEIGHT + (uint32_t)tcnt3_final;
        ticks = ((uint32_t)(uint16_t)(ovf1_at_t5 - ovf1_at_t2)) * FREQ_OVERFLOW_WEIGHT;
        ticks += (uint32_t)icr_t5;
        ticks -= (uint32_t)icr_t2;

        /* Consume the result. Without this a second read with no new gate would hand back the same
         * measurement as if it were fresh, which is how a stale reading becomes a believed one. */
        state = FREQ_IDLE;
    }

    if (ticks == 0uL || edges == 0uL) {
        return false; /* no window and no edges is not a reading of zero, it is no reading */
    }

    /* Deliberately float, not double: on avr-gcc `double` is 32 bits unless the build passes -mdouble=64,
     * so writing `double` here would silently mean something different depending on a flag. Float is good
     * enough by a wide margin, and the bound is worth stating because this routine's whole purpose is a
     * 15 ppm correction:
     *   edges is at most about 1.2e6 and ticks at most about 1.2e7, both below 2^24, so both convert
     *   exactly; the product is about 1.2e13, rounded to 24 bits, and the divide rounds once more. The
     *   total is under 0.12 ppm, i.e. a hundred times smaller than the error being fixed. */
    *hz = ((float)edges * (float)F_CPU_HZ) / (float)ticks;
    return true;
}

/* Period. The original computed 1 / f with no guard; with its arithmetic a zero frequency returned
 * +FLT_MAX, which then printed as a believable large number instead of an error. */
bool freq_period_read(float *seconds, uint16_t timeout_ms) {
    float hz;

    if (!freq_read(&hz, timeout_ms)) {
        return false;
    }
    if (hz <= 0.0f) {
        return false;
    }
    *seconds = 1.0f / hz;
    return true;
}
