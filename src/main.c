/* main.c -- reset, initialisation, the 1 ms tick and the main loop.
 *
 * Replaces boot_reset 0x01B30, boot_port_init 0x01D02, boot_soft_reset 0x01D56, main 0x0953C and
 * init_settings_flags 0x095CE, and adds the power-on relay exercise hook, the hour meter and power-up
 * counter, and the warm-up indication.
 *
 * This file is where every requirement the other subsystems accumulated has to actually land.
 *
 * ---- Three structural fixes live here ----------------------------------------------------------------
 * 1. **A 1 ms tick, which the original does not have at all.** Of the ATmega128's 35 vectors the original
 *    populates five, and Timer0 is untouched, so every duration in it is a
 *    busy-wait or a count of loop iterations -- the 1000 ms error message, the 3000-iteration frequency
 *    timeout, the key debounce, the buzzer pulse. A loop count is not a duration: it changes with the
 *    compiler, the optimisation level and whatever else the loop is doing. One tick replaces all of them,
 *    costs about 20 cycles per millisecond, and is the reason nothing in this port needs to busy-wait with
 *    interrupts off.
 * 2. **Full peripheral initialisation on every entry** (a defect of the original). The original's soft reset jumps to
 *    zero without re-initialising the UART receive interrupt or the timer enables, so a `*RST` left the
 *    instrument in a state that depended on what it had been doing before.
 * 3. **The loop never blocks.** `meas_task()` and `cal_sweep_task()` are state machines that do at most one
 *    conversion's worth of work per call; the loop services the host link, the keys and the watchdog
 *    between them. That is the other half of the host-link fix -- the receive ring stops bytes being lost, and this
 *    stops commands waiting half a minute to run.
 *
 * ---- And the watchdog, which the original has not got --------------------------------------------------
 * The original has no watchdog and no timeout on anything, so a converter that stopped answering
 * left the instrument hung with its last reading on the display, looking alive. The watchdog is armed here and
 * petted in the loop, and it is deliberately NOT petted inside any long operation other than through the
 * state machines, so a genuinely stuck state machine resets rather than spinning for ever.
 *
 * A converter timeout does NOT reset the meter. With /DRDY stopped, twenty simulated seconds later the
 * meter has not reset -- `meas_task()` has entered MEAS_FAULT, cleared `reading.valid`, and retries once a
 * second, with this loop still feeding the watchdog on every pass.
 *
 * That behaviour is right. A converter that has genuinely died would make a resetting instrument reboot for ever, which is a worse
 * failure than one showing dashes and retrying -- and a converter that recovers is picked up by the retry
 * without anyone power-cycling. What matters is that the user can TELL: the fault was latched
 * in meas.c and never consumed, and the dashes it produces are the same dashes a normal restart shows. The
 * remote layer now drains it into the error queue (exec.c, host_link_faults).
 *
 * The watchdog still does its job for the case it is for -- a state machine wedged inside a single pass,
 * which stops the loop reaching wdt_reset() at all.
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include "board.h"
#include "main.h"
#include "cal/cal.h"
#include "eeprom_map.h"
#include "hal/adcbus.h"
#include "hal/buzzer.h"
#include "hal/eeprom.h"
#include "hal/freq.h"
#include "hal/keys.h"
#include "hal/uart1.h"
#include "hal/vfd.h"
#include "meas/meas.h"
#include "remote/remote.h"
#include "serv/serv.h"
#include "state.h"
#include "ui/display.h"
#include "ui/settings.h"
#include "ui/menu.h"
#include "version.h"

/* The one instance of the whole-instrument state (state.h). */
instrument_t g_state;

/* ---- The 1 ms tick ------------------------------------------------------------------------------------
 * Timer0 in CTC mode, prescaler 64, OCR0 = 187. The period is
 *
 *     64 * (187 + 1) / 12 MHz = 12032 / 12000000 s = 1.00267 ms,   +0.267 %
 *
 * Prescaler 8 with OCR0 = 149 would give (149 + 1) * 8 / 12 MHz = 100 MICROseconds, not 1 ms: a tick ten
 * times too fast, which would make the 30-minute warm-up expire in 3 minutes, the 60-second lockout in
 * 6 seconds, the 50 ms key debounce in 5 ms, and the hour meter count an hour every 6 minutes.
 *
 * NO 8-BIT TIMER GIVES EXACTLY 1 ms AT 12 MHz. It needs prescaler * (OCR0 + 1) = 12000, and for the
 * prescalers Timer0 offers on this part (1, 8, 32, 64, 128, 256, 1024) the required OCR0 + 1 is 12000,
 * 1500, 375, 187.5, 93.75, 46.875 and 11.72 -- the first three are above the 8-bit range and the rest are
 * not whole numbers. So 1.00267 ms is the closest achievable, and the error is stated rather than hidden:
 *
 *     30 min warm-up  +4.8 s        60 s lockout  +0.16 s      50 ms debounce  +0.13 ms
 *     10 min session  +1.6 s        1 h of meter  +9.6 s
 *
 * None of those changes a decision anyone makes. The hour meter is the only one that accumulates, at about
 * a day per decade, and it is a service-interval indication rather than a measurement.
 *
 * The alternative considered and rejected: keep the 100 us period and count ten interrupts per
 * millisecond. That is exact, but it runs the ISR ten times as often for no gain -- at the measured ISR
 * cost it would spend over a tenth of the CPU on timekeeping. Timer1 and Timer3 are both taken by the
 * frequency counter (hal/freq.c), so the 16-bit option that would be exact is not available. */
#define TICK_OCR0 187u

/* How many millisecond ticks of menu timekeeping one pass of the loop may catch up on. A pass that took
 * longer than this many milliseconds has something slow in it, and running the whole backlog would make
 * the next pass slower still. */
#define MENU_TICK_CATCHUP_MAX 8u

static volatile uint32_t ms_since_boot;

/* What may run in here, and what may not.
 *
 * `menu_tick()` is deliberately NOT in this list. It writes the display -- message timeouts, the blink, the busy
 * screens -- and the main loop writes the display too, through `display_update()` and the key scan. Both
 * drive the same PT6311 lines, and a PT6311 transaction is a /STB-framed sequence of bit-banged clock
 * pulses. An interrupt in the middle of one does not delay it; it interleaves a second frame inside it, and
 * what the controller then latches is neither. The symptom would have been occasional wrong glyphs and
 * dropped key presses, appearing at random and never reproducing.
 *
 * So this ISR does only what is safe to do at any instant: increment counters that nothing else writes.
 * `keys_tick_1ms` and `serv_tick_1ms` are counter increments. `buzzer_tick_1ms` drives PA4, which no other
 * code touches -- the front-end relays share PORTA, but they are only ever written from the main loop, so
 * the read-modify-write here can lose the buzzer bit and never a relay bit. `meas_tick_1ms` advances the
 * converter's own timers and touches no port.
 *
 * `menu_tick()` runs from the main loop instead, once per elapsed millisecond. It is not called here. */
ISR(TIMER0_COMP_vect) {
    ms_since_boot++;
    keys_tick_1ms();
    buzzer_tick_1ms();
    meas_tick_1ms();
    serv_tick_1ms();
}

static uint32_t millis(void) {
    uint32_t t;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = ms_since_boot; }
    return t;
}

static void tick_init(void) {
    /* CTC, prescaler 64. CS02 alone selects /64 on this part: Timer0's divisor set is
     * /1, /8, /32, /64, /128, /256, /1024 mapped onto CS02:0 = 001..111, anchored at both ends inside this
     * codebase -- the boot loader uses CS02|CS01|CS00 with OCR0 = 0xEA and gets 1024 * 235 / 12 MHz =
     * 20.05 ms, which its own comment states and which is only true for /1024; CS01 alone would select /8,
     * giving a 125 us tick with this OCR0. */
    TCCR0 = (1u << WGM01) | (1u << CS02);
    OCR0 = TICK_OCR0;
    TCNT0 = 0u;
    TIMSK |= (1u << OCIE0);
}

/* ---- Ports -------------------------------------------------------------------------------------------
 * The reset-time port state the original writes at 0x01D02, reproduced exactly from board.h. Defect 12 is
 * KEEP: the write ORDER is the original's, and the values are the original's. What each bit DOES
 * electrically is tagged UNVERIFIED in board.h, with the check; nothing here depends on it, because this
 * replays the writes rather than reasoning about them. */
static void port_init(void) {
    DDRA = BOARD_DDRA;
    PORTA = BOARD_PORTA;
    DDRB = BOARD_DDRB;
    PORTB = BOARD_PORTB;
    DDRC = BOARD_DDRC;
    PORTC = BOARD_PORTC;
    DDRD = BOARD_DDRD;
    PORTD = BOARD_PORTD;
    DDRE = BOARD_DDRE;
    PORTE = BOARD_PORTE;
    DDRF = BOARD_DDRF;
    PORTF = BOARD_PORTF;
    DDRG = BOARD_DDRG;
    PORTG = BOARD_PORTG;
}

/* ---- Settings and state defaults ---------------------------------------------------------------------
 * The five operator settings come from EEPROM, as the original's 0x095CE did. Everything else here is a
 * field that has no initialiser in the original: the statics that used to hold these defaults
 * were removed when the state moved into g_state, so without this they would start at zero and a
 * zero-valued setting is not always the harmless one. */
static void state_init(void) {
    uint8_t i;

    g_state.func = FUNC_DCV;
    g_state.rate = RATE_MEDIUM;
    g_state.trigger = TRIG_IMM;

    for (i = 0u; i < FUNC_COUNT; i++) {
        g_state.range[i].manual = false;
        g_state.range[i].index = 0u;
        g_state.second_display[i] = 0u;
        g_state.rel[i].active = false;
        g_state.rel[i].reference = 0.0f;
        g_state.rel[i].raw_reference = 0.0f;
    }

    /* EEPROM-backed operator settings. A byte that has never been written reads 0xFF, so each one falls
     * back to the value the meter actually ships with rather than to zero.
     *
     * The load is `settings_load()` in ui/settings.c and nowhere else: `settings_store()` writes through
     * the same module, so a setting added there and not loaded here would be stored and never read back. */
    settings_load();

    /* display_enabled has no initialiser, and false would have meant a blank panel at power-on. */
    g_state.display_enabled = true;

    /* The unit channels. 75 ohm is the .data initialiser for the dBm reference impedance (flash
     * 0x1125); the dBm reference is an int16 in the instrument, not a float. */
    for (i = 0u; i < 2u; i++) {
        g_state.unit_channel[i].mode = UNIT_NATIVE;
        g_state.unit_channel[i].db_ref = 1.0f;
        g_state.unit_channel[i].dbm_ref = 75;
    }
    g_state.math.db_ref = 1.0f;
    g_state.math.dbm_ref = 75;
    g_state.math.percent_ref = 1.0f;

    /* The comparator limits. `g_state` lives in .bss, so leaving these out gave
     * both of them 0.0 -- and the original's initialised-data image holds +1.0 and -1.0 at RAM 0x011E and
     * 0x0122 (flash 0x113B and 0x113F).
     *
     * At 0 and 0 the COMP function is not merely wrong, it is actively hostile: every reading above zero
     * is HI, every reading below zero is LO, and the beeper sounds on all of them (meas.c:1243). A user
     * switching COMP on to see what it does would get a meter that shrieks at every measurement. The
     * original passes anything between -1 and +1 silently. */
    g_state.math.limit_hi = 1.0f;
    g_state.math.limit_lo = -1.0f;

    /* hold_window is a FRACTION and the original never writes it, so it had no defined
     * value at all. One per cent of the first reading is the menu's own default. */
    g_state.hold_window = 0.01f;
    g_state.hold_count = 3u;

    g_state.minmax_view = MINMAX_VIEW_LIVE;
    g_state.remote = false;
    g_state.menu_mode = false;
}

/* ---- The power-up counter and the hour meter --------------------------------------------------
 * The counter is bumped once per boot. The hour meter is wear levelled over EE_HOURS_SLOTS slots, written
 * once an hour, with the slot chosen round-robin and the highest sequence number winning on read -- see
 * eeprom_map.h for the endurance arithmetic. Both treat an erased value as "start from zero", which is
 * the lesson of the boot initialiser: on a meter that has not run this firmware every one of these bytes reads 0xFF. */
static uint16_t hours_now;

/* The sequence number the next write will use. It MUST continue from the highest one already stored, which
 * is why hours_load() leaves it behind rather than hours_store() owning a counter of its own.
 *
 * With a per-boot counter starting at zero, the hour meter would go BACKWARDS. The slot is chosen by
 * `hours % 8`, so a second run rewrites slots the first run used, with sequence numbers starting again at
 * 1 -- and `hours_load()` picks the highest sequence it can see. After a reboot the old run's seq 20 still
 * outranks the new run's seq 1..19, so the meter reloads the old hours and loses up to nineteen hours of
 * running; and as the old slots are gradually overwritten the winner changes, so the displayed figure can
 * jump to a different stale value. A user who power-cycles once an hour would see the same number for ever.
 *
 * An hour meter whose reading depends on how recently the meter was switched off is not an hour meter. */
static uint16_t hours_seq;

static void hours_load(void) {
    uint16_t best_seq = 0u;
    uint8_t i;

    hours_now = 0u;
    for (i = 0u; i < EE_HOURS_SLOTS; i++) {
        const uint16_t base = (uint16_t)(EE_HOURS_BASE + i * EE_HOURS_SLOT_SIZE);
        const uint16_t hours = ee_read_u16(base);
        const uint16_t seq = ee_read_u16((uint16_t)(base + 2u));

        if (seq != 0xFFFFu && seq >= best_seq) {
            best_seq = seq;
            hours_now = (hours == 0xFFFFu) ? 0u : hours;
        }
    }
    hours_seq = best_seq; /* the next write continues the series rather than restarting it */
}

static void hours_store(void) {
    const uint8_t slot = (uint8_t)(hours_now % EE_HOURS_SLOTS);
    const uint16_t base = (uint16_t)(EE_HOURS_BASE + slot * EE_HOURS_SLOT_SIZE);

    /* 0xFFFF is the erased value and means "this slot has never been written", so the series must never
     * land on it. At one write an hour, reaching 0xFFFE takes 65534 hours -- 7.5 years of continuous
     * running -- and stopping there is right: the sequence only has to order the eight slots against each
     * other, and a saturated value keeps ordering them correctly for ever after. Wrapping to 0 would make
     * the newest slot look like the oldest. */
    if (hours_seq < 0xFFFEu) {
        hours_seq++;
    }
    (void)ee_write_u16(base, hours_now);
    (void)ee_write_u16((uint16_t)(base + 2u), hours_seq);
}

static void powerup_count_bump(void) {
    uint16_t n = ee_read_u16(EE_PWRUP_COUNT);

    if (n == 0xFFFFu) {
        n = 0u; /* never written: this is the first power-up under this firmware */
    }
    if (n < 0xFFFEu) {
        (void)ee_write_u16(EE_PWRUP_COUNT, (uint16_t)(n + 1u));
    }
}

uint16_t main_power_on_hours(void) {
    return hours_now;
}

/* ---- The warm-up indication ------------------------------------------------------------------
 * The instrument's specification assumes a 30 minute warm-up, and CAL RANGE warns when it has not had one.
 * Exposed rather than kept private because two callers need the same answer and must not disagree about
 * it: the panel indication and the calibration gate. */
#define WARMUP_MS (30UL * 60UL * 1000UL)

bool main_warmed_up(void) {
    return millis() >= WARMUP_MS;
}

/* ---- The soft reset ---------------------------------------------------------------------------------
 * The original jumped to zero (0x01D56) without re-initialising anything, which is a defect of the original. A
 * watchdog reset is used instead: it re-enters through the real reset vector, so every peripheral is set
 * up again from scratch and nothing depends on what the instrument was doing beforehand. */
void main_soft_reset(void) {
    cli();
    wdt_enable(WDTO_15MS);
    for (;;) {
    }
}

/* ORIGINAL_BUGS G.1, "lock-ups and traps": an interrupt that is enabled but has no handler.
 *
 * avr-libc routes every vector that has no ISR() to `__bad_interrupt`, and in the release build that is
 * literally `jmp 0` -- disassembled from a release build at 0x172E:
 *     0000172e <__bad_interrupt>:  172e: 0c 94 00 00  jmp 0  ; 0x0 <__vectors>
 * That is the same jump-to-zero the soft reset above exists to avoid: it re-enters the
 * program with every peripheral still configured, the interrupt flags untouched and the stack pointer
 * wherever the fault left it, so the restart runs on the state that produced the fault. BADISR_vect is
 * avr-libc's documented override for all of those vectors at once (`avr/interrupt.h:245-257`, where
 * BADISR_vect is defined as __vector_default).
 *
 * Routing it through the watchdog turns a spurious interrupt into a real reset through the reset vector,
 * with every peripheral initialised from scratch. This handler is not expected to run at all; if it does,
 * the instrument comes back in a defined state rather than a half-initialised one. */
ISR(BADISR_vect) {
    main_soft_reset();
}

/* ---- Stack overflow detection --------------------------------------------------------------
 * The ATmega128 has 4096 bytes of internal SRAM, 0x0100 to 0x10FF, with the stack at the top growing down and
 * the static data at the bottom growing up. Nothing stops the two meeting. The guard is a few bytes in
 * `.noinit`, which the linker places directly above `.bss` and so directly below the stack's free space
 * (`_end`): a stack that grows as far as the static data has to go through the guard first, so it is
 * disturbed before any variable is. The loop looks at it once a pass and resets through the watchdog when
 * it has changed, which is the safe outcome: the state that overflowed is discarded and every peripheral is
 * initialised again. `.noinit` is not cleared by the start-up code, so the guard is written here at every
 * start. Volatile because nothing in the program writes it again, and the compiler would otherwise fold the
 * test away.
 *
 * Measured on a release build (linker map): `_end` = 0x80099D and RAMEND = 0x10FF (avr-libc, atmega128),
 * so the stack has 0x1100 - 0x99D = 1891 bytes before it reaches the static data. The deepest path known is
 * the formatter's big-integer digit generator, called from the display path, at about 200 bytes. */
#define STACK_GUARD_BYTES 8u
#define STACK_GUARD_FILL 0xA5u

static volatile uint8_t stack_guard[STACK_GUARD_BYTES] __attribute__((section(".noinit")));

static void stack_guard_arm(void) {
    for (uint8_t i = 0u; i < STACK_GUARD_BYTES; i++) {
        stack_guard[i] = STACK_GUARD_FILL;
    }
}

static bool stack_guard_intact(void) {
    for (uint8_t i = 0u; i < STACK_GUARD_BYTES; i++) {
        if (stack_guard[i] != STACK_GUARD_FILL) {
            return false;
        }
    }
    return true;
}

/* ---- Reset and the main loop ------------------------------------------------------------------------ */

static void hardware_init(void) {
    /* The watchdog may have brought us here.
     *
     * WDRF is cleared because nothing else clears it: ATmega128A datasheet p.76, MCUCSR -- "The bit is
     * reset by a Power-on Reset, or by writing a logic zero to the flag". Left set, it would still read as
     * the reset source after an unrelated later reset.
     *
     * wdt_disable() is belt and braces, not a necessity on this part, and the reason is worth stating
     * because the two AVR generations differ. On the ATmega88/168 family WDRF forces WDE to 1, so a
     * watchdog reset really does come back with the watchdog armed and a boot loop follows. Not here:
     * WDTCR's reset value is 0x00 including WDE (datasheet p.77), and with this meter's fuses -- efuse
     * 0xFF, so WDTON and M103C both unprogrammed -- Table 15-1 on p.73 gives safety
     * level 1 and "WDT Initial State: Disabled". So the watchdog is already off here, and the boot loader,
     * which BOOTRST makes the first code to run after every reset, needs no watchdog handling of its own.
     * That matters: main_soft_reset() arms a 15 ms watchdog deliberately, and if WDE survived a reset the
     * loader could never finish its banner wait. */
    MCUCSR &= (uint8_t)~(1u << WDRF);
    wdt_disable();

    port_init();
    tick_init();

    /* The lamp test, second in start-up as the original has it (0x09540). It is not decoration: the
     * PT6311B datasheet (p.14 note 5) says display RAM is UNDEFINED at first power-up, and this is what
     * clears it -- every cell lit, held, then both controllers cleared. Without it the splash can be
     * written over whatever the RAM powered up holding. It costs about 0.8 s and feeds the watchdog
     * throughout. */
    vfd_lamp_test();

    vfd_init();
    display_init();
    keys_init();
    buzzer_init();
    adc_init();
    freq_init();

    state_init();
    uart1_init(g_state.settings.baud_index);
    remote_init();
    serv_init();
    menu_init();
    meas_init();

    sei();
}

int main(void) {
    uint32_t last_hour_ms = 0u;

    stack_guard_arm();
    hardware_init();

    hours_load();
    powerup_count_bump();
    cal_boot_defaults_init();

    display_splash();

    /* The quick relay exercise at power-on. It runs before the first measurement so that the first
     * reading is taken on worked contacts. Only an explicit stored 0 disables it -- an erased byte means
     * ON, per the constants in eeprom_map.h. */
    if (ee_read_u8(EE_SET_RLY_BOOT) != RLY_BOOT_STORED_OFF) {
        meas_relay_exercise_start(false); /* the ~1.5 s pass; `true` is the 60 s menu sequence */
    }

    wdt_enable(WDTO_2S);

    for (;;) {
        wdt_reset();
        if (!stack_guard_intact()) {
            main_soft_reset();
        }

        /* A calibration sweep owns the converter while it runs, so it is advanced before the measurement
         * and the measurement is skipped meanwhile. */
        if (cal_sweep_busy()) {
            (void)cal_sweep_task();
        } else {
            meas_task();
        }

        /* The host link. `remote_poll()` assembles at most one line per pass and runs it, so a stream of
         * commands cannot starve the measurement -- and it is also what finishes a sweep: it watches
         * `cal_sweep_state()` and sends the closing `!` when the last cell is done. Without this call a
         * sweep would complete silently and the client would wait for ever.
         *
         * IT RUNS AFTER `meas_task()` AND BEFORE THE READING IS TAKEN, so that `FETCh?` is answered.
         * `fetch_service()`, at the end of
         * `remote_poll()`, replies only while `meas_reading_ready()` is true -- which is only true between
         * `meas_task()` setting MEAS_COMPLETE and `meas_take_reading()` clearing it to MEAS_IDLE. With
         * `remote_poll()` first in the pass, that window opened and shut entirely between two of its calls,
         * so it was never once observed and `:FETC?` set a pending flag that nothing ever answered. The
         * vendor utility polls `Fetc?` for every reading, so it would have timed out on
         * all of them.
         *
         * Ordering was the right fix rather than a flag: the window exists, it is one statement long, and
         * the reader just had to be inside it. */
        remote_poll();

        /* A completed pass has to be consumed or the machine stays at MEAS_COMPLETE and never starts
         * another: nothing inside `meas.c` takes its own reading. The display and `FETCh?` both read
         * `g_state.reading`, so the order here matters -- take it, then draw. */
        if (meas_reading_ready()) {
            meas_take_reading();
        }

        /* Keys. A function key pressed inside a menu closes it without executing, and main re-dispatches
         * it -- menu_mode must still be set for that second dispatch, which is
         * why it is cleared afterwards and not by menu.c. */
        {
            const uint8_t key = keys_poll();

            if (key != KEY_NONE) {
                const bool was_in_menu = menu_active();

                serv_note_front_panel_key();

                /* A key acknowledges the ERR lamp -- but only when the lamp was actually on
                 * show. `display_annunciators()` does not draw the measurement
                 * lamps while a menu is open, and draws nothing at all while the display is disabled by
                 * `:DISPlay:ENABle OFF`. Acknowledging regardless meant an error raised during a menu
                 * operation was cleared by the next keystroke of that same operation, and the operator
                 * never saw there had been one.
                 *
                 * An acknowledgement is supposed to mean "I have seen it". If it could not have been seen,
                 * the key does not acknowledge it, and the lamp is there to be noticed when the menu
                 * closes. The error QUEUE is never touched either way. */
                if (!menu_active() && g_state.display_enabled) {
                    remote_error_acknowledge();
                }
                keys_dispatch(key);
                if (was_in_menu && !menu_active() && key >= 0x11u && key <= 0x16u) {
                    keys_dispatch(key);
                    g_state.menu_mode = false;
                }
            }
        }

        /* The menu's timekeeping, out here rather than in the ISR. Once per elapsed
         * millisecond, and catching up if a pass took longer than one tick, so a message timeout or a
         * blink phase still means what it says. Bounded, so a long pass cannot turn into a long burst of
         * menu work that then starves the loop it is running in. */
        {
            static uint32_t menu_ms_done;
            const uint32_t now = millis();
            uint8_t done = 0u;

            if (now < menu_ms_done) {
                menu_ms_done = now; /* the counter wrapped */
            }
            while (menu_ms_done != now && done < MENU_TICK_CATCHUP_MAX) {
                menu_ms_done++;
                done++;
                if (menu_tick()) {
                    /* That tick ran an ACTIVITY step, not just timekeeping -- a sweep cell, a relay
                     * exercise step, or a ZERO CAL capture that blocks for tens of milliseconds on the
                     * converter. Catching up eight of those back to back means seconds with no key scan,
                     * no remote_poll and no display_update, and the "any key aborts" the operator was
                     * promised is not serviced until the batch ends.
                     *
                     * Timekeeping can be caught up in a batch because it is arithmetic. Work cannot, so
                     * the backlog waits: the loop goes round, everything else gets its turn, and the next
                     * pass takes the next step. */
                    break;
                }
            }
            /* `done` counts UP, so it is only ever compared against a constant and can never wrap into a
             * value that means the opposite of what it says. A countdown tested as `budget-- != 0u` wraps
             * from 0 to 255 on the last iteration, the resync never fires, and after any stall the loop
             * crawls forward eight milliseconds per pass. */
            if (done == MENU_TICK_CATCHUP_MAX && menu_ms_done != now) {
                menu_ms_done = now; /* too far behind to catch up: resynchronise rather than crawl */
            }
        }

        display_update();

        /* The hour meter, once an hour. */
        if ((millis() - last_hour_ms) >= 3600000UL) {
            last_hour_ms = millis();
            if (hours_now < 0xFFFEu) {
                hours_now++;
                hours_store();
            }
        }
    }
}
