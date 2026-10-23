/* adcbus.c -- bit-banged bus to the external measurement chip (PORTB, SPI mode 3 framing).
 *
 * Pins: PB0 /CS (active low), PB1 SCLK (idles high), PB2 DIN (to chip), PB3 DOUT (from chip),
 * PB4 /DRDY (low = result ready). Data is shifted MSB first; DIN changes while SCLK is low and
 * DOUT is sampled just after SCLK returns high.
 *
 * The chip is a TI ADS1216Y. The byte sequences reproduce the original. The bit timing
 * reproduces its slope (15 cycles per delay unit, 1.25 us at 12 MHz) and keeps every datasheet-bound
 * gap at or above both the datasheet minimum with margin and the original's gap (see below). The bus
 * write time is about 14.6 us per bit by static count: per bit BUS_DELAY 1+2+1+2 = 162 cycles = 13.5 us
 * of delay plus the port and loop instructions.
 */
#include "adcbus.h"

#include <avr/io.h>
#include <util/delay.h>

#include "../board.h"

/* Delay primitive. The original routine 0x3578 costs, counted instruction by instruction,
 * 13 + 15n cycles in its body plus 5 cycles of call setup (6 at the two call sites
 * 0xF2B0 and 0xF2D6). BUS_DELAY(n) models the 5-cycle case, so every gap built from it is a
 * cycle-accurate LOWER bound: the instructions between the delays (port writes, the shift, loop
 * control) only add to it, and an interrupt can only lengthen it. The argument must be a constant. */
#define BUS_DELAY_BODY_FIXED_CYCLES 13UL
#define BUS_DELAY_CALL_SETUP_CYCLES 5UL
#define BUS_DELAY_CYCLES_PER_UNIT 15UL
#define BUS_DELAY_COST(units)                                                                      \
    (BUS_DELAY_CALL_SETUP_CYCLES + BUS_DELAY_BODY_FIXED_CYCLES +                                   \
     BUS_DELAY_CYCLES_PER_UNIT * (units))
#define BUS_DELAY(units) __builtin_avr_delay_cycles(BUS_DELAY_COST(units))

/* Converter timing limits: TI ADS1216Y, datasheet SBAS171D, timing table on p.8. All limits are multiples of tOSC = 1 / fOSC.
 *
 * UNVERIFIED: ADS_FOSC_HZ. The converter clock is taken as 4.910 MHz;
 * the crystal may instead be 4.9152 MHz. To check: read the crystal marking or measure XOUT
 * (converter pin 24). The lower frequency is used deliberately: it gives the longer tOSC and so the
 * larger absolute limits, which is the conservative side for every minimum below. */
#define ADS_FOSC_HZ 4910000UL
#define NS_PER_S 1000000000ULL

#define ADS_T6_TOSC 50u  /* RDATA byte last SCLK edge to first data SCLK edge (p.8, t6). */
#define ADS_T10_TOSC 16u /* /CS low after the final SCLK edge (p.8, t10). */
#define ADS_RESET_TOSC 16u /* After RESET, before the next command (p.8, RESET note). */

/* Safety margin over a datasheet minimum, as a percentage: 1.25 = 125 %. */
#define ADS_MARGIN_PERCENT 125ULL

/* Gaps the original produces, from the exact cycle count at 12 MHz. They are
 * kept as floors so the port is never faster than a unit known to work. */
#define ADS_ORIGINAL_RDATA_GAP_NS 23000ULL
#define ADS_ORIGINAL_CS_HOLD_NS 7700ULL
#define ADS_ORIGINAL_RESET_GAP_NS 7700ULL

/* Datasheet minimum in nanoseconds, rounded up. At 4.910 MHz: t6 = 50 tOSC = 10.18 us (1.25 x =
 * 12.7 us, so the original's 23 us is about 2.3 x the limit); t10 = RESET = 16 tOSC = 3.26 us. */
#define ADS_MIN_NS(tosc_count) (((tosc_count) * NS_PER_S + ADS_FOSC_HZ - 1u) / ADS_FOSC_HZ)
#define ADS_MARGINED_NS(tosc_count) ((ADS_MIN_NS(tosc_count) * ADS_MARGIN_PERCENT + 99u) / 100u)
#define ADS_MAX_NS(a, b) ((a) > (b) ? (a) : (b))

#define ADS_T6_MIN_NS ADS_MIN_NS(ADS_T6_TOSC)
#define ADS_T6_GAP_NS ADS_MAX_NS(ADS_MARGINED_NS(ADS_T6_TOSC), ADS_ORIGINAL_RDATA_GAP_NS)
#define ADS_T10_HOLD_NS ADS_MAX_NS(ADS_MARGINED_NS(ADS_T10_TOSC), ADS_ORIGINAL_CS_HOLD_NS)
#define ADS_RESET_GAP_NS ADS_MAX_NS(ADS_MARGINED_NS(ADS_RESET_TOSC), ADS_ORIGINAL_RESET_GAP_NS)

/* Nanoseconds to CPU cycles at F_CPU_HZ, rounded up. */
#define NS_TO_CYCLES(ns) (((ns) * (uint64_t)F_CPU_HZ + NS_PER_S - 1u) / NS_PER_S)

#define ADS_T6_GAP_CYCLES NS_TO_CYCLES(ADS_T6_GAP_NS)
#define ADS_T10_HOLD_CYCLES NS_TO_CYCLES(ADS_T10_HOLD_NS)
#define ADS_RESET_GAP_CYCLES NS_TO_CYCLES(ADS_RESET_GAP_NS)

/* Delay-only lower bounds of the gaps the byte routines below produce.
 *  - WRITE_TAIL: after the last rising SCLK edge of a written byte (BUS_DELAY 1 and 2).
 *  - WRITE_LEAD: from /CS low to the first falling SCLK edge of a written byte (BUS_DELAY 2).
 *  - READ_LEAD:  from /CS low to the first falling SCLK edge of a read byte (BUS_DELAY 10). */
#define WRITE_TAIL_CYCLES (BUS_DELAY_COST(1) + BUS_DELAY_COST(2))
#define WRITE_LEAD_CYCLES BUS_DELAY_COST(2)
#define READ_LEAD_CYCLES BUS_DELAY_COST(10)

/* The remainder each limit still needs beyond what the byte routines already provide. */
#define RDATA_GAP_TOPUP_CYCLES (ADS_T6_GAP_CYCLES - (WRITE_TAIL_CYCLES + READ_LEAD_CYCLES))
#define CS_HOLD_TOPUP_CYCLES (ADS_T10_HOLD_CYCLES - WRITE_TAIL_CYCLES)

_Static_assert(ADS_T6_GAP_CYCLES > WRITE_TAIL_CYCLES + READ_LEAD_CYCLES,
               "the RDATA gap needs a top-up delay; the formula below assumes one");
_Static_assert(ADS_T10_HOLD_CYCLES > WRITE_TAIL_CYCLES,
               "the /CS hold needs a top-up delay; the formula below assumes one");
/* Two commands in a row (any command, RESET included): last edge of the first, then the lead of
 * the next write, with or without /CS released in between. */
_Static_assert(WRITE_TAIL_CYCLES + WRITE_LEAD_CYCLES >= ADS_RESET_GAP_CYCLES,
               "back-to-back commands must respect the post-RESET gap");
/* The read path ends with a read byte, then a BUS_DELAY(10), then /CS high. */
_Static_assert(BUS_DELAY_COST(10) >= ADS_T10_HOLD_CYCLES,
               "/CS hold after a data read must respect t10");

#define ADC_MASK(bit) ((uint8_t)(1u << (bit)))

/* Command bytes. */
#define ADC_CMD_RDATA 0x01u
#define ADC_CMD_WREG 0x50u
#define ADC_CMD_SELFCAL 0xF0u
#define ADC_CMD_SELFOCAL 0xF1u
#define ADC_CMD_SELFGCAL 0xF2u
#define ADC_CMD_SYSOCAL 0xF3u
#define ADC_CMD_SYSGCAL 0xF4u

/* Result length in bytes (24-bit two's complement, MSB first). */
#define ADC_RESULT_BYTES 3u

static inline void cs_low(void)
{
    ADC_PORT &= (uint8_t)~ADC_MASK(ADC_CS);
}

static inline void cs_high(void)
{
    ADC_PORT |= ADC_MASK(ADC_CS);
}

/* Ends a transfer whose last byte was written: holds /CS low until t10 is met, then releases it. */
static inline void finish_after_write(void)
{
    __builtin_avr_delay_cycles(CS_HOLD_TOPUP_CYCLES);
    cs_high();
}

void adc_init(void)
{
    ADC_PORT |= ADC_MASK(ADC_CS) | ADC_MASK(ADC_SCLK) | ADC_MASK(ADC_DIN) | ADC_MASK(ADC_DOUT) |
                ADC_MASK(ADC_DRDY);
    DDRB |= ADC_MASK(ADC_CS) | ADC_MASK(ADC_SCLK) | ADC_MASK(ADC_DIN);
    DDRB &= (uint8_t) ~(ADC_MASK(ADC_DOUT) | ADC_MASK(ADC_DRDY));
}

/* Selects the chip and clocks one byte out. /CS is left asserted. */
void adc_write_byte(uint8_t b)
{
    cs_low();
    BUS_DELAY(2);
    for (uint8_t i = 0; i < 8; i++) {
        ADC_PORT &= (uint8_t)~ADC_MASK(ADC_SCLK);
        BUS_DELAY(1);
        if (b & 0x80u) {
            ADC_PORT |= ADC_MASK(ADC_DIN);
        } else {
            ADC_PORT &= (uint8_t)~ADC_MASK(ADC_DIN);
        }
        BUS_DELAY(2);
        ADC_PORT |= ADC_MASK(ADC_SCLK);
        BUS_DELAY(1);
        b = (uint8_t)(b << 1);
        BUS_DELAY(2);
    }
}

/* Selects the chip and clocks one byte in. /CS is left asserted. */
uint8_t adc_read_byte(void)
{
    uint8_t value = 0;

    cs_low();
    BUS_DELAY(10);
    for (uint8_t i = 0; i < 8; i++) {
        ADC_PORT &= (uint8_t)~ADC_MASK(ADC_SCLK);
        BUS_DELAY(3);
        ADC_PORT |= ADC_MASK(ADC_SCLK);
        value = (uint8_t)(value << 1);
        if (ADC_PIN & ADC_MASK(ADC_DOUT)) {
            value |= 1u;
        }
        BUS_DELAY(1);
    }
    BUS_DELAY(10);
    return value;
}

/* Write command, a zero count byte, then the value. */
void adc_write_register(uint8_t reg, uint8_t value)
{
    cs_low();
    adc_write_byte((uint8_t)(reg | ADC_CMD_WREG));
    adc_write_byte(0x00);
    adc_write_byte(value);
    finish_after_write();
}

void adc_command(uint8_t cmd)
{
    cs_low();
    adc_write_byte(cmd);
    finish_after_write();
}

/* Polls /DRDY about every 100 microseconds. The original waited forever. */
bool adc_wait_ready(uint16_t timeout_ms)
{
    for (uint32_t polls = (uint32_t)timeout_ms * 10u; polls != 0; polls--) {
        if ((ADC_PIN & ADC_MASK(ADC_DRDY)) == 0) {
            return true;
        }
        _delay_us(100);
    }
    return (ADC_PIN & ADC_MASK(ADC_DRDY)) == 0;
}

bool adc_read_result(int32_t *out_code, uint16_t timeout_ms)
{
    if (!adc_wait_ready(timeout_ms)) {
        return false;
    }

    cs_low();
    BUS_DELAY(2);
    adc_write_byte(ADC_CMD_RDATA);
    /* t6: the write tail plus the first read byte's lead-in fall short of the required gap. */
    __builtin_avr_delay_cycles(RDATA_GAP_TOPUP_CYCLES);

    uint32_t raw = 0;
    for (uint8_t i = 0; i < ADC_RESULT_BYTES; i++) {
        raw = (raw << 8) | adc_read_byte();
    }
    BUS_DELAY(10);
    cs_high();

    if (raw & 0x800000UL) {
        raw |= 0xFF000000UL;
    }
    *out_code = (int32_t)raw;
    return true;
}
