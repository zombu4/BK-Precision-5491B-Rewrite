/* adcbus.h -- bit-banged bus to the external measurement chip (PORTB, SPI mode 3 framing).
 * FROZEN. meas/ consumes it.
 *
 * The chip is the TI ADS1216 (opcodes checked against ADS1216 SBAS171D p.26). It
 * is clocked with SCLK idling high (POL = 1, SBAS171D p.8 and p.18), which is SPI mode 3 framing. The
 * data-rate registers 8 and 9 are written by meas.c. Reproduce the original byte sequences EXACTLY; never
 * "improve" the bus and never invent chip behaviour -- anything unclear is marked UNVERIFIED.
 *
 * /DRDY is polled with a bounded timeout. adc_wait_ready() returns false on timeout; the
 * caller reports an error. The original hung forever.
 */
#ifndef BK_ADCBUS_H
#define BK_ADCBUS_H
#include <stdbool.h>
#include <stdint.h>
#define ADC_DRDY_TIMEOUT_MS 2000u
void adc_init(void);
void adc_write_byte(uint8_t b);
uint8_t adc_read_byte(void);
void adc_write_register(uint8_t reg, uint8_t value);
void adc_command(uint8_t cmd);        /* SELFCAL / SELFGCAL / SELFOCAL / SYSOCAL / SYSGCAL */
bool adc_wait_ready(uint16_t timeout_ms);
bool adc_read_result(int32_t *out_code, uint16_t timeout_ms); /* 24-bit signed result */
#endif
