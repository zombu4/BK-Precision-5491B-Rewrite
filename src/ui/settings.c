/* settings.c -- operator settings mirrored between RAM (g_state.settings) and EEPROM.
 *
 * Original: the five settings bytes were read once by boot_load_settings (0x95CE) into RAM 0x0E1B-0x0E1F and
 * written one byte at a time by the SYS menu editors (0x0EA4C, 0x541A) with no verify. The baud divisor was
 * programmed into UBRR1L only (0x5588), so the 600 / 1200 / 2400 baud divisors, which exceed 8 bits, were
 * truncated.
 *
 * Here every SYS setting goes through the same three steps: update g_state.settings, settings_apply(), then
 * settings_store(). Echo, terminator, beeper and key click are read live from g_state.settings by uart1 and
 * the buzzer, so applying them is immediate; the baud rate is pushed into the UART with its full 12-bit
 * divisor.
 */
#include "settings.h"

#include "../eeprom_map.h"
#include "../hal/eeprom.h"
#include "../hal/uart1.h"

#define BAUD_DEFAULT_INDEX 2u /* 9600 baud, the factory setting */

/* Index order is EEPROM-visible (EE_SET_BAUD): baud = 600 * 2^(6 - index). */
const uint32_t baud_table[] = {38400UL, 19200UL, 9600UL, 4800UL, 2400UL, 1200UL, 600UL};
const uint8_t baud_table_count = (uint8_t)(sizeof baud_table / sizeof baud_table[0]);

void settings_load(void) {
    settings_t *s = &g_state.settings;
    const uint8_t baud = ee_read_u8(EE_SET_BAUD);

    /* The original used the byte unchecked, so a blank EEPROM indexed far outside its baud table. */
    s->baud_index = (baud < baud_table_count) ? baud : BAUD_DEFAULT_INDEX;
    s->echo = ee_read_u8(EE_SET_ECHO) != 0u;
    s->tx_terminator = (ee_read_u8(EE_SET_TXTERM) != 0u) ? 1u : 0u;
    s->beeper = ee_read_u8(EE_SET_BEEPER) != 0u;
    s->key_click = ee_read_u8(EE_SET_KEYCLICK) != 0u;
}

/* EEPROM cells have a limited write life, so an unchanged byte is not written again. */
static bool store_byte(uint16_t addr, uint8_t value) {
    if (ee_read_u8(addr) == value) {
        return true;
    }
    return ee_write_u8(addr, value);
}

bool settings_store(void) {
    const settings_t *s = &g_state.settings;
    bool ok = true;

    ok = store_byte(EE_SET_BAUD, s->baud_index) && ok;
    ok = store_byte(EE_SET_ECHO, s->echo ? 1u : 0u) && ok;
    ok = store_byte(EE_SET_TXTERM, s->tx_terminator ? 1u : 0u) && ok;
    ok = store_byte(EE_SET_BEEPER, s->beeper ? 1u : 0u) && ok;
    ok = store_byte(EE_SET_KEYCLICK, s->key_click ? 1u : 0u) && ok;
    return ok;
}

void settings_apply(void) {
    uint8_t baud = g_state.settings.baud_index;

    if (baud >= baud_table_count) {
        baud = BAUD_DEFAULT_INDEX;
    }
    uart1_set_baud(baud);
}
