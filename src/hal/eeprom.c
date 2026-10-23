/* eeprom.c -- EEPROM access with atomic, verified, bounds-checked writes.
 *
 * Replaces the original's eight primitives: read_byte 0x030B2, read_block 0x030C0, read_word 0x030E8,
 * read_dword 0x03116, write_byte 0x03144, write_block 0x03154, write_word 0x0317C, write_dword 0x0319E.
 *
 * The original's write_byte, read from the image at 0x03144, is six instructions:
 *     sbic EECR,EEWE / rjmp back      wait for a previous write
 *     out EEARH / out EEARL / out EEDR
 *     sbi EECR,EEMWE
 *     sbi EECR,EEWE
 * and read_byte at 0x030B2 is the same wait, the address, sbi EECR,EERE, in EEDR.
 *
 * THREE DEFECTS OF THAT CODE ARE FIXED HERE:
 *
 * 1. The EEMWE-to-EEWE window is four clock cycles. The original sets them back to back with interrupts
 *    ENABLED, so any interrupt landing between the two `sbi`s makes the hardware drop the write silently --
 *    no error, no retry, and the byte keeps its old value. This instrument has a 1 ms timer tick, a UART
 *    receive interrupt and two counter interrupts, so the window is hit in normal use. Every write here is
 *    inside ATOMIC_BLOCK.
 * 2. Nothing was ever verified. A dropped or failed byte was indistinguishable from a good one, which is
 *    one way a calibration record becomes half-written. Every write here is read back and compared, and the
 *    functions return false on a mismatch so the caller can refuse to go on.
 * 3. No address was ever checked. The original's `:SCAL:COEF` index arithmetic can run past the end of a
 *    table, and with a 12-bit address register a stray index wraps inside the 4 KB space
 *    and lands on another record. Every access here is bounds-checked against EE_SIZE and refuses rather
 *    than wrapping.
 *
 * ONE DELIBERATE ADDITION, which is not a behaviour change: a byte that already holds the wanted value is
 * not rewritten. The cell is rated for 100,000 erase/write cycles, the original's sweeps rewrote whole
 * float tables unconditionally, and an instrument may be old. Skipping an identical byte is invisible
 * to every caller, cannot affect the signature-last ordering that cal.c relies on, and takes a sweep that
 * changes nothing from thousands of cycles to none.
 *
 * Record writers must still write the signature word LAST; that ordering is cal.c's job, not
 * this file's.
 */
#include "eeprom.h"

#include <avr/io.h>
#include <string.h>
#include <util/atomic.h>

#include "../eeprom_map.h"

/* ---- Primitives ---------------------------------------------------------------------------------------- */

static void wait_ready(void) {
    while ((EECR & (1u << EEWE)) != 0u) {
        /* A write in progress takes 8.5 ms (ATmega128 doc2467 Table 2: 8448 RC cycles). The original waited the same way, at 0x03144. */
    }
}

static uint8_t read_raw(uint16_t addr) {
    wait_ready();
    EEAR = addr;
    EECR |= (1u << EERE);
    return EEDR;
}

/* True when the byte ends up holding `v`, whether or not a write was needed. */
static bool write_raw(uint16_t addr, uint8_t v) {
    if (read_raw(addr) == v) {
        return true;
    }

    wait_ready();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        EEAR = addr;
        EEDR = v;
        EECR |= (1u << EEMWE);
        EECR |= (1u << EEWE);
    }
    return read_raw(addr) == v; /* read_raw waits for the write to finish first */
}

/* ---- Reads --------------------------------------------------------------------------------------------- */

/* An out-of-range read gives 0xFF, which is what an erased cell reads and what every caller already treats
 * as "no data". It cannot be mistaken for a valid signature: neither 0x5554 nor 0x5161 contains 0xFF. */
uint8_t ee_read_u8(uint16_t addr) {
    return (addr < EE_SIZE) ? read_raw(addr) : 0xFFu;
}

uint16_t ee_read_u16(uint16_t addr) {
    return (uint16_t)ee_read_u8(addr) | ((uint16_t)ee_read_u8((uint16_t)(addr + 1u)) << 8);
}

float ee_read_f32(uint16_t addr) {
    union {
        float f;
        uint8_t b[4];
    } v;
    uint8_t i;

    for (i = 0u; i < 4u; i++) {
        v.b[i] = ee_read_u8((uint16_t)(addr + i));
    }
    return v.f;
}

void ee_read_block(uint16_t addr, void *dst, uint16_t len) {
    uint8_t *out = (uint8_t *)dst;
    uint16_t i;

    for (i = 0u; i < len; i++) {
        out[i] = ee_read_u8((uint16_t)(addr + i));
    }
}

/* ---- Writes -------------------------------------------------------------------------------------------- */

bool ee_write_u8(uint16_t addr, uint8_t v) {
    return (addr < EE_SIZE) && write_raw(addr, v);
}

/* A multi-byte write stops at the first byte that will not verify, so a caller that checks the result knows
 * the object is incomplete. For a record that matters, cal.c orders the signature last, which makes an
 * interrupted record INVALID rather than half-valid. */
bool ee_write_u16(uint16_t addr, uint16_t v) {
    return ee_write_u8(addr, (uint8_t)(v & 0xFFu)) &&
           ee_write_u8((uint16_t)(addr + 1u), (uint8_t)(v >> 8));
}

bool ee_write_f32(uint16_t addr, float v) {
    union {
        float f;
        uint8_t b[4];
    } u;
    uint8_t i;

    u.f = v;
    for (i = 0u; i < 4u; i++) {
        if (!ee_write_u8((uint16_t)(addr + i), u.b[i])) {
            return false;
        }
    }
    return true;
}

bool ee_write_block(uint16_t addr, const void *src, uint16_t len) {
    const uint8_t *in = (const uint8_t *)src;
    uint16_t i;

    for (i = 0u; i < len; i++) {
        if (!ee_write_u8((uint16_t)(addr + i), in[i])) {
            return false;
        }
    }
    return true;
}
