/* eeprom.h -- EEPROM access with atomic, verified writes.
 * FROZEN. Every module reads through this; nobody touches EEARH/EEDR directly.
 *
 * The original wrote EEMWE/EEWE with interrupts enabled and no verify, so an interrupt
 * between the two writes silently dropped a byte. Here each byte write is inside ATOMIC_BLOCK and the
 * block write verifies by reading back. Record writers must write the signature word LAST.
 */
#ifndef BK_EE_H
#define BK_EE_H
#include <stdbool.h>
#include <stdint.h>
uint8_t ee_read_u8(uint16_t addr);
uint16_t ee_read_u16(uint16_t addr);
float ee_read_f32(uint16_t addr);
void ee_read_block(uint16_t addr, void *dst, uint16_t len);
bool ee_write_u8(uint16_t addr, uint8_t v);              /* verified */
bool ee_write_u16(uint16_t addr, uint16_t v);            /* verified */
bool ee_write_f32(uint16_t addr, float v);               /* verified */
bool ee_write_block(uint16_t addr, const void *src, uint16_t len); /* verified */
#endif
