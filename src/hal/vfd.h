/* vfd.h -- two PT6311-class VFD controllers (PC0 CLK, PC1 DIN, PC4 /STB main, PC3 /STB secondary).
 * FROZEN.
 *
 * Bytes are clocked LSB first. Commands: 0x3A mode, 0x87/0x8F display off / on brightness 7,
 * 0x40 data write auto-increment, 0xC0|addr address. Display memory is 3 bytes per character cell
 * (high glyph byte at 3n, low at 3n+1). Address 0x1B carries a flag byte (bit 5 = minus sign).
 * Controller 1 also returns the key-scan bytes (see keys.h) -- do not drive /STB1 from two places.
 */
#ifndef BK_VFD_H
#define BK_VFD_H
#include <stdbool.h>
#include <stdint.h>
typedef enum { VFD_MAIN = 0, VFD_SECOND = 1 } vfd_dev_t;
#define VFD_TEXT_CELLS 9u   /* text printers stop after 9 cells */
#define VFD_NUM_CELLS 6u    /* numeric printers stop after 6 */
#define VFD_FLAG_ADDR 0x1Bu
#define VFD_FLAG_MINUS 0x20u
void vfd_init(void);
void vfd_clear(vfd_dev_t dev);
void vfd_lamp_test(void);
void vfd_write_cell(vfd_dev_t dev, uint8_t cell, uint16_t glyph);
void vfd_write_flags(uint8_t flags);
void vfd_text(vfd_dev_t dev, const char *s);
void vfd_set_annunciator(uint8_t annun_id, bool on); /* writes 0x00/0xFF at the table address */
void vfd_read_keyscan(uint8_t out[3]);               /* command 0x42 to controller 1 */

/* ---- Display brightness ---------------------------------------------------------------------
 * `level` is 0 (dimmest) to VFD_BRIGHT_MAX (brightest). VFD_BRIGHT_DEFAULT is what an unset setting gives.
 *
 * VERIFIED against the datasheet: PT6311B V1.0 (October 2009), COMMAND 4 "DISPLAY CONTROL COMMANDS",
 * page 9. The command byte is
 *
 *     1 0 - - b3 b2 b1 b0      bits 5-4 "Not Relevant"
 *                 b3 = display:  0 = off (key scan continues), 1 = on
 *                 b2..b0 = dimming
 *
 * which confirms what the original's own two constants implied -- 0x87 is display-off and 0x8F display-on,
 * both at dimming 7.
 *
 * THE STEPS ARE NOT LINEAR, and that is the thing worth knowing. From the datasheet table:
 *
 *     000 = 1/16   001 = 2/16   010 = 4/16   011 = 10/16
 *     100 = 11/16  101 = 12/16  110 = 13/16  111 = 14/16
 *
 * The chip has exactly these 8 steps ("8-Step Dimming Circuitry", datasheet features list). The menu
 * offers all 8, shown as 1 (code 0, 1/16) to 8 (code 7, 14/16), because
 * the top five alone (10/16 to 14/16) gave too little visible change.
 *
 * Power-on state, from the same page: 1/16 pulse width with the display OFF. So the display must be
 * explicitly turned on at init; it does not default to visible. */
#define VFD_BRIGHT_MAX 7u
#define VFD_BRIGHT_DEFAULT 7u
void vfd_set_brightness(uint8_t level);
#endif
