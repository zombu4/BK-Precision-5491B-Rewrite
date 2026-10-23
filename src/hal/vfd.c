/* vfd.c -- bit-banged driver for the two PT6311-class VFD controllers.
 *
 * Pins (board.h): PC0 CLK, PC1 DIN, PC2 DOUT (key data), PC4 /STB of the main controller, PC3 /STB of the
 * secondary one. Every byte is clocked out LSB first. A frame is /STB low, bytes, /STB high; a data write
 * is two frames in a row: the 0x40 data command, then the address command (0xC0 | address) with its data
 * bytes. Display memory has three bytes per character cell: glyph high byte at 3n, low byte at 3n+1.
 *
 * Original: disp_send_byte 0x1F3C, disp_write_byte 0x1F60, disp_write_word 0x1F8A, disp_clear_all 0x1FD4,
 * disp_lamp_test 0x2026, the controller set-up of disp_init_both_simple 0x2106, the text printers 0x256E
 * and 0x2B4E, and the key read half of key_scan_read 0x3774.
 *
 * PORTC also carries front-end bits (PC5-PC7). All accesses below are single-bit sbi/cbi, which the
 * compiler emits for constant bit operations on this low I/O register, so they cannot lose a bit
 * written by another module between read and write.
 */
#include "vfd.h"

#include <avr/cpufunc.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "../board.h"
#include "../tables.h"

#define VFD_MASK(bit) ((uint8_t)(1u << (bit)))

/* Commands. */
#define CMD_MODE 0x3Au
#define CMD_DISPLAY_OFF 0x87u

/* The display-on command carries the dimming field in its low three bits, so there is no such thing as a
 * constant "display on": it is always "on, at some brightness". The old `CMD_DISPLAY_ON 0x8Fu` was on at
 * dimming 7, and `vfd_init()` -- which the display refresh calls every frame -- resent it, so the BRIGHT
 * setting would be undone on the next refresh. Everything that
 * turns the display on now asks for the current level. */
static uint8_t display_on_command(void);

/* The display-control byte is 0b10xx1nnn: bit 3 lights the display and the low three bits set the
 * dimming, which is why the original's blank (0x87) and light (0x8F) differ in bit 3 alone and both carry
 * nnn = 7. Verified against PT6311B V1.0 page 9; see
 * vfd.h for the full table and for why the steps are not evenly spaced. */
#define CMD_DISPLAY_BASE 0x88u
#define CMD_DATA_WRITE 0x40u
#define CMD_KEY_READ 0x42u
#define CMD_ADDRESS 0xC0u

/* Display memory size per controller, in bytes. */
#define MAIN_RAM_BYTES 0x24u
#define SECOND_RAM_BYTES 0x21u

#define LAMP_ON 0xFFu
#define LAMP_OFF 0x00u

/* Annunciator table rows 0-11 sit on the main controller, rows 12-23 on the secondary one. */
#define ANNUN_FIRST_SECOND_ROW 12u

#define KEYSCAN_BYTES 3u

#define LAMP_TEST_MS 800u

/* Bit timing. The original loop at 0x1F3C needed about 24 cycles per bit: CLK was high for about 16 of
 * them (1.3 us) and low for about 7 (0.6 us). Under -Os the loop below would be much faster, so the two
 * phases are padded to the original widths. */
#define CLK_HIGH_PAD_CYCLES 12UL
#define CLK_LOW_PAD_CYCLES 3UL

static inline void stb_low(vfd_dev_t dev) {
    if (dev == VFD_MAIN) {
        VFD_PORT &= (uint8_t)~VFD_MASK(VFD_STB1);
    } else {
        VFD_PORT &= (uint8_t)~VFD_MASK(VFD_STB2);
    }
}

static inline void stb_high(vfd_dev_t dev) {
    if (dev == VFD_MAIN) {
        VFD_PORT |= VFD_MASK(VFD_STB1);
    } else {
        VFD_PORT |= VFD_MASK(VFD_STB2);
    }
}

/* Original disp_send_byte (0x1F3C): one byte, LSB first, /STB handled by the caller. */
static void shift_out(uint8_t value) {
    for (uint8_t bit = 0; bit < 8u; bit++) {
        VFD_PORT &= (uint8_t)~VFD_MASK(VFD_CLK);
        if (value & 1u) {
            VFD_PORT |= VFD_MASK(VFD_DIN);
        } else {
            VFD_PORT &= (uint8_t)~VFD_MASK(VFD_DIN);
        }
        __builtin_avr_delay_cycles(CLK_LOW_PAD_CYCLES);
        VFD_PORT |= VFD_MASK(VFD_CLK);
        __builtin_avr_delay_cycles(CLK_HIGH_PAD_CYCLES);
        value >>= 1;
    }
}

/* One single-byte command frame. */
static void command(vfd_dev_t dev, uint8_t cmd) {
    stb_low(dev);
    shift_out(cmd);
    stb_high(dev);
}

/* The two frames that start every data write: 0x40, then the address command. /STB is left low, ready
 * for the data bytes. */
static void begin_write(vfd_dev_t dev, uint8_t address) {
    command(dev, CMD_DATA_WRITE);
    stb_low(dev);
    shift_out((uint8_t)(CMD_ADDRESS | address));
}

static void write_bytes(vfd_dev_t dev, uint8_t address, const uint8_t *data, uint8_t count) {
    begin_write(dev, address);
    for (uint8_t i = 0; i < count; i++) {
        shift_out(data[i]);
    }
    stb_high(dev);
}

/* Blocking wait that keeps the watchdog fed; only the boot-time lamp test uses it. */
static void wait_ms(uint16_t ms) {
    while (ms-- != 0u) {
        wdt_reset();
        _delay_ms(1);
    }
}

void vfd_init(void) {
    for (uint8_t d = 0; d <= (uint8_t)VFD_SECOND; d++) {
        const vfd_dev_t dev = (vfd_dev_t)d;
        command(dev, CMD_MODE);
        command(dev, display_on_command());
        command(dev, CMD_DATA_WRITE);
    }
}

void vfd_clear(vfd_dev_t dev) {
    const uint8_t count = (dev == VFD_MAIN) ? MAIN_RAM_BYTES : SECOND_RAM_BYTES;

    begin_write(dev, 0u);
    for (uint8_t i = 0; i < count; i++) {
        shift_out(0u);
    }
    stb_high(dev);
}

/* Both controllers: mode, display off, auto-increment data, every cell lit, display on; then a pause to
 * look at it and a clear. */
void vfd_lamp_test(void) {
    for (uint8_t d = 0; d <= (uint8_t)VFD_SECOND; d++) {
        const vfd_dev_t dev = (vfd_dev_t)d;
        const uint8_t count = (dev == VFD_MAIN) ? MAIN_RAM_BYTES : SECOND_RAM_BYTES;

        command(dev, CMD_MODE);
        command(dev, CMD_DISPLAY_OFF);
        begin_write(dev, 0u);
        for (uint8_t i = 0; i < count; i++) {
            shift_out(0xFFu);
        }
        stb_high(dev);
        command(dev, display_on_command());
    }
    wait_ms(LAMP_TEST_MS);
    vfd_clear(VFD_MAIN);
    vfd_clear(VFD_SECOND);
}

void vfd_write_cell(vfd_dev_t dev, uint8_t cell, uint16_t glyph) {
    const uint8_t bytes[2] = {(uint8_t)(glyph >> 8), (uint8_t)glyph};

    write_bytes(dev, (uint8_t)(3u * cell), bytes, 2u);
}

void vfd_write_flags(uint8_t flags) {
    vfd_write_cell(VFD_MAIN, VFD_FLAG_ADDR / 3u, (uint16_t)((uint16_t)flags << 8));
}

/* ---- Text -------------------------------------------------------------------------------------------- */


/* Symbol glyph indexes in tables.h order: m k u @ - + % / & n d. */
enum { SYM_M, SYM_K, SYM_U, SYM_AT, SYM_MINUS, SYM_PLUS, SYM_PERCENT, SYM_SLASH, SYM_AMP, SYM_N, SYM_D };

#define GLYPH_DP 0x0080u
#define GLYPH_COLON 0x4080u /* dp bit plus bit 14 */
#define GLYPH_BAR 0x8001u   /* '-' in free text */
#define GLYPH_CARET 0x0014u

/* Glyph for one character of free text (original 0x256E / 0x2B4E). Returns false for characters the
 * printers skip. *takes_marks is set when a following '.' or ':' joins the cell. */
static bool text_glyph(char c, uint16_t *glyph, bool *takes_marks) {
    *takes_marks = true;
    if (c == ' ') {
        *glyph = 0u;
    } else if (c == '-') {
        *glyph = GLYPH_BAR;
    } else if (c >= 'A' && c <= 'Z') {
        *glyph = tbl_glyph_alpha((uint8_t)(c - 'A'));
    } else if (c >= '0' && c <= '9') {
        *glyph = tbl_glyph_digit((uint8_t)(c - '0'));
    } else {
        *takes_marks = false;
        switch (c) {
        case '^':
            *glyph = GLYPH_CARET;
            break;
        case 'm':
            *glyph = tbl_glyph_symbol(SYM_M);
            break;
        case 'k':
            *glyph = tbl_glyph_symbol(SYM_K);
            break;
        case 'u':
            *glyph = tbl_glyph_symbol(SYM_U);
            break;
        case '@':
            *glyph = tbl_glyph_symbol(SYM_AT);
            break;
        case '+':
            *glyph = tbl_glyph_symbol(SYM_PLUS);
            break;
        case '%':
            *glyph = tbl_glyph_symbol(SYM_PERCENT);
            break;
        case '/':
            *glyph = tbl_glyph_symbol(SYM_SLASH);
            break;
        case '&':
            *glyph = tbl_glyph_symbol(SYM_AMP);
            break;
        case 'n':
            *glyph = tbl_glyph_symbol(SYM_N);
            break;
        case 'd':
            *glyph = tbl_glyph_symbol(SYM_D);
            break;
        default:
            return false;
        }
    }
    return true;
}

/* Streams a string into consecutive cells from cell 0, with one address frame for the whole run. Stops at
 * the end of the string or after VFD_TEXT_CELLS cells. Characters without a glyph are skipped. A '.' or
 * ':' after a letter, digit, space or '-' sets the decimal point (or colon) of that cell instead of
 * taking a cell of its own. */
static void print_text(vfd_dev_t dev, const char *s) {
    uint8_t cells = 0;

    begin_write(dev, 0u);
    while (*s != '\0') {
        uint16_t glyph;
        bool takes_marks;

        if (text_glyph(*s, &glyph, &takes_marks)) {
            if (takes_marks) {
                const char next = s[1];

                if (next == '.') {
                    glyph |= GLYPH_DP;
                    s++;
                } else if (next == ':') {
                    glyph |= GLYPH_COLON;
                    s++;
                }
            }
            shift_out((uint8_t)(glyph >> 8));
            shift_out((uint8_t)glyph);
            stb_high(dev);
            stb_low(dev);
            cells++;
            if (cells == VFD_TEXT_CELLS) {
                break;
            }
            shift_out((uint8_t)(CMD_ADDRESS | (3u * cells)));
        }
        s++;
    }
    stb_high(dev);
}

void vfd_text(vfd_dev_t dev, const char *s) { print_text(dev, s); }

/* ---- Annunciators and keys --------------------------------------------------------------------------- */

void vfd_set_annunciator(uint8_t annun_id, bool on) {
    if (annun_id >= ANNUN_COUNT) {
        return;
    }
    const vfd_dev_t dev = (annun_id < ANNUN_FIRST_SECOND_ROW) ? VFD_MAIN : VFD_SECOND;
    const uint8_t address = pgm_read_byte(&annun_table[annun_id].vfd_addr);
    const uint8_t value = on ? LAMP_ON : LAMP_OFF;

    write_bytes(dev, address, &value, 1u);
}

static uint8_t brightness_level = VFD_BRIGHT_DEFAULT;

/* Declared up with the command constants, because that is where a reader looks for "how is the display
 * turned on". */
static uint8_t display_on_command(void) {
    return (uint8_t)(CMD_DISPLAY_BASE + brightness_level);
}

void vfd_set_brightness(uint8_t level) {
    if (level > VFD_BRIGHT_MAX) {
        level = VFD_BRIGHT_MAX;
    }
    brightness_level = level;
    /* Both controllers, so the two halves of the panel cannot end up at different brightnesses -- which
     * would look like a hardware fault rather than a setting. */
    command(VFD_MAIN, (uint8_t)(CMD_DISPLAY_BASE + level));
    command(VFD_SECOND, (uint8_t)(CMD_DISPLAY_BASE + level));
}

/* Command 0x42 on the main controller, then three bytes clocked in LSB first. The original stopped at the
 * first non-empty byte; reading all three is the same wire protocol and the caller decodes them. */
void vfd_read_keyscan(uint8_t out[3]) {
    VFD_PORT &= (uint8_t)~VFD_MASK(VFD_STB1);
    shift_out(CMD_KEY_READ);
    /* The original waits here with delay_us(1) at 0x3786, inside key_scan_read 0x3774. That call costs
     * 5 + 13 + 15 = 33 cycles, i.e. 2.75 us at 12 MHz, not the 1 us that `_delay_us(1)` gives -- so this
     * gap used to be 1.75 us shorter than the original's. There is no datasheet for the display
     * controller to check a shorter gap against, so the original's figure is the floor. */
    __builtin_avr_delay_cycles(ORIG_DELAY_US_CYCLES(1));
    for (uint8_t i = 0; i < KEYSCAN_BYTES; i++) {
        uint8_t value = 0;

        for (uint8_t bit = 0; bit < 8u; bit++) {
            value >>= 1;
            VFD_PORT |= VFD_MASK(VFD_CLK);
            _NOP();
            _NOP();
            _NOP();
            VFD_PORT &= (uint8_t)~VFD_MASK(VFD_CLK);
            _NOP();
            _NOP();
            _NOP();
            if (VFD_PIN & VFD_MASK(VFD_DOUT)) {
                value |= 0x80u;
            }
        }
        out[i] = value;
    }
    VFD_PORT |= VFD_MASK(VFD_STB1);
}
