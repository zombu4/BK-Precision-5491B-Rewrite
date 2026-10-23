/*
 * boot.c - BK 5491B boot loader (separate program in the boot section, 0x1F800 and up).
 *
 * Behaviour (ported from the original loader):
 *   - Initialise UART1 (38400 nominal, 8 data bits, 1 stop bit, no parity), Timer0 (one tick every
 *     20.05 ms) and the front-panel display controller, then read one key-scan byte.
 *   - Unless that byte is 0x03 (both horizontal arrow keys held) start the application at once.
 *   - With the arrow keys held, print a banner and wait about 4 s for ENTER. ENTER starts an
 *     XMODEM-CRC download into flash from address 0 upwards; no ENTER means run the application.
 *   - 'C' is sent every ~1.36 s until the host starts a block with SOH.
 *
 * This is a stand-alone program: it must not use any application module. Interrupts are never enabled.
 *
 * Differences from the original (each one fixes a defect listed in ORIGINAL_BUGS):
 *   1. Nothing at or above BOOT_START is ever written, and the first block must carry a plausible
 *      reset vector (a JMP into the application area) before any flash is touched.
 *   1a. A PARTLY WRITTEN IMAGE IS NEVER STARTED. EE_BOOT_XFER is set before the first page is written and
 *      cleared only after the whole image has arrived and verified; this loader refuses to start the
 *      application while it is set, whatever the keys say. Without it, an interrupted download left the
 *      new image's first pages and the old image's remainder in flash, and the next POWER-UP ran that
 *      mixture without anyone choosing to -- recoverable only with an ISP programmer.
 *   2. A final partial page is padded with 0xFF, not with the previous page's data.
 *   3. Every page is read back and compared after it is written.
 *   4. The receiver resynchronises on SOH/EOT, every wait is bounded, and a bad frame is always
 *      consumed completely before it is rejected with NAK. Too many NAKs end the transfer.
 *   5. The banner no longer claims a 'd' command: the code never read the serial line, the real trigger
 *      is the ENTER key, so the banner now says so. (The serial 'd' command was not implemented.)
 *   7. A TRUNCATED IMAGE IS NEVER STARTED. XMODEM carries no length and an early EOT is a legal end of
 *      transfer, so a host that died mid-file, or a short file, passed every per-frame and per-page check
 *      above and was started. The image sent is now the application, 0xFF-padded to whole frames, plus one
 *      trailer frame (magic "BK", length, CRC16). The loader holds each frame back by one, so at EOT the
 *      frame in hand is the trailer and is never written. The CRC is computed over the flash contents
 *      after the last page is written, and EE_BOOT_XFER is cleared only if it matches (PROTOCOL_CHANGES G.5).
 *   6. UART1 runs at 38461.5 baud (double speed, UBRR1 = 38) with 1 stop bit, like the application. The
 *      original's 39474 baud was +2.8 % off 38400, and it echoed 'o', '@' and 'n' for each key it saw.
 */

#include <avr/boot.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/delay.h>

/* Only for EE_BOOT_XFER and its sentinel. The loader calls no application code; it carries its own
 * EEPROM access below, because a stand-alone program that linked against application modules would
 * stop being stand-alone. */
#include "../eeprom_map.h"

#ifndef F_CPU
#define F_CPU 12000000UL
#endif

#define BOOT_START 0x1F800UL
#define PAGE_BYTES 256U
#define FRAME_BYTES 128U

_Static_assert(SPM_PAGESIZE == PAGE_BYTES, "flash page size");
_Static_assert(BOOT_START % PAGE_BYTES == 0, "boot section is page aligned");
_Static_assert(PAGE_BYTES % FRAME_BYTES == 0, "frames tile a page");

/* UART1 in double-speed mode: baud = 12 000 000 / (8 * (UBRR1 + 1)). UBRR1 = 38 gives 12e6 / 312 =
 * 38 461.5 baud, +0.16 % on 38400 (ATmega128 doc2467, USART baud rate equation for U2X = 1). The
 * original's UBRR1 = 18 with U2X = 0 gave 12e6 / (16 * 19) = 39 474 baud, +2.8 %. */
#define UBRR1_VALUE 38U

/* Timer0 in CTC mode, OCR0 = 234, prescaler 1024: one OCF0 tick every 20.05 ms. */
#define TIMER0_COMPARE 0xEAU
#define TIMER0_CTC_DIV1024 (_BV(WGM01) | _BV(CS02) | _BV(CS01) | _BV(CS00))

/* Display controller on PORTC. PC2 (DOUT) is the only input. */
#define VFD_CLK PC0
#define VFD_DIN PC1
#define VFD_DOUT PC2
#define VFD_STB PC4
#define PORTC_DIR 0xFBU
#define PORTC_IDLE 0x3FU

#define VFD_CMD_MODE 0x3BU
#define VFD_CMD_DISPLAY 0x8BU
#define VFD_CMD_READ_KEYS 0x42U
#define VFD_INIT_SETTLE_MS 100

#define KEY_ARROWS 0x03U /* both horizontal arrow keys */
#define KEY_ENTER 0x10U

#define ENTER_WAIT_TICKS 200U      /* about 4.0 s */
#define REQUEST_CRC_TICKS 67U      /* 'C' is sent when the count exceeds this: every 68 ticks */
#define BYTE_TIMEOUT_TICKS 50U     /* about 1 s between bytes inside a frame */
#define HEADER_TIMEOUT_TICKS 250U  /* about 5 s between frames */
#define MAX_CONSECUTIVE_NAKS 10U

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define CAN 0x18

/* A JMP instruction is 0x940C followed by a 16-bit word address. */
#define JMP_OPCODE_LO 0x0CU
#define JMP_OPCODE_HI 0x94U

/* DEFAULT: on a failed transfer the loader stays resident and offers the download again. */
typedef enum {
    XFER_OK = 0,
    XFER_ERR_LIMIT = 'L',   /* image would reach the boot section */
    XFER_ERR_IMAGE = 'I',   /* first block is not a plausible application reset vector */
    XFER_ERR_VERIFY = 'V',  /* page read-back differs from what was written */
    XFER_ERR_CRC = 'C',     /* whole-image CRC in flash differs from the trailer's */
    XFER_ERR_RETRY = 'T'    /* host silent or too many rejected frames */
} xfer_status_t;

typedef enum { POLL_NONE, POLL_ARROWS, POLL_ENTER, POLL_OTHER } poll_result_t;

typedef enum { FRAME_NEW, FRAME_DUPLICATE, FRAME_REJECT } frame_result_t;

static const char banner[] = "Press ENTER to download, others run app.\n\r";

static uint8_t page_buf[PAGE_BYTES];

/* The most recent accepted frame, not yet appended to page_buf. Holding one frame back is what lets the
 * loader recognise the trailer at EOT without ever having written it to flash. */
static uint8_t hold_buf[FRAME_BYTES];

/* Trailer frame: [0] 0x42, [1] 0x4B, [2..5] application length (uint32, little-endian), [6..7] CRC16 of
 * the application bytes (big-endian, the same order as the frame CRC), [8..127] 0xFF. */
#define TRAILER_MAGIC_0 0x42U
#define TRAILER_MAGIC_1 0x4BU

/* ------------------------------------------------------------------------------------------------ */
/* UART1                                                                                            */

static void uart_init(void) {
    UBRR1H = 0;
    UBRR1L = UBRR1_VALUE;
    UCSR1A = _BV(U2X1);
    UCSR1B = _BV(RXEN1) | _BV(TXEN1);
    UCSR1C = _BV(UCSZ11) | _BV(UCSZ10); /* 8 data bits, 1 stop bit, no parity */
}

static void uart_putc(uint8_t c) {
    while (!(UCSR1A & _BV(UDRE1))) {
    }
    UDR1 = c;
}

static void uart_puts(const char *s) {
    while (*s != '\0') {
        uart_putc((uint8_t)*s++);
    }
}

/* One received byte, or -1 if none is waiting or the byte had a frame, overrun or parity error. */
static int16_t uart_try_getc(void) {
    if (!(UCSR1A & _BV(RXC1))) {
        return -1;
    }
    uint8_t status = UCSR1A;
    uint8_t data = UDR1;
    if (status & (_BV(FE1) | _BV(DOR1) | _BV(UPE1))) {
        return -1;
    }
    return data;
}

/* ------------------------------------------------------------------------------------------------ */
/* Timer0 ticks                                                                                     */

static void tick_clear(void) {
    TIFR = _BV(OCF0);
}

static bool tick_elapsed(void) {
    if (TIFR & _BV(OCF0)) {
        TIFR = _BV(OCF0);
        return true;
    }
    return false;
}

/* Waits for a byte for at most about ticks * 20 ms (ticks >= 1). Returns -1 on timeout. */
/* Drain the line until it has been quiet for PURGE_IDLE_TICKS, then return (the standard XMODEM purge).
 *
 * Without it, a rejected frame left the receiver somewhere in the middle of the sender's stream, and
 * `wait_header()` took the next SOH or EOT it saw as a header. A payload byte of 0x04 is an ordinary value
 * -- it is part of any image that contains one -- so a single dropped byte could shift the stream, be read
 * as EOT, end the transfer, clear the in-progress flag, and start a TRUNCATED image that the loader then
 * believed was complete. That is the one failure the flag exists to prevent, arriving through the front
 * door.
 *
 * Two ticks at 20.05 ms each is about 40 ms of silence. One 133-byte frame at 38461.5 baud 8N1 (10 bits per
 * character) takes 133 * 10 / 38461.5 = 34.6 ms, so a sender still transmitting a frame cannot be quiet that
 * long, and a sender waiting for our NAK is quiet immediately. */
#define PURGE_IDLE_TICKS 2u

static int16_t uart_getc_timeout(uint16_t ticks);

static void uart_purge(void) {
    while (uart_getc_timeout(PURGE_IDLE_TICKS) >= 0) {
    }
}

static int16_t uart_getc_timeout(uint16_t ticks) {
    tick_clear();
    for (;;) {
        int16_t c = uart_try_getc();
        if (c >= 0) {
            return c;
        }
        if (tick_elapsed() && --ticks == 0) {
            return -1;
        }
    }
}

/* ------------------------------------------------------------------------------------------------ */
/* Front-panel display controller (bit-banged on PORTC)                                             */

static inline void settle(void) {
    __asm__ volatile("nop\n\tnop\n\tnop");
}

static void vfd_send_byte(uint8_t value) {
    for (uint8_t bit = 0; bit < 8; bit++) {
        PORTC &= (uint8_t)~_BV(VFD_CLK);
        if (value & 1U) {
            PORTC |= _BV(VFD_DIN);
        } else {
            PORTC &= (uint8_t)~_BV(VFD_DIN);
        }
        PORTC |= _BV(VFD_CLK);
        value >>= 1;
    }
}

/* First key-scan data byte, LSB first on DOUT. */
static uint8_t vfd_read_keys(void) {
    uint8_t keys = 0;

    PORTC &= (uint8_t)~_BV(VFD_STB);
    vfd_send_byte(VFD_CMD_READ_KEYS);
    _delay_us(3);
    for (uint8_t bit = 0; bit < 8; bit++) {
        keys >>= 1;
        PORTC |= _BV(VFD_CLK);
        settle();
        PORTC &= (uint8_t)~_BV(VFD_CLK);
        settle();
        if (PINC & _BV(VFD_DOUT)) {
            keys |= 0x80U;
        }
    }
    PORTC |= _BV(VFD_STB);
    return keys;
}

static void vfd_init(void) {
    PORTC &= (uint8_t)~_BV(VFD_STB);
    vfd_send_byte(VFD_CMD_MODE);
    PORTC |= _BV(VFD_STB);
    PORTC &= (uint8_t)~_BV(VFD_STB);
    vfd_send_byte(VFD_CMD_DISPLAY);
    PORTC |= _BV(VFD_STB);
    _delay_ms(VFD_INIT_SETTLE_MS);
}

/*
 * Reports a key event: nothing while no key is held or the same key stays held. A new key byte is
 * read twice (the second read is the one that counts). The original also echoed 'o', '@' or 'n' on the
 * UART for each new key; those were debug output nothing reads, and they are gone.
 */
static poll_result_t poll_key(void) {
    static uint8_t last_key;

    uint8_t key = vfd_read_keys();
    if (key == 0) {
        last_key = 0;
        return POLL_NONE;
    }
    if (key == last_key) {
        return POLL_NONE;
    }
    key = vfd_read_keys();
    last_key = key;
    if (key == KEY_ARROWS) {
        return POLL_ARROWS;
    }
    if (key == KEY_ENTER) {
        return POLL_ENTER;
    }
    return POLL_OTHER;
}

/* ------------------------------------------------------------------------------------------------ */
/* Hand-off to the application                                                                      */

static void start_app(void) __attribute__((noreturn));

static void start_app(void) {
    UCSR1A = _BV(TXC1) | _BV(U2X1); /* TXC1 is write-one-to-clear; U2X1 stays set so "OK" keeps its rate */
    uart_puts("OK\r\n");
    while (!(UCSR1A & _BV(TXC1))) {
    }
    MCUCR = _BV(IVCE);
    MCUCR = 0;
    RAMPZ = 0;
    __asm__ volatile("jmp 0");
    __builtin_unreachable();
}

/* ------------------------------------------------------------------------------------------------ */
/* Flash programming                                                                                */

/* Waits for the SPM operation and re-enables the read-while-write section if it was locked out. */
static void spm_complete(void) {
    boot_spm_busy_wait();
    while (boot_rww_busy()) {
        boot_rww_enable();
        boot_spm_busy_wait();
    }
}

/* Erases, programs and verifies the page at byte address addr from page_buf. */
/* ---- EEPROM, the loader's own ----------------------------------------------------------------------
 * The ATmega128's write sequence needs EEMWE set and EEWE set within four cycles, with no interrupt in
 * between. Interrupts are never enabled in this loader (see the file header), so there is nothing to
 * disable -- but the two stores still have to be adjacent, which is why this is written out rather than
 * borrowed from the application's hal/eeprom.c. */
static uint8_t ee_get(uint16_t addr) {
    while ((EECR & _BV(EEWE)) != 0u) {
    }
    EEAR = addr;
    EECR |= _BV(EERE);
    return EEDR;
}

static void ee_put(uint16_t addr, uint8_t value) {
    while ((EECR & _BV(EEWE)) != 0u) {
    }
    EEAR = addr;
    EEDR = value;
    EECR |= _BV(EEMWE);
    EECR |= _BV(EEWE);
}

/* Write only when it differs. EEPROM cells have a limited write life and this one is touched on every
 * download; a meter reflashed weekly for ten years would otherwise spend 520 writes here for no reason. */
static void xfer_flag_set(bool active) {
    const uint8_t want = active ? EE_BOOT_XFER_ACTIVE : 0x00u;

    if (ee_get(EE_BOOT_XFER) != want) {
        ee_put(EE_BOOT_XFER, want);
    }
}

static bool xfer_flag_active(void) {
    return ee_get(EE_BOOT_XFER) == EE_BOOT_XFER_ACTIVE;
}

static xfer_status_t flash_write_page(uint32_t addr) {
    if (addr >= BOOT_START) {
        return XFER_ERR_LIMIT;
    }

    boot_spm_busy_wait();
    boot_page_erase(addr);
    spm_complete();
    for (uint16_t i = 0; i < PAGE_BYTES; i += 2) {
        uint16_t word = (uint16_t)(page_buf[i] | ((uint16_t)page_buf[i + 1] << 8));
        boot_page_fill(addr + i, word);
    }
    boot_page_write(addr);
    spm_complete();

    for (uint16_t i = 0; i < PAGE_BYTES; i++) {
        if (pgm_read_byte_far(addr + i) != page_buf[i]) {
            return XFER_ERR_VERIFY;
        }
    }
    return XFER_OK;
}

/* ------------------------------------------------------------------------------------------------ */
/* XMODEM-CRC receiver                                                                              */

/* CRC-16/XMODEM: polynomial 0x1021, initial value 0, MSB first. */
/* The only copy of the polynomial: the frame check and the whole-image check both run through it, so
 * they cannot disagree about what a CRC is. */
static uint16_t crc16_xmodem_byte(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x8000U) {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t crc16_xmodem(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0;
    while (len-- > 0) {
        crc = crc16_xmodem_byte(crc, *data++);
    }
    return crc;
}

static bool read_byte(uint8_t *out) {
    int16_t c = uart_getc_timeout(BYTE_TIMEOUT_TICKS);
    if (c < 0) {
        return false;
    }
    *out = (uint8_t)c;
    return true;
}

/*
 * Reads the rest of a frame (the SOH is already consumed) into payload. The whole frame is always
 * consumed, so a rejected frame never leaves part of itself on the line; a timeout means the line is
 * idle already. DUPLICATE is the previous block again (the host missed our ACK).
 */
static frame_result_t receive_frame(uint8_t expected, bool have_previous, uint8_t *payload) {
    uint8_t block;
    uint8_t complement;
    uint8_t crc_hi;
    uint8_t crc_lo;

    if (!read_byte(&block) || !read_byte(&complement)) {
        return FRAME_REJECT;
    }
    for (uint8_t i = 0; i < FRAME_BYTES; i++) {
        if (!read_byte(&payload[i])) {
            return FRAME_REJECT;
        }
    }
    if (!read_byte(&crc_hi) || !read_byte(&crc_lo)) {
        return FRAME_REJECT;
    }

    uint16_t received_crc = (uint16_t)(((uint16_t)crc_hi << 8) | crc_lo);
    const uint8_t block_inverted = (uint8_t)(~(unsigned)block);
    if (block_inverted != complement || crc16_xmodem(payload, FRAME_BYTES) != received_crc) {
        return FRAME_REJECT;
    }
    if (block == expected) {
        return FRAME_NEW;
    }
    if (have_previous && block == (uint8_t)(expected - 1U)) {
        return FRAME_DUPLICATE;
    }
    return FRAME_REJECT;
}

/* True if the first four bytes are a JMP to a non-zero word address inside the application area. */
static bool image_header_valid(const uint8_t *head) {
    uint16_t target = (uint16_t)(head[2] | ((uint16_t)head[3] << 8));
    return head[0] == JMP_OPCODE_LO && head[1] == JMP_OPCODE_HI && target != 0 && target < BOOT_START / 2;
}

/*
 * Waits for the next SOH or EOT, ignoring anything else (resynchronisation). While the host is silent
 * a NAK goes out every HEADER_TIMEOUT_TICKS. Returns -1 once MAX_CONSECUTIVE_NAKS is exceeded.
 */
static int16_t wait_header(uint8_t *naks) {
    for (;;) {
        int16_t c = uart_getc_timeout(HEADER_TIMEOUT_TICKS);
        if (c == SOH || c == EOT) {
            return c;
        }
        if (c < 0) {
            if (++*naks > MAX_CONSECUTIVE_NAKS) {
                return -1;
            }
            uart_putc(NAK);
        }
    }
}

/*
 * EOT: hold_buf is the trailer, not image data. Checks the trailer, writes the last partial page, then
 * checks the CRC of what is really in flash. Returns XFER_OK only if the whole image is there.
 *
 * Without this an image cut short by a dead host or a truncated file passed every per-frame and per-page
 * check and was started; XMODEM has no length of its own and an early EOT is legal.
 */
static xfer_status_t image_finish(uint16_t frames, uint32_t page_addr, uint16_t fill) {
    /* A trailer alone is not an image. */
    if (frames < 2) {
        return XFER_ERR_IMAGE;
    }

    /* A held frame that is not a trailer means the host sent a bare image, or a different file. The length
     * may not exceed what arrived (a short transfer) nor the application area. */
    uint32_t length = (uint32_t)hold_buf[2] | ((uint32_t)hold_buf[3] << 8) | ((uint32_t)hold_buf[4] << 16) |
                      ((uint32_t)hold_buf[5] << 24);
    if (hold_buf[0] != TRAILER_MAGIC_0 || hold_buf[1] != TRAILER_MAGIC_1 || length == 0 ||
        length > BOOT_START || length > (uint32_t)(frames - 1U) * FRAME_BYTES) {
        return XFER_ERR_IMAGE;
    }

    if (fill != 0) {
        for (uint16_t i = fill; i < PAGE_BYTES; i++) {
            page_buf[i] = 0xFF;
        }
        xfer_status_t status = flash_write_page(page_addr);
        if (status != XFER_OK) {
            return status;
        }
    }

    /* The CRC is read from flash, not from the receive path, so it also covers a page that was never
     * written. Per-page read-back proves each page equals what was sent; only this proves the pages sent
     * are the image the trailer describes. */
    uint16_t crc = 0;
    for (uint32_t addr = 0; addr < length; addr++) {
        crc = crc16_xmodem_byte(crc, pgm_read_byte_far(addr));
    }
    if (crc != (uint16_t)(((uint16_t)hold_buf[6] << 8) | hold_buf[7])) {
        return XFER_ERR_CRC;
    }
    return XFER_OK;
}

/* Receives the image whose first header byte is header. The ACK for a block follows its flash write. */
static xfer_status_t transfer(int16_t header) {
    uint32_t page_addr = 0;
    uint16_t fill = 0;
    uint16_t frames = 0; /* frames accepted so far, the trailer included */
    uint8_t expected = 1;
    uint8_t naks = 0;
    bool have_previous = false;

    /* Before any flash is touched. From here until this function returns XFER_OK, a power cut leaves the
     * flag set and the next reset stays in the loader. It is deliberately set even for a
     * transfer that fails before its first page write -- the cost is one EEPROM write and a boot that
     * waits for a download, and the alternative is reasoning about exactly which failures can have written
     * a page, which is the kind of reasoning that was wrong the first time. */
    xfer_flag_set(true);

    for (;;) {
        if (header == EOT) {
            xfer_status_t status = image_finish(frames, page_addr, fill);
            if (status != XFER_OK) {
                return status; /* the flag stays set: the half-written application is never started */
            }
            uart_putc(ACK);
            /* The trailer matched and the CRC of the flash contents matched it. This is the only place the
             * flag is cleared. */
            xfer_flag_set(false);
            return XFER_OK;
        }

        frame_result_t result = receive_frame(expected, have_previous, &page_buf[fill]);
        if (result == FRAME_REJECT) {
            if (++naks > MAX_CONSECUTIVE_NAKS) {
                return XFER_ERR_RETRY;
            }
            /* Out of step with the sender: discard the rest of whatever it is sending before asking for a
             * retransmission, so the next byte we treat as a header really is one. */
            uart_purge();
            uart_putc(NAK);
        } else {
            if (result == FRAME_NEW) {
                /* The new frame sits at page_buf[fill]. Early reject on the first frame, before flash. */
                if (frames == 0 && !image_header_valid(&page_buf[fill])) {
                    return XFER_ERR_IMAGE;
                }
                /* Swap: the previously held frame lands in page_buf and becomes image data; the new frame
                 * is held back until the next frame or EOT shows whether it is data or the trailer. */
                for (uint8_t i = 0; i < FRAME_BYTES; i++) {
                    uint8_t newest = page_buf[fill + i];
                    page_buf[fill + i] = hold_buf[i];
                    hold_buf[i] = newest;
                }
                if (frames != 0) {
                    fill += FRAME_BYTES;
                    if (fill == PAGE_BYTES) {
                        xfer_status_t status = flash_write_page(page_addr);
                        if (status != XFER_OK) {
                            return status;
                        }
                        page_addr += PAGE_BYTES;
                        fill = 0;
                    }
                }
                frames++;
                expected++;
                have_previous = true;
            }
            naks = 0;
            uart_putc(ACK);
        }

        header = wait_header(&naks);
        if (header < 0) {
            return XFER_ERR_RETRY;
        }
    }
}

/* Sends 'C' every 68 ticks until the host starts a block. Anything but SOH is ignored. */
static void request_download(void) {
    uint8_t ticks = 0;

    tick_clear();
    for (;;) {
        if (uart_try_getc() == SOH) {
            return;
        }
        if (tick_elapsed() && ++ticks > REQUEST_CRC_TICKS) {
            uart_putc('C');
            ticks = 0;
        }
    }
}

static void report_failure(xfer_status_t status) {
    uart_putc(CAN);
    uart_putc(CAN);
    uart_puts("FAIL ");
    uart_putc((uint8_t)status);
    uart_puts("\r\n");
}

/* ------------------------------------------------------------------------------------------------ */

int main(void) {
    MCUCR = _BV(IVCE);
    MCUCR = _BV(IVSEL);

    uart_init();
    OCR0 = TIMER0_COMPARE;
    TCCR0 = TIMER0_CTC_DIV1024;
    DDRC = PORTC_DIR;
    PORTC = PORTC_IDLE;
    vfd_init();

    /* A guard on the key shortcut alone is not a guard: an incomplete image that skipped this
     * `start_app()` would fall straight into the banner loop below, which starts the application on its
     * own after ENTER_WAIT_TICKS, so the half-written image would run anyway, four seconds later.
     *
     * `incomplete` therefore gates EVERY path out of this function that can start the application. With it set
     * the only exit is a transfer that completes and verifies -- which is the state the operator needs to
     * be in, and is reachable over the serial port with no programmer. */
    const bool incomplete = xfer_flag_active();

    if (incomplete) {
        uart_puts("Incomplete image. Download required.\n\r");
    } else if (poll_key() != POLL_ARROWS) {
        start_app();
    }

    uart_puts(banner);
    tick_clear();
    uint8_t ticks = 0;
    for (;;) {
        if (poll_key() == POLL_ENTER) {
            break;
        }
        if (tick_elapsed() && ++ticks > ENTER_WAIT_TICKS && !incomplete) {
            start_app();
        }
    }

    for (;;) {
        request_download();
        xfer_status_t status = transfer(SOH);
        if (status == XFER_OK) {
            start_app();
        }
        report_failure(status);
    }
}
