/*
 * sim_5491b.c - simavr test harness for the 5491B firmware.
 *
 * Runs the compiled boot loader and application on a simulated ATmega128 and executes 23 fixed tests.
 * This is host code, not firmware. Nothing here touches a serial port, avrdude or a real meter.
 *
 * Modelled peripherals (pin map from src/board.h):
 *   - ADS1216 converter on PORTB, bit-banged by src/hal/adcbus.c: slave samples DIN on the rising SCLK edge
 *     (adcbus.c:148-156) and presents DOUT while SCLK is low, i.e. on the falling edge (adcbus.c:171-176).
 *   - PT6311 display controllers on PORTC: frame capture per /STB, and the key-scan read of command 0x42
 *     (boot.c:185-203, vfd.c:326-354), LSB first.
 *   - UART1 host link with a byte queue in each direction and a paced feeder.
 *   - EEPROM preloaded from an image and read back at the end of a test.
 *
 * What the harness does NOT model : analogue front end, relays, buzzer,
 * the frequency counter inputs, ADC timing of the converter beyond /DRDY, real UART line timing beyond what
 * simavr's avr_uart.c provides (it models one byte time per received byte, avr_uart.c:avr_uart_baud_write).
 *
 * Usage: sim_5491b --app app.elf --boot boot.elf --eeprom eeprom_a.bin [--only N] [--reset-pc ADDR]
 *   --only N        run test N (1..23) only
 *   --reset-pc ADDR override the reset vector of every selected test (the tests choose 0x1F800 for boot
 *                   loader tests and 0 for application tests by themselves; this option exists to isolate
 *                   or to model the fuse setting differently)
 */
#define _GNU_SOURCE
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avr_eeprom.h"
#include "avr_ioport.h"
#include "avr_uart.h"
#include "avr_watchdog.h"
#include "sim_avr.h"
#include "sim_cycle_timers.h"
#include "sim_elf.h"
#include "sim_irq.h"

/* ------------------------------------------------------------------------------------------------ */
/* Constants                                                                                        */

#define F_CPU_HZ 12000000ULL
#define MS(n) ((uint64_t)(n) * 12000ULL)
#define SEC(n) ((uint64_t)(n) * 12000000ULL)

#define FLASH_BYTES 0x20000u
#define BOOT_START 0x1F800u    /* boot.c:36 */
#define APP_REGION_END 0x11200u /* the application region test 1 watches */
#define EEPROM_BYTES 4096u

#define FRAME_BYTES 128u /* boot.c:38 */
#define PAGE_BYTES 256u  /* boot.c:37 */

#define XM_SOH 0x01
#define XM_EOT 0x04
#define XM_ACK 0x06
#define XM_NAK 0x15
#define XM_CAN 0x18 /* boot.c:73-77 */
#define XM_FILL 0xFFu /* pad byte of the unused tail of the last data frame (PROTOCOL_CHANGES G.5) */

/* The trailer block that follows the image (boot.c:125-128): magic "BK", the application length as a
 * little-endian uint32, the CRC16-XMODEM of the application bytes (high byte first), then 0xFF. */
#define TRAILER_MAGIC_0 0x42u
#define TRAILER_MAGIC_1 0x4Bu

#define KEY_ARROWS 0x03u /* boot.c:64 */
#define KEY_ENTER 0x10u  /* boot.c:65 */

/* Data-space addresses of the UART1 registers (iom128.h of the avr-gcc toolchain used by build.py). */
#define ADDR_UBRR1H 0x98u
#define ADDR_UBRR1L 0x99u
#define ADDR_UCSR1B 0x9Au
#define ADDR_UCSR1A 0x9Bu
#define ADDR_UCSR1C 0x9Du
#define ADDR_WDTCR 0x41u /* I/O address 0x21 */
#define WDTCR_WDE 0x08u
#define UCSR1B_RXCIE1 0x80u

#define RX_MAX (1u << 20)
#define TX_Q_MAX (1u << 16)
#define VFD_FRAMES_MAX 200000u
#define VFD_FRAME_BYTES 64u

/* EEPROM regions the application writes by design (src/eeprom_map.h lines 156, 210, 219-222). */
#define EE_SERV_FAILS 0x0A74u
#define EE_PWRUP_COUNT 0x0A90u
#define EE_HOURS_BASE 0x0A94u
#define EE_HOURS_END 0x0AB4u
#define EE_SET_ECHO 0x0865u /* src/eeprom_map.h:128 */
#define EE_ID_MODEL 0x0810u
#define EE_ID_SERIAL 0x0867u
#define EE_ID_MODEL_BYTES 28u
#define EE_ID_SERIAL_BYTES 20u

/* ------------------------------------------------------------------------------------------------ */
/* Global state                                                                                     */

typedef enum { T_PASS, T_FAIL, T_SKIP } result_t;

typedef struct {
    char port;
    uint8_t mask;
    uint8_t value;
} ext_port_t;

typedef enum { ADS_FREE_RUN, ADS_NEVER_READY } ads_mode_t;
typedef enum { ADS_CMD, ADS_RREG_COUNT, ADS_WREG_COUNT, ADS_WREG_DATA } ads_state_t;

typedef struct {
    uint8_t dev; /* 0 main (PC4), 1 second (PC3) */
    uint8_t n;
    uint8_t b[VFD_FRAME_BYTES];
} vfd_frame_t;

/* Display memory of the two controllers, kept as the frames arrive (see vfd_model_frame()). */
#define VFD_RAM_BYTES 0x24u /* vfd.c:49-50: main 0x24 bytes, second 0x21 */
typedef struct {
    uint8_t b[2][VFD_RAM_BYTES];
} vfd_ram_t;

/* One display-control command (0x88 to 0x8F), numbered by the frame it was in. */
typedef struct {
    uint64_t seq;
    uint8_t dev;
    uint8_t byte;
} vfd_ctl_t;
#define VFD_CTL_MAX (1u << 16)

static struct {
    avr_t *avr;
    uint64_t base_cycles;
    unsigned resets;
    int last_state;

    /* configuration of the current test */
    uint8_t keys[3];
    bool echo;

    /* images */
    uint8_t ee_initial[EEPROM_BYTES];

    /* region counters (steps whose next pc lay in each region) */
    uint64_t app_steps;
    uint64_t boot_steps;
    uint64_t stray_steps;

    /* UART */
    avr_irq_t *uart_in;
    uint8_t *rx_buf;
    uint64_t *rx_time;
    size_t rx_len;
    bool rx_overflow;
    uint8_t *tx_q;
    size_t tx_head;
    size_t tx_tail;
    uint64_t next_feed;
    uint64_t last_fed;

    /* converter */
    struct {
        ads_mode_t mode;
        uint32_t conv_delay_us;
        uint32_t cal_delay_us;
        int32_t code;
        uint8_t regs[16];
        bool cs, sclk, din;
        ads_state_t st;
        uint8_t rx_shift;
        uint8_t rx_bits;
        uint8_t reg_ptr;
        uint8_t left;
        bool tx_active;
        bool tx_is_rdata;
        uint8_t tx_bytes[16];
        uint8_t tx_len;
        uint8_t tx_bits_out;
        unsigned n_rdata, n_wreg, n_rreg, n_cal, n_reset, n_unknown;
        uint8_t last_unknown;
    } ads;

    /* display controllers */
    struct {
        bool clk, din;
        bool stb[2];
        int active;
        uint8_t shift;
        uint8_t bits;
        vfd_frame_t cur;
        bool reading_keys;
        unsigned nfall;
        vfd_frame_t *frames;
        size_t nframes;
        unsigned key_reads;

        /* Not limited by the size of frames[]: the application redraws the panel after every measurement
         * pass (main.c:555, display.c:867-881), which is tens of frames per pass, so a run of a few seconds
         * fills frames[]. These two are kept up to date for every frame regardless. */
        uint64_t seq; /* frames completed so far, whether or not they were stored in frames[] */
        vfd_ram_t ram;
        vfd_ctl_t ctl[VFD_CTL_MAX];
        size_t nctl;
        bool ctl_overflow;
    } vfd;

    ext_port_t ext_b;
    ext_port_t ext_c;
} S;

static const char *g_app_path;
static const char *g_boot_path;
static const char *g_eeprom_path;
static int g_reset_pc_override = -1;

static uint8_t g_app_img[FLASH_BYTES];
static uint32_t g_app_len;
static uint8_t g_boot_img[FLASH_BYTES];
static uint32_t g_boot_lo;
static uint32_t g_boot_hi;
static uint8_t g_ee_file[EEPROM_BYTES];

static char g_why[2048];

#define FAILF(...)                                                                                       \
    do {                                                                                                 \
        snprintf(g_why, sizeof g_why, __VA_ARGS__);                                                      \
        return T_FAIL;                                                                                   \
    } while (0)
#define REQUIRE(cond, ...)                                                                               \
    do {                                                                                                 \
        if (!(cond)) {                                                                                   \
            FAILF(__VA_ARGS__);                                                                          \
        }                                                                                                \
    } while (0)

static void info(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void info(const char *fmt, ...) {
    va_list ap;
    printf("       ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}

static uint64_t now(void) { return S.base_cycles + S.avr->cycle; }

static double seconds(uint64_t cycles) { return (double)cycles / (double)F_CPU_HZ; }

/* ------------------------------------------------------------------------------------------------ */
/* Pin driving                                                                                      */

static avr_irq_t *pin_irq(char port, unsigned bit) {
    return avr_io_getirq(S.avr, AVR_IOCTL_IOPORT_GETIRQ(port), (int)bit);
}

/*
 * Drives an input pin. avr_ioport.c:avr_ioport_update_irqs() re-raises an input pin with its PORT bit set
 * (internal pull-up) on every PORT write, which would overwrite a value driven only through the IRQ. The
 * external pull state (AVR_IOCTL_IOPORT_SET_EXTERNAL) is consulted first by that function, so the driven
 * value is also stored there.
 */
static void drive_pin(ext_port_t *ep, unsigned bit, bool level) {
    avr_ioport_external_t e;

    ep->mask = (uint8_t)(ep->mask | (1u << bit));
    ep->value = (uint8_t)((ep->value & ~(1u << bit)) | (level ? (1u << bit) : 0u));
    e.name = (unsigned long)ep->port;
    e.mask = ep->mask;
    e.value = ep->value;
    avr_ioctl(S.avr, AVR_IOCTL_IOPORT_SET_EXTERNAL(ep->port), &e);
    avr_raise_irq(pin_irq(ep->port, bit), level ? 1u : 0u);
}

/* After an AVR reset the PIN registers are zero (sim_avr.c:avr_reset) while the IRQ value is unchanged, and
 * the IRQ is filtered; toggling the line restores the PIN register. */
static void redrive_pins(void) {
    ext_port_t *ports[2] = {&S.ext_b, &S.ext_c};

    for (unsigned p = 0; p < 2; p++) {
        for (unsigned bit = 0; bit < 8; bit++) {
            if (ports[p]->mask & (1u << bit)) {
                bool level = (ports[p]->value >> bit) & 1u;

                avr_raise_irq(pin_irq(ports[p]->port, bit), level ? 0u : 1u);
                avr_raise_irq(pin_irq(ports[p]->port, bit), level ? 1u : 0u);
            }
        }
    }
}

/* ------------------------------------------------------------------------------------------------ */
/* ADS1216 model                                                                                    */

#define ADS_CS 0u
#define ADS_SCLK 1u
#define ADS_DIN 2u
#define ADS_DOUT 3u
#define ADS_DRDY 4u

/* Command bytes: adcbus.c:105-112 and meas.c:93. */
#define ADS_CMD_RDATA 0x01u
#define ADS_CMD_RREG 0x10u
#define ADS_CMD_WREG 0x50u
#define ADS_CMD_SELFCAL 0xF0u
#define ADS_CMD_SYSGCAL 0xF4u
#define ADS_CMD_RESET 0xFEu

/* When each RDATA command was received, for the tests that need to know when the converter was read. */
#define RDATA_LOG_MAX (1u << 16)
static uint64_t g_rdata_t[RDATA_LOG_MAX];
static size_t g_rdata_n;

static void ads_set_drdy(bool low) { drive_pin(&S.ext_b, ADS_DRDY, !low); }

static avr_cycle_count_t ads_drdy_timer(avr_t *avr, avr_cycle_count_t when, void *param) {
    (void)avr;
    (void)when;
    (void)param;
    ads_set_drdy(true);
    return 0;
}

/* A conversion starts: /DRDY goes high now and falls after delay_us, unless the model never signals. */
static void ads_start_conversion(uint32_t delay_us) {
    avr_cycle_timer_cancel(S.avr, ads_drdy_timer, NULL);
    ads_set_drdy(false);
    if (S.ads.mode == ADS_FREE_RUN) {
        avr_cycle_timer_register_usec(S.avr, delay_us, ads_drdy_timer, NULL);
    }
}

static void ads_set_mode(ads_mode_t mode) {
    S.ads.mode = mode;
    if (mode == ADS_NEVER_READY) {
        avr_cycle_timer_cancel(S.avr, ads_drdy_timer, NULL);
        ads_set_drdy(false);
    } else {
        ads_start_conversion(S.ads.conv_delay_us);
    }
}

static void ads_arm_tx(const uint8_t *bytes, unsigned len, bool is_rdata) {
    memcpy(S.ads.tx_bytes, bytes, len);
    S.ads.tx_len = (uint8_t)len;
    S.ads.tx_bits_out = 0;
    S.ads.tx_active = true;
    S.ads.tx_is_rdata = is_rdata;
}

static void ads_byte(uint8_t b) {
    switch (S.ads.st) {
    case ADS_CMD:
        if ((b & 0xF0u) == ADS_CMD_RREG) {
            S.ads.reg_ptr = b & 0x0Fu;
            S.ads.st = ADS_RREG_COUNT;
        } else if ((b & 0xF0u) == ADS_CMD_WREG) {
            S.ads.reg_ptr = b & 0x0Fu;
            S.ads.st = ADS_WREG_COUNT;
        } else if (b == ADS_CMD_RDATA) {
            uint32_t v = (uint32_t)S.ads.code & 0xFFFFFFu;
            uint8_t out[3] = {(uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};

            S.ads.n_rdata++;
            if (g_rdata_n < RDATA_LOG_MAX) {
                g_rdata_t[g_rdata_n++] = now();
            }
            ads_arm_tx(out, 3, true);
        } else if (b >= ADS_CMD_SELFCAL && b <= ADS_CMD_SYSGCAL) {
            S.ads.n_cal++;
            ads_start_conversion(S.ads.cal_delay_us);
        } else if (b == ADS_CMD_RESET) {
            S.ads.n_reset++;
            memset(S.ads.regs, 0, sizeof S.ads.regs);
            ads_start_conversion(S.ads.conv_delay_us);
        } else {
            S.ads.n_unknown++;
            S.ads.last_unknown = b;
        }
        break;
    case ADS_RREG_COUNT: {
        unsigned count = (unsigned)(b & 0x0Fu) + 1u;
        uint8_t out[16];

        for (unsigned i = 0; i < count; i++) {
            out[i] = S.ads.regs[(S.ads.reg_ptr + i) & 0x0Fu];
        }
        S.ads.n_rreg++;
        ads_arm_tx(out, count, false);
        S.ads.st = ADS_CMD;
        break;
    }
    case ADS_WREG_COUNT:
        S.ads.left = (uint8_t)((b & 0x0Fu) + 1u);
        S.ads.st = ADS_WREG_DATA;
        break;
    case ADS_WREG_DATA:
        S.ads.regs[S.ads.reg_ptr & 0x0Fu] = b;
        S.ads.reg_ptr++;
        if (--S.ads.left == 0) {
            S.ads.n_wreg++;
            S.ads.st = ADS_CMD;
        }
        break;
    }
}

static void ads_tx_done(void) {
    S.ads.tx_active = false;
    drive_pin(&S.ext_b, ADS_DOUT, false);
    if (S.ads.tx_is_rdata) {
        /* The result has been read: /DRDY returns high and the next conversion starts. */
        ads_start_conversion(S.ads.conv_delay_us);
    }
}

static void ads_sclk_rising(void) {
    if (S.ads.tx_active) {
        if (S.ads.tx_bits_out == 8u * S.ads.tx_len) {
            ads_tx_done();
        }
        return;
    }
    S.ads.rx_shift = (uint8_t)((S.ads.rx_shift << 1) | (S.ads.din ? 1u : 0u));
    if (++S.ads.rx_bits == 8u) {
        S.ads.rx_bits = 0;
        ads_byte(S.ads.rx_shift);
    }
}

static void ads_sclk_falling(void) {
    if (S.ads.tx_active && S.ads.tx_bits_out < 8u * S.ads.tx_len) {
        unsigned n = S.ads.tx_bits_out;
        bool bit = (S.ads.tx_bytes[n / 8u] >> (7u - n % 8u)) & 1u;

        drive_pin(&S.ext_b, ADS_DOUT, bit);
        S.ads.tx_bits_out++;
    }
}

static void ads_pin_notify(avr_irq_t *irq, uint32_t value, void *param) {
    unsigned pin = (unsigned)(uintptr_t)param;
    bool level = value != 0;

    (void)irq;
    switch (pin) {
    case ADS_CS:
        if (level == S.ads.cs) {
            return;
        }
        S.ads.cs = level;
        S.ads.st = ADS_CMD;
        S.ads.rx_bits = 0;
        S.ads.tx_active = false;
        break;
    case ADS_SCLK:
        if (level == S.ads.sclk) {
            return;
        }
        S.ads.sclk = level;
        if (!S.ads.cs) {
            if (level) {
                ads_sclk_rising();
            } else {
                ads_sclk_falling();
            }
        }
        break;
    case ADS_DIN:
        S.ads.din = level;
        break;
    default:
        break;
    }
}

static void ads_attach(void) {
    S.ads.cs = true;
    S.ads.sclk = true;
    S.ads.din = false;
    S.ads.st = ADS_CMD;
    S.ads.rx_bits = 0;
    S.ads.tx_active = false;
    for (unsigned pin = ADS_CS; pin <= ADS_DIN; pin++) {
        avr_irq_register_notify(pin_irq('B', pin), ads_pin_notify, (void *)(uintptr_t)pin);
    }
    S.ext_b.port = 'B';
    drive_pin(&S.ext_b, ADS_DOUT, false);
    ads_set_drdy(false);
}

/* ------------------------------------------------------------------------------------------------ */
/* Display controller model                                                                         */

#define VFD_CLK 0u
#define VFD_DIN 1u
#define VFD_DOUT 2u
#define VFD_STB_SECOND 3u
#define VFD_STB_MAIN 4u
#define VFD_CMD_READ_KEYS 0x42u /* boot.c:61 */

#define VFD_CMD_ADDRESS 0xC0u  /* vfd.c:46 */
#define VFD_DISPLAY_BASE 0x88u /* vfd.c:43: display-control command 0b10xx1nnn, nnn = dimming code */

/* Applies one completed frame to the display-memory model and the display-control log. A data write is the
 * frame 0x40, then a frame that starts with 0xC0 | address and carries the data bytes (vfd.c:107-121), the
 * address counting up after each byte. A display-control command is a frame of one byte. */
static void vfd_model_frame(const vfd_frame_t *f) {
    S.vfd.seq++;
    if (f->n >= 2u && (f->b[0] & 0xC0u) == VFD_CMD_ADDRESS) {
        unsigned addr = f->b[0] & 0x3Fu;

        for (unsigned k = 1; k < f->n && k < VFD_FRAME_BYTES; k++, addr++) {
            if (addr < VFD_RAM_BYTES) {
                S.vfd.ram.b[f->dev][addr] = f->b[k];
            }
        }
    } else if (f->n == 1u && (f->b[0] & 0xF8u) == VFD_DISPLAY_BASE) {
        if (S.vfd.nctl < VFD_CTL_MAX) {
            S.vfd.ctl[S.vfd.nctl++] = (vfd_ctl_t){.seq = S.vfd.seq, .dev = f->dev, .byte = f->b[0]};
        } else {
            S.vfd.ctl_overflow = true;
        }
    }
}

static void vfd_frame_end(void) {
    if (S.vfd.active < 0) {
        return;
    }
    if (S.vfd.cur.n > 0) {
        vfd_model_frame(&S.vfd.cur);
        if (S.vfd.nframes < VFD_FRAMES_MAX) {
            S.vfd.frames[S.vfd.nframes++] = S.vfd.cur;
        }
    }
    if (S.vfd.reading_keys) {
        S.vfd.key_reads++;
    }
    S.vfd.active = -1;
    S.vfd.reading_keys = false;
    drive_pin(&S.ext_c, VFD_DOUT, false);
}

static void vfd_pin_notify(avr_irq_t *irq, uint32_t value, void *param) {
    unsigned pin = (unsigned)(uintptr_t)param;
    bool level = value != 0;

    (void)irq;
    switch (pin) {
    case VFD_DIN:
        S.vfd.din = level;
        break;
    case VFD_CLK:
        if (level == S.vfd.clk) {
            return;
        }
        S.vfd.clk = level;
        if (S.vfd.active < 0) {
            return;
        }
        if (level) {
            if (S.vfd.reading_keys) {
                return;
            }
            S.vfd.shift = (uint8_t)((S.vfd.shift >> 1) | (S.vfd.din ? 0x80u : 0u));
            if (++S.vfd.bits == 8u) {
                S.vfd.bits = 0;
                if (S.vfd.cur.n < VFD_FRAME_BYTES) {
                    S.vfd.cur.b[S.vfd.cur.n] = S.vfd.shift;
                }
                S.vfd.cur.n++;
                if (S.vfd.cur.n == 1 && S.vfd.shift == VFD_CMD_READ_KEYS && S.vfd.active == 0) {
                    S.vfd.reading_keys = true;
                    S.vfd.nfall = 0;
                }
            }
        } else if (S.vfd.reading_keys) {
            unsigned idx = S.vfd.nfall / 8u;
            unsigned bit = S.vfd.nfall % 8u;
            bool v = idx < 3u && ((S.keys[idx] >> bit) & 1u);

            drive_pin(&S.ext_c, VFD_DOUT, v);
            S.vfd.nfall++;
        }
        break;
    case VFD_STB_MAIN:
    case VFD_STB_SECOND: {
        unsigned dev = (pin == VFD_STB_MAIN) ? 0u : 1u;

        if (level == S.vfd.stb[dev]) {
            return;
        }
        S.vfd.stb[dev] = level;
        if (!level) {
            S.vfd.active = (int)dev;
            S.vfd.bits = 0;
            S.vfd.shift = 0;
            memset(&S.vfd.cur, 0, sizeof S.vfd.cur);
            S.vfd.cur.dev = (uint8_t)dev;
            S.vfd.reading_keys = false;
        } else if (S.vfd.active == (int)dev) {
            vfd_frame_end();
        }
        break;
    }
    default:
        break;
    }
}

static void vfd_attach(void) {
    S.vfd.clk = true;
    S.vfd.stb[0] = true;
    S.vfd.stb[1] = true;
    S.vfd.active = -1;
    for (unsigned pin = VFD_CLK; pin <= VFD_STB_MAIN; pin++) {
        if (pin == VFD_DOUT) {
            continue;
        }
        avr_irq_register_notify(pin_irq('C', pin), vfd_pin_notify, (void *)(uintptr_t)pin);
    }
    S.ext_c.port = 'C';
    drive_pin(&S.ext_c, VFD_DOUT, false);
}

static unsigned vfd_count_cmd(unsigned dev, uint8_t cmd) {
    unsigned n = 0;

    for (size_t i = 0; i < S.vfd.nframes; i++) {
        if (S.vfd.frames[i].dev == dev && S.vfd.frames[i].n >= 1 && S.vfd.frames[i].b[0] == cmd) {
            n++;
        }
    }
    return n;
}

/* ------------------------------------------------------------------------------------------------ */
/* UART1                                                                                            */

static void uart_out_notify(avr_irq_t *irq, uint32_t value, void *param) {
    (void)irq;
    (void)param;
    if (S.rx_len < RX_MAX) {
        S.rx_buf[S.rx_len] = (uint8_t)value;
        S.rx_time[S.rx_len] = now();
        S.rx_len++;
    } else {
        S.rx_overflow = true;
    }
}

/* One received byte occupies this many CPU cycles on simavr's UART: avr_uart.c:avr_uart_baud_write(). */
static uint64_t uart_byte_cycles(void) {
    static const unsigned databits[8] = {5, 6, 7, 8, 8, 8, 8, 9};
    const uint8_t *d = S.avr->data;
    unsigned ubrr = (unsigned)d[ADDR_UBRR1L] | ((unsigned)(d[ADDR_UBRR1H] & 0x0Fu) << 8);
    bool u2x = (d[ADDR_UCSR1A] & 0x02u) != 0;
    unsigned ucsz = ((d[ADDR_UCSR1C] >> 1) & 3u) | (unsigned)(((d[ADDR_UCSR1B] >> 2) & 1u) << 2);
    unsigned stop = 1u + ((d[ADDR_UCSR1C] >> 3) & 1u);
    unsigned word = 1u + databits[ucsz] + 1u + stop;
    uint64_t per_bit = (uint64_t)(ubrr + 1u) * 8u * (u2x ? 1u : 2u);

    return per_bit * word;
}

static bool uart_rx_enabled(void) { return (S.avr->data[ADDR_UCSR1B] & 0x10u) != 0; }

static void host_queue(const uint8_t *bytes, size_t n) {
    for (size_t i = 0; i < n; i++) {
        S.tx_q[S.tx_tail++ % TX_Q_MAX] = bytes[i];
    }
}

static void host_queue_str(const char *s) { host_queue((const uint8_t *)s, strlen(s)); }

static bool host_queue_empty(void) { return S.tx_head == S.tx_tail; }

static void host_feed(void) {
    if (S.tx_head == S.tx_tail || !uart_rx_enabled()) {
        return;
    }
    uint64_t t = now();

    if (t < S.next_feed) {
        return;
    }
    avr_raise_irq(S.uart_in, S.tx_q[S.tx_head++ % TX_Q_MAX]);
    S.last_fed = t;
    S.next_feed = t + uart_byte_cycles();
}

/* ------------------------------------------------------------------------------------------------ */
/* Image loading                                                                                    */

/* Copies the flash chunks of an ELF into img with the same rule as simavr's do_chunk() (sim_elf.c:99-128):
 * p_filesz bytes of data, then zero fill up to p_memsz. */
static bool load_image(const char *path, uint8_t *img, uint32_t *lo, uint32_t *hi) {
    elf_firmware_t fw;

    memset(&fw, 0, sizeof fw);
    memset(img, 0xFF, FLASH_BYTES);
    if (elf_read_firmware(path, &fw) != 0) {
        fprintf(stderr, "cannot read ELF %s\n", path);
        return false;
    }
    *lo = 0xFFFFFFFFu;
    *hi = 0;
    for (fw_chunk_t *c = fw.chunks; c != NULL; c = c->next) {
        if (c->addr >= AVR_SEGMENT_OFFSET_DATA) {
            continue;
        }
        uint32_t csize = c->size < c->fill_size ? c->size : c->fill_size;
        uint32_t total = c->fill_size > csize ? c->fill_size : csize;

        if (c->addr + total > FLASH_BYTES) {
            fprintf(stderr, "chunk beyond flash in %s\n", path);
            return false;
        }
        memcpy(img + c->addr, c->data, csize);
        memset(img + c->addr + csize, 0, total - csize);
        if (c->addr < *lo) {
            *lo = c->addr;
        }
        if (c->addr + total > *hi) {
            *hi = c->addr + total;
        }
    }
    return *hi != 0;
}

static bool load_file(const char *path, uint8_t *dst, size_t size) {
    FILE *f = fopen(path, "rb");

    if (f == NULL) {
        fprintf(stderr, "cannot open %s: %s\n", path, strerror(errno));
        return false;
    }
    size_t n = fread(dst, 1, size, f);
    fclose(f);
    return n == size;
}

/* ------------------------------------------------------------------------------------------------ */
/* Session set-up and run loop                                                                      */

typedef struct {
    uint32_t reset_pc;
    bool load_boot;
    bool load_app;
    uint8_t app_fill; /* flash filler for the application area when load_app is false */
    uint8_t key0;
} open_cfg_t;

static void sleep_none(avr_t *avr, avr_cycle_count_t how_long) {
    (void)avr;
    (void)how_long;
}

static void session_close(void) {
    if (S.avr != NULL) {
        avr_terminate(S.avr);
        S.avr = NULL;
    }
    free(S.rx_buf);
    free(S.rx_time);
    free(S.tx_q);
    free(S.vfd.frames);
    memset(&S, 0, sizeof S);
}

/*
 * simavr defect worked around here: the atmega128 core declares the watchdog with interrupt vector 0
 * (cores/sim_mega128.c:80, AVR_WATCHDOG_DECLARE_128(WDTCR, _VECTOR(0))), so avr_register_vector() returns
 * before it initialises the watchdog's IRQ array (sim_interrupts.c:64). On the second avr_reset() the watchdog
 * reset handler registers its notify again, finds the hook already present and dereferences the IRQ's
 * NULL pool (sim_irq.c:162), which crashes simavr on every watchdog reset. Giving the IRQ array a valid pool
 * removes the crash and changes nothing else.
 */
static void patch_watchdog_irq_pool(void) {
    static const char *names[2] = {"wdt.pending", "wdt.running"};

    for (avr_io_t *io = S.avr->io_port; io != NULL; io = io->next) {
        if (io->kind != NULL && strcmp(io->kind, "watchdog") == 0) {
            avr_watchdog_t *w = (avr_watchdog_t *)io;

            if (w->watchdog.irq[0].pool == NULL) {
                avr_init_irq(&S.avr->irq_pool, w->watchdog.irq, 0, 2, names);
            }
        }
    }
}

static bool session_open(const open_cfg_t *cfg) {
    session_close();
    g_rdata_n = 0;
    S.rx_buf = calloc(RX_MAX, 1);
    S.rx_time = calloc(RX_MAX, sizeof(uint64_t));
    S.tx_q = calloc(TX_Q_MAX, 1);
    S.vfd.frames = calloc(VFD_FRAMES_MAX, sizeof(vfd_frame_t));
    if (!S.rx_buf || !S.rx_time || !S.tx_q || !S.vfd.frames) {
        fprintf(stderr, "out of memory\n");
        return false;
    }
    memcpy(S.ee_initial, g_ee_file, EEPROM_BYTES);
    S.echo = S.ee_initial[EE_SET_ECHO] != 0;
    S.keys[0] = cfg->key0;

    S.avr = avr_make_mcu_by_name("atmega128");
    if (S.avr == NULL) {
        fprintf(stderr, "atmega128 core is not registered\n");
        return false;
    }
    S.avr->frequency = (uint32_t)F_CPU_HZ;
    avr_init(S.avr);
    patch_watchdog_irq_pool();
    S.avr->sleep = sleep_none;

    if (cfg->load_app) {
        memcpy(S.avr->flash, g_app_img, g_app_len);
    } else {
        memset(S.avr->flash, cfg->app_fill, BOOT_START);
    }
    if (cfg->load_boot) {
        memcpy(S.avr->flash + g_boot_lo, g_boot_img + g_boot_lo, g_boot_hi - g_boot_lo);
    }
    S.avr->codeend = S.avr->flashend;

    uint32_t pc = g_reset_pc_override >= 0 ? (uint32_t)g_reset_pc_override : cfg->reset_pc;
    S.avr->reset_pc = pc;
    S.avr->pc = pc;

    avr_eeprom_desc_t desc = {.ee = S.ee_initial, .offset = 0, .size = EEPROM_BYTES};
    if (avr_ioctl(S.avr, AVR_IOCTL_EEPROM_SET, &desc) < 0) {
        fprintf(stderr, "EEPROM load failed\n");
        return false;
    }

    uint32_t flags = 0;
    avr_ioctl(S.avr, AVR_IOCTL_UART_GET_FLAGS('1'), &flags);
    flags &= ~(uint32_t)AVR_UART_FLAG_STDIO;
    avr_ioctl(S.avr, AVR_IOCTL_UART_SET_FLAGS('1'), &flags);
    avr_irq_register_notify(avr_io_getirq(S.avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUTPUT), uart_out_notify,
                            NULL);
    S.uart_in = avr_io_getirq(S.avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_INPUT);

    S.ads.mode = ADS_FREE_RUN;
    S.ads.conv_delay_us = 20000;
    S.ads.cal_delay_us = 20000 * 14;
    S.ads.code = 0x100000;
    ads_attach();
    vfd_attach();
    ads_set_mode(ADS_FREE_RUN);
    return true;
}

static void on_avr_reset(void) {
    S.ads.st = ADS_CMD;
    S.ads.rx_bits = 0;
    S.ads.tx_active = false;
    S.ads.cs = true;
    S.ads.sclk = true;
    S.vfd.active = -1;
    S.vfd.reading_keys = false;
    S.vfd.clk = true;
    S.vfd.stb[0] = true;
    S.vfd.stb[1] = true;
    redrive_pins();
    ads_set_mode(S.ads.mode);
}

/* Executes one instruction (plus timers). Returns false once the core is no longer running. */
static bool step(void) {
    host_feed();
    uint64_t before = S.avr->cycle;
    int st = avr_run(S.avr);

    if (st == cpu_Done && (S.avr->data[ADDR_WDTCR] & WDTCR_WDE) != 0) {
        /* simavr ends the run at a self-jump with interrupts off (sim_avr.c:431, gracefully_quit). With the
         * watchdog enabled the real chip keeps running that loop until the watchdog resets it, which is how
         * the firmware's *RST works (thid.c:321-327). The core is resumed so the watchdog timer can fire. */
        S.avr->state = cpu_Running;
        st = cpu_Running;
    }
    S.last_state = st;
    if (S.avr->cycle < before) {
        S.base_cycles += before;
        S.resets++;
        on_avr_reset();
    }
    uint32_t pc = S.avr->pc;

    if (pc < APP_REGION_END) {
        S.app_steps++;
    } else if (pc >= BOOT_START) {
        S.boot_steps++;
    } else {
        S.stray_steps++;
    }
    return st == cpu_Running || st == cpu_Sleeping;
}

typedef enum { RUN_OK, RUN_TIMEOUT, RUN_STOPPED } run_status_t;
typedef bool (*pred_fn)(void *ctx);

static run_status_t run_until(pred_fn pred, void *ctx, uint64_t budget) {
    uint64_t end = now() + budget;

    for (;;) {
        if (pred != NULL && pred(ctx)) {
            return RUN_OK;
        }
        if (now() >= end) {
            return pred == NULL ? RUN_OK : RUN_TIMEOUT;
        }
        if (!step()) {
            return RUN_STOPPED;
        }
    }
}

static const char *state_name(int st) {
    switch (st) {
    case cpu_Limbo:
        return "Limbo";
    case cpu_Stopped:
        return "Stopped";
    case cpu_Running:
        return "Running";
    case cpu_Sleeping:
        return "Sleeping";
    case cpu_Done:
        return "Done";
    case cpu_Crashed:
        return "Crashed";
    default:
        return "other";
    }
}

static const char *stopped_text(void) {
    static char buf[96];

    snprintf(buf, sizeof buf, "core stopped: state %s, pc 0x%05X", state_name(S.last_state), S.avr->pc);
    return buf;
}

/* ---- predicates and waits ---- */

static bool pred_queue_empty(void *ctx) {
    (void)ctx;
    return host_queue_empty();
}

typedef struct {
    const uint8_t *needle;
    size_t nlen;
    size_t from;
    size_t scanned;
    size_t end;
} needle_ctx_t;

static bool pred_needle(void *c) {
    needle_ctx_t *n = c;

    if (S.rx_len == n->scanned) {
        return false;
    }
    size_t start = n->scanned > n->from + n->nlen ? n->scanned - n->nlen + 1 : n->from;

    n->scanned = S.rx_len;
    if (S.rx_len < start + n->nlen) {
        return false;
    }
    uint8_t *hit = memmem(S.rx_buf + start, S.rx_len - start, n->needle, n->nlen);

    if (hit == NULL) {
        return false;
    }
    n->end = (size_t)(hit - S.rx_buf) + n->nlen;
    return true;
}

/* Waits until the byte string occurs in the received stream at or after from. */
static run_status_t wait_rx(const void *needle, size_t nlen, size_t from, uint64_t budget, size_t *end_pos) {
    needle_ctx_t n = {.needle = needle, .nlen = nlen, .from = from, .scanned = from};
    run_status_t r = run_until(pred_needle, &n, budget);

    if (r == RUN_OK && end_pos != NULL) {
        *end_pos = n.end;
    }
    return r;
}

static run_status_t wait_rx_str(const char *s, size_t from, uint64_t budget, size_t *end_pos) {
    return wait_rx(s, strlen(s), from, budget, end_pos);
}

typedef struct {
    size_t pos;
} pos_ctx_t;

static bool pred_rx_beyond(void *c) { return S.rx_len > ((pos_ctx_t *)c)->pos; }

/* Next received byte at index pos. Returns -1 on timeout, -2 when the core stopped. */
static int wait_rx_byte(size_t pos, uint64_t budget) {
    pos_ctx_t c = {pos};
    run_status_t r = run_until(pred_rx_beyond, &c, budget);

    if (r == RUN_OK) {
        return S.rx_buf[pos];
    }
    return r == RUN_STOPPED ? -2 : -1;
}

static run_status_t run_for(uint64_t cycles) { return run_until(NULL, NULL, cycles); }

typedef struct {
    unsigned dev;
    uint8_t cmd;
    unsigned min_count;
} vfd_cmd_ctx_t;

static bool pred_vfd_cmd(void *c) {
    vfd_cmd_ctx_t *v = c;

    return vfd_count_cmd(v->dev, v->cmd) >= v->min_count;
}

/* The application's uart1_init() enables the receive interrupt (uart1.c:384); the boot loader never does
 * (boot.c:106). Bytes sent before that are discarded when the application reconfigures the UART. */
static bool pred_app_uart_ready(void *c) {
    (void)c;
    return (S.avr->data[ADDR_UCSR1B] & UCSR1B_RXCIE1) != 0;
}

static bool pred_resets(void *c) { return S.resets >= *(unsigned *)c; }

static bool pred_rdata(void *c) { return S.ads.n_rdata >= *(unsigned *)c; }

/* ------------------------------------------------------------------------------------------------ */
/* CRC and XMODEM sender                                                                            */

/* CRC-16/XMODEM, polynomial 0x1021, initial value 0 (boot.c:301-315). */
static uint16_t crc16_xmodem(const uint8_t *data, size_t len) {
    uint16_t crc = 0;

    while (len-- > 0) {
        crc ^= (uint16_t)((uint16_t)*data++ << 8);
        for (unsigned bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Frame layout: SOH, block, ~block, 128 bytes, CRC high, CRC low (boot.c:331-361). If flip_byte is not
 * negative, that payload byte is inverted after the CRC was computed, so the frame carries a wrong CRC. */
static void xm_queue_frame(uint8_t blk, const uint8_t *payload, int flip_byte) {
    uint8_t f[3 + FRAME_BYTES + 2];
    uint16_t crc = crc16_xmodem(payload, FRAME_BYTES);

    f[0] = XM_SOH;
    f[1] = blk;
    f[2] = (uint8_t)~blk;
    memcpy(f + 3, payload, FRAME_BYTES);
    f[3 + FRAME_BYTES] = (uint8_t)(crc >> 8);
    f[4 + FRAME_BYTES] = (uint8_t)crc;
    if (flip_byte >= 0) {
        f[3 + flip_byte] = (uint8_t)~f[3 + flip_byte];
    }
    host_queue(f, sizeof f);
}

/* Reads the loader's answer to a frame: skips 'C' requests that were already in flight. Returns the byte,
 * -1 on timeout, -2 when the core stopped. *pos is advanced past it. */
static int xm_response(size_t *pos, uint64_t budget) {
    for (;;) {
        int b = wait_rx_byte(*pos, budget);

        if (b < 0) {
            return b;
        }
        (*pos)++;
        if (b != 'C') {
            return b;
        }
    }
}

typedef struct {
    bool ok;
    unsigned frames_acked;
    int bad_response; /* the byte that was not ACK, or -1 timeout, -2 core stopped */
    unsigned naks;
} xm_result_t;

/* Sends one block and waits for its ACK, retransmitting after a NAK up to three times. Returns false, with
 * r->bad_response set, when the loader answers anything else. */
static bool xm_send_block(uint8_t blk, const uint8_t *payload, size_t *pos, xm_result_t *r) {
    for (unsigned attempt = 0;; attempt++) {
        xm_queue_frame(blk, payload, -1);
        int resp = xm_response(pos, SEC(3));

        if (resp == XM_ACK) {
            r->frames_acked++;
            return true;
        }
        if (resp == XM_NAK && attempt < 3) {
            r->naks++;
            continue;
        }
        r->bad_response = resp;
        return false;
    }
}

/* Sends len bytes of img as XMODEM-CRC data blocks starting at block 1, the last block padded with 0xFF.
 * No trailer and no EOT. r.ok is true when every block was acknowledged. */
static xm_result_t xm_send_data(const uint8_t *img, size_t len, size_t *pos) {
    xm_result_t r = {.ok = false, .frames_acked = 0, .bad_response = 0, .naks = 0};
    size_t frames = (len + FRAME_BYTES - 1) / FRAME_BYTES;

    for (size_t i = 0; i < frames; i++) {
        uint8_t payload[FRAME_BYTES];
        size_t n = len - i * FRAME_BYTES;

        n = n > FRAME_BYTES ? FRAME_BYTES : n;
        memset(payload, XM_FILL, sizeof payload);
        memcpy(payload, img + i * FRAME_BYTES, n);
        if (!xm_send_block((uint8_t)(i + 1), payload, pos, &r)) {
            return r;
        }
    }
    r.ok = true;
    return r;
}

/* The trailer block for an application of `length` bytes whose CRC16-XMODEM is `crc`. */
static void xm_trailer(uint8_t t[FRAME_BYTES], uint32_t length, uint16_t crc) {
    memset(t, 0xFF, FRAME_BYTES);
    t[0] = TRAILER_MAGIC_0;
    t[1] = TRAILER_MAGIC_1;
    t[2] = (uint8_t)length;
    t[3] = (uint8_t)(length >> 8);
    t[4] = (uint8_t)(length >> 16);
    t[5] = (uint8_t)(length >> 24);
    t[6] = (uint8_t)(crc >> 8);
    t[7] = (uint8_t)crc;
}

/* The wire image: len bytes of img as data blocks, then the trailer block carrying the given length and CRC,
 * then EOT. A correct sender passes the true length and CRC of img[0, len); the tests of a damaged transfer
 * pass something else. */
static xm_result_t xm_send_stream(const uint8_t *img, size_t len, uint32_t trailer_len, uint16_t trailer_crc,
                                  size_t *pos) {
    xm_result_t r = xm_send_data(img, len, pos);

    if (!r.ok) {
        return r;
    }
    uint8_t trailer[FRAME_BYTES];
    size_t frames = (len + FRAME_BYTES - 1) / FRAME_BYTES;

    r.ok = false;
    xm_trailer(trailer, trailer_len, trailer_crc);
    if (!xm_send_block((uint8_t)(frames + 1), trailer, pos, &r)) {
        return r;
    }
    uint8_t eot = XM_EOT;

    host_queue(&eot, 1);
    int resp = xm_response(pos, SEC(3));

    if (resp != XM_ACK) {
        r.bad_response = resp;
        return r;
    }
    r.ok = true;
    return r;
}

/* A correct transfer of img[0, len): data blocks, a trailer that tells the truth, EOT. */
static xm_result_t xm_send_image(const uint8_t *img, size_t len, size_t *pos) {
    return xm_send_stream(img, len, (uint32_t)len, crc16_xmodem(img, len), pos);
}

/* ------------------------------------------------------------------------------------------------ */
/* Application host-link helpers                                                                    */

typedef struct {
    char text[512];
    uint64_t first_byte_latency; /* last command byte reaching the AVR to first reply byte */
} reply_t;

static const char *run_text(run_status_t r) {
    return r == RUN_STOPPED ? stopped_text() : "timeout";
}

typedef struct {
    size_t from;
} nl_ctx_t;

static bool pred_newline(void *c) {
    nl_ctx_t *n = c;

    return S.rx_len > n->from && memchr(S.rx_buf + n->from, '\n', S.rx_len - n->from) != NULL;
}

/* Sends one command line and reads the one reply line that follows the echo. */
static run_status_t host_query(const char *cmd, reply_t *r, uint64_t budget) {
    size_t from = S.rx_len;
    size_t echo = S.echo ? strlen(cmd) + 1 : 0;

    host_queue_str(cmd);
    host_queue_str("\n");
    run_status_t st = run_until(pred_queue_empty, NULL, budget);

    if (st != RUN_OK) {
        return st;
    }
    uint64_t arrive = S.last_fed + uart_byte_cycles();
    nl_ctx_t n = {from + echo};

    st = run_until(pred_newline, &n, budget);
    if (st != RUN_OK) {
        return st;
    }
    const uint8_t *nl = memchr(S.rx_buf + from + echo, '\n', S.rx_len - (from + echo));
    size_t len = (size_t)(nl - (S.rx_buf + from + echo));

    len = len < sizeof r->text - 1 ? len : sizeof r->text - 1;
    memcpy(r->text, S.rx_buf + from + echo, len);
    r->text[len] = '\0';
    r->first_byte_latency = S.rx_time[from + echo] > arrive ? S.rx_time[from + echo] - arrive : 0;
    return RUN_OK;
}

/* Sends a command that has no reply and waits until its echo is complete. */
static run_status_t host_send(const char *cmd, uint64_t budget) {
    size_t from = S.rx_len;
    size_t echo = S.echo ? strlen(cmd) + 1 : 0;

    host_queue_str(cmd);
    host_queue_str("\n");
    run_status_t st = run_until(pred_queue_empty, NULL, budget);

    if (st != RUN_OK) {
        return st;
    }
    if (echo > 0) {
        pos_ctx_t c = {from + echo - 1};

        st = run_until(pred_rx_beyond, &c, budget);
    }
    return st;
}

static void app_open(void) {
    open_cfg_t cfg = {.reset_pc = 0, .load_boot = false, .load_app = true, .app_fill = 0xFF, .key0 = 0};

    if (!session_open(&cfg)) {
        exit(2);
    }
}

/* ------------------------------------------------------------------------------------------------ */
/* Flash and EEPROM checks                                                                          */

static bool flash_equals(uint32_t addr, const uint8_t *expect, size_t len, uint32_t *first_diff) {
    for (size_t i = 0; i < len; i++) {
        if (S.avr->flash[addr + i] != expect[i]) {
            *first_diff = (uint32_t)(addr + i);
            return false;
        }
    }
    return true;
}

static bool flash_all(uint32_t addr, size_t len, uint8_t value, uint32_t *first_diff) {
    for (size_t i = 0; i < len; i++) {
        if (S.avr->flash[addr + i] != value) {
            *first_diff = (uint32_t)(addr + i);
            return false;
        }
    }
    return true;
}

/* The first 'n' bytes of the application image used as a test image. */
static const uint8_t *app_image(void) { return g_app_img; }

/* ------------------------------------------------------------------------------------------------ */
/* Boot loader sessions                                                                             */

static void boot_open(bool real_app, uint8_t key0) {
    open_cfg_t cfg = {.reset_pc = BOOT_START, .load_boot = true, .load_app = real_app, .app_fill = 0x5A, .key0 = key0};

    if (!session_open(&cfg)) {
        exit(2);
    }
}

/* Banner, then ENTER, then the first 'C'. Leaves the loader waiting for a block. *pos is the receive index
 * after that 'C'. */
static result_t boot_to_download(size_t *pos) {
    run_status_t r = wait_rx_str("Press ENTER to download", 0, SEC(3), pos);

    REQUIRE(r == RUN_OK, "banner not printed: %s", run_text(r));
    S.keys[0] = KEY_ENTER;
    r = wait_rx_str("C", *pos, SEC(6), pos);
    REQUIRE(r == RUN_OK, "no 'C' request after ENTER: %s", run_text(r));
    return T_PASS;
}

/* ------------------------------------------------------------------------------------------------ */
/* Tests                                                                                            */

/* 1. No key at reset: the application starts and the loader prints no banner. */
static result_t test_01(void) {
    size_t pos = 0;

    boot_open(true, 0);
    run_status_t r = wait_rx_str("OK\r\n", 0, SEC(2), &pos);

    REQUIRE(r == RUN_OK, "loader did not hand over (no \"OK\" line): %s", run_text(r));
    vfd_cmd_ctx_t v = {.dev = 0, .cmd = 0x3A, .min_count = 1};

    r = run_until(pred_vfd_cmd, &v, SEC(1));
    REQUIRE(r == RUN_OK, "application did not start (no display mode command 0x3A): %s", run_text(r));
    REQUIRE(S.app_steps > 0, "pc never entered the application region (< 0x%X)", APP_REGION_END);
    REQUIRE(memmem(S.rx_buf, S.rx_len, "Press ENTER", 11) == NULL, "loader printed its banner with no key held");
    info("handover after %.3f s, app steps %" PRIu64, seconds(S.rx_time[pos - 1]), S.app_steps);
    return T_PASS;
}

/* Common tail of tests 2, 4, 7: "OK" line, flash contents, loader area unchanged. */
static uint8_t g_boot_copy[FLASH_BYTES - BOOT_START];

static void boot_snapshot(void) { memcpy(g_boot_copy, S.avr->flash + BOOT_START, sizeof g_boot_copy); }

static bool boot_unchanged(uint32_t *diff) {
    return flash_equals(BOOT_START, g_boot_copy, sizeof g_boot_copy, diff);
}

/* 2. Arrow keys + ENTER + the whole application image over XMODEM-CRC. */
static result_t test_02(void) {
    size_t pos;
    uint32_t diff;
    const size_t len = g_app_len;

    boot_open(false, KEY_ARROWS);
    boot_snapshot();
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        return t;
    }
    xm_result_t x = xm_send_image(app_image(), len, &pos);

    REQUIRE(x.ok, "transfer failed after %u acked frames, response %d (0x%02X)", x.frames_acked, x.bad_response,
            x.bad_response & 0xFF);
    run_status_t r = wait_rx_str("OK\r\n", pos, SEC(2), &pos);

    REQUIRE(r == RUN_OK, "no \"OK\" line after the transfer: %s", run_text(r));
    REQUIRE(flash_equals(0, app_image(), len, &diff), "flash differs from the image at 0x%05X (flash %02X, image %02X)",
            diff, S.avr->flash[diff], app_image()[diff]);
    size_t padded = (len + PAGE_BYTES - 1) / PAGE_BYTES * PAGE_BYTES;
    size_t frame_end = (len + FRAME_BYTES - 1) / FRAME_BYTES * FRAME_BYTES;

    for (size_t i = len; i < padded; i++) {
        /* The sender pads the last data frame with 0xFF and the loader pads the rest of the page with 0xFF
         * (boot.c:503-506); the trailer frame is never written. */
        REQUIRE(S.avr->flash[i] == 0xFFu, "tail byte 0x%05zX is %02X, expected FF%s", i, S.avr->flash[i],
                i < frame_end ? " (sender pad)" : " (loader pad)");
    }
    REQUIRE(boot_unchanged(&diff), "boot loader area changed at 0x%05X", diff);
    REQUIRE(flash_all((uint32_t)padded, BOOT_START - padded, 0x5A, &diff),
            "flash beyond the image changed at 0x%05X", diff);

    S.app_steps = 0;
    vfd_cmd_ctx_t v = {.dev = 0, .cmd = 0x3A, .min_count = 1};

    r = run_until(pred_vfd_cmd, &v, SEC(1));
    REQUIRE(r == RUN_OK, "the written application did not start: %s", run_text(r));
    REQUIRE(S.app_steps > 0, "pc never entered the application region");
    reply_t rep;

    r = run_until(pred_app_uart_ready, NULL, SEC(30));
    REQUIRE(r == RUN_OK, "the written application never enabled its receive interrupt: %s", run_text(r));
    r = host_query("*IDN?", &rep, SEC(60));
    REQUIRE(r == RUN_OK, "the written application does not answer *IDN?: %s", run_text(r));
    info("%u frames, %zu bytes written, application answers: %s", x.frames_acked, len, rep.text);
    return T_PASS;
}

/* Builds a test image: the first frames of the application (so the reset vector is valid), then a
 * deterministic pattern. */
static void make_image(uint8_t *img, size_t len) {
    for (size_t i = 0; i < len; i++) {
        img[i] = i < g_app_len ? g_app_img[i] : (uint8_t)(i % 251u);
    }
}

/* 3. An image that reaches the loader's own area is refused and the loader is unchanged. */
static result_t test_03(void) {
    size_t pos;
    uint32_t diff;
    const size_t len = BOOT_START + PAGE_BYTES; /* one page past the limit */
    uint8_t *img = malloc(len);

    boot_open(false, KEY_ARROWS);
    boot_snapshot();
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        free(img);
        return t;
    }
    make_image(img, len);
    xm_result_t x = xm_send_image(img, len, &pos);

    REQUIRE(!x.ok, "an image reaching 0x%05X was accepted", BOOT_START);
    /* The loader holds each frame back by one (boot.c:570-576), so a page is written when the frame AFTER its
     * second frame arrives. The page at BOOT_START holds data frames BOOT_START/128+1 and +2; the frame that
     * follows them is the trailer, which therefore triggers the refused write and is not acknowledged. */
    unsigned expect_acked = BOOT_START / FRAME_BYTES + 2;
    REQUIRE(x.bad_response == XM_CAN, "expected CAN after %u acked frames, got response %d", expect_acked,
            x.bad_response);
    REQUIRE(x.frames_acked == expect_acked, "frames acked %u, expected %u", x.frames_acked, expect_acked);
    run_status_t r = wait_rx_str("FAIL L", pos, SEC(1), &pos);

    REQUIRE(r == RUN_OK, "loader did not report the limit failure: %s", run_text(r));
    REQUIRE(boot_unchanged(&diff), "BOOT LOADER AREA CHANGED at 0x%05X (flash %02X, was %02X)", diff,
            S.avr->flash[diff], g_boot_copy[diff - BOOT_START]);
    REQUIRE(flash_equals(0, img, BOOT_START, &diff), "pages below the limit differ from the image at 0x%05X", diff);
    r = wait_rx_str("C", pos, SEC(3), &pos);
    REQUIRE(r == RUN_OK, "loader did not resume requesting a download: %s", run_text(r));
    REQUIRE(S.app_steps == 0, "the application region was executed");
    info("%u frames acked, refused at 0x%05X, boot area unchanged", x.frames_acked, BOOT_START);
    free(img);
    return T_PASS;
}

/* 4. A corrupted block is answered with NAK, then the retransmission succeeds. */
static result_t test_04(void) {
    size_t pos;
    uint32_t diff;
    const size_t len = 3 * FRAME_BYTES;
    uint8_t payload[FRAME_BYTES];

    boot_open(false, KEY_ARROWS);
    boot_snapshot();
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        return t;
    }
    xm_queue_frame(1, app_image(), -1);
    REQUIRE(xm_response(&pos, SEC(3)) == XM_ACK, "block 1 was not acknowledged");
    memcpy(payload, app_image() + FRAME_BYTES, FRAME_BYTES);
    xm_queue_frame(2, payload, 17);
    int resp = xm_response(&pos, SEC(3));

    REQUIRE(resp == XM_NAK, "corrupted block 2 answered with %d, expected NAK (0x15)", resp);
    xm_queue_frame(2, payload, -1);
    resp = xm_response(&pos, SEC(3));
    REQUIRE(resp == XM_ACK, "retransmitted block 2 answered with %d, expected ACK", resp);
    xm_queue_frame(3, app_image() + 2 * FRAME_BYTES, -1);
    REQUIRE(xm_response(&pos, SEC(3)) == XM_ACK, "block 3 was not acknowledged");
    xm_trailer(payload, (uint32_t)len, crc16_xmodem(app_image(), len));
    xm_queue_frame(4, payload, -1);
    REQUIRE(xm_response(&pos, SEC(3)) == XM_ACK, "the trailer block (block 4) was not acknowledged");
    uint8_t eot = XM_EOT;

    host_queue(&eot, 1);
    REQUIRE(xm_response(&pos, SEC(3)) == XM_ACK, "EOT was not acknowledged");
    run_status_t r = wait_rx_str("OK\r\n", pos, SEC(2), &pos);

    REQUIRE(r == RUN_OK, "no \"OK\" line: %s", run_text(r));
    REQUIRE(flash_equals(0, app_image(), len, &diff),
            "flash differs from the image at 0x%05X (flash %02X, image %02X)", diff, S.avr->flash[diff],
            app_image()[diff]);
    REQUIRE(boot_unchanged(&diff), "boot loader area changed at 0x%05X", diff);
    info("NAK for the corrupted block, retransmission accepted, 3 frames and a trailer sent, 3 frames in flash");
    return T_PASS;
}

/* 5. The host goes silent in the middle of a frame. */
static result_t test_05(void) {
    size_t pos;
    uint32_t diff;
    uint8_t partial[3 + 50];

    boot_open(false, KEY_ARROWS);
    boot_snapshot();
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        return t;
    }
    S.app_steps = 0;
    /* Five frames: with the one-frame hold-back (boot.c:570-576) the fifth is what completes the second page. */
    xm_result_t x = xm_send_data(app_image(), 5 * FRAME_BYTES, &pos);

    REQUIRE(x.ok, "the first five frames were not acknowledged (response %d)", x.bad_response);
    partial[0] = XM_SOH;
    partial[1] = 6;
    partial[2] = (uint8_t)~6;
    memcpy(partial + 3, app_image() + 5 * FRAME_BYTES, 50);
    host_queue(partial, sizeof partial);
    run_status_t r = run_until(pred_queue_empty, NULL, SEC(2));

    REQUIRE(r == RUN_OK, "partial frame not delivered: %s", run_text(r));
    uint64_t stall_start = now();

    r = wait_rx_str("FAIL T", pos, SEC(120), &pos);
    REQUIRE(r == RUN_OK, "the loader never gave up on the stalled transfer within 120 s: %s", run_text(r));
    double give_up = seconds(now() - stall_start);
    size_t first_c, second_c;

    r = wait_rx_str("C", pos, SEC(3), &first_c);
    REQUIRE(r == RUN_OK, "loader did not resume requesting a download after giving up: %s", run_text(r));
    r = wait_rx_str("C", first_c, SEC(3), &second_c);
    REQUIRE(r == RUN_OK, "no second 'C' request: %s", run_text(r));
    REQUIRE(S.app_steps == 0, "the application region was executed after the stall (%" PRIu64 " steps)",
            S.app_steps);
    REQUIRE(boot_unchanged(&diff), "boot loader area changed at 0x%05X", diff);
    REQUIRE(flash_equals(0, app_image(), 2 * PAGE_BYTES, &diff),
            "the two completed pages differ from the image at 0x%05X", diff);
    double c_interval = seconds(S.rx_time[second_c - 1] - S.rx_time[first_c - 1]);

    info("gave up %.2f s after the stall, 'C' spacing %.3f s (boot.c:68 states 1.36 s)", give_up, c_interval);
    info("two completed pages (512 bytes) stay in flash after the abort; nothing marks them invalid");

    /* THE POINT OF THIS TEST, and until now it was not tested at all.
     *
     * This block was labelled "observation only": it reset the part, waited two seconds, and `info()`d
     * whether the application had started. It asserted NOTHING, so the test passed while the firmware did
     * the one thing the test's own name says it must not -- and the fix would have been reported on the strength
     * of a suite containing this. It would not even have observed the fault: the banner path starts the
     * application after ENTER_WAIT_TICKS, about 4.0 s, and the wait here was 2.
     *
     * The dangerous case is a POWER CYCLE, not the failed transfer. After an aborted download the loader
     * sits there offering a retry, which is fine while the power stays on; the flash now holds the new
     * image's first pages and the old image's remainder, and the next power-up is what runs that mixture.
     * So: reset with no key held, let the full banner timeout elapse with margin, and require that the
     * application never executes a single instruction. */
    uint64_t cycles_before_reset = S.avr->cycle;

    avr_reset(S.avr);
    S.base_cycles += cycles_before_reset;
    on_avr_reset();
    S.keys[0] = 0;
    S.app_steps = 0;

    /* ENTER_WAIT_TICKS (200) at the 20.05 ms tick is about 4.0 s; ten seconds is comfortably past it. */
    r = run_for(SEC(10));
    REQUIRE(r == RUN_OK, "%s", stopped_text());
    info("10 s after a reset with no key held: %" PRIu64 " application step(s)", S.app_steps);
    REQUIRE(S.app_steps == 0,
            "the loader STARTED the partially written image after a power cycle (%" PRIu64 " steps) -- this is "
            "the brick case the transfer-in-progress flag exists to prevent",
            S.app_steps);
    return T_PASS;
}

/* 6. The first block carries no valid reset vector. */
static result_t test_06(void) {
    size_t pos;
    uint32_t diff;
    uint8_t payload[FRAME_BYTES];
    static const uint8_t bad_target[4] = {0x0C, 0x94, 0xFF, 0xFF}; /* JMP 0xFFFF word address: beyond 0xFC00 */
    static const uint8_t zero_target[4] = {0x0C, 0x94, 0x00, 0x00}; /* JMP 0 */

    boot_open(false, KEY_ARROWS);
    boot_snapshot();
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        return t;
    }
    S.app_steps = 0;
    for (unsigned variant = 0; variant < 3; variant++) {
        const char *what = variant == 0 ? "all 0xFF" : variant == 1 ? "JMP to 0xFFFF" : "JMP to 0";

        memset(payload, 0xFF, sizeof payload);
        if (variant == 1) {
            memcpy(payload, bad_target, 4);
        } else if (variant == 2) {
            memcpy(payload, zero_target, 4);
        }
        xm_queue_frame(1, payload, -1);
        run_status_t r = wait_rx_str("FAIL I", pos, SEC(3), &pos);

        REQUIRE(r == RUN_OK, "first block %s was not refused with FAIL I: %s", what, run_text(r));
        r = wait_rx_str("C", pos, SEC(3), &pos);
        REQUIRE(r == RUN_OK, "loader did not stay resident after refusing %s: %s", what, run_text(r));
    }
    REQUIRE(memchr(S.rx_buf, XM_ACK, S.rx_len) == NULL, "the loader acknowledged a block with an invalid vector");
    REQUIRE(flash_all(0, BOOT_START, 0x5A, &diff), "flash was written by a refused image at 0x%05X", diff);
    REQUIRE(boot_unchanged(&diff), "boot loader area changed at 0x%05X", diff);
    REQUIRE(S.app_steps == 0, "the application region was executed");
    info("three invalid first blocks refused, nothing written, loader resident");
    return T_PASS;
}

/* 7. A final partial page is padded with 0xFF. */
static result_t test_07(void) {
    size_t pos;
    uint32_t diff;
    const size_t len = 3 * FRAME_BYTES; /* one and a half pages */

    boot_open(false, KEY_ARROWS);
    boot_snapshot();
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        return t;
    }
    xm_result_t x = xm_send_image(app_image(), len, &pos);

    REQUIRE(x.ok, "transfer failed after %u acked frames, response %d", x.frames_acked, x.bad_response);
    run_status_t r = wait_rx_str("OK\r\n", pos, SEC(2), &pos);

    REQUIRE(r == RUN_OK, "no \"OK\" line: %s", run_text(r));
    REQUIRE(flash_equals(0, app_image(), len, &diff), "image bytes differ at 0x%05X", diff);
    REQUIRE(flash_all((uint32_t)len, 2 * PAGE_BYTES - len, 0xFF, &diff),
            "padding of the last page is not 0xFF at 0x%05X (flash %02X; the filler before the transfer was 5A)",
            diff, S.avr->flash[diff]);
    REQUIRE(flash_all(2 * PAGE_BYTES, 16, 0x5A, &diff), "the page after the image changed at 0x%05X", diff);
    info("bytes 0x%zX-0x%X of the last page read 0xFF", len, 2 * PAGE_BYTES - 1);
    return T_PASS;
}

/* Region names for the EEPROM differences the application writes by design. */
static const char *ee_region(unsigned off) {
    if (off >= EE_PWRUP_COUNT && off < EE_PWRUP_COUNT + 2) {
        return "EE_PWRUP_COUNT (eeprom_map.h:210)";
    }
    if (off >= EE_HOURS_BASE && off < EE_HOURS_END) {
        return "EE_HOURS slots (eeprom_map.h:219-222)";
    }
    if (off == EE_SERV_FAILS) {
        return "EE_SERV_FAILS (eeprom_map.h:156)";
    }
    return "not a documented runtime-written location";
}

/* ee_report_diff() lived here and was called by nobody: the one test that compares a whole EEPROM image,
 * T08, has its own loop with its own pass/fail rule. It is deleted rather than kept "in case", which is the
 * same rule the firmware's called-by-nobody gate applies to src/ -- six functions in this port compiled,
 * linked and were called by nobody. ee_region() survives
 * because T08's loop now uses it to name the region a stray write landed in. */

/* The bytes this firmware is allowed to change on a normal boot, and nothing else.
 *
 * A "byte-identical" criterion conflicts with the power-up
 * counter: a counter that counts power-ups writes EEPROM on power-up. Dropping the check would have lost
 * the thing it was really for -- catching a write nobody intended, which is how an existing meter's
 * calibration gets damaged. So the criterion is narrowed rather than relaxed: every vendor-defined byte is
 * still asserted unchanged, and the counter is asserted to move by exactly one.
 *
 * EE_PWRUP_COUNT is 0x0A90 and **two** bytes wide -- `eeprom_map.h:210` says u16, `powerup_count_bump()`
 * writes `ee_write_u16`, and all three readers (main.c, handlers.c, menu.c) read u16. The first version of
 * this test read four bytes, because the hour meter next door uses 4-byte slots, and duly reported the
 * count going from 0 to 4294901761 -- which is 0xFFFF0001, the two bytes the firmware wrote plus two it
 * never touches. The test was wrong, not the firmware. Worth leaving in the record: a test that asserts a
 * width nothing implements fails loudly and looks exactly like a firmware defect.
 *
 * It sits inside the span this firmware added at 0x0A82-0x0AC5. Nothing below 0x0A80 is ours. */
#define EE_PWRUP_COUNT_ADDR 0x0A90u
#define EE_PWRUP_COUNT_LEN 2u

static uint32_t ee_u16(const uint8_t *ee, unsigned addr) {
    return (uint32_t)ee[addr] | ((uint32_t)ee[addr + 1u] << 8);
}

/* 8. Boots to the measurement loop; nothing vendor-defined changes and the power-up count rises by one. */
static result_t test_08(void) {
    uint8_t after[EEPROM_BYTES];

    app_open();
    unsigned want = 5;
    run_status_t r = run_until(pred_rdata, &want, SEC(30));

    REQUIRE(r == RUN_OK, "the measurement loop never read 5 conversions in 30 s: %s", run_text(r));
    r = run_for(SEC(10));
    REQUIRE(r == RUN_OK, "%s", stopped_text());
    REQUIRE(S.resets == 0, "the core reset %u time(s) while running", S.resets);
    REQUIRE(vfd_count_cmd(0, 0x3A) >= 1, "the application never initialised the display");
    avr_eeprom_desc_t desc = {.ee = after, .offset = 0, .size = EEPROM_BYTES};

    REQUIRE(avr_ioctl(S.avr, AVR_IOCTL_EEPROM_GET, &desc) >= 0, "EEPROM read-back failed");
    info("measurement loop: %u RDATA reads, %u register writes, %u self-calibrations, %zu display frames, %u key scans",
         S.ads.n_rdata, S.ads.n_wreg, S.ads.n_cal, S.vfd.nframes, S.vfd.key_reads);
    REQUIRE(S.ads.n_unknown == 0, "the firmware sent %u converter command(s) the model does not know (last 0x%02X)",
            S.ads.n_unknown, S.ads.last_unknown);
    /* Every changed byte must be inside EE_PWRUP_COUNT. A diff anywhere else -- above all anywhere below
     * 0x0A80, which is the vendor's -- is the failure this test exists to catch. */
    unsigned diffs = 0;
    unsigned outside = 0;

    for (unsigned a = 0; a < EEPROM_BYTES; a++) {
        if (S.ee_initial[a] == after[a]) {
            continue;
        }
        diffs++;
        if (a < EE_PWRUP_COUNT_ADDR || a >= EE_PWRUP_COUNT_ADDR + EE_PWRUP_COUNT_LEN) {
            outside++;
            /* ee_region() names what the address belongs to, with its eeprom_map.h line. A bare address
             * in a failure message sends the reader to the map by hand; this says "EE_SERV_FAILS" and
             * where that is written down. */
            info("EEPROM 0x%04X: %02X -> %02X  OUTSIDE the power-up counter -- %s", a, S.ee_initial[a],
                 after[a], ee_region(a));
        }
    }
    REQUIRE(outside == 0, "%u byte(s) changed outside EE_PWRUP_COUNT (listed above)", outside);

    /* An erased counter reads as 0xFFFFFFFF and must be treated as zero, so the first boot of this
     * firmware on a meter that has never run it lands on exactly 1. That is worth
     * asserting rather than assuming. */
    uint32_t before_raw = ee_u16(S.ee_initial, EE_PWRUP_COUNT_ADDR);
    uint32_t before = (before_raw == 0xFFFFu) ? 0u : before_raw;
    uint32_t after_count = ee_u16(after, EE_PWRUP_COUNT_ADDR);

    info("power-up count: %lu -> %lu (raw before 0x%04lX), %u byte(s) changed in total",
         (unsigned long)before, (unsigned long)after_count, (unsigned long)before_raw, diffs);
    REQUIRE(after_count == before + 1u, "power-up count moved from %lu to %lu, expected exactly +1",
            (unsigned long)before, (unsigned long)after_count);
    return T_PASS;
}

static const char *model_prefix(char *buf, size_t size) {
    size_t n = 0;

    while (n < EE_ID_MODEL_BYTES && n + 1 < size && g_ee_file[EE_ID_MODEL + n] != 0) {
        buf[n] = (char)g_ee_file[EE_ID_MODEL + n];
        n++;
    }
    buf[n] = '\0';
    return buf;
}

static const char *serial_text(char *buf, size_t size) {
    size_t n = 0;

    while (n < EE_ID_SERIAL_BYTES && n + 1 < size && g_ee_file[EE_ID_SERIAL + n] != 0) {
        buf[n] = (char)g_ee_file[EE_ID_SERIAL + n];
        n++;
    }
    buf[n] = '\0';
    return buf;
}

/* 9. *IDN? returns a reply. */
static result_t test_09(void) {
    reply_t rep;
    char model[40], serial[40], expect_head[48];

    app_open();
    run_status_t r = host_query("*IDN?", &rep, SEC(40));

    REQUIRE(r == RUN_OK, "no reply to *IDN?: %s", run_text(r));
    snprintf(expect_head, sizeof expect_head, "%s,", model_prefix(model, sizeof model));
    serial_text(serial, sizeof serial);
    size_t rl = strlen(rep.text), sl = strlen(serial);

    REQUIRE(strncmp(rep.text, expect_head, strlen(expect_head)) == 0, "reply \"%s\" does not start with the EEPROM model \"%s\"",
            rep.text, expect_head);
    REQUIRE(rl > sl + 1 && strcmp(rep.text + rl - sl, serial) == 0 && rep.text[rl - sl - 1] == ',',
            "reply \"%s\" does not end with \",%s\"", rep.text, serial);
    REQUIRE(rl > strlen(expect_head) + sl + 2, "reply \"%s\" carries no version field", rep.text);
    info("*IDN? -> \"%s\"", rep.text);
    return T_PASS;
}

/* 10. :SERV:UNLOCK "6666" succeeds. */
static result_t test_10(void) {
    reply_t rep;

    app_open();
    run_status_t r = host_query(":SERV:STAT?", &rep, SEC(40));

    REQUIRE(r == RUN_OK, "no reply to :SERV:STAT?: %s", run_text(r));
    REQUIRE(strcmp(rep.text, "LOCKED") == 0, "state before unlock is \"%s\", expected LOCKED", rep.text);
    r = host_send(":SERV:UNLOCK \"6666\"", SEC(10));
    REQUIRE(r == RUN_OK, "unlock command not delivered: %s", run_text(r));
    r = host_query(":SERV:STAT?", &rep, SEC(10));
    REQUIRE(r == RUN_OK, "no reply to :SERV:STAT? after unlock: %s", run_text(r));
    REQUIRE(strcmp(rep.text, "UNLOCKED") == 0, "state after unlock is \"%s\", expected UNLOCKED", rep.text);
    r = host_query(":SYST:ERR?", &rep, SEC(10));
    REQUIRE(r == RUN_OK, "no reply to :SYST:ERR?: %s", run_text(r));
    REQUIRE(strcmp(rep.text, "NO ERROR!") == 0, "the unlock left an error: \"%s\"", rep.text);
    info("LOCKED -> UNLOCKED, error queue empty");
    return T_PASS;
}

/* 11. :SCAL:COEF? returns coefficients after the unlock. */
static result_t test_11(void) {
    reply_t rep;

    app_open();
    run_status_t r = host_send(":SERV:UNLOCK \"6666\"", SEC(40));

    REQUIRE(r == RUN_OK, "unlock command not delivered: %s", run_text(r));
    r = host_query(":SCAL:COEF?", &rep, SEC(20));
    REQUIRE(r == RUN_OK, "no reply to :SCAL:COEF?: %s", run_text(r));
    unsigned fields = 0;
    const char *p = rep.text;

    while (*p != '\0') {
        char *end;

        (void)strtod(p, &end);
        REQUIRE(end != p, "field %u of \"%s\" is not a number", fields + 1, rep.text);
        fields++;
        p = end;
        if (*p == ',') {
            p++;
        } else {
            REQUIRE(*p == '\0', "unexpected text \"%s\" in the reply", p);
        }
    }
    REQUIRE(fields == 6, "reply \"%s\" has %u numeric fields, expected 6 (f0,f1 for medium, fast, slow)", rep.text,
            fields);
    info(":SCAL:COEF? -> \"%s\"", rep.text);
    return T_PASS;
}

/* 12. An error does not stall the next command. */
static result_t test_12(void) {
    reply_t base, after, err;
    uint64_t t_error_done;

    app_open();
    run_status_t r = host_query("*IDN?", &base, SEC(40)); /* the first answer waits for the splash screen */

    REQUIRE(r == RUN_OK, "no reply to the first *IDN?: %s", run_text(r));
    info("first reply after power-up began %.0f ms after the command", seconds(base.first_byte_latency) * 1000.0);
    r = run_for(MS(200));
    REQUIRE(r == RUN_OK, "%s", stopped_text());
    r = host_query("*IDN?", &base, SEC(5)); /* the baseline: same command, no error before it */
    REQUIRE(r == RUN_OK, "no reply to the baseline *IDN?: %s", run_text(r));
    r = run_for(MS(200));
    REQUIRE(r == RUN_OK, "%s", stopped_text());
    /* The bad command and the next command are sent back to back, the case in which the original lost
     * the second command's bytes (uart1.c:9-17). */
    size_t from = S.rx_len;
    size_t echo = S.echo ? strlen("NONSENSE") + 1 + strlen("*IDN?") + 1 : 0;

    host_queue_str("NONSENSE\n");
    host_queue_str("*IDN?\n");
    r = run_until(pred_queue_empty, NULL, SEC(5));
    REQUIRE(r == RUN_OK, "burst not delivered: %s", run_text(r));
    t_error_done = S.last_fed + uart_byte_cycles();
    nl_ctx_t n = {from + echo};

    r = run_until(pred_newline, &n, SEC(5));
    REQUIRE(r == RUN_OK, "no reply to *IDN? sent right after an error (within 5 s): %s", run_text(r));
    const uint8_t *nl = memchr(S.rx_buf + from + echo, '\n', S.rx_len - (from + echo));
    size_t len = (size_t)(nl - (S.rx_buf + from + echo));

    memcpy(after.text, S.rx_buf + from + echo, len < sizeof after.text - 1 ? len : sizeof after.text - 1);
    after.text[len < sizeof after.text - 1 ? len : sizeof after.text - 1] = '\0';
    uint64_t latency = S.rx_time[from + echo] > t_error_done ? S.rx_time[from + echo] - t_error_done : 0;

    REQUIRE(strcmp(after.text, base.text) == 0, "reply after the error is \"%s\", baseline \"%s\"", after.text, base.text);
    REQUIRE(latency < MS(500), "the reply began %.3f s after the command; the original's lockout was 1 s", seconds(latency));
    REQUIRE(latency <= base.first_byte_latency + MS(100),
            "reply latency after an error %.3f s exceeds the baseline %.3f s by more than 100 ms", seconds(latency),
            seconds(base.first_byte_latency));
    r = host_query(":SYST:ERR?", &err, SEC(5));
    REQUIRE(r == RUN_OK, "no reply to :SYST:ERR?: %s", run_text(r));
    REQUIRE(strcmp(err.text, "NO ERROR!") != 0, "the bad command raised no error (queue reads \"%s\")", err.text);
    info("latency after error %.1f ms, baseline %.1f ms, error text \"%s\"", seconds(latency) * 1000.0,
         seconds(base.first_byte_latency) * 1000.0, err.text);
    return T_PASS;
}

/* 13. Empty lines are ignored. */
static result_t test_13(void) {
    reply_t rep;

    app_open();
    run_status_t r = host_query("*IDN?", &rep, SEC(40)); /* the firmware is up and listening */

    REQUIRE(r == RUN_OK, "no reply to *IDN?: %s", run_text(r));
    size_t from = S.rx_len;
    static const char blank[] = "\n\r\n\n\r";

    host_queue_str(blank);
    r = run_until(pred_queue_empty, NULL, SEC(5));
    REQUIRE(r == RUN_OK, "blank lines not delivered: %s", run_text(r));
    r = run_for(MS(500));
    REQUIRE(r == RUN_OK, "%s", stopped_text());
    size_t expect = S.echo ? strlen(blank) : 0;

    REQUIRE(S.rx_len - from == expect, "blank lines produced %zu byte(s) of output, expected only the %zu echoed byte(s)",
            S.rx_len - from, expect);
    r = host_query(":SYST:ERR?", &rep, SEC(5));
    REQUIRE(r == RUN_OK, "no reply to :SYST:ERR? after blank lines: %s", run_text(r));
    REQUIRE(strcmp(rep.text, "NO ERROR!") == 0, "blank lines raised an error: \"%s\"", rep.text);
    info("blank lines produced no output and no error");
    return T_PASS;
}

/* Control for test 14: *RST arms the 15 ms watchdog and spins (thid.c:321-327). If the harness cannot see
 * that reset, a missing reset in the hung-converter scenario would say nothing about the firmware. */
static const char *watchdog_control(void) {
    reply_t rep;
    unsigned one = 1;

    app_open();
    if (host_query("*IDN?", &rep, SEC(40)) != RUN_OK) {
        return "the application did not answer *IDN? before *RST";
    }
    run_status_t r = host_send("*RST", SEC(5));

    if (r != RUN_OK && S.resets == 0) {
        static char buf[160];

        snprintf(buf, sizeof buf, "*RST was not delivered (%s)", run_text(r));
        return buf;
    }
    /* The budget is 4 s, not 2, and the reason is a simavr modelling limit rather than slack.
     *
     * `main_soft_reset()` arms a 15 ms watchdog, but simavr does not act on the new prescaler. In
     * `avr_watchdog_write` (simavr/sim/avr_watchdog.c:108-118) `old_wdp` is read AFTER
     * `avr_core_watch_write` has already stored the new value, so `wdp_changed` compares the new prescaler
     * with itself and is always false; with the watchdog already enabled `enable_changed` is false too, and
     * `avr_watchdog_set_cycle_count_and_timer` returns at its guard (:76-79) without registering a cycle
     * timer. So the 15 ms arm has no effect here and what actually expires is the timer the last WDR
     * registered, at the main loop's WDTO_2S period: 2048 << 7 = 262144 counts, times 12 MHz / 128 kHz,
     * is 24,576,000 cycles = 2.048 s. A 2 s budget cannot see it.
     *
     * This is the simulator, not the firmware: on the part a WDTCR write takes effect immediately
     * (ATmega128A datasheet p.77). The control asks only "does *RST reset the instrument", and 4 s answers
     * that without asserting a precision simavr does not have. It still fails if no reset comes at all.
     *
     * It passed at 2 s until 2026-10-23 because the reply used to be paced at 1 ms per character, so the
     * firmware reached the spin well after its last WDR and the remainder of the 2.048 s fitted inside the
     * budget. Removing that pause (ORIGINAL_BUGS #8) left the full period to run. */
    r = run_until(pred_resets, &one, SEC(4));

    if (r != RUN_OK) {
        static char buf[160];

        snprintf(buf, sizeof buf, "no reset within 4 s of *RST (%s, WDTCR 0x%02X)", run_text(r),
                 S.avr->data[ADDR_WDTCR]);
        return buf;
    }
    r = run_until(pred_app_uart_ready, NULL, SEC(30));
    if (r != RUN_OK) {
        return "the application did not restart after the reset";
    }
    if (host_query("*IDN?", &rep, SEC(60)) != RUN_OK) {
        return "the application did not answer *IDN? after the reset";
    }
    return NULL;
}

/* 14. A hung converter: the meter retries and reports, and does NOT reset.
 *
 * A reset would be the obvious thing to assert, but the firmware does something else: the /DRDY timeout enters MEAS_FAULT, invalidates the reading
 * so the panel shows dashes, and retries once a second, with the main loop still reaching wdt_reset().
 *
 * That behaviour is a design decision. A converter that has genuinely died would make a resetting
 * instrument reboot for ever, which is worse than one showing dashes and retrying -- and a converter that
 * recovers is picked up by the retry with nobody power-cycling. What was missing was that nobody could
 * TELL, since those dashes are the same dashes a normal restart shows; the latched fault is drained into the
 * error queue.
 *
 * So this now asserts the three things that decision actually promises: no reset, the retry continuing,
 * and an error the host can read. The *RST control still runs first, because "no reset happened" is only
 * evidence if a reset would have been visible. */
static result_t test_14(void) {
    const char *control = watchdog_control();

    if (control != NULL) {
        snprintf(g_why, sizeof g_why,
                 "control failed (%s): without a visible *RST watchdog reset, 'the firmware did not reset' is not a "
                 "fact about the firmware",
                 control);
        return T_SKIP;
    }
    info("control: *RST produces a watchdog reset and the application restarts, so a reset would be seen");
    app_open();
    unsigned want = 5;
    run_status_t r = run_until(pred_rdata, &want, SEC(30));

    REQUIRE(r == RUN_OK, "the measurement loop never started: %s", run_text(r));
    r = run_for(SEC(2));
    REQUIRE(r == RUN_OK, "%s", stopped_text());
    REQUIRE(S.resets == 0, "the core reset %u time(s) before the converter was stopped", S.resets);
    unsigned reads_before = S.ads.n_rdata;
    unsigned cal_before = S.ads.n_cal;
    size_t frames_before = S.vfd.nframes;

    ads_set_mode(ADS_NEVER_READY);

    r = run_for(SEC(20));
    if (r == RUN_STOPPED) {
        FAILF("%s -- the core stopped instead of carrying on", stopped_text());
    }
    REQUIRE(r == RUN_OK, "%s", run_text(r));

    info("20 s without /DRDY: %u reset(s), %u self-calibration retr(ies), %zu display frames, core %s",
         S.resets, S.ads.n_cal - cal_before, S.vfd.nframes - frames_before, state_name(S.last_state));

    /* 1. It did not reset. */
    REQUIRE(S.resets == 0, "the meter reset %u time(s); the design is retry-and-report, not reset",
            S.resets);

    /* 2. It is still alive and still trying. The display frames prove the main loop is running, and the
     *    re-issued self-calibrations prove the measurement state machine is retrying rather than wedged. */
    REQUIRE(S.vfd.nframes > frames_before,
            "the display stopped being written, so the main loop is not running");
    REQUIRE(S.ads.n_cal > cal_before,
            "the converter was never retried after it went silent (self-calibrations before %u, after %u)",
            cal_before, S.ads.n_cal);
    REQUIRE(S.ads.n_rdata >= reads_before,
            "RDATA count went backwards (before %u, after %u)", reads_before, S.ads.n_rdata);

    /* 3. A host can find out. This part matters: the fault is
     *    latched in meas.c and never consumed, so the only external sign was dashes that look exactly like
     *    a meter which has not taken its first reading. */
    reply_t err;

    r = host_query(":SYST:ERR?", &err, SEC(5));
    REQUIRE(r == RUN_OK, ":SYST:ERR? did not answer while the converter was hung: %s", run_text(r));
    info(":SYST:ERR? -> \"%s\"", err.text);
    REQUIRE(strcmp(err.text, "NO ERROR!") != 0,
            ":SYST:ERR? reported no error although the converter had been silent for 20 s");
    return T_PASS;
}

/* 15. The :THID calibration-count query, in the spelling the vendor is proven to use on this unit.
 *
 * After flashing, a user can send this and expect the meter's calibration count (5 in the test image) as
 * the check that the :THID block was not written by the flash. The vendor tools use `:thid:cal:coun ?`
 * -- note the abbreviation and the space before the '?'. This test establishes that OUR port accepts that
 * spelling, so the step cannot fail on syntax and be read as a damaged identity block.
 *
 * The count comes from the EEPROM image, which the harness loads, so this asserts the number
 * the meter will actually give rather than a constant typed in here. */
static result_t test_15(void) {
    reply_t rep;
    unsigned want;

    app_open();
    /* EE_THID_CAL_COUNT is 0x0A26, a little-endian u16 (eeprom_map.h). */
    want = (unsigned)S.ee_initial[0x0A26] | ((unsigned)S.ee_initial[0x0A27] << 8);

    run_status_t r = host_query(":thid:cal:coun ?", &rep, SEC(40));

    REQUIRE(r == RUN_OK, "no reply to \":thid:cal:coun ?\": %s", run_text(r));
    info(":thid:cal:coun ? -> \"%s\"  (EEPROM holds %u)", rep.text, want);

    char expect[16];
    snprintf(expect, sizeof expect, "%u", want);
    REQUIRE(strcmp(rep.text, expect) == 0,
            "reply \"%s\" does not match the count in the EEPROM image (%u)", rep.text, want);

    /* And it must not have been a syntax error that happened to print a number. */
    r = host_query(":SYST:ERR?", &rep, SEC(10));
    REQUIRE(r == RUN_OK, "no reply to :SYST:ERR?: %s", run_text(r));
    REQUIRE(strcmp(rep.text, "NO ERROR!") == 0, "the query raised an error: \"%s\"", rep.text);
    return T_PASS;
}

/* ------------------------------------------------------------------------------------------------ */
/* SYS item 7, ZERO CAL: tests 16 to 18                                                         */
/*
 * One sweep is driven from the front panel with the key model and watched while it runs; the three tests
 * assert different things about the same observation, so it is made once per process and cached.
 *
 * Path through the menu, every step from src:
 *   SHIFT, MENU       dispatch_shifted() KEY_MENU -> menu_enter(), page A:MATH (keys.c:521-548, menu.c:2810-2819)
 *   NEXT, NEXT        page_key(): MATH -> TRIG -> SYS (menu.c:2746-2752, page order menu.c:390)
 *   RANGE_DOWN        KEY_DOWN on a page -> the item list (menu.c:2754-2756)
 *   NEXT x6           item_key(): item 0 -> item 6, "7:ZERO CAL" (menu.c:2785-2787, sys_items[] menu.c:368-375)
 *   RANGE_DOWN        start_editor(): a protected item with the lock closed opens the KEY prompt
 *                     (menu.c:2732-2735, item_is_protected menu.c:340-342)
 *   RANGE_UP x6, NEXT four times over: integer_key() steps a digit with UP and moves the cursor with NEXT
 *                     (menu.c:1754-1764, digit_step menu.c:879-885); the field starts at 0000 (menu.c:1678)
 *   AUTO (ENTER)      integer_enter(): serv_unlock(code, SERV_FROM_PANEL) then open_item() -> the CAL:SHORT
 *                     prompt (menu.c:1716-1722, 2674-2679)
 *   AUTO (ENTER)      prompt_run(): PA_USER_ZERO -> begin_busy(ACT_ZERO_CAL) (menu.c:2308-2312)
 * After that menu_tick() runs one zero_cal_tick() per call: one meas_zero_capture() and one
 * cal_write_user_zero() for DC V ranges 0..4 and then ohm ranges 0..5 (menu.c:2165-2179).
 *
 * Scan bits are the ones keys.c:43 assigns to byte 0, bit 0 first.
 */
#define ZK_PREV 0x01u
#define ZK_NEXT 0x02u
#define ZK_DOWN 0x04u /* KEY_RANGE_DOWN, which menu.c:62 calls KEY_DOWN */
#define ZK_UP 0x08u  /* KEY_RANGE_UP, menu.c:61 */
#define ZK_ENTER 0x10u /* KEY_AUTO, menu.c:65 */
#define ZK_MENU 0x20u
#define ZK_SHIFT 0x40u

#define KEY_HOLD_MS 150u /* keys.h:34 KEY_DEBOUNCE_MS is 50: the same code must be seen on two scans 50 ms apart */
#define KEY_GAP_MS 100u

#define EE_UZERO_DCV 0x0880u /* src/eeprom_map.h:130 */
#define EE_UZERO_DCV_SIG 0x0894u /* eeprom_map.h:131 */
#define EE_UZERO_OHM 0x0898u /* eeprom_map.h:132 */
#define ZS_DCV_RANGES 5u     /* eeprom_map.h:130 "5 floats" */
#define ZS_OHM_RANGES 6u     /* eeprom_map.h:132 "6 floats" */
#define ZS_RECORDS (ZS_DCV_RANGES + ZS_OHM_RANGES)

/* Front-end bits, from board.h:75-99: the ports and the bits of each that switch relays. Other bits of
 * the same ports are the buzzer (PA4) and the two serial buses (PB0-4, PC0-4), which are not front end.
 * The ADS1216 and VFD models would otherwise make the comparison depend on bus traffic.
 *
 * PB6 (FE_B_OHM_SRC) is left out of the comparison on purpose. It is set by the ohm and diode functions and
 * no function ever clears it (meas.c:710-721, ORIGINAL_BUGS.md #103, status BLOCKED-ON-HARDWARE: its electrical
 * role is not established). The sweep therefore leaves it set whatever restore is done, so including it would make
 * the test fail on a behaviour that is carried over from the original and recorded there. PA2
 * (FE_A_OHM_SRC_EN, board.h:77), which fe_dcv() clears (meas.c:557), is compared. */
#define FE_PORTS 7u
static const struct {
    char name;
    uint16_t data_addr; /* I/O address + 0x20; iom128.h: PORTA 0x1B, B 0x18, C 0x15, D 0x12, E 0x03, F 0x62(mem), G 0x65(mem) */
    uint8_t mask;
} FE_MAP[FE_PORTS] = {
    {'A', 0x3Bu, 0xEFu}, /* PA0-3, PA5-7 */
    {'B', 0x38u, 0xA0u}, /* PB5, PB7 */
    {'C', 0x35u, 0xE0u}, /* PC5-7 */
    {'D', 0x32u, 0x20u}, /* PD5 */
    {'E', 0x23u, 0x1Cu}, /* PE2-4 */
    {'F', 0x62u, 0x0Eu}, /* PF1-3 */
    {'G', 0x65u, 0x18u}, /* PG3-4 */
};

typedef struct {
    uint8_t p[FE_PORTS];
} fe_state_t;

static fe_state_t fe_sample(void) {
    fe_state_t s;

    for (unsigned i = 0; i < FE_PORTS; i++) {
        s.p[i] = (uint8_t)(S.avr->data[FE_MAP[i].data_addr] & FE_MAP[i].mask);
    }
    return s;
}

static bool fe_equal(const fe_state_t *a, const fe_state_t *b) { return memcmp(a, b, sizeof *a) == 0; }

static const char *fe_text(const fe_state_t *s) {
    static char buf[4][48];
    static unsigned n;
    char *out = buf[n++ % 4u];

    snprintf(out, sizeof buf[0], "A%02X B%02X C%02X D%02X E%02X F%02X G%02X", s->p[0], s->p[1], s->p[2], s->p[3],
             s->p[4], s->p[5], s->p[6]);
    return out;
}

#define FE_DISTINCT_MAX 16u
#define SENTINEL_F32 1.0f

typedef struct {
    bool ran;
    const char *fail; /* set when the sweep could not be driven or observed: nothing below is then valid */
    bool timed_out;   /* the sweep was driven but had not written all eleven records within the budget */

    uint8_t ee_poisoned[EEPROM_BYTES]; /* EEPROM just before the sweep, spans holding the sentinel */
    uint8_t ee_after[EEPROM_BYTES];
    uint64_t t_start;
    uint64_t t_rec[ZS_RECORDS]; /* when the record first differed from the sentinel, 0 = never */
    fe_state_t fe_before;
    fe_state_t fe_at_rec[ZS_RECORDS]; /* front end at that moment, i.e. after the capture released the converter */
    fe_state_t fe_seen[FE_DISTINCT_MAX];
    unsigned fe_seen_n; /* distinct states different from fe_before seen while the sweep ran */
    bool ohm_src_seen;  /* PA2, FE_A_OHM_SRC_EN, was high at some point during the sweep */

    unsigned scans;
    uint64_t max_scan_gap;
    uint64_t gap_end;       /* when the longest gap ended */
    size_t frames_during;   /* display frames written between the start and the end */

    bool idn_answered;
    uint64_t idn_latency;
    char idn_text[64];
} zs_t;

static zs_t ZS;

static uint16_t zs_addr(unsigned i) {
    return i < ZS_DCV_RANGES ? (uint16_t)(EE_UZERO_DCV + 4u * i) : (uint16_t)(EE_UZERO_OHM + 4u * (i - ZS_DCV_RANGES));
}

static float f32_at(const uint8_t *ee, unsigned addr) {
    float v;

    memcpy(&v, ee + addr, sizeof v);
    return v;
}

/* The simulated chip's EEPROM array, for per-step inspection. The ioctl would copy 4 KiB per step. */
static uint8_t *eeprom_array(void) {
    for (avr_io_t *io = S.avr->io_port; io != NULL; io = io->next) {
        if (io->kind != NULL && strcmp(io->kind, "eeprom") == 0) {
            return ((avr_eeprom_t *)io)->eeprom;
        }
    }
    return NULL;
}

static run_status_t key_tap(uint8_t scan_bit) {
    S.keys[0] = scan_bit;
    run_status_t r = run_for(MS(KEY_HOLD_MS));

    S.keys[0] = 0;
    return r == RUN_OK ? run_for(MS(KEY_GAP_MS)) : r;
}

static const char *key_taps(const uint8_t *bits, size_t n) {
    for (size_t i = 0; i < n; i++) {
        run_status_t r = key_tap(bits[i]);

        if (r != RUN_OK) {
            return "the core stopped while keys were being pressed";
        }
    }
    return NULL;
}

/* Brings the menu to the CAL:SHORT prompt of SYS item 7, entering the service code on the panel. */
static const char *zs_navigate(void) {
    static const uint8_t to_item[] = {ZK_SHIFT, ZK_MENU, ZK_NEXT, ZK_NEXT, ZK_DOWN, ZK_NEXT, ZK_NEXT,
                                      ZK_NEXT,  ZK_NEXT, ZK_NEXT, ZK_NEXT, ZK_DOWN};
    const char *err = key_taps(to_item, sizeof to_item);

    if (err != NULL) {
        return err;
    }
    for (unsigned digit = 0; digit < 4u; digit++) {
        for (unsigned up = 0; up < 6u; up++) { /* '0' -> '6' */
            if (key_tap(ZK_UP) != RUN_OK) {
                return "the core stopped while the service code was being entered";
            }
        }
        if (key_tap(ZK_NEXT) != RUN_OK) {
            return "the core stopped while the service code was being entered";
        }
    }
    return key_tap(ZK_ENTER) == RUN_OK ? NULL : "the core stopped after the service code was entered";
}

/*
 * What the converter reads while the input is shorted, per captured range.
 *
 * A real instrument with a short on the jacks does not read zero counts: it reads its own offset, and that
 * offset is what the stored factory zero was measured against. cal_sample() is (2.86e-7 * code) * scale -
 * zero (cal.c:38, cal.c:480-486), so the code a short produces is zero / (2.86e-7 * scale). With a constant
 * code the first range passes and the second is refused as implausible (meas.c:2375-2384, observed in this
 * harness), which ends the sweep with FAILED after one record. The factory zeros are read from the EEPROM image the harness
 * loaded (eeprom_map.h:90-91, 14 bytes per range, first float); the scales are tables.c:65 and tables.c:69.
 * The code for the next capture is set when the record of the previous one appears, which is the order
 * menu.c:2165-2171 runs them in.
 */
static const float ZS_SCALE_DCV[ZS_DCV_RANGES] = {495.55f, 247.775f, 25.0025f, 2.5002f, 0.25f};
static const float ZS_SCALE_OHM[ZS_OHM_RANGES] = {24593496.0f, 2705284.5f, 245934.9f, 24593.4f, 2459.3f, 307.41f};
#define ZS_VOLTS_PER_COUNT 2.86e-7f
#define EE_ZERO_OHM_ADDR 0x0438u /* eeprom_map.h:90 */
#define EE_ZERO_DCV_ADDR 0x048Cu /* eeprom_map.h:91 */
#define EE_ZERO_RECORD 14u       /* eeprom_map.h:52 */

static int32_t zs_short_code(unsigned i) {
    const bool dc = i < ZS_DCV_RANGES;
    const unsigned r = dc ? i : i - ZS_DCV_RANGES;
    const float zero = f32_at(S.ee_initial, (dc ? EE_ZERO_DCV_ADDR : EE_ZERO_OHM_ADDR) + EE_ZERO_RECORD * r);
    const float scale = dc ? ZS_SCALE_DCV[r] : ZS_SCALE_OHM[r];

    return (int32_t)lrintf(zero / (ZS_VOLTS_PER_COUNT * scale));
}

static void zs_record_fe(const fe_state_t *now_fe) {
    if (fe_equal(now_fe, &ZS.fe_before)) {
        return;
    }
    for (unsigned i = 0; i < ZS.fe_seen_n; i++) {
        if (fe_equal(now_fe, &ZS.fe_seen[i])) {
            return;
        }
    }
    if (ZS.fe_seen_n < FE_DISTINCT_MAX) {
        ZS.fe_seen[ZS.fe_seen_n++] = *now_fe;
    }
}

#define ZS_SWEEP_BUDGET SEC(20) /* the sweep takes about 7 s of simulated time */
#define ZS_IDN_AFTER MS(500)
#define ZS_AFTER_LAST MS(2000)
#define ZS_LAST_WRITE_SETTLE MS(100)

static void zs_run(void) {
    uint8_t *ee;
    uint8_t sentinel[4];
    float one = SENTINEL_F32;
    bool idn_sent = false;
    bool idn_arrived = false;
    uint64_t idn_arrive = 0;
    size_t idn_from = 0;
    size_t idn_echo = 0;
    uint64_t last_scan;
    unsigned seen_scans;
    size_t frames_start;
    unsigned want = 5;
    unsigned recorded = 0;
    uint64_t end;

    if (ZS.ran) {
        return;
    }
    ZS.ran = true;
    memcpy(sentinel, &one, sizeof sentinel);
    app_open();

    if (run_until(pred_rdata, &want, SEC(30)) != RUN_OK || run_for(SEC(4)) != RUN_OK) {
        ZS.fail = "the measurement loop never started";
        return;
    }
    /* The live measurement must not move the relays by itself while the sweep runs, or "the state before"
     * would not be one state: autorange follows the input, and the input changes from one capture to the
     * next. RANGE_DOWN (keys.c:330-346) selects manual ranging and steps towards the lowest range, the
     * extra presses stopping at the end (keys.c:338-341); RANGE_UP (keys.c:311-328) then steps back one,
     * to DC V range 3. fe_dcv() writes every bit it owns for a range below 4 (meas.c:560-564), whereas for
     * range 4 it leaves PA1 as it found it (meas.c:560-564, 587-593), which would make the state before
     * depend on history. */
    S.ads.code = 0;
    for (unsigned i = 0; i < 7u; i++) {
        if (key_tap(i < 6u ? ZK_DOWN : ZK_UP) != RUN_OK) {
            ZS.fail = "the core stopped while the range was being fixed";
            return;
        }
    }
    if (run_for(SEC(2)) != RUN_OK) {
        ZS.fail = "the core stopped while the range was being fixed";
        return;
    }
    ee = eeprom_array();
    if (ee == NULL) {
        ZS.fail = "the simulated EEPROM array could not be found";
        return;
    }
    for (unsigned i = 0; i < ZS_RECORDS; i++) {
        memcpy(ee + zs_addr(i), sentinel, sizeof sentinel);
    }
    memcpy(ZS.ee_poisoned, ee, EEPROM_BYTES);

    const char *nav = zs_navigate();

    if (nav != NULL) {
        ZS.fail = nav;
        return;
    }
    S.ads.code = zs_short_code(0);
    ZS.fe_before = fe_sample();
    ZS.t_start = now();
    last_scan = ZS.t_start;
    seen_scans = S.vfd.key_reads;
    frames_start = S.vfd.nframes;
    end = ZS.t_start + ZS_SWEEP_BUDGET;

    S.keys[0] = ZK_ENTER; /* PA_USER_ZERO starts when this is accepted, 50 ms of debounce later */
    uint64_t release_at = ZS.t_start + MS(KEY_HOLD_MS);

    for (;;) {
        const uint64_t t = now();

        if (S.keys[0] != 0 && t >= release_at) {
            S.keys[0] = 0;
        }
        if (t >= end) {
            ZS.timed_out = true;
            break;
        }
        if (!step()) {
            ZS.fail = stopped_text();
            break;
        }
        const uint64_t t2 = now();

        if (S.vfd.key_reads != seen_scans) {
            seen_scans = S.vfd.key_reads;
            ZS.scans++;
            if (t2 - last_scan > ZS.max_scan_gap) {
                ZS.max_scan_gap = t2 - last_scan;
                ZS.gap_end = t2;
            }
            last_scan = t2;
        }
        const fe_state_t fe = fe_sample();

        zs_record_fe(&fe);
        if ((S.avr->data[0x3Bu] & 0x04u) != 0) { /* PORTA bit 2 = PA2, board.h:77 */
            ZS.ohm_src_seen = true;
        }
        for (unsigned i = 0; i < ZS_RECORDS; i++) {
            if (ZS.t_rec[i] == 0 && memcmp(ee + zs_addr(i), sentinel, sizeof sentinel) != 0) {
                ZS.t_rec[i] = t2;
                ZS.fe_at_rec[i] = fe;
                recorded++;
                if (i + 1u < ZS_RECORDS) {
                    S.ads.code = zs_short_code(i + 1u);
                }
            }
        }
        if (!idn_sent && t2 >= ZS.t_start + ZS_IDN_AFTER) {
            idn_sent = true;
            idn_from = S.rx_len;
            idn_echo = S.echo ? strlen("*IDN?") + 1u : 0u;
            host_queue_str("*IDN?\n");
        }
        if (idn_sent && !idn_arrived && host_queue_empty()) {
            idn_arrived = true;
            idn_arrive = S.last_fed + uart_byte_cycles();
        }
        if (idn_arrived && !ZS.idn_answered) {
            const uint8_t *nl = S.rx_len > idn_from + idn_echo
                                    ? memchr(S.rx_buf + idn_from + idn_echo, '\n', S.rx_len - (idn_from + idn_echo))
                                    : NULL;

            if (nl != NULL) {
                size_t len = (size_t)(nl - (S.rx_buf + idn_from + idn_echo));

                ZS.idn_answered = true;
                ZS.idn_latency = S.rx_time[idn_from + idn_echo] > idn_arrive ? S.rx_time[idn_from + idn_echo] - idn_arrive : 0;
                len = len < sizeof ZS.idn_text - 1 ? len : sizeof ZS.idn_text - 1;
                memcpy(ZS.idn_text, S.rx_buf + idn_from + idn_echo, len);
                ZS.idn_text[len] = '\0';
            }
        }
        /* The last record is noticed at its first changed byte; the rest of its four bytes is still being
         * written, so the run goes on for a moment before the EEPROM is read back. */
        if (recorded == ZS_RECORDS && t2 > ZS.t_rec[ZS_RECORDS - 1] + ZS_LAST_WRITE_SETTLE &&
            (ZS.idn_answered || t2 > ZS.t_rec[ZS_RECORDS - 1] + ZS_AFTER_LAST)) {
            break;
        }
    }
    ZS.frames_during = S.vfd.nframes - frames_start;
    S.keys[0] = 0;
    if (ZS.fail != NULL || ZS.timed_out) {
        info("sweep diagnostics: %u of %u records written, %u key scans, converter: %u RDATA, %u register writes, %u "
             "self-calibrations, %u unknown command(s)",
             recorded, ZS_RECORDS, ZS.scans, S.ads.n_rdata, S.ads.n_wreg, S.ads.n_cal, S.ads.n_unknown);
    }

    avr_eeprom_desc_t desc = {.ee = ZS.ee_after, .offset = 0, .size = EEPROM_BYTES};

    if (avr_ioctl(S.avr, AVR_IOCTL_EEPROM_GET, &desc) < 0 && ZS.fail == NULL) {
        ZS.fail = "EEPROM read-back failed";
    }
}

static bool ee_documented_runtime_write(unsigned a) {
    return a == EE_SERV_FAILS || (a >= EE_PWRUP_COUNT && a < EE_PWRUP_COUNT + 2u) ||
           (a >= EE_HOURS_BASE && a < EE_HOURS_END);
}

/* 16. The sweep stores one finite float in each of the eleven user-zero slots, in order, and nothing else. */
static result_t test_16(void) {
    zs_run();
    REQUIRE(ZS.fail == NULL, "the ZERO CAL sweep could not be driven to its end: %s", ZS.fail);
    for (unsigned i = 0; i < ZS_RECORDS; i++) {
        const unsigned a = zs_addr(i);
        const float v = f32_at(ZS.ee_after, a);
        const char *name = i < ZS_DCV_RANGES ? "DC V" : "ohm";
        const unsigned r = i < ZS_DCV_RANGES ? i : i - ZS_DCV_RANGES;

        REQUIRE(ZS.t_rec[i] != 0, "%s range %u: the record at 0x%04X was never written (still the sentinel)", name, r, a);
        REQUIRE(memcmp(ZS.ee_after + a, ZS.ee_poisoned + a, 4) != 0, "%s range %u: 0x%04X is unchanged at the end", name, r, a);
        REQUIRE(isfinite(v), "%s range %u: the stored zero at 0x%04X is not finite (bytes %02X %02X %02X %02X)", name, r, a,
                ZS.ee_after[a], ZS.ee_after[a + 1], ZS.ee_after[a + 2], ZS.ee_after[a + 3]);
        if (i > 0) {
            REQUIRE(ZS.t_rec[i] > ZS.t_rec[i - 1], "%s range %u was written before record %u: the sweep order is wrong",
                    name, r, i - 1u);
        }
        info("%-4s range %u  0x%04X = %.9g  (written %.3f s after ENTER)", name, r, a, (double)v,
             seconds(ZS.t_rec[i] - ZS.t_start));
    }
    unsigned stray = 0;

    for (unsigned a = 0; a < EEPROM_BYTES; a++) {
        const bool in_span = (a >= EE_UZERO_DCV && a < EE_UZERO_DCV + 4u * ZS_DCV_RANGES) ||
                             (a >= EE_UZERO_OHM && a < EE_UZERO_OHM + 4u * ZS_OHM_RANGES);

        if (ZS.ee_after[a] != ZS.ee_poisoned[a] && !in_span && !ee_documented_runtime_write(a)) {
            if (stray++ < 16u) {
                info("EEPROM 0x%04X: %02X -> %02X  outside the two user-zero spans", a, ZS.ee_poisoned[a], ZS.ee_after[a]);
            }
        }
    }
    REQUIRE(stray == 0, "%u EEPROM byte(s) changed outside the two user-zero spans (the DC V block signature at 0x%04X "
            "must not move)", stray, EE_UZERO_DCV_SIG);
    return T_PASS;
}

/* 17. After the sweep the relays are where they were: the ohm source is not left on the input jacks. */
static result_t test_17(void) {
    zs_run();
    REQUIRE(ZS.fail == NULL, "the ZERO CAL sweep could not be driven to its end: %s", ZS.fail);
    REQUIRE(ZS.t_rec[ZS_RECORDS - 1] != 0, "the last ohm record was never written, so the end state cannot be sampled");
    if (ZS.fe_seen_n == 0) {
        snprintf(g_why, sizeof g_why,
                 "the front-end ports never left their pre-sweep state (%s) while the sweep ran, so a restore "
                 "check could not distinguish restored from never moved",
                 fe_text(&ZS.fe_before));
        return T_SKIP;
    }
    REQUIRE(ZS.ohm_src_seen, "the ohm source enable (PA2, board.h:77) was never high during the sweep, so there is "
            "nothing for the restore to undo");
    info("front end before: %s;  %u other state(s) seen during the sweep", fe_text(&ZS.fe_before), ZS.fe_seen_n);
    for (unsigned i = 0; i < ZS_RECORDS; i++) {
        const char *name = i < ZS_DCV_RANGES ? "DC V" : "ohm";
        const unsigned r = i < ZS_DCV_RANGES ? i : i - ZS_DCV_RANGES;

        REQUIRE((ZS.fe_at_rec[i].p[0] & 0x04u) == 0,
                "after the %s range %u capture the ohm source enable (PA2) is still set: %s", name, r,
                fe_text(&ZS.fe_at_rec[i]));
        REQUIRE(fe_equal(&ZS.fe_at_rec[i], &ZS.fe_before),
                "after the %s range %u capture the front end is %s, before the sweep it was %s", name, r,
                fe_text(&ZS.fe_at_rec[i]), fe_text(&ZS.fe_before));
    }
    info("after each of the 11 captures the front end is back to the pre-sweep state");
    return T_PASS;
}

/* 18. The sweep takes one capture per main-loop pass; the loop keeps scanning keys, drawing and answering. */
static result_t test_18(void) {
    uint64_t longest_capture = 0;

    zs_run();
    REQUIRE(ZS.fail == NULL, "the ZERO CAL sweep could not be driven to its end: %s", ZS.fail);
    for (unsigned i = 1; i < ZS_RECORDS; i++) {
        REQUIRE(ZS.t_rec[i] != 0 && ZS.t_rec[i - 1] != 0, "record %u or %u never written", i - 1u, i);
        if (ZS.t_rec[i] - ZS.t_rec[i - 1] > longest_capture) {
            longest_capture = ZS.t_rec[i] - ZS.t_rec[i - 1];
        }
    }
    info("records are %.0f ms apart at most; %u key scans, longest gap between scans %.0f ms; %zu display frames during",
         seconds(longest_capture) * 1000.0, ZS.scans, seconds(ZS.max_scan_gap) * 1000.0, ZS.frames_during);
    REQUIRE(ZS.scans > ZS_RECORDS, "only %u key scan(s) in the whole sweep of %u captures: the loop is not running between captures",
            ZS.scans, ZS_RECORDS);
    REQUIRE(ZS.max_scan_gap <= 2u * longest_capture,
            "the key scan was starved for %.0f ms in one stretch; one capture takes at most %.0f ms. Several captures ran back to "
            "back",
            seconds(ZS.max_scan_gap) * 1000.0, seconds(longest_capture) * 1000.0);
    REQUIRE(ZS.frames_during > 0, "the display was not written once while the sweep ran");
    REQUIRE(ZS.idn_answered, "*IDN? sent %.1f s into the sweep was not answered before the sweep ended",
            seconds(ZS_IDN_AFTER));
    info("*IDN? answered in %.0f ms: \"%s\"", seconds(ZS.idn_latency) * 1000.0, ZS.idn_text);
    REQUIRE(ZS.idn_latency <= 2u * longest_capture + MS(100),
            "*IDN? took %.0f ms to begin its reply during the sweep; one capture takes at most %.0f ms",
            seconds(ZS.idn_latency) * 1000.0, seconds(longest_capture) * 1000.0);
    return T_PASS;
}

/* ------------------------------------------------------------------------------------------------ */
/* SYS item 8, RLY EXER: test 19                                                                    */
/*
 * What is measured, and how it is read from the simulated chip (the analogue front end itself is not modelled,
 * so every figure comes from the port registers the firmware writes and from the converter's RDATA log):
 *
 *   front-end state   the masked PORTA..PORTG bits of FE_MAP, sampled after every instruction. A change is an
 *                     event. A state is HELD when it lasts EX_HELD or longer; the intermediate states inside
 *                     one fe_apply() last microseconds and are not counted.
 *   guard reads       RDATA commands. The exercise takes conversions only for its input guard, so a held state
 *                     during which the converter is read is a guard state, and a held state during which it is
 *                     not read is a relay pattern.
 *   pattern rate      pattern states entered per second, over EX_WINDOW from the start of the run. A pattern
 *                     whose state equals the guard state it follows (DC V range 0) causes no port change and
 *                     is not seen; the count is made the same way for every firmware this test is run on.
 *   unread time       the longest stretch without an RDATA, which is the longest time the input was unchecked.
 *
 * The run is started from the panel exactly as a user does it: SHIFT, MENU, NEXT, NEXT (page SYS), DOWN
 * (item list), NEXT x7 (item 8, "RLY EXER", sys_items[] in menu.c), DOWN (the NO INPUT prompt), ENTER.
 */
#define EX_EVENTS_MAX 60000u
#define EX_HELD MS(10)
#define EX_WINDOW SEC(8)
#define EX_AFTER_INPUT SEC(3)
#define EX_INPUT_NEAR MS(10)  /* a port write this soon after an RDATA is the write that follows a guard read */
#define EX_INPUT_CODE 0x10000 /* 65536 x 2.86e-7 V x 495.55 = 9.3 V on DC V range 0 (ZS_SCALE_DCV[0]) */
#define EX_INPUT_VOLTS 9.3
#define EX_BOOT_OBSERVE SEC(8)
#define EX_BOOT_RUN_MS MS(1500) /* MEAS_RELAY_BOOT_MS, meas.c */
/* tDATA at 60 Hz, 128 x 639 / 4.910 MHz = 16.658 ms (meas.c TDATA_US_60HZ is the rounded-down 16600). The firmware runs at
 * 60 Hz unless the EEPROM holds 1, and the harness's default converter period is 20 ms, so the test sets this one. */
#define EX_CONVERSION_US 16658u
#define EX_STOP_LIMIT SEC(1)
#define EX_FINAL_STILL MS(1500)

/* The pattern rate of the firmware before the guard was moved off the per-pattern path, measured by this test
 * on that firmware and kept here as the figure the new rate is compared with. */
#define EX_OLD_PATTERNS_PER_S 5.50

typedef struct {
    uint64_t t;
    fe_state_t fe;
} ex_event_t;

static struct {
    ex_event_t ev[EX_EVENTS_MAX];
    size_t n;
    bool overflow;
    fe_state_t last;
    fe_state_t before;
    bool started;
    uint64_t t0;
    uint64_t t_enter;
    bool input_done;
    bool input_late;
    uint64_t t_input;
    uint64_t t_end;
} EX;

static void ex_input_apply(uint64_t t, bool late) {
    S.ads.code = EX_INPUT_CODE;
    EX.t_input = t;
    EX.input_done = true;
    EX.input_late = late;
}

static void ex_note(void) {
    const fe_state_t cur = fe_sample();
    const uint64_t t = now();

    if (fe_equal(&cur, &EX.last)) {
        return;
    }
    if (EX.n < EX_EVENTS_MAX) {
        EX.ev[EX.n].t = t;
        EX.ev[EX.n].fe = cur;
        EX.n++;
    } else {
        EX.overflow = true;
    }
    if (!EX.started) {
        EX.started = true;
        EX.t0 = t;
    }
    /* The port write right after a guard read. Applying the input here is the worst case for the exercise,
     * since the next guard read is then as far away as it can be. */
    if (EX.t_enter != 0 && t >= EX.t0 + EX_WINDOW && !EX.input_done && g_rdata_n > 0 && t - g_rdata_t[g_rdata_n - 1] < EX_INPUT_NEAR) {
        ex_input_apply(t, false);
    }
    EX.last = cur;
}

static bool ex_pump(uint64_t end) {
    while (now() < end) {
        if (!step()) {
            return false;
        }
        ex_note();
    }
    return true;
}

static uint64_t ex_dwell(size_t i) { return (i + 1u < EX.n ? EX.ev[i + 1u].t : EX.t_end) - EX.ev[i].t; }

static unsigned ex_reads_between(uint64_t from, uint64_t to) {
    unsigned n = 0;

    for (size_t i = 0; i < g_rdata_n; i++) {
        if (g_rdata_t[i] >= from && g_rdata_t[i] < to) {
            n++;
        }
    }
    return n;
}

/* Pattern states entered in [from, to): held, and not read from the converter while they last. */
static unsigned ex_pattern_entries(uint64_t from, uint64_t to) {
    unsigned n = 0;

    for (size_t i = 0; i < EX.n; i++) {
        const uint64_t d = ex_dwell(i);

        if (EX.ev[i].t >= from && EX.ev[i].t < to && d >= EX_HELD &&
            ex_reads_between(EX.ev[i].t, EX.ev[i].t + d) == 0) {
            n++;
        }
    }
    return n;
}

/* The longest stretch inside [from, to) with no RDATA, counting the two ends as if they were reads. */
static uint64_t ex_longest_unread(uint64_t from, uint64_t to) {
    uint64_t prev = from;
    uint64_t longest = 0;

    for (size_t i = 0; i < g_rdata_n; i++) {
        if (g_rdata_t[i] >= from && g_rdata_t[i] < to) {
            longest = g_rdata_t[i] - prev > longest ? g_rdata_t[i] - prev : longest;
            prev = g_rdata_t[i];
        }
    }
    return to - prev > longest ? to - prev : longest;
}

/* The power-on run, read from the events of a session that has just started. It begins with the first port
 * write at or before the first converter read (the guard of the first step) and ends when the last pattern
 * has been held, which is the moment the front end is put back. Returns the number of patterns it held. */
static unsigned ex_boot_run(uint64_t *length) {
    uint64_t begin = 0;
    uint64_t end = 0;
    unsigned patterns = 0;

    if (g_rdata_n == 0) {
        *length = 0;
        return 0;
    }
    for (size_t i = 0; i < EX.n; i++) {
        const uint64_t d = ex_dwell(i);

        if (EX.ev[i].t <= g_rdata_t[0]) {
            begin = EX.ev[i].t;
        }
        if (d >= EX_HELD && ex_reads_between(EX.ev[i].t, EX.ev[i].t + d) == 0 && EX.ev[i].t > g_rdata_t[0]) {
            end = EX.ev[i].t + d;
            patterns++;
        }
    }
    *length = end > begin ? end - begin : 0;
    return patterns;
}

/* 19. The relay exercise steps fast, the input guard is still run at least once a second, a live input stops
 * it, and the power-on run still fits its 1.5 s. */
static result_t test_19(void) {
    static const uint8_t to_item[] = {ZK_SHIFT, ZK_MENU, ZK_NEXT, ZK_NEXT, ZK_DOWN, ZK_NEXT, ZK_NEXT,
                                      ZK_NEXT,  ZK_NEXT, ZK_NEXT, ZK_NEXT, ZK_NEXT, ZK_DOWN};
    const char *err;
    uint64_t boot_length;
    unsigned boot_patterns;

    memset(&EX, 0, sizeof EX);
    app_open();
    S.ads.conv_delay_us = EX_CONVERSION_US;
    S.ads.code = 0; /* no input: the power-on run is not stopped by the guard */
    EX.last = fe_sample();
    REQUIRE(ex_pump(EX_BOOT_OBSERVE), "%s", stopped_text());
    EX.t_end = now();
    boot_patterns = ex_boot_run(&boot_length);

    memset(&EX, 0, sizeof EX);
    for (unsigned i = 0; i < 7u; i++) { /* the manual DC V range 3 that zs_run() also uses */
        REQUIRE(key_tap(i < 6u ? ZK_DOWN : ZK_UP) == RUN_OK, "%s", stopped_text());
    }
    REQUIRE(run_for(SEC(2)) == RUN_OK, "%s", stopped_text());
    err = key_taps(to_item, sizeof to_item);
    REQUIRE(err == NULL, "%s", err);
    EX.last = fe_sample();
    EX.t_enter = now();
    S.keys[0] = ZK_ENTER;
    REQUIRE(ex_pump(EX.t_enter + MS(KEY_HOLD_MS)), "%s", stopped_text());
    S.keys[0] = 0;
    for (;;) {
        const uint64_t t = now();

        if (!EX.started && t > EX.t_enter + SEC(3)) {
            FAILF("the exercise never began: the front end did not change in 3 s after ENTER");
        }
        if (EX.started && !EX.input_done && t > EX.t0 + EX_WINDOW + SEC(5)) {
            ex_input_apply(t, true); /* no guard read was seen after the window: the input goes in regardless */
        }
        if (EX.input_done && t >= EX.t_input + EX_AFTER_INPUT) {
            break;
        }
        REQUIRE(step(), "%s", stopped_text());
        ex_note();
    }
    EX.t_end = now();
    REQUIRE(!EX.overflow, "more than %u front-end changes were logged", EX_EVENTS_MAX);

    const unsigned patterns = ex_pattern_entries(EX.t0, EX.t0 + EX_WINDOW);
    const double rate = (double)patterns / seconds(EX_WINDOW);
    const uint64_t unread = ex_longest_unread(EX.t0, EX.t0 + EX_WINDOW);
    const uint64_t t_last = EX.ev[EX.n - 1u].t;
    const uint64_t stop = t_last - EX.t_input;
    const bool stopped = EX.t_end - t_last >= EX_FINAL_STILL;
    bool ok = true;

    info("power-on run: %u patterns held, %.0f ms from its first guard to the front end being put back (limit %.0f ms)",
         boot_patterns, seconds(boot_length) * 1000.0, seconds(EX_BOOT_RUN_MS) * 1000.0);
    info("pattern rate: %u patterns in %.0f s = %.2f /s (before the change: %.2f /s, ratio %.2f)", patterns,
         seconds(EX_WINDOW), rate, EX_OLD_PATTERNS_PER_S, rate / EX_OLD_PATTERNS_PER_S);
    info("longest time without a guard read in the window: %.0f ms", seconds(unread) * 1000.0);
    info("input of %.1f V applied %.3f s into the run%s; last front-end change %.0f ms later, %.1f s before the end",
         EX_INPUT_VOLTS, seconds(EX.t_input - EX.t0),
         EX.input_late ? " (no guard read was seen, so at a fixed time)" : " right after a guard read",
         seconds(stop) * 1000.0, seconds(EX.t_end - t_last));

    if (boot_patterns < 3u || boot_length > EX_BOOT_RUN_MS) {
        ok = false;
        snprintf(g_why + strlen(g_why), sizeof g_why - strlen(g_why),
                 "[boot] the power-on run held %u pattern(s) and lasted %.0f ms (it must run, and end within %.0f ms); ",
                 boot_patterns, seconds(boot_length) * 1000.0, seconds(EX_BOOT_RUN_MS) * 1000.0);
    }
    if (rate < 1.5 * EX_OLD_PATTERNS_PER_S) {
        ok = false;
        snprintf(g_why + strlen(g_why), sizeof g_why - strlen(g_why),
                 "[rate] %.2f patterns/s is less than 1.5 x the old %.2f; ", rate, EX_OLD_PATTERNS_PER_S);
    }
    if (unread > SEC(1)) {
        ok = false;
        snprintf(g_why + strlen(g_why), sizeof g_why - strlen(g_why),
                 "[guard] the input was left unchecked for %.0f ms; ", seconds(unread) * 1000.0);
    }
    if (EX.input_late || !stopped || stop > EX_STOP_LIMIT) {
        ok = false;
        snprintf(g_why + strlen(g_why), sizeof g_why - strlen(g_why),
                 "[input] an input applied mid-run did not stop the exercise within 1 s (%s); ",
                 EX.input_late ? "no guard read followed the window, so none could notice it"
                               : (stopped ? "it stopped late" : "it was still stepping at the end"));
    }
    return ok ? T_PASS : T_FAIL;
}

/* ------------------------------------------------------------------------------------------------ */
/* Panel tests: what the display controllers were told                                              */

#define VFD_MAIN 0u
#define VFD_SECOND 1u
#define VFD_FLAG_ADDR 0x1Bu    /* vfd.h:16 */
#define VFD_FLAG_MINUS 0x20u   /* vfd.h:17 */
#define VFD_TEXT_CELLS 9u      /* vfd.h:14 */
#define VFD_LAMP_CELLS 10u     /* annun_table (tables.c:158-163): the lamp bytes are 0x02, 0x05 ... 0x1D, the third byte of cells 0-9 */
#define VFD_DIM_CODES 8u

#define EE_SET_BRIGHT 0x0A84u    /* src/eeprom_map.h:201 */
#define EE_BOOT_XFER 0x0E74u     /* src/eeprom_map.h:253 */
#define EE_BOOT_XFER_ACTIVE 0x5Au /* src/eeprom_map.h:254 */

#define MENU_MESSAGE_MS 1000u /* menu.c:78: the SAVED message */
#define BRIGHT_ITEM_INDEX 14u /* position of BRIGHT in sys_items[], menu.c:374-393 */

/* Glyphs, from src/tables.c:19-26: a cell is the high byte at address 3n and the low byte at 3n+1. */
static const uint16_t GLYPH_DIGIT[10] = {0x2362, 0x0808, 0x8361, 0x0343, 0xA203, 0xA143, 0xA163, 0x0508, 0xA363, 0xA343};
static const uint16_t GLYPH_BRIGHT[6] = {0x0B4B /* B */, 0xA325 /* R */, 0x0948 /* I */,
                                         0x2163 /* G */, 0xA223 /* H */, 0x0908 /* T */};

static void vfd_snapshot(vfd_ram_t *ram) { *ram = S.vfd.ram; }

static uint16_t ram_glyph(const vfd_ram_t *ram, unsigned dev, unsigned cell) {
    return (uint16_t)(((unsigned)ram->b[dev][3u * cell] << 8) | ram->b[dev][3u * cell + 1u]);
}

/* Cells first .. first+count-1 of a controller whose glyph is not blank. */
static unsigned ram_cells_lit(const vfd_ram_t *ram, unsigned dev, unsigned first, unsigned count) {
    unsigned n = 0;

    for (unsigned c = first; c < first + count; c++) {
        n += ram_glyph(ram, dev, c) != 0u;
    }
    return n;
}

/* Lamp bytes that are not zero, on both controllers. */
static unsigned ram_lamps_lit(const vfd_ram_t *ram) {
    unsigned n = 0;

    for (unsigned dev = 0; dev < 2u; dev++) {
        for (unsigned c = 0; c < VFD_LAMP_CELLS; c++) {
            n += ram->b[dev][3u * c + 2u] != 0u;
        }
    }
    return n;
}

/* True when the first n cells of the text line hold exactly want[] and the rest of the nine are blank. */
static bool ram_line_is(const vfd_ram_t *ram, unsigned dev, const uint16_t *want, unsigned n) {
    for (unsigned c = 0; c < VFD_TEXT_CELLS; c++) {
        if (ram_glyph(ram, dev, c) != (c < n ? want[c] : 0u)) {
            return false;
        }
    }
    return true;
}

static const char *ram_line_text(const vfd_ram_t *ram, unsigned dev) {
    static char buf[4][VFD_TEXT_CELLS * 5u + 1u];
    static unsigned next;
    char *out = buf[next++ % 4u];

    out[0] = '\0';
    for (unsigned c = 0; c < VFD_TEXT_CELLS; c++) {
        snprintf(out + strlen(out), sizeof buf[0] - strlen(out), "%s%04X", c ? " " : "", ram_glyph(ram, dev, c));
    }
    return out;
}

/* The last display-control byte (0x88 to 0x8F) sent to controller dev by the frames after number `mark`
 * (S.vfd.seq at the time), or -1 when there was none. */
static int last_display_on(uint64_t mark, unsigned dev) {
    int last = -1;

    for (size_t i = 0; i < S.vfd.nctl; i++) {
        if (S.vfd.ctl[i].seq > mark && S.vfd.ctl[i].dev == dev) {
            last = S.vfd.ctl[i].byte;
        }
    }
    return last;
}

static run_status_t key_tap_group(unsigned group, uint8_t scan_bit) {
    S.keys[group] = scan_bit;
    run_status_t r = run_for(MS(KEY_HOLD_MS));

    S.keys[group] = 0;
    return r == RUN_OK ? run_for(MS(KEY_GAP_MS)) : r;
}

#define TAP(group, bit)                                                                                  \
    do {                                                                                                 \
        if (key_tap_group((group), (bit)) != RUN_OK) {                                                   \
            FAILF("%s", stopped_text());                                                                 \
        }                                                                                                \
    } while (0)

/* The application is running its measurement loop and has settled. */
static result_t measuring_start(void) {
    unsigned want = 5;
    run_status_t r = run_until(pred_rdata, &want, SEC(30));

    REQUIRE(r == RUN_OK, "the measurement loop never started: %s", run_text(r));
    r = run_for(SEC(2));
    REQUIRE(r == RUN_OK, "%s", stopped_text());
    return T_PASS;
}

/* The digit shown by the open BRIGHT choice editor: 1 to 8, or 0 when the main line is anything but exactly
 * one such digit in cell 0 with the other eight cells blank. The value blinks, visible for 310 ms of every
 * 600 (menu.c:75-76), so a blank line is looked at again 300 ms later, which falls in a visible phase
 * wherever the first look fell. */
static unsigned bright_digit(vfd_ram_t *ram) {
    vfd_snapshot(ram);
    if (ram_cells_lit(ram, VFD_MAIN, 0, VFD_TEXT_CELLS) == 0u) {
        if (run_for(MS(300)) != RUN_OK) {
            return 0;
        }
        vfd_snapshot(ram);
    }
    for (unsigned d = 1; d <= VFD_DIM_CODES; d++) {
        if (ram_glyph(ram, VFD_MAIN, 0) == GLYPH_DIGIT[d]) {
            return ram_cells_lit(ram, VFD_MAIN, 1, VFD_TEXT_CELLS - 1u) == 0u ? d : 0u;
        }
    }
    return 0;
}

/* 20. BRIGHT offers the chip's eight dimming codes, shown as 1 to 8, drives the matching display-control byte
 * to both controllers, and keeps the choice across a reset. */
static result_t test_20(void) {
    static const uint8_t to_sys_items[] = {ZK_SHIFT, ZK_MENU, ZK_NEXT, ZK_NEXT, ZK_DOWN};
    vfd_ram_t ram;

    app_open();
    result_t t = measuring_start();

    if (t != T_PASS) {
        return t;
    }
    uint8_t *ee = eeprom_array();

    REQUIRE(ee != NULL, "the simulated EEPROM array could not be found");
    /* An unset or out-of-range byte means the brightest level (menu.c:912-917, vfd.h:52). */
    const unsigned initial = ee[EE_SET_BRIGHT] > VFD_DIM_CODES - 1u ? VFD_DIM_CODES - 1u : ee[EE_SET_BRIGHT];
    const char *err = key_taps(to_sys_items, sizeof to_sys_items);

    REQUIRE(err == NULL, "%s", err);
    for (unsigned i = 0; i < BRIGHT_ITEM_INDEX; i++) {
        TAP(0, ZK_NEXT);
    }
    vfd_snapshot(&ram);
    REQUIRE(ram_line_is(&ram, VFD_MAIN, GLYPH_BRIGHT, 6), "SYS item %u does not read BRIGHT, the main line is: %s",
            BRIGHT_ITEM_INDEX, ram_line_text(&ram, VFD_MAIN));
    TAP(0, ZK_DOWN);
    unsigned digit = bright_digit(&ram);

    REQUIRE(digit == initial + 1u, "the editor opened showing %u (main line %s), expected %u for stored code %u", digit,
            ram_line_text(&ram, VFD_MAIN), initial + 1u, initial);
    REQUIRE(ram_line_is(&ram, VFD_SECOND, GLYPH_BRIGHT, 6), "the second line does not name BRIGHT: %s",
            ram_line_text(&ram, VFD_SECOND));

    /* 1. Eight steps. Eight presses of NEXT must visit eight different digits, each one a single character
     * from 1 to 8 (a fraction such as 10/16 occupies more cells and reads as 0), step by one, and wrap from 8
     * to 1. Five choices would repeat after five presses, sixteen would reach a two-character value. */
    unsigned seen[VFD_DIM_CODES + 1u] = {0};
    bool wrapped = false;
    unsigned prev = digit;

    seen[prev]++;
    for (unsigned press = 1; press <= VFD_DIM_CODES; press++) {
        TAP(0, ZK_NEXT);
        digit = bright_digit(&ram);
        REQUIRE(digit != 0u, "after %u presses the line is not a single digit 1-8: %s", press,
                ram_line_text(&ram, VFD_MAIN));
        REQUIRE(digit == (prev == VFD_DIM_CODES ? 1u : prev + 1u), "press %u went from %u to %u", press, prev, digit);
        wrapped |= prev == VFD_DIM_CODES;
        seen[digit]++;
        prev = digit;
    }
    unsigned distinct = 0;

    for (unsigned d = 1; d <= VFD_DIM_CODES; d++) {
        distinct += seen[d] != 0u;
    }
    REQUIRE(distinct == VFD_DIM_CODES, "%u distinct choices seen in a full turn, expected %u", distinct, VFD_DIM_CODES);
    REQUIRE(wrapped, "stepping past 8 never wrapped to 1");
    REQUIRE(digit == initial + 1u, "after eight presses the line shows %u, expected the starting %u", digit,
            initial + 1u);
    TAP(0, ZK_PREV);
    digit = bright_digit(&ram);
    REQUIRE(digit == (initial == 0u ? VFD_DIM_CODES : initial), "PREV from %u gave %u", initial + 1u, digit);
    TAP(0, ZK_NEXT);

    /* 2. A choice that is only previewed is dimmed on the wire at once and undone when the editor is left with
     * UP (menu.c:1975-1979, 1988-1994). */
    for (unsigned i = 0; i < 3u; i++) {
        TAP(0, ZK_NEXT);
    }
    const unsigned previewed = (initial + 3u) % VFD_DIM_CODES;

    for (unsigned dev = 0; dev < 2u; dev++) {
        REQUIRE(last_display_on(0, dev) == (int)(VFD_DISPLAY_BASE + previewed),
                "controller %u: the previewed choice %u did not reach the wire (last display-control byte 0x%02X)", dev,
                previewed + 1u, (unsigned)last_display_on(0, dev));
    }
    uint64_t mark = S.vfd.seq;

    TAP(0, ZK_UP);
    REQUIRE(run_for(MS(100)) == RUN_OK, "%s", stopped_text());
    for (unsigned dev = 0; dev < 2u; dev++) {
        REQUIRE(last_display_on(mark, dev) == (int)(VFD_DISPLAY_BASE + initial),
                "controller %u: leaving without ENTER left the dimming at 0x%02X, expected the stored 0x%02X", dev,
                (unsigned)last_display_on(mark, dev), VFD_DISPLAY_BASE + initial);
    }

    /* 3. Every choice, entered. The last one is neither the factory default (code 7) nor the stored value the
     * session started with, so the reset below can tell a kept choice from an untouched one. */
    const unsigned last = initial == 2u ? 3u : 2u;
    unsigned order[VFD_DIM_CODES];
    unsigned n = 0;

    for (unsigned k = 0; k < VFD_DIM_CODES; k++) {
        if (k != last) {
            order[n++] = k;
        }
    }
    order[n++] = last;
    unsigned stored = initial;

    for (unsigned i = 0; i < VFD_DIM_CODES; i++) {
        const unsigned k = order[i];
        const unsigned steps = (k + VFD_DIM_CODES - stored) % VFD_DIM_CODES;
        const uint8_t want = (uint8_t)(VFD_DISPLAY_BASE + k);

        TAP(0, ZK_DOWN);
        digit = bright_digit(&ram);
        REQUIRE(digit == stored + 1u, "choice %u: the editor opened showing %u, expected the stored %u (%s)", k + 1u, digit,
                stored + 1u, ram_line_text(&ram, VFD_MAIN));
        for (unsigned s = 0; s < steps; s++) {
            TAP(0, ZK_NEXT);
        }
        digit = bright_digit(&ram);
        REQUIRE(digit == k + 1u, "choice %u: the line shows %u (%s)", k + 1u, digit, ram_line_text(&ram, VFD_MAIN));
        mark = S.vfd.seq;
        TAP(0, ZK_ENTER);
        REQUIRE(run_for(MS(MENU_MESSAGE_MS + 300u)) == RUN_OK, "%s", stopped_text());
        REQUIRE(!S.vfd.ctl_overflow, "the display-control log is full");
        for (unsigned dev = 0; dev < 2u; dev++) {
            unsigned sent = 0;

            for (size_t e = 0; e < S.vfd.nctl; e++) {
                if (S.vfd.ctl[e].seq > mark && S.vfd.ctl[e].dev == dev) {
                    REQUIRE(S.vfd.ctl[e].byte == want,
                            "choice %u, controller %u: display-control byte 0x%02X after ENTER, expected 0x%02X", k + 1u,
                            dev, S.vfd.ctl[e].byte, want);
                    sent++;
                }
            }
            REQUIRE(sent > 0u, "choice %u: ENTER sent no display-control byte to controller %u (expected 0x%02X)", k + 1u,
                    dev, want);
        }
        REQUIRE(ee[EE_SET_BRIGHT] == k, "choice %u: EEPROM 0x%04X holds 0x%02X, expected 0x%02X", k + 1u, EE_SET_BRIGHT,
                ee[EE_SET_BRIGHT], k);
        stored = k;
    }

    /* 4. The stored choice survives a reset: the firmware sends it again at start-up and keeps sending it. */
    REQUIRE(stored == last && stored != VFD_DIM_CODES - 1u && stored != initial, "internal: the last choice is not distinctive");
    mark = S.vfd.seq;
    uint64_t cycles_before_reset = S.avr->cycle;

    avr_reset(S.avr);
    S.base_cycles += cycles_before_reset;
    on_avr_reset();
    S.keys[0] = 0;
    REQUIRE(run_for(SEC(8)) == RUN_OK, "%s", stopped_text());
    REQUIRE(!S.vfd.ctl_overflow, "the display-control log is full");
    ee = eeprom_array();
    REQUIRE(ee != NULL && ee[EE_SET_BRIGHT] == stored, "after the reset EEPROM 0x%04X holds 0x%02X, expected 0x%02X",
            EE_SET_BRIGHT, ee == NULL ? 0u : ee[EE_SET_BRIGHT], stored);
    const uint8_t want = (uint8_t)(VFD_DISPLAY_BASE + stored);

    for (unsigned dev = 0; dev < 2u; dev++) {
        bool found = false;
        unsigned after = 0;

        for (size_t e = 0; e < S.vfd.nctl; e++) {
            if (S.vfd.ctl[e].seq <= mark || S.vfd.ctl[e].dev != dev) {
                continue;
            }
            if (found) {
                REQUIRE(S.vfd.ctl[e].byte == want,
                        "controller %u: display-control 0x%02X after restart, the stored choice is 0x%02X", dev,
                        S.vfd.ctl[e].byte, want);
                after++;
            } else if (S.vfd.ctl[e].byte == want) {
                found = true;
            }
        }
        REQUIRE(found, "controller %u: the firmware never sent the stored dimming code 0x%02X after the reset", dev, want);
        REQUIRE(after > 0u, "controller %u: the stored dimming code was sent once and never again, so nothing shows it holds",
                dev);
    }
    info("8 choices shown 1-8, ENTER sent 0x88-0x8F to both controllers, choice %u (0x%02X) kept after a reset", stored + 1u,
         want);
    return T_PASS;
}

/* The shortcut keys of the three reference editors, all in scan byte 1 (keys.c:44): REL bit 1, DIODE bit 3,
 * ACDC bit 4, pressed after SHIFT (keys.c:494-533). */
typedef struct {
    const char *name;
    uint8_t scan_bit;
} shortcut_t;

static const shortcut_t SHORTCUTS[3] = {
    {"dB REF, SHIFT+ACDC", 0x10u},
    {"dBm REF, SHIFT+DIODE", 0x08u},
    {"PERC REF, SHIFT+REL", 0x02u},
};

/* The converter code that reads -0.05 V on DC V range 4 (scale 0.25, tables.c:187; 2.86e-7 V per count,
 * ZS_VOLTS_PER_COUNT). It stays negative under every mode the three shortcuts turn on: 20 log10(0.05) = -26 dB,
 * 10 log10(0.05^2 / 75 / 0.001) = -14.8 dBm (meas.c:1172-1196) and 100 (-0.05 - 1) / 1 = -105 % (meas.c:1227-1236,
 * reference 1.0, main.c:220-222). */
#define NEGATIVE_INPUT_CODE (-700000)
#define SHORTCUT_SETTLE SEC(4)

static result_t shortcut_case(const shortcut_t *sc, bool confirm) {
    vfd_ram_t before;
    vfd_ram_t open;
    vfd_ram_t after;

    app_open();
    result_t t = measuring_start();

    if (t != T_PASS) {
        return t;
    }
    /* Manual ranging at the lowest DC V range, as zs_run() does (keys.c:330-346), then a negative input. */
    S.ads.code = 0;
    for (unsigned i = 0; i < 6u; i++) {
        TAP(0, ZK_DOWN);
    }
    S.ads.code = NEGATIVE_INPUT_CODE;
    REQUIRE(run_for(SHORTCUT_SETTLE) == RUN_OK, "%s", stopped_text());

    /* The reading is negative and drawn: without this a clear flag proves nothing. */
    vfd_snapshot(&before);
    REQUIRE((before.b[VFD_MAIN][VFD_FLAG_ADDR] & VFD_FLAG_MINUS) != 0u,
            "%s: the minus flag is not set on a negative reading (flag byte 0x%02X), the test cannot observe it clearing",
            sc->name, before.b[VFD_MAIN][VFD_FLAG_ADDR]);
    REQUIRE(ram_cells_lit(&before, VFD_MAIN, 0, 6) >= 3u, "%s: no reading is drawn before the editor opens (%s)", sc->name,
            ram_line_text(&before, VFD_MAIN));
    REQUIRE(ram_lamps_lit(&before) > 0u, "%s: no measurement lamp is lit before the editor opens", sc->name);

    TAP(0, ZK_SHIFT);
    TAP(1, sc->scan_bit);
    REQUIRE(run_for(MS(300)) == RUN_OK, "%s", stopped_text());
    vfd_snapshot(&open);
    REQUIRE(ram_cells_lit(&open, VFD_SECOND, 0, VFD_TEXT_CELLS) > 0u,
            "%s: the editor did not open (no caption on the second line, main line %s)", sc->name,
            ram_line_text(&open, VFD_MAIN));
    REQUIRE(memcmp(open.b[VFD_MAIN], before.b[VFD_MAIN], VFD_FLAG_ADDR) != 0, "%s: the main line is unchanged by the editor",
            sc->name);
    REQUIRE((open.b[VFD_MAIN][VFD_FLAG_ADDR] & VFD_FLAG_MINUS) == 0u,
            "%s: the minus flag is still set while the editor is open (flag byte 0x%02X)", sc->name,
            open.b[VFD_MAIN][VFD_FLAG_ADDR]);
    REQUIRE(ram_lamps_lit(&open) == 0u, "%s: %u measurement lamp byte(s) are lit while the editor is open", sc->name,
            ram_lamps_lit(&open));

    if (confirm) {
        TAP(0, ZK_ENTER);
        REQUIRE(run_for(MS(MENU_MESSAGE_MS)) == RUN_OK, "%s", stopped_text());
    } else {
        TAP(0, ZK_MENU);
    }
    REQUIRE(run_for(SHORTCUT_SETTLE) == RUN_OK, "%s", stopped_text());
    vfd_snapshot(&after);
    REQUIRE(memcmp(after.b[VFD_MAIN], open.b[VFD_MAIN], VFD_RAM_BYTES) != 0,
            "%s, %s: the panel still shows the editor (main line %s)", sc->name, confirm ? "confirmed" : "cancelled",
            ram_line_text(&after, VFD_MAIN));
    REQUIRE((after.b[VFD_MAIN][VFD_FLAG_ADDR] & VFD_FLAG_MINUS) != 0u,
            "%s, %s: the minus flag did not come back (flag byte 0x%02X)", sc->name, confirm ? "confirmed" : "cancelled",
            after.b[VFD_MAIN][VFD_FLAG_ADDR]);
    REQUIRE(ram_cells_lit(&after, VFD_MAIN, 0, 6) >= 3u && ram_cells_lit(&after, VFD_MAIN, 6, 3) > 0u,
            "%s, %s: the reading is not drawn again (%s)", sc->name, confirm ? "confirmed" : "cancelled",
            ram_line_text(&after, VFD_MAIN));
    REQUIRE(ram_lamps_lit(&after) > 0u, "%s, %s: no measurement lamp came back", sc->name, confirm ? "confirmed" : "cancelled");
    if (!confirm) {
        REQUIRE(memcmp(&after, &before, sizeof after) == 0,
                "%s, cancelled: the panel is not what it was before the editor (main %s, was %s)", sc->name,
                ram_line_text(&after, VFD_MAIN), ram_line_text(&before, VFD_MAIN));
    }
    return T_PASS;
}

/* 21. The reference editors opened from the SHIFT layer clear the measurement's minus sign and lamps while they
 * are open, and both ways out redraw the measurement, the minus sign included. */
static result_t test_21(void) {
    for (unsigned i = 0; i < 3u; i++) {
        for (unsigned confirm = 0; confirm < 2u; confirm++) {
            result_t t = shortcut_case(&SHORTCUTS[i], confirm != 0u);

            if (t != T_PASS) {
                return t;
            }
            info("%s: editor clears minus flag and lamps, %s redraws the reading with the minus flag", SHORTCUTS[i].name,
                 confirm ? "ENTER" : "MENU");
        }
    }
    return T_PASS;
}

/* What a refused transfer must leave behind. x is the sender's result, pos the receive index. statuses lists
 * the FAIL letters that count as the right refusal. The incomplete flag stays set, the application region is
 * never executed, the loader keeps asking for a download, and after a reset with no key held the application
 * is still not started. */
static result_t refused_transfer(const xm_result_t *x, size_t pos, const char *statuses) {
    uint32_t diff;

    REQUIRE(!x->ok, "the loader accepted a damaged transfer");
    REQUIRE(x->bad_response == XM_CAN, "the loader answered %d (0x%02X), expected CAN", x->bad_response,
            x->bad_response & 0xFF);
    run_status_t r = wait_rx_str("FAIL ", pos, SEC(3), &pos);

    REQUIRE(r == RUN_OK, "no \"FAIL\" line after the damaged transfer: %s", run_text(r));
    int status = wait_rx_byte(pos, SEC(1));

    REQUIRE(status > 0 && strchr(statuses, status) != NULL, "FAIL status %d is not one of \"%s\"", status, statuses);
    pos++;
    const uint8_t *ee = eeprom_array();

    REQUIRE(ee != NULL, "the simulated EEPROM array could not be found");
    REQUIRE(ee[EE_BOOT_XFER] == EE_BOOT_XFER_ACTIVE, "the incomplete flag 0x%04X was cleared (0x%02X) by a refused transfer",
            EE_BOOT_XFER, ee[EE_BOOT_XFER]);
    REQUIRE(memmem(S.rx_buf, S.rx_len, "OK\r\n", 4) == NULL, "the loader reported success");
    r = wait_rx_str("C", pos, SEC(3), &pos);
    REQUIRE(r == RUN_OK, "the loader did not resume requesting a download: %s", run_text(r));
    REQUIRE(S.app_steps == 0, "the application region was executed (%" PRIu64 " steps)", S.app_steps);
    REQUIRE(boot_unchanged(&diff), "boot loader area changed at 0x%05X", diff);

    uint64_t cycles_before_reset = S.avr->cycle;

    avr_reset(S.avr);
    S.base_cycles += cycles_before_reset;
    on_avr_reset();
    S.keys[0] = 0;
    S.app_steps = 0;
    r = run_for(SEC(10)); /* past ENTER_WAIT_TICKS, about 4.0 s */
    REQUIRE(r == RUN_OK, "%s", stopped_text());
    REQUIRE(S.app_steps == 0, "the loader started the refused image after a power cycle (%" PRIu64 " steps)", S.app_steps);
    return T_PASS;
}

/* 22. A transfer that ends cleanly but short is refused: the trailer claims more application bytes than were
 * sent. XMODEM carries no length, so before the trailer this was accepted and started (PROTOCOL_CHANGES G.5). */
static result_t test_22(void) {
    size_t pos;
    const size_t sent = 8 * FRAME_BYTES;

    REQUIRE(g_app_len > sent, "internal: the application image (%u bytes) is not longer than the truncated transfer", g_app_len);
    boot_open(false, KEY_ARROWS);
    boot_snapshot();
    const uint8_t *ee = eeprom_array();

    REQUIRE(ee != NULL && ee[EE_BOOT_XFER] != EE_BOOT_XFER_ACTIVE, "internal: the incomplete flag is set before the transfer");
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        return t;
    }
    S.app_steps = 0;
    xm_result_t x = xm_send_stream(app_image(), sent, g_app_len, crc16_xmodem(app_image(), g_app_len), &pos);

    t = refused_transfer(&x, pos, "IC");
    if (t == T_PASS) {
        info("%zu of %u bytes sent, trailer claims %u: refused, flag kept, application not run, not started after a reset", sent,
             g_app_len, g_app_len);
    }
    return t;
}

/* 23. A complete transfer whose trailer carries the wrong CRC is refused with status 'C'. */
static result_t test_23(void) {
    size_t pos;
    const size_t len = 4 * FRAME_BYTES;

    boot_open(false, KEY_ARROWS);
    boot_snapshot();
    const uint8_t *ee = eeprom_array();

    REQUIRE(ee != NULL && ee[EE_BOOT_XFER] != EE_BOOT_XFER_ACTIVE, "internal: the incomplete flag is set before the transfer");
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        return t;
    }
    S.app_steps = 0;
    xm_result_t x = xm_send_stream(app_image(), len, (uint32_t)len, (uint16_t)(crc16_xmodem(app_image(), len) ^ 0x0001u), &pos);

    t = refused_transfer(&x, pos, "C");
    if (t == T_PASS) {
        info("%zu bytes sent whole, trailer CRC off by one bit: refused with FAIL C, flag kept, not started after a reset", len);
    }
    return t;
}

/* ------------------------------------------------------------------------------------------------ */

typedef struct {
    int num;
    const char *name;
    result_t (*fn)(void);
} test_t;

/* ORIGINAL_BUGS #100, the boot loader's serial link. The original ran 12 MHz / (16 x (18+1)) = 39473.7 baud
 * against a host's 38400 -- +2.8 %, the same arithmetic defect as #90 in the application -- and framed with
 * two stop bits. Both are now fixed, and this test pins the registers so neither can come back.
 *
 * The figures: with U2X the divisor is 8 instead of 16, so UBRR1 = 38 gives 12e6 / (8 x 39) = 38461.5 baud,
 * which is +0.16 % against 38400. A UART tolerates a few per cent, so +2.8 % did not stop the original
 * working -- which is exactly why a defect like this survives unless something asserts the number.
 *
 * The registers are read after the loader has reached its download prompt, so they are the values it
 * actually configured, not power-on defaults. */
static result_t test_24(void) {
    size_t pos;

    boot_open(false, KEY_ARROWS);
    result_t t = boot_to_download(&pos);

    if (t != T_PASS) {
        return t;
    }

    const uint8_t *d = S.avr->data;
    const unsigned ubrr = (unsigned)d[ADDR_UBRR1L] | ((unsigned)(d[ADDR_UBRR1H] & 0x0Fu) << 8);
    const bool u2x = (d[ADDR_UCSR1A] & 0x02u) != 0;
    const unsigned stop = 1u + ((d[ADDR_UCSR1C] >> 3) & 1u);
    const unsigned ucsz = ((d[ADDR_UCSR1C] >> 1) & 3u) | (unsigned)(((d[ADDR_UCSR1B] >> 2) & 1u) << 2);

    REQUIRE(u2x, "the loader did not set U2X1: the divisor is 16, not 8");
    REQUIRE(ubrr == 38u, "UBRR1 is %u, expected 38", ubrr);
    REQUIRE(stop == 1u, "the loader framed with %u stop bits, expected 1", stop);
    REQUIRE(ucsz == 3u, "the loader set UCSZ %u, expected 3 (8 data bits)", ucsz);

    /* The arithmetic the two constants above stand for, asserted as a number so a future edit that keeps
     * U2X but changes UBRR is caught too. */
    const double baud = 12000000.0 / (8.0 * (double)(ubrr + 1u));
    const double error_pct = (baud - 38400.0) / 38400.0 * 100.0;

    REQUIRE(error_pct > -1.0 && error_pct < 1.0, "the loader runs at %.1f baud, %+.2f %% from 38400", baud,
            error_pct);

    /* The debug echoes the original emitted ("o", "@", "n") are gone. This cannot be checked by scanning
     * the whole stream for those letters -- the banner itself is "Press ENTER to download, others run
     * app.", which contains both an 'o' and an 'n'. The first version of this test did exactly that and
     * failed on the banner's own text.
     *
     * What is checked instead is the part of the stream where only the protocol may speak: once the loader
     * is at its download prompt it sends the XMODEM-CRC request 'C' and nothing else until a frame
     * arrives. A stray echo there would corrupt what a host is parsing. */
    size_t prompt_from = pos;

    (void)run_for(SEC(2));
    REQUIRE(S.rx_len > prompt_from, "the loader sent nothing further at its download prompt");
    for (size_t i = prompt_from; i < S.rx_len; i++) {
        const uint8_t c = S.rx_buf[i];

        REQUIRE(c == (uint8_t)'C', "at the download prompt the loader sent 0x%02X ('%c') at offset %zu; only "
                                   "the XMODEM-CRC request 'C' belongs here",
                c, (c >= 0x20u && c < 0x7Fu) ? (char)c : '.', i);
    }
    return T_PASS;
}

static const test_t TESTS[] = {
    {1, "boot: no key held, application starts, no banner", test_01},
    {2, "boot: arrows+ENTER, full image over XMODEM-CRC, flash exact, app runs", test_02},
    {3, "boot: image reaching 0x1F800 refused, loader area unchanged", test_03},
    {4, "boot: corrupted block NAKed, retransmission accepted", test_04},
    {5, "boot: stalled transfer times out, resynchronises, partial image not run", test_05},
    {6, "boot: first block without a valid reset vector refused", test_06},
    {7, "boot: final partial page padded with 0xFF", test_07},
    {8, "app: boots to measurement loop; no vendor byte changes, power-up count +1", test_08},
    {9, "app: *IDN? answers", test_09},
    {10, "app: :SERV:UNLOCK \"6666\" succeeds", test_10},
    {11, "app: :SCAL:COEF? answers after unlock", test_11},
    {12, "app: error does not stall the next command", test_12},
    {13, "app: empty lines are ignored", test_13},
    {14, "app: hung converter -> no reset, keeps retrying, reports an error", test_14},
    {15, "app: :thid:cal:coun ? answers the EEPROM's calibration count", test_15},
    {16, "zero cal: SYS item 7 stores 5 DC V and 6 ohm user zeros, in order, nothing else", test_16},
    {17, "zero cal: the front end is restored after every capture", test_17},
    {18, "zero cal: one capture per pass, key scan, display and *IDN? keep being serviced", test_18},
    {19, "relay exercise: patterns step at least 1.5x faster, guard read at least once a second, input stops it", test_19},
    {20, "panel: BRIGHT offers 8 choices shown 1-8, ENTER sends 0x88-0x8F to both controllers, choice kept after reset", test_20},
    {21, "panel: SHIFT reference editors clear the minus flag and lamps, both exits redraw the reading and the flag", test_21},
    {22, "boot: truncated transfer (trailer claims more than was sent) refused, flag kept, app not run", test_22},
    {23, "boot: complete transfer with a wrong trailer CRC refused with FAIL C, flag kept, app not run", test_23},
    {24, "boot: link is 38461.5 baud (U2X, UBRR1 38), 8 data bits, 1 stop bit, no debug echoes", test_24},
};

int main(int argc, char **argv) {
    int only = 0;

    setvbuf(stdout, NULL, _IOLBF, 0);
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--app") && i + 1 < argc) {
            g_app_path = argv[++i];
        } else if (!strcmp(argv[i], "--boot") && i + 1 < argc) {
            g_boot_path = argv[++i];
        } else if (!strcmp(argv[i], "--eeprom") && i + 1 < argc) {
            g_eeprom_path = argv[++i];
        } else if (!strcmp(argv[i], "--only") && i + 1 < argc) {
            only = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--reset-pc") && i + 1 < argc) {
            g_reset_pc_override = (int)strtol(argv[++i], NULL, 0);
        } else {
            fprintf(stderr, "unknown or incomplete option %s\n", argv[i]);
            return 2;
        }
    }
    if (!g_app_path || !g_boot_path || !g_eeprom_path) {
        fprintf(stderr, "usage: %s --app app.elf --boot boot.elf --eeprom eeprom.bin [--only N] [--reset-pc ADDR]\n", argv[0]);
        return 2;
    }
    uint32_t lo;

    if (!load_image(g_app_path, g_app_img, &lo, &g_app_len) || !load_image(g_boot_path, g_boot_img, &g_boot_lo, &g_boot_hi) ||
        !load_file(g_eeprom_path, g_ee_file, EEPROM_BYTES)) {
        return 2;
    }
    (void)lo;
    printf("app image: 0x00000-0x%05X (%u bytes), boot image: 0x%05X-0x%05X, EEPROM image %u bytes, echo %s\n",
           g_app_len - 1, g_app_len, g_boot_lo, g_boot_hi - 1, EEPROM_BYTES, g_ee_file[EE_SET_ECHO] ? "on" : "off");

    unsigned pass = 0, fail = 0, skip = 0, ran = 0;

    for (size_t i = 0; i < sizeof TESTS / sizeof TESTS[0]; i++) {
        const test_t *t = &TESTS[i];

        if (only != 0 && only != t->num) {
            continue;
        }
        ran++;
        g_why[0] = '\0';
        printf("---- T%02d %s\n", t->num, t->name);
        result_t r = t->fn();
        const char *label = r == T_PASS ? "PASS" : r == T_FAIL ? "FAIL" : "SKIPPED";

        printf("[%s] T%02d %s%s%s\n", label, t->num, t->name, g_why[0] ? ": " : "", g_why);
        if (r == T_PASS) {
            pass++;
        } else if (r == T_FAIL) {
            fail++;
        } else {
            skip++;
        }
        session_close();
    }
    printf("SUMMARY: %u tests run: %u PASS, %u FAIL, %u SKIPPED\n", ran, pass, fail, skip);
    return fail == 0 ? 0 : 1;
}
