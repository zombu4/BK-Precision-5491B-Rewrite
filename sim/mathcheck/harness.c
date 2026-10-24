/* simavr harness: loads an ELF, runs until the trailer 0xDEADBEEF has arrived on UART0, writes the bytes to a file.
 * Usage: harness mathcheck.elf out.bin */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "avr_uart.h"
#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_irq.h"

static uint8_t *buf;
static size_t len, cap;

static void on_byte(avr_irq_t *irq, uint32_t value, void *param) {
    (void)irq; (void)param;
    if (len == cap) { cap = cap ? cap * 2 : 65536; buf = realloc(buf, cap); }
    buf[len++] = (uint8_t)value;
}

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: harness in.elf out.bin\n"); return 2; }
    elf_firmware_t fw;
    memset(&fw, 0, sizeof fw);
    if (elf_read_firmware(argv[1], &fw) != 0) { fprintf(stderr, "cannot read %s\n", argv[1]); return 2; }
    avr_t *avr = avr_make_mcu_by_name("atmega128");
    if (!avr) { fprintf(stderr, "no atmega128 core\n"); return 2; }
    avr_init(avr);
    avr->frequency = 16000000;
    avr_load_firmware(avr, &fw);
    uint32_t flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~(uint32_t)AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), on_byte, NULL);
    for (;;) {
        int st = avr_run(avr);
        if (st == cpu_Done || st == cpu_Crashed) break;
        if (len >= 4 && buf[len-4] == 0xEF && buf[len-3] == 0xBE && buf[len-2] == 0xAD && buf[len-1] == 0xDE) break;
        if (avr->cycle > 4000000000ULL) { fprintf(stderr, "cycle limit\n"); break; }
    }
    fprintf(stderr, "bytes %zu cycles %llu\n", len, (unsigned long long)avr->cycle);
    FILE *f = fopen(argv[2], "wb");
    fwrite(buf, 1, len, f);
    fclose(f);
    return 0;
}
