/* mathcheck: log10f and sqrtf of avr-libc 2.2.0 on an ATmega128, inputs from inputs.h (PROGMEM).
 * Output on UART0: for each input one 8-byte record (input word, result word), little endian,
 * log10f table first, then sqrtf table, then the trailer word 0xDEADBEEF. */
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <stdint.h>
#include "inputs.h"

static void tx(uint8_t b) {
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = b;
}

static void tx32(uint32_t w) {
    tx((uint8_t)w);
    tx((uint8_t)(w >> 8));
    tx((uint8_t)(w >> 16));
    tx((uint8_t)(w >> 24));
}

typedef union {
    float f;
    uint32_t u;
} fu_t;

int main(void) {
    UCSR0B = (1 << TXEN0);
    for (uint16_t i = 0; i < N_LOG; i++) {
        fu_t x, r;
        x.u = pgm_read_dword(&in_log[i]);
        r.f = log10f(x.f);
        tx32(x.u);
        tx32(r.u);
    }
    for (uint16_t i = 0; i < N_SQRT; i++) {
        fu_t x, r;
        x.u = pgm_read_dword(&in_sqrt[i]);
        r.f = sqrtf(x.f);
        tx32(x.u);
        tx32(r.u);
    }
    tx32(0xDEADBEEFUL);
    for (;;) {
    }
}
