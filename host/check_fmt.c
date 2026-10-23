/* check_fmt.c -- host driver for the fmt module. Not part of the firmware image.
 *
 * Built and run by check_fmt.py on the development machine. It compiles the real src/fmt/fmt.c
 * (the non-AVR branch of its one preprocessor guard) and answers one request per input line, so that the
 * Python side can compare every answer with an exact oracle built from Python's decimal module.
 *
 * Request lines, one per line on stdin:
 *   E <hexbits> <decimals>          fmt_float_e_prec
 *   X <hexbits> <decimals>          fmt_float_E_prec
 *   F <hexbits> <width> <decimals>  fmt_fixed
 *   I <int32>                       fmt_int
 *   W <n>                           fmt_pow10f, answered as hex bits
 *   P <text>                        fmt_parse_float, answered as "<status> <hexbits>"
 *   L <text>                        fmt_parse_long, answered as "<status> <value>"
 *
 * Every answer is one line. A formatter answer is the exact bytes the function stored, wrapped in | marks
 * so that leading and trailing spaces survive the round trip.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/fmt/fmt.c"

static float from_bits(unsigned long u) {
    union {
        float f;
        uint32_t u;
    } b;

    b.u = (uint32_t)u;
    return b.f;
}

static unsigned long to_bits(float f) {
    union {
        float f;
        uint32_t u;
    } b;

    b.f = f;
    return (unsigned long)b.u;
}

/* The buffer is deliberately bigger than any answer, and a sentinel after it is checked every time so that
 * an overrun in the module under test is caught here rather than silently passing. */
#define BUF 64
#define GUARD 0xA5

static char buf[BUF + 8];

static void reset(void) {
    memset(buf, GUARD, sizeof buf);
}

static void answer(size_t n) {
    size_t i;

    for (i = BUF; i < sizeof buf; i++) {
        if ((unsigned char)buf[i] != GUARD) {
            printf("OVERRUN\n");
            return;
        }
    }
    if (n > BUF) {
        printf("BADLEN %zu\n", n);
        return;
    }
    putchar('|');
    for (i = 0; i < n; i++) {
        putchar(buf[i]);
    }
    printf("|\n");
}

int main(void) {
    char line[256];

    while (fgets(line, sizeof line, stdin) != NULL) {
        char kind = line[0];
        char *rest = line + 1;
        size_t len = strlen(line);

        while (len > 0u && (line[len - 1u] == '\n' || line[len - 1u] == '\r')) {
            line[--len] = '\0';
        }
        if (len == 0u) {
            continue;
        }
        while (*rest == ' ') {
            rest++;
        }
        reset();

        if (kind == 'E' || kind == 'X') {
            unsigned long u;
            unsigned d;

            if (sscanf(rest, "%lx %u", &u, &d) != 2) {
                printf("BADREQ\n");
                continue;
            }
            answer(kind == 'E' ? fmt_float_e_prec(buf, BUF, from_bits(u), (uint8_t)d)
                               : fmt_float_E_prec(buf, BUF, from_bits(u), (uint8_t)d));
        } else if (kind == 'F') {
            unsigned long u;
            unsigned w;
            unsigned d;

            if (sscanf(rest, "%lx %u %u", &u, &w, &d) != 3) {
                printf("BADREQ\n");
                continue;
            }
            answer(fmt_fixed(buf, BUF, from_bits(u), (uint8_t)w, (uint8_t)d));
        } else if (kind == 'I') {
            long v;

            if (sscanf(rest, "%ld", &v) != 1) {
                printf("BADREQ\n");
                continue;
            }
            answer(fmt_int(buf, BUF, (int32_t)v));
        } else if (kind == 'W') {
            int n;

            if (sscanf(rest, "%d", &n) != 1) {
                printf("BADREQ\n");
                continue;
            }
            printf("%08lx\n", to_bits(fmt_pow10f((int8_t)n)));
        } else if (kind == 'P') {
            const char *end = NULL;
            float v = 0.0f;
            fmt_status_t st = fmt_parse_float(rest, &end, &v);

            printf("%d %08lx %d\n", (int)st, to_bits(v), (int)(end - rest));
        } else if (kind == 'L') {
            const char *end = NULL;
            int32_t v = 0;
            fmt_status_t st = fmt_parse_long(rest, &end, &v);

            printf("%d %ld %d\n", (int)st, (long)v, (int)(end - rest));
        } else {
            printf("BADREQ\n");
        }
        fflush(stdout);
    }
    return 0;
}
