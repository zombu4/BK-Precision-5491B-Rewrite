/* reply.c -- reply lines, the error queue and the front-panel LOCAL transition.
 *
 * Replies go out through uart1 and always end with the configured terminator (LF, or CR
 * when settings.tx_terminator is non-zero). Number text comes from fmt.h only.
 *
 * Reporting an error is bounded work. The original blanked the display for 1000 ms in a busy
 * loop with interrupts disabled, which lost every byte sent meanwhile. Here an error is queued, beeped,
 * shown once on the front panel and the rest of the line is dropped; nothing waits.
 */
#include <avr/pgmspace.h>
#include "../fmathf.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../fmt/fmt.h"
#include "../hal/buzzer.h"
#include "../hal/uart1.h"
#include "../state.h"
#include "../tables.h"
#include "../ui/display.h"
#include "scpi.h"

#define ERROR_QUEUE_DEPTH 8u
#define REPLY_RAM_STRING_MAX 50u
#define OVERLOAD_THRESHOLD 2147483520.0f
#define FETCH_PENDING_MAX 255u

/* The fragment of the service lock refusal. It is not one of the original's 14 fragments, so it is not in
 * error_fragment[] (tables.h); SCPI_FRAGMENT_LOCKED is the index one past that table. */
static const char fragment_locked[] PROGMEM = "BUS:LOCKED";

/* The error queue: the oldest error is read first. PROTOCOL_CHANGES G.3 #10 replaces the original's single
 * latched error (0x070F8, RAM M0F55/M0F54) with this queue; when it is full a new error is dropped, so the
 * eight kept are the first eight and the oldest-first order of what :SYSTem:ERRor? returns stays unbroken. */
static uint8_t queue[ERROR_QUEUE_DEPTH]; /* packed (first << 4) | second */
static uint8_t queue_head;
static uint8_t queue_count;

/* FETCh? requests that have not been answered yet; see scpi_fetch_request(). g_state.fetch_pending mirrors
 * "count is not zero" for the code that only needs the flag. */
static uint8_t fetch_count;

/* The flash text of one fragment; an unknown index gives the blank fragment 0. */
static const char *message_text(uint8_t index) {
    if (index == SCPI_FRAGMENT_LOCKED) {
        return fragment_locked;
    }
    if (index >= ERROR_FRAGMENT_COUNT) {
        index = 0u;
    }
    return (const char *)pgm_read_ptr(&error_fragment[index]);
}

static uint8_t pack(uint8_t first, uint8_t second) {
    return (uint8_t)(((unsigned)first << 4) | (second & 0x0Fu));
}

static void record_error(uint8_t first, uint8_t second) {
    uint8_t packed = pack(first, second);

    if (queue_count < ERROR_QUEUE_DEPTH) {
        queue[(uint8_t)(((unsigned)queue_head + queue_count) % ERROR_QUEUE_DEPTH)] = packed;
        queue_count++;
    }
    /* The ERR lamp. It means "an error has happened that you have not
     * acknowledged": lit here, put out by a front-panel key that could have been seen or by the read that
     * empties the queue. #60 defined it as "the queue is non-empty", which is tidier and wrong for a
     * meter with no host -- nothing would ever read the queue, so the lamp stayed on until a reset.
     *
     * The original READS this flag and honours it (0x0F95E tests bit 0 of RAM 0x0F61 and writes the lamp
     * cell), and NOTHING IN THE WHOLE IMAGE EVER SETS IT -- the single bit-0 write is a clear at 0x09636
     * in the start-up initialiser. So the vendor's ERR lamp is dead silicon: wired up, read every frame,
     * and never raised. Every pre-existing firmware bug is fixed, so ours works. */
    g_state.error_indicated = true;
    buzzer_error();
    display_show_text_P(message_text(first));
}

/* The first error of a line wins: later ones come from handlers that found the aftermath of the first. */
void scpi_error_report(uint8_t first, uint8_t second) {
    if (scpi_line_failed()) {
        return;
    }
    record_error(first, second);
    scpi_line_abort();
}

/* THE LAMP AND THE QUEUE MEAN DIFFERENT THINGS, and conflating them was the
 * defect. #60 wired the lamp to "the queue holds something", which left a stand-alone meter with no way to
 * put it out: one framing error on an idle RS-232 line, or one converter timeout, lit ERR and kept it lit
 * until a reset, because nothing was ever going to read the queue -- there is no host.
 *
 * The lamp now means "an error has happened that you have not acknowledged", which is what an ERR lamp on
 * an instrument conventionally means and the only reading that works without a host. It is put out by a
 * front-panel key OR by draining the queue, because both are somebody noticing. The QUEUE is untouched by
 * the acknowledgement: `:SYST:ERR?` still returns every error in order afterwards, so a host that connects
 * later loses nothing. */
void remote_error_acknowledge(void) {
    g_state.error_indicated = false;
}

void remote_error_push(scpi_err_t e) {
    switch (e) {
    case SCPI_ERR_BAD_COMMAND:
        record_error(1u, 2u);
        break;
    case SCPI_ERR_SYNTAX:
        record_error(3u, 4u);
        break;
    case SCPI_ERR_PARAMETER:
        record_error(9u, 11u);
        break;
    case SCPI_ERR_DATA:
        record_error(5u, 4u);
        break;
    case SCPI_ERR_LOCKED:
        record_error(SCPI_FRAGMENT_LOCKED, 2u);
        break;
    default:
        return;
    }
    scpi_line_abort();
}

/* Takes the oldest queued error; false when none is queued. */
static bool queue_pop(uint8_t *packed) {
    if (queue_count == 0u) {
        return false;
    }
    *packed = queue[queue_head];
    queue_head = (uint8_t)((queue_head + 1u) % ERROR_QUEUE_DEPTH);
    queue_count--;
    if (queue_count == 0u) {
        g_state.error_indicated = false; /* the last one has been read: the lamp goes out */
    }
    return true;
}

/* :SYSTem:ERRor? -- the text of the oldest queued error, which is then removed; "NO ERROR!" (flash 0x02BC)
 * when the queue is empty. Two errors in a row are read back as two answers, in the order they happened. */
void scpi_reply_last_error(void) {
    char text[24];
    uint8_t packed;

    if (!queue_pop(&packed)) {
        remote_reply_P(PSTR("NO ERROR!"));
        return;
    }
    strcpy_P(text, message_text((uint8_t)(packed >> 4)));
    strcat_P(text, message_text((uint8_t)(packed & 0x0Fu)));
    remote_reply(text);
}

void scpi_reply_init(void) {
    queue_head = 0u;
    queue_count = 0u;
    g_state.error_indicated = false;
    fetch_count = 0u;
    g_state.fetch_pending = false;
}

/* ---- Reply lines --------------------------------------------------------------------------------------- */

void remote_reply(const char *s) {
    uart1_write(s);
    uart1_write_terminator();
}

void remote_reply_P(const char *s_flash) {
    uart1_write_P(s_flash);
    uart1_write_terminator();
}

static void float_text(char *dst, size_t size, float v, uint8_t decimals) {
    size_t n = fmt_float_e_prec(dst, size, v, decimals);

    dst[n < size ? n : size - 1u] = '\0';
}

/* The plain %e layout, 6 decimals: the CALC, UNIT and HOLD replies. */
void remote_reply_float(float v) {
    char text[24];

    float_text(text, sizeof text, v, SCPI_DECIMALS_STANDARD);
    remote_reply(text);
}

void remote_reply_int(int32_t v) {
    char text[16];
    size_t n = fmt_int(text, sizeof text, v);

    text[n < sizeof text ? n : sizeof text - 1u] = '\0';
    remote_reply(text);
}

void scpi_reply_1(void) {
    remote_reply_P(PSTR("1"));
}
void scpi_reply_0(void) {
    remote_reply_P(PSTR("0"));
}
void scpi_reply_bang(void) {
    remote_reply_P(PSTR("!"));
}

void scpi_reply_bool(bool value) {
    if (value) {
        scpi_reply_1();
    } else {
        scpi_reply_0();
    }
}

/* A RAM string, at most 50 characters. */
void scpi_reply_ram_string(const char *text) {
    char line[REPLY_RAM_STRING_MAX + 1u];

    strncpy(line, text, REPLY_RAM_STRING_MAX);
    line[REPLY_RAM_STRING_MAX] = '\0';
    remote_reply(line);
}

/* Reading style used by FETCh? and the reference queries: a value at or beyond the overload threshold reads
 * the given text, everything else is the number with the given decimals. Returns the length written.
 *
 * The original printed "0.000" (flash 0x0A11) for any magnitude under 1e-7, which turned a 20 MHz period of
 * 5e-8 s or a current under 100 nA into zero. PROTOCOL_CHANGES G.2 #41 plans the true value in the unchanged
 * %e layout, so the threshold is gone and an exact zero prints as the formatter prints it ("0.0e0"). */
static size_t clamped_text(char *dst, size_t size, float value, uint8_t decimals, const char *overload_P) {
    if (fabsf(value) > OVERLOAD_THRESHOLD) {
        strncpy_P(dst, overload_P, size);
    } else {
        float_text(dst, size, value, decimals);
    }
    dst[size - 1u] = '\0';
    return strlen(dst);
}

/* The CALC, FREQuency:REFerence? and V/I/R:REFerence? replies, which differ only in the decimals. */
void scpi_reply_float_clamped(float value, uint8_t decimals) {
    char text[24];

    clamped_text(text, sizeof text, value, decimals, PSTR("9.9999e37"));
    remote_reply(text);
}

/* <func>:REFerence? -- the reference of function f. The V, I and R families answer with 8 decimals (0x11538,
 * format "%.8e"); FREQuency and PERiod with 6 (0x10A2C, format "%e"). */
void scpi_reply_reference(func_t f, uint8_t decimals) {
    scpi_reply_float_clamped(g_state.rel[f].reference, decimals);
}

/* FETCh? -- "<primary>" or "<primary>,<secondary>", sent at the end of the next completed measurement.
 *
 * Every request is answered once, each by its own conversion (PROTOCOL_CHANGES G.2 #40). The original kept a
 * single flag bit (M0F62 bit 5), so two requests before the next conversion gave one reply and the client
 * waited for the other. The count saturates at FETCH_PENDING_MAX; a request beyond that is refused (false)
 * rather than forgotten. */
bool scpi_fetch_request(void) {
    if (fetch_count >= FETCH_PENDING_MAX) {
        return false;
    }
    fetch_count++;
    g_state.fetch_pending = true;
    return true;
}

bool scpi_fetch_pending(void) {
    return fetch_count != 0u;
}

/* Answers one pending request with the reading that has just completed. */
void scpi_reply_reading(void) {
    char line[56];
    size_t n;

    if (fetch_count == 0u) {
        return;
    }
    fetch_count--;
    g_state.fetch_pending = (fetch_count != 0u);
    n = clamped_text(line, 24u, g_state.reading.primary, SCPI_DECIMALS_READING, PSTR("9.9e37"));
    if (g_state.dual_display) {
        line[n++] = ',';
        clamped_text(line + n, sizeof line - n, g_state.reading.secondary, SCPI_DECIMALS_READING,
                     PSTR("9.9e37"));
    }
    remote_reply(line);
}

/* HOLD:WINDow? -- the window in percent. */
void scpi_reply_hold_window(void) {
    remote_reply_float(g_state.hold_window * 100.0f);
}

void scpi_reply_hold_count(void) {
    remote_reply_int((int32_t)g_state.hold_count);
}

/* The front-panel LOCAL key: leave remote control. */
void scpi_enter_local(void) {
    g_state.remote = false;
    display_annunciators();
    buzzer_click();
}
