/* display.c -- reading text, function label, annunciators and the refresh
 * cycle.
 *
 * Layout of the main display (controller 1): cells 0-5 hold the number (five
 * digits plus the SI prefix), cells 6-8 the three-character function label,
 * cell 9 the flag byte (bit 5 = minus sign, bits 2/3 and 6/7 = the DC / AC
 * markers used by the dB and AC+DC modes). The secondary display (controller 2)
 * has the same arrangement. Annunciator lamps are single bytes at the third
 * byte of a cell (tables.h).
 *
 * Original: disp_update 0x9C3E, disp_hold_refresh_loop 0xAEB4, the reading
 * formatters 0xBD08-0xCA0A, disp_init_both 0x20B2, the numeric printers 0x213A
 * / 0x2354, the simple text printer 0x2D50, disp_update_function_label 0xF322,
 * disp_update_second_label 0xF508, disp_update_annunciators 0xF744,
 * disp_update_math_label 0xFA26 and boot_splash 0x5240.
 *
 * Text pipeline: a formatter produces a numtext_t -- sign character, six body
 * characters, SI prefix -- exactly the 8 characters the original kept at RAM
 * 0x0F19-0x0F20. The numeric printer takes the sign off the front (it becomes
 * the minus flag) and turns up to six of the remaining characters into cells.
 * Number digits come from fmt_fixed(), the original's "%#7.nf" with n from the
 * per-range format tables.
 *
 * State the original kept in scattered flag bytes, and where it lives now:
 *  - "second display on" (RAM 0x0F60 bit 2): g_state.dual_display. draw_second() draws it after the
 *    number on every frame (0x9C92-0x9C98).
 *  - MAX/MIN view (RAM 0x0130): g_state.minmax_view. draw_second() shows the stored extreme and the MAX
 *    or MIN lamp follows the same field (0x9C9C, 0xF9CE).
 *  - display enable (RAM 0x0F61 bit 2, written by :DISPlay:ENABle at 0x10E10 and 0x16DA4):
 *    g_state.display_enabled, see draw_frame().
 *  - dB / dBm unit in force (RAM 0x0135, copied from 0x0133 / 0x0134 by 0x166F4 / 0x16BB6): g_state.math
 *    db_on / dbm_on. The remote module mirrors a :UNIT:VOLTage selection into those two flags.
 *  - limit-test result (RAM 0x0132): derived from the reading and the limits, as the test itself does.
 *  - ERR lamp (RAM 0x0F61 bit 0, read at 0xF95E): g_state.error_indicated.
 *  - "restart" (RAM 0x0F60 bit 4): g_state.reading.valid is false.
 */
#include "display.h"

#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "../fmathf.h"
#include <string.h>
#include <util/delay.h>

#include "../fmt/fmt.h"
#include "../hal/buzzer.h"
#include "../hal/vfd.h"
#include "../tables.h"
#include "menu.h"

/* ---- Layout constants ---------------------------------------------------------------------------------- */

#define LABEL_CELL 6u /* display address 0x12 */
#define LABEL_CELLS 3u
#define LABEL_ADDRESS 0x12u
#define FLAG_CELL (VFD_FLAG_ADDR / 3u)

_Static_assert(DISPLAY_TEXT_CELLS == VFD_TEXT_CELLS, "the main line's text printer stops at the text cells");
_Static_assert(DISPLAY_TEXT_CELLS == FLAG_CELL,
               "the text cells end where the flag cell begins, on both lines");

/* Unit-pattern byte positions (display addresses) and patterns. A unit byte
 * replaces the low byte of the label cell it addresses, so it is merged into
 * that cell when the label is drawn. */
#define UNIT_ADDR_NORMAL 0x19u /* low byte of label cell 8 */
#define UNIT_ADDR_FREQ 0x16u   /* low byte of label cell 7 */
#define UNIT_ADDR_PERCENT 0x13u
#define UNIT_ADDR_DB 0x16u
#define UNIT_PAT_VOLT_AMP 0xE0u
#define UNIT_PAT_OHM 0xA2u
#define UNIT_PAT_FREQ 0xD0u
#define UNIT_PAT_DIODE 0xB0u
#define UNIT_PAT_ACDC 0xCAu
#define UNIT_PAT_PERCENT 0x9Bu
#define UNIT_PAT_DB 0xCBu
#define UNIT_PAT_DBM 0xABu

/* Flag-byte marker bits for the dB, dBm and AC+DC modes. */
#define FLAGS_DC 0x0Cu
#define FLAGS_AC 0xC0u
#define FLAGS_ACDC 0xCCu

#define TEXT_CAPACITY 16u
#define FIELD_WIDTH 7u /* the original's "%#7.nf" field (formats at RAM 0x0232-0x0319) */

#define SPLASH_MS 1200u
#define SPLASH_REVISION_CELL 4u /* display address 0x0C */

/* ---- Text ---------------------------------------------------------------------------------------------- */

static const char txt_overload[] PROGMEM = " OVL.D  ";
static const char txt_open[] PROGMEM = " OPEN ";
static const char txt_restart[] PROGMEM = "--.----";
static const char txt_revision[] PROGMEM = "001";
static const char blank_line[] PROGMEM = "";

static const char lbl_vdc[] PROGMEM = "VDC";
static const char lbl_vac[] PROGMEM = "VAC";
static const char lbl_adc[] PROGMEM = "ADC";
static const char lbl_aac[] PROGMEM = "AAC";
static const char lbl_ohm[] PROGMEM = "OHM";
static const char lbl_hz[] PROGMEM = "HZ ";
static const char lbl_sec[] PROGMEM = "SEC";
static const char lbl_dcv[] PROGMEM = "DCV";
static const char lbl_blank[] PROGMEM = "   ";
static const char lbl_percent[] PROGMEM = "%  ";
static const char lbl_db[] PROGMEM = "dB ";
static const char lbl_db_short[] PROGMEM = "dB"; /* two cells: the third keeps what was there */
static const char lbl_dbm[] PROGMEM = "dBm";
static const char lbl_vad[] PROGMEM = "VAD";
static const char lbl_aad[] PROGMEM = "AAD";

/* Label of the primary function, DCV ... DIODE. */
static const char *const function_label[FUNC_DIODE + 1u] PROGMEM = {
    lbl_vdc, lbl_vac, lbl_adc, lbl_aac, lbl_ohm, lbl_hz, lbl_sec, lbl_ohm, lbl_dcv,
};

typedef struct {
    char c[TEXT_CAPACITY]; /* [0] sign, [1..6] body, [7] SI prefix, then NUL */
} numtext_t;

static void put_cells(vfd_dev_t dev, uint8_t first_cell, const uint16_t *glyphs, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        vfd_write_cell(dev, (uint8_t)(first_cell + i), glyphs[i]);
    }
}

/* Draws a flash string at a cell position, at most up to the flag cell. */
static void put_text_P(vfd_dev_t dev, uint8_t first_cell, const char *text_flash) {
    uint16_t glyphs[VFD_TEXT_CELLS];
    const uint8_t n =
        tbl_text_cells(text_flash, true, TEXT_PRINTER_TEXT, glyphs, (uint8_t)(VFD_TEXT_CELLS - first_cell));

    put_cells(dev, first_cell, glyphs, n);
}

/* ---- Display state ------------------------------------------------------------------------------------- */

static struct {
    uint8_t flags[2];  /* flag byte last written to cell 9 of each controller */
    uint16_t layout;   /* what the clear-and-redraw decision depends on, see layout_key() */
    bool layout_valid; /* false until the first draw */
    bool second_text;  /* a menu has text on the second line */
} disp;

static void write_flags(vfd_dev_t dev, uint8_t flags) {
    disp.flags[dev] = flags;
    if (dev == VFD_MAIN) {
        vfd_write_flags(flags);
    } else {
        vfd_write_cell(dev, FLAG_CELL, (uint16_t)((uint16_t)flags << 8));
    }
}

/* Original disp_print_number_ctrl1 / ctrl2 (0x213A / 0x2354): the first
 * character is the sign and is consumed -- '-' sets the minus flag, anything
 * else clears it -- then up to six cells follow. */
static void print_number(vfd_dev_t dev, const numtext_t *t) {
    uint8_t flags = disp.flags[dev];
    uint16_t glyphs[VFD_NUM_CELLS];

    if (t->c[0] == '-') {
        flags |= VFD_FLAG_MINUS;
    } else {
        flags &= (uint8_t)~VFD_FLAG_MINUS;
    }
    write_flags(dev, flags);
    put_cells(dev, 0u, glyphs, tbl_text_cells(t->c + 1, false, TEXT_PRINTER_NUMBER, glyphs, VFD_NUM_CELLS));
}

/* ---- Number formatting --------------------------------------------------------------------------------- */

/* Fills a numtext_t from a scaled value: the first seven characters of
 * "%#7.<decimals>f" (longer text is cut, as the original's buffer cut it), the
 * sign over the first, blanks or minus signs in characters 1-3 turned into
 * zeros (original fmt_blank_to_zero3 0x3906), and the SI prefix. */
static void render(numtext_t *t, float shown, uint8_t decimals, char prefix) {
    char raw[24];

    fmt_fixed(raw, sizeof raw, shown, FIELD_WIDTH, decimals);
    memcpy(t->c, raw, FIELD_WIDTH);
    t->c[0] = (shown < 0.0f) ? '-' : ' ';
    for (size_t i = 1; i <= 3u; i++) {
        if (t->c[i] == ' ' || t->c[i] == '-') {
            t->c[i] = '0';
        }
    }
    t->c[FIELD_WIDTH] = prefix;
    t->c[FIELD_WIDTH + 1u] = '\0';
}

static void set_text_P(numtext_t *t, const char *text_flash) {
    strncpy_P(t->c, text_flash, sizeof t->c - 1u);
    t->c[sizeof t->c - 1u] = '\0';
}

/* Decimals by scaled magnitude: >= 999.995 -> 1, >= 99.9995 -> 2, >= 9.99995 ->
 * 3, else 4. */
static uint8_t decimals_for(float mag) {
    if (mag >= 999.995f) {
        return 1u;
    }
    if (mag >= 99.9995f) {
        return 2u;
    }
    return (mag >= 9.99995f) ? 3u : 4u;
}

/* The same without the one-decimal step (the MHz and millisecond bands never
 * use it). */
static uint8_t decimals_for_wide(float mag) {
    if (mag >= 99.9995f) {
        return 2u;
    }
    return (mag >= 9.99995f) ? 3u : 4u;
}

/* Original fmt_reading_auto_prefix 0xBD08: SI prefix chosen from the magnitude,
 * decimals from the scaled magnitude. */
static void fmt_reading_auto_prefix(numtext_t *t, float x) {
    const float mag = fabsf(x);
    float scaled;
    char prefix;

    if (mag >= 1e6f) {
        scaled = x / 1e6f;
        prefix = 'M';
    } else if (mag >= 1e3f) {
        scaled = x / 1e3f;
        prefix = 'k';
    } else if (mag >= 1.0f) {
        scaled = x;
        prefix = ' ';
    } else if (mag >= 1e-3f) {
        scaled = x / 1e-3f;
        prefix = 'm';
    } else {
        scaled = x / 1e-6f;
        prefix = 'u';
    }
    render(t, scaled, decimals_for(fabsf(scaled)), prefix);
}

/* Original fmt_reading_plain 0xBF20: no scaling, blank prefix (dB and dBm). */
static void fmt_reading_plain(numtext_t *t, float x) {
    render(t, x, decimals_for(fabsf(x)), ' ');
}

/* Decimals per range, from the original's "%#7.nf" format arrays (RAM
 * 0x0232-0x0319). The image holds identical copies for DCV and ACV, and for DCA
 * and ACA. */
static const uint8_t volt_decimals[5] PROGMEM = {1u, 2u, 3u, 4u, 2u};
static const uint8_t amp_decimals[5] PROGMEM = {3u, 4u, 2u, 3u, 4u};
static const uint8_t ohm_decimals[6] PROGMEM = {3u, 4u, 2u, 3u, 4u, 2u};

static uint8_t table_decimals(const uint8_t *table_flash, uint8_t count, uint8_t range) {
    return pgm_read_byte(&table_flash[range < count ? range : (uint8_t)(count - 1u)]);
}

/* Original fmt_value_dcv / acv / acdc_v (0xC128, 0xC1B8, 0xC982): the lowest
 * range (500 mV) in millivolts. */
static void fmt_value_volts(numtext_t *t, float x, uint8_t range) {
    const uint8_t decimals = table_decimals(volt_decimals, 5u, range);

    if (range == 4u) {
        render(t, x / 1e-3f, decimals, 'm');
    } else {
        render(t, x, decimals, ' ');
    }
}

/* Original fmt_value_dca / aca / acdc_a (0xC32A, 0xC3BA, 0xCA0A): ranges 2-4 in
 * milliamps. */
static void fmt_value_amps(numtext_t *t, float x, uint8_t range) {
    const uint8_t decimals = table_decimals(amp_decimals, 5u, range);

    if (range >= 2u) {
        render(t, x / 1e-3f, decimals, 'm');
    } else {
        render(t, x, decimals, ' ');
    }
}

/* Original fmt_value_ohm 0xC24E: range 5 in ohms, 2-4 in kilohms, 0-1 in
 * megohms. */
static void fmt_value_ohm(numtext_t *t, float x, uint8_t range) {
    const uint8_t decimals = table_decimals(ohm_decimals, 6u, range);

    if (range >= 5u) {
        render(t, x, decimals, ' ');
    } else if (range >= 2u) {
        render(t, x / 1e3f, decimals, 'k');
    } else {
        render(t, x / 1e6f, decimals, 'M');
    }
}

/* Original fmt_value_freq 0xC450. An exact zero is shown with a blank prefix: the original gave it 'm'
 * (0xC620-0xC628), so 0 Hz read as milli-hertz (ORIGINAL_BUGS 77). */
static void fmt_value_freq(numtext_t *t, float x) {
    const float mag = fabsf(x);

    if (mag >= 1e6f) {
        const float scaled = x / 1e6f;

        render(t, scaled, decimals_for_wide(fabsf(scaled)), 'M');
    } else if (mag >= 1e3f) {
        const float scaled = x / 1e3f;

        render(t, scaled, decimals_for(fabsf(scaled)), 'k');
    } else {
        render(t, x, decimals_for(mag), ' ');
    }
}

/* Original fmt_value_period 0xC636: milliseconds, microseconds or nanoseconds.
 */
static void fmt_value_period(numtext_t *t, float x) {
    const float mag = fabsf(x);

    if (mag >= 1e-3f) {
        const float scaled = x / 1e-3f;

        render(t, scaled, decimals_for_wide(fabsf(scaled)), 'm');
    } else if (mag >= 1e-6f) {
        const float scaled = x / 1e-6f;

        render(t, scaled, decimals_for(fabsf(scaled)), 'u');
    } else {
        const float scaled = x / 1e-9f;

        render(t, scaled, decimals_for(fabsf(scaled)), 'n');
    }
}

/* Original fmt_value_diode 0xC816. */
static void fmt_value_diode(numtext_t *t, float x) {
    render(t, x, 4u, ' ');
}

/* The beeper half of original fmt_value_cont 0xC852: on below the range's threshold, off otherwise. The
 * original did it inside the formatter; here it is a separate step that only the live reading takes, so that
 * formatting a stored MAX or MIN for the second display cannot drive the tone. The text itself is a
 * resistance (fmt_value_ohm). */
static void continuity_tone(float x, uint8_t range) {
    const uint8_t index = (range < CONTINUITY_RANGE_COUNT) ? range : (uint8_t)(CONTINUITY_RANGE_COUNT - 1u);

    buzzer_set(x < pgm_read_float(&continuity_threshold[index]));
}

/* Original fmt_reading_dispatch 0xC01A, the formatting half. */
static void fmt_reading_dispatch(numtext_t *t, float x, func_t f, uint8_t range) {
    switch (f) {
    case FUNC_DCA:
    case FUNC_ACA:
    case FUNC_ACDCI:
        fmt_value_amps(t, x, range);
        break;
    case FUNC_OHM:
    case FUNC_CONT:
        fmt_value_ohm(t, x, range);
        break;
    case FUNC_FREQ:
        fmt_value_freq(t, x);
        break;
    case FUNC_PERIOD:
        fmt_value_period(t, x);
        break;
    case FUNC_DIODE:
        fmt_value_diode(t, x);
        break;
    case FUNC_DCV:
    case FUNC_ACV:
    case FUNC_ACDCV:
    default:
        fmt_value_volts(t, x, range);
        break;
    }
}

/* ---- Mode helpers -------------------------------------------------------------------------------------- */

static bool is_volt_function(func_t f) {
    return f == FUNC_DCV || f == FUNC_ACV || f == FUNC_ACDCV;
}

static bool db_mode_active(func_t f) {
    return is_volt_function(f) && (g_state.math.db_on || g_state.math.dbm_on);
}

typedef enum { FMT_AUTO_PREFIX, FMT_PLAIN, FMT_PER_FUNCTION } format_mode_t;

/* Which formatter the main display uses (original disp_update 0x9C3E): PERCENT
 * scales automatically, dB and dBm are plain, everything else follows the
 * function. */
static format_mode_t format_mode(func_t f) {
    if (g_state.math.percent_on) {
        return FMT_AUTO_PREFIX;
    }
    return db_mode_active(f) ? FMT_PLAIN : FMT_PER_FUNCTION;
}

static void format_value(numtext_t *t, float x, func_t f) {
    switch (format_mode(f)) {
    case FMT_AUTO_PREFIX:
        fmt_reading_auto_prefix(t, x);
        break;
    case FMT_PLAIN:
        fmt_reading_plain(t, x);
        break;
    case FMT_PER_FUNCTION:
        fmt_reading_dispatch(t, x, f, g_state.range[f].index);
        break;
    }
}

/* The primary reading as text, including the restart (dashes) and overload
 * cases. */
static void format_primary(numtext_t *t) {
    const func_t f = g_state.func;

    if (!g_state.reading.valid) {
        set_text_P(t, txt_restart);
    } else if (g_state.reading.overload) {
        if (f == FUNC_CONT && format_mode(f) == FMT_PER_FUNCTION) {
            set_text_P(t, txt_open);
            buzzer_set(false);
        } else {
            set_text_P(t, txt_overload);
        }
    } else {
        format_value(t, g_state.reading.primary, f);
        if (f == FUNC_CONT && format_mode(f) == FMT_PER_FUNCTION) {
            continuity_tone(g_state.reading.primary, g_state.range[f].index);
        }
    }
}

/* Limit-test outcome, as the test itself computes it: 0 none, 1 within, 2 high,
 * 3 low. */
static uint8_t limit_result(void) {
    const math_state_t *m = &g_state.math;
    const float x = g_state.reading.primary;

    if (!m->limit_on || g_state.func == FUNC_CONT) {
        return 0u;
    }
    if (g_state.reading.overload || m->limit_hi < x) {
        return 2u;
    }
    return (x < m->limit_lo) ? 3u : 1u;
}

/* ---- Function label ------------------------------------------------------------------------------------ */

typedef struct {
    const char *label; /* flash string, three cells */
    uint8_t flags;     /* flag-byte marker bits */
    uint8_t unit_addr;
    uint8_t unit_pattern;
} chrome_t;

/* Original disp_update_function_label 0xF322: what the main display shows next
 * to the number. */
static chrome_t chrome_for_state(void) {
    const func_t f = g_state.func;
    chrome_t c = {lbl_blank, 0u, UNIT_ADDR_NORMAL, UNIT_PAT_VOLT_AMP};

    if (f <= FUNC_DIODE) {
        c.label = (const char *)pgm_read_ptr(&function_label[f]);
    }
    switch (f) {
    case FUNC_OHM:
    case FUNC_CONT:
        c.unit_pattern = UNIT_PAT_OHM;
        break;
    case FUNC_FREQ:
        c.unit_addr = UNIT_ADDR_FREQ;
        c.unit_pattern = UNIT_PAT_FREQ;
        break;
    case FUNC_DIODE:
        c.unit_pattern = UNIT_PAT_DIODE;
        break;
    case FUNC_ACDCV:
    case FUNC_ACDCI:
        c.unit_pattern = UNIT_PAT_ACDC;
        break;
    default:
        break;
    }

    if (g_state.math.percent_on) {
        c.label = lbl_percent;
        c.unit_addr = UNIT_ADDR_PERCENT;
        c.unit_pattern = UNIT_PAT_PERCENT;
    } else if (db_mode_active(f)) {
        c.flags = (f == FUNC_DCV) ? FLAGS_DC : ((f == FUNC_ACV) ? FLAGS_AC : FLAGS_ACDC);
        if (g_state.math.db_on) {
            c.label = lbl_db;
            c.unit_addr = UNIT_ADDR_DB;
            c.unit_pattern = UNIT_PAT_DB;
        } else {
            c.label = lbl_dbm;
            c.unit_pattern = UNIT_PAT_DBM;
        }
    } else if (f == FUNC_ACDCV || f == FUNC_ACDCI) {
        c.flags = FLAGS_ACDC;
        c.label = (f == FUNC_ACDCV) ? lbl_vad : lbl_aad;
    }
    return c;
}

/* Draws the label on the main display with the unit byte merged into the cell
 * it belongs to. */
static void draw_function_label(void) {
    const chrome_t c = chrome_for_state();
    uint16_t glyphs[LABEL_CELLS];
    uint8_t n = tbl_text_cells(c.label, true, TEXT_PRINTER_TEXT, glyphs, LABEL_CELLS);
    const uint8_t unit_cell = (uint8_t)((c.unit_addr - LABEL_ADDRESS) / 3u);

    while (n <= unit_cell) {
        glyphs[n++] = 0u;
    }
    glyphs[unit_cell] = (uint16_t)((glyphs[unit_cell] & 0xFF00u) | c.unit_pattern);
    put_cells(VFD_MAIN, LABEL_CELL, glyphs, n);
    disp.flags[VFD_MAIN] = c.flags;
}

/* ---- Second display ------------------------------------------------------------------------------------ */

/* How a second-display value is formatted, and which function's range it
 * follows. */
typedef enum { SK_VOLT_DC, SK_VOLT_AC, SK_AMP_DC, SK_AMP_AC, SK_FREQ, SK_PLAIN } second_kind_t;

typedef struct {
    const char *label; /* flash string */
    uint8_t kind;      /* second_kind_t */
} second_entry_t;

/* Selector tables, indexed by g_state.second_display[]: label and formatter
 * (original 0xF508 and the secondary stages 0x9CBA-0xA270). */
static const second_entry_t second_dcv[4] PROGMEM = {
    {lbl_vac, SK_VOLT_DC}, {lbl_hz, SK_FREQ}, {lbl_db, SK_PLAIN}, {lbl_dbm, SK_PLAIN}};
static const second_entry_t second_acv[4] PROGMEM = {
    {lbl_vdc, SK_VOLT_AC}, {lbl_hz, SK_FREQ}, {lbl_db_short, SK_PLAIN}, {lbl_dbm, SK_PLAIN}};
static const second_entry_t second_dca[2] PROGMEM = {{lbl_aac, SK_AMP_DC}, {lbl_hz, SK_FREQ}};
static const second_entry_t second_aca[2] PROGMEM = {{lbl_adc, SK_AMP_AC}, {lbl_hz, SK_FREQ}};
static const second_entry_t second_source[2] PROGMEM = {{lbl_vac, SK_VOLT_AC}, {lbl_aac, SK_AMP_AC}};
static const second_entry_t second_acdcv[5] PROGMEM = {
    {lbl_hz, SK_FREQ}, {lbl_db, SK_PLAIN}, {lbl_dbm, SK_PLAIN}, {lbl_vdc, SK_VOLT_DC}, {lbl_vac, SK_VOLT_DC}};
static const second_entry_t second_acdci[3] PROGMEM = {
    {lbl_hz, SK_FREQ}, {lbl_aac, SK_AMP_DC}, {lbl_adc, SK_AMP_DC}};

/* FREQ and PERIOD share one selector, RAM 0x012C. */
static uint8_t second_slot(func_t f) {
    return (f == FUNC_PERIOD) ? (uint8_t)FUNC_FREQ : (uint8_t)f;
}

/* Looks up the second-display entry selected for function f. */
static bool second_entry(func_t f, second_entry_t *out) {
    const second_entry_t *table;
    uint8_t count;

    switch (f) {
    case FUNC_DCV:
        table = second_dcv;
        count = 4u;
        break;
    case FUNC_ACV:
        table = second_acv;
        count = 4u;
        break;
    case FUNC_DCA:
        table = second_dca;
        count = 2u;
        break;
    case FUNC_ACA:
        table = second_aca;
        count = 2u;
        break;
    case FUNC_FREQ:
    case FUNC_PERIOD:
        table = second_source;
        count = 2u;
        break;
    case FUNC_ACDCV:
        table = second_acdcv;
        count = 5u;
        break;
    case FUNC_ACDCI:
        table = second_acdci;
        count = 3u;
        break;
    default:
        return false;
    }
    const uint8_t sel = g_state.second_display[second_slot(f)];

    if (sel >= count) {
        return false;
    }
    memcpy_P(out, &table[sel], sizeof *out);
    return true;
}

/* The AC volt and AC amp formatters (original 0xC1B8, 0xC3BA) index the range array by the ACTIVE function
 * (0x154F0 reads RAM 0x0B12), not by ACV or ACA, and the DC ones (0xC128, 0xC32A) by a fixed DCV or DCA
 * slot. For the secondary of FREQ and PERIOD, which is their amplitude input, that makes the AC formatters
 * follow range[FUNC_FREQ] -- the range the measurement autoranged that input on (meas.c range_of(pass.func),
 * so `pass.range` is the active function's). Reading range[FUNC_ACV] there would apply the range of a
 * function that is not measuring. */
static void format_second(numtext_t *t, float v, uint8_t kind) {
    const uint8_t active_range = g_state.range[g_state.func].index;

    switch ((second_kind_t)kind) {
    case SK_VOLT_DC:
        fmt_value_volts(t, v, g_state.range[FUNC_DCV].index);
        break;
    case SK_VOLT_AC:
        fmt_value_volts(t, v, active_range);
        break;
    case SK_AMP_DC:
        fmt_value_amps(t, v, g_state.range[FUNC_DCA].index);
        break;
    case SK_AMP_AC:
        fmt_value_amps(t, v, active_range);
        break;
    case SK_FREQ:
        fmt_value_freq(t, v);
        break;
    case SK_PLAIN:
        fmt_reading_plain(t, v);
        break;
    }
}

/* Original disp_update_math_label 0xFA26: the unit label shown with a MAX or
 * MIN value. */
static void draw_statistic_label(func_t f) {
    const char *label;

    if (g_state.math.percent_on) {
        label = lbl_percent;
    } else if (db_mode_active(f)) {
        label = g_state.math.db_on ? lbl_db : lbl_dbm;
    } else if (f == FUNC_ACDCV) {
        label = lbl_vad;
    } else if (f == FUNC_ACDCI) {
        label = lbl_aad;
    } else if (f <= FUNC_DIODE) {
        label = (const char *)pgm_read_ptr(&function_label[f]);
    } else {
        label = lbl_blank;
    }
    put_text_P(VFD_SECOND, LABEL_CELL, label);
}

/* ---- Annunciators -------------------------------------------------------------------------------------- */

static void lamp(uint8_t id, bool on) {
    vfd_set_annunciator(id, on);
}

/* Original disp_update_annunciators 0xF744, the lamps. Each id is an
 * annun_table row and was checked against the row the original addresses (Z =
 * 0x0500 + low byte loaded before the helper calls at 0xF74E-0xF9FA): FAST 0,
 * MED 3, SLOW 4, TRIG 5, HOLD 6, AUTO 7, REL 8, MATH 9, BEEP 10, DIOD 11, COMP
 * 12, HI 15, IN 16, LO 17, RMT 18, MAX 20, MIN 21, ERR 22, SHIFT 23. */
static void draw_lamps(void) {
    const instrument_t *g = &g_state;
    const func_t f = g->func;
    bool fast = false;
    bool medium = false;
    bool slow = false;
    const uint8_t result = limit_result();
    const bool auto_range = !g->range[f].manual && (f < FUNC_CONT || f == FUNC_ACDCV || f == FUNC_ACDCI);
    const bool math_lamp = g->math.percent_on || db_mode_active(f);

    if (f == FUNC_CONT) {
        fast = true;
    } else if (f == FUNC_DIODE) {
        medium = true;
    } else if (g->rate == RATE_FAST) {
        fast = true;
    } else if (g->rate == RATE_MEDIUM) {
        medium = true;
    } else {
        slow = true;
    }
    lamp(ANN_FAST, fast);
    lamp(ANN_MED, medium);
    lamp(ANN_SLOW, slow);
    if (f == FUNC_CONT) {
        lamp(ANN_BEEP, true);
    } else if (f == FUNC_DIODE) {
        lamp(ANN_DIOD, true);
    }
    lamp(ANN_REL, g->rel[f].active);
    lamp(ANN_AUTO, auto_range);
    lamp(ANN_TRIG, g->trigger != TRIG_IMM);
    lamp(ANN_MATH, math_lamp);
    lamp(ANN_COMP, g->math.limit_on);
    lamp(ANN_IN, result == 1u);
    lamp(ANN_LO, result == 3u);
    lamp(ANN_HI, result == 2u);
    lamp(ANN_RMT, g->remote);
    lamp(ANN_SHIFT, g->shift_active);
    lamp(ANN_HOLD, g->hold_active);
    lamp(ANN_ERR, g->error_indicated);
    lamp(ANN_MAX, g->minmax_view == MINMAX_VIEW_MAX);
    lamp(ANN_MIN, g->minmax_view == MINMAX_VIEW_MIN);
}

/* Label, minus sign, lamps: everything around the number (original
 * disp_init_both 0x20B2 after the controller set-up, plus 0xF744). */
static void draw_chrome(void) {
    draw_function_label();
    if (g_state.reading.primary < 0.0f) {
        disp.flags[VFD_MAIN] |= VFD_FLAG_MINUS;
    }
    draw_lamps();
    write_flags(VFD_MAIN, disp.flags[VFD_MAIN]);
}

/* ---- The second display's frame ------------------------------------------------------------------------ */

/* Original disp_update 0x9C3E, after the number: the secondary text on controller 2 when the second display
 * is on (0x9C92-0x9C98), then the stored MAX or MIN text when RAM 0x0130 is 1 or 2 (0x9C9C-0x9CB4); the two
 * are never on together, because the MAX/MIN key ends the dual display (keys.c plain_maxmin). The hold loop
 * 0xAEB4 ends in the same two steps (0xAEFC-0xAF1C), so one routine serves both.
 *
 * A secondary value the measurement did not produce is shown as the restart dashes, and an out-of-range one
 * as the same overload text the primary uses: the original formats the secondary into its own buffer with
 * the same overload copy (0x9D08: flash 0x04F2 into RAM 0x0F19). */
static void draw_second(void) {
    const func_t f = g_state.func;
    numtext_t t;

    if (g_state.minmax_view != MINMAX_VIEW_LIVE) {
        const float extreme =
            (g_state.minmax_view == MINMAX_VIEW_MAX) ? g_state.minmax.max : g_state.minmax.min;

        draw_statistic_label(f);
        format_value(&t, extreme, f);
    } else {
        second_entry_t entry;

        if (!g_state.dual_display || !second_entry(f, &entry)) {
            return;
        }
        put_text_P(VFD_SECOND, LABEL_CELL, entry.label);
        if (!g_state.reading.valid || !g_state.reading.secondary_valid) {
            set_text_P(&t, txt_restart);
        } else if (g_state.reading.secondary_overload) {
            set_text_P(&t, txt_overload);
        } else {
            format_second(&t, g_state.reading.secondary, entry.kind);
        }
    }
    print_number(VFD_SECOND, &t);
}

/* ---- Layout changes ------------------------------------------------------------------------------------ */

/* Everything that decides what the fixed parts of the panel look like: the function, the math mode, and
 * which of the second display's views is up. The second display's own label and selector are in it because
 * the labels are not all three cells wide ("dB" leaves the third cell as it was), so a change of selector
 * has to start from a clear panel; the original cleared on every such change (0x15D62 after the key). */
static uint16_t layout_key(void) {
    const func_t f = g_state.func;
    uint16_t key = (uint16_t)f; /* 0-10: bits 0-3 */

    if (g_state.math.percent_on) {
        key |= 0x0010u;
    }
    if (g_state.math.db_on) {
        key |= 0x0020u;
    }
    if (g_state.math.dbm_on) {
        key |= 0x0040u;
    }
    key |= (uint16_t)((uint16_t)g_state.minmax_view << 7); /* 0-2: bits 7-8 */
    if (g_state.dual_display) {
        key |= 0x0200u;
        key |= (uint16_t)((uint16_t)(g_state.second_display[second_slot(f)] & 0x07u) << 10);
    }
    return key;
}

static void clear_panel(void) {
    disp.second_text = false;
    disp.flags[VFD_MAIN] = 0u;
    disp.flags[VFD_SECOND] = 0u;
    vfd_clear(VFD_MAIN);
    vfd_clear(VFD_SECOND);
}

/* A new function or math mode changes the label, the unit byte and which lamps
 * can be lit. The original cleared both controllers and redrew everything on
 * every such change (0xF000 and its siblings); this does the same the first
 * time any entry point sees the change. Returns true when it cleared. Never
 * touches the display while a menu owns it. */
static bool sync_layout(void) {
    const uint16_t key = layout_key();

    if (menu_active() || (disp.layout_valid && key == disp.layout)) {
        return false;
    }
    disp.layout = key;
    disp.layout_valid = true;
    clear_panel();
    return true;
}

/* ---- Public interface ---------------------------------------------------------------------------------- */

void display_init(void) {
    memset(&disp, 0, sizeof disp);
    vfd_init();
}

/* Original disp_clear_all 0x1FD4: both controllers' display memory and the flag-byte shadows. The next
 * measurement frame has no layout to compare with, so it redraws label, flags and lamps from scratch -- which
 * is what the original does when a menu hands the panel back (0xE368-0xE376). */
void display_clear_all(void) {
    clear_panel();
    disp.layout_valid = false;
}

/* Original boot_splash 0x5240, called by main 0x953C right after disp_lamp_test (0x9540), which main.c
 * calls before vfd_init(): the lamp test also clears the display memory that the PT6311
 * datasheet leaves undefined at power-up (page 14, note 5), so the cells this splash does not write start
 * blank. The original's EEPROM read of the model text was never used: the REVISION menu item read the
 * EEPROM itself, and now shows the firmware version.
 *
 * The text has a purpose: the message the user set in SYS > BOOT MSG, or over
 * `:DISP:BOOT`, is what the panel shows at power-up. `ui_boot_text()` returns the shipped banner when
 * nothing has been set, so an untouched meter splashes exactly as the original did. */
void display_splash(void) {
    char banner[EE_BOOT_MSG_LEN + 1u];

    ui_boot_text(banner);
    vfd_text(VFD_MAIN, banner);
    put_text_P(VFD_SECOND, SPLASH_REVISION_CELL, txt_revision);
    buzzer_set(true);
    for (uint16_t ms = 0; ms < BUZZ_PULSE_MS; ms++) {
        wdt_reset();
        _delay_ms(1);
    }
    buzzer_set(false);
    for (uint16_t ms = 0; ms < SPLASH_MS; ms++) {
        wdt_reset();
        _delay_ms(1);
    }
}

/* The main refresh: number, controller set-up (re-sent every time, as the
 * original did), label, lamps, and the second display or MAX/MIN text. It is also what the original's
 * hold loop (0xAEB4) drew: while HOLD is on, g_state.reading is the held reading, and the main loop's own
 * pass serves the keyboard that the original loop served.
 *
 * With the display disabled (:DISPlay:ENABle OFF) the original formats the
 * reading, with its beeper side effect, and then draws nothing: the test of RAM
 * 0x0F61 bit 2 at 0x9C7E-0x9C84 skips the number print, disp_init_both, the
 * lamps and the secondary display, and the same test at 0xAF22 skips the hold
 * loop's redraw. Nothing is blanked; the panel keeps showing what it showed. */
void display_update(void) {
    numtext_t t;

    if (menu_active()) {
        return;
    }
    sync_layout();
    format_primary(&t);
    if (!g_state.display_enabled) {
        return;
    }
    print_number(VFD_MAIN, &t);
    vfd_init();
    draw_chrome();
    draw_second();
}

/* One line of menu or message text from cell 0, blank-filled to the nine text cells. Every menu string of
 * the original is stored blank-padded to the display width and the printer runs to the terminator
 * (0x2D50-0x2F16), so a shorter screen overwrites the whole line; a printer that stopped at the last
 * character left the previous screen's characters in the cells after it ("ON" over the longer title before it). */
static void show_line(vfd_dev_t dev, const char *s, bool in_flash) {
    uint16_t glyphs[DISPLAY_TEXT_CELLS];
    uint8_t n = tbl_text_cells(s, in_flash, TEXT_PRINTER_TEXT, glyphs, DISPLAY_TEXT_CELLS);

    while (n < DISPLAY_TEXT_CELLS) {
        glyphs[n++] = 0u;
    }
    put_cells(dev, 0u, glyphs, n);
}

/* The line, with the cell that `char_index` falls in drawn blank. For a blinking cursor.
 *
 * The layout is computed once from the whole text and then one cell is cleared, so the off phase occupies
 * exactly the same cells as the on phase. Blanking the CHARACTER instead moves the rest of the line
 * whenever that character was carrying a '.' or ':' -- see tbl_cell_of_char. */
void display_show_text_blank_cell(const char *s, uint8_t char_index) {
    uint16_t glyphs[DISPLAY_TEXT_CELLS];
    uint8_t n = tbl_text_cells(s, false, TEXT_PRINTER_TEXT, glyphs, DISPLAY_TEXT_CELLS);
    const uint8_t cell = tbl_cell_of_char(s, false, TEXT_PRINTER_TEXT, char_index);

    if (cell < n) {
        glyphs[cell] = 0u;
    }
    while (n < DISPLAY_TEXT_CELLS) {
        glyphs[n++] = 0u;
    }
    put_cells(VFD_MAIN, 0u, glyphs, n);
}

void display_show_text(const char *s) {
    show_line(VFD_MAIN, s, false);
}

void display_show_text_P(const char *s_flash) {
    show_line(VFD_MAIN, s_flash, true);
}

/* The second line belongs to the measurement screen, so a menu that lights it has to hand it back clean.
 * Lighting it therefore forgets the layout: the first display_update() after the menu closes clears both
 * controllers and draws the measurement from scratch, which is what a menu that was entered through
 * display_clear_all() gets anyway, and which a numeric editor opened from the SHIFT layer, over a live
 * measurement screen with the second display off, would otherwise leave behind as stale text. */
void display_show_second_P(const char *s_flash) {
    if (s_flash == NULL) {
        if (disp.second_text) {
            show_line(VFD_SECOND, blank_line, true);
            disp.second_text = false;
        }
        return;
    }
    show_line(VFD_SECOND, s_flash, true);
    disp.second_text = true;
    disp.layout_valid = false;
}

/* Lamps only, so a menu can call it while its text is on the main display.
 * After a layout change the label and flag byte are redrawn as well (original
 * 0xF000 and its siblings). While a menu owns the panel it draws nothing: the original clears the panel on
 * entry and, inside the menu, writes only the lamp bytes of the lamps the menu itself changes (SHIFT
 * 0xEA0C-0xEA22, TRIG 0xE942-0xE950); the menu does that itself, and release_display() redraws all of it. */
void display_annunciators(void) {
    if (menu_active()) {
        return;
    }
    if (sync_layout()) {
        draw_chrome();
    } else {
        draw_lamps();
    }
}
