/* test_menu_text.c -- host check that every text the front panel can show fits its line. Not part of the
 * firmware.
 *
 * The vendor's menu texts did not fit the display and the vendor's "MEU" had been copied along
 * with them. A text that is too long is cut, or it moves the cells after it, so this check makes one
 * impossible to ship: it fails the host test run, and the host test run is part of the build.
 *
 * It looks at what the firmware ships and not at a list of its own. tables.c and menu.c are compiled into
 * this program as they are (the real glyph tables, the real cell rule, the real screen code), with a fake
 * EEPROM and stubs for the modules menu.c calls. Three things are checked:
 *
 *   1. Every string in menu_text[], in the menu's local text list, and in error_fragment[].
 *   2. Every SCREEN menu.c builds from them. The menu composes lines at run time -- a choice value, an
 *      edit field with its cursor, a countdown, "GAIN+0.12%" -- and those widths exist nowhere in a table,
 *      so the real render() is driven through every page, item, choice, editor, prompt, busy state, view
 *      and character editor, in both phases of the blink, and the lines it hands the display are measured.
 *   3. The cell rule itself, against the glyph tables: every character of every shipped text must have a
 *      glyph (the same rule the BOOT MSG picker uses), and a '.' or ':' must stand right behind a letter or
 *      a digit, because only there does the printer merge it into that cell.
 *
 * A text is measured by tbl_text_cells(), the function the display driver prints with, so "how many cells"
 * is not re-implemented here. The limit is DISPLAY_TEXT_CELLS, which display.c asserts against the flag
 * cell that follows the text cells on both controllers.
 *
 * Usage:  test_menu_text            exits 1 and names each offender
 *         test_menu_text --list     also prints every text with its cell count (this is how MENU_TEXTS.md
 *                                   was checked)
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/tables.c"
#include "../src/ui/menu.c"

/* ---- Fake EEPROM -------------------------------------------------------------------------------------
 * Filled with the longest things the panel reads from it, so the views are measured at their widest. */
#define FAKE_EE_SIZE 4096u
static uint8_t fake_ee[FAKE_EE_SIZE];

uint8_t ee_read_u8(uint16_t addr) {
    return (addr < FAKE_EE_SIZE) ? fake_ee[addr] : 0xFFu;
}
uint16_t ee_read_u16(uint16_t addr) {
    return (uint16_t)(ee_read_u8(addr) | ((uint16_t)ee_read_u8((uint16_t)(addr + 1u)) << 8));
}
float ee_read_f32(uint16_t addr) {
    float v;
    memcpy(&v, &fake_ee[addr], sizeof v);
    return v;
}
void ee_read_block(uint16_t addr, void *dst, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        ((uint8_t *)dst)[i] = ee_read_u8((uint16_t)(addr + i));
    }
}
bool ee_write_u8(uint16_t addr, uint8_t v) {
    if (addr >= FAKE_EE_SIZE) {
        return false;
    }
    fake_ee[addr] = v;
    return true;
}
bool ee_write_u16(uint16_t addr, uint16_t v) {
    return ee_write_u8(addr, (uint8_t)(v & 0xFFu)) && ee_write_u8((uint16_t)(addr + 1u), (uint8_t)(v >> 8));
}
bool ee_write_f32(uint16_t addr, float v) {
    memcpy(&fake_ee[addr], &v, sizeof v);
    return true;
}
bool ee_write_block(uint16_t addr, const void *src, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        if (!ee_write_u8((uint16_t)(addr + i), ((const uint8_t *)src)[i])) {
            return false;
        }
    }
    return true;
}

/* ---- What the display driver is handed ---------------------------------------------------------------- */
#define CAPTURE_MAX 64u
static char shown_main[CAPTURE_MAX];
static char shown_second[CAPTURE_MAX];

static void capture_flash(char *dst, const char *src_flash) {
    size_t n = 0;

    if (src_flash != NULL) {
        for (; pgm_read_byte(src_flash + n) != 0 && n + 1u < CAPTURE_MAX; n++) {
            dst[n] = (char)pgm_read_byte(src_flash + n);
        }
    }
    dst[n] = '\0';
}

void display_show_text(const char *s) {
    strncpy(shown_main, s, CAPTURE_MAX - 1u);
    shown_main[CAPTURE_MAX - 1u] = '\0';
}
/* The blinking-cursor path. It captures the SAME text as the on phase, which is the property
 * worth asserting: the off phase blanks a cell AFTER layout, so it occupies exactly the cells the on phase
 * did. The old code substituted a space for the character, and that moved the rest of the line whenever
 * the character carried a decimal point -- which is what made the display appear to scroll. */
void display_show_text_blank_cell(const char *s, uint8_t char_index) {
    (void)char_index;
    strncpy(shown_main, s, CAPTURE_MAX - 1u);
    shown_main[CAPTURE_MAX - 1u] = '\0';
}
void display_show_text_P(const char *s_flash) {
    capture_flash(shown_main, s_flash);
}
void display_show_second_P(const char *s_flash) {
    capture_flash(shown_second, s_flash);
}
void display_clear_all(void) {
}
void display_annunciators(void) {
}
void display_update(void) {
}

/* ---- Stubs for the modules menu.c calls --------------------------------------------------------------- */
instrument_t g_state;

void vfd_set_annunciator(uint8_t annun_id, bool on) {
    (void)annun_id;
    (void)on;
}
void vfd_write_cell(vfd_dev_t dev, uint8_t cell, uint16_t glyph) {
    (void)dev;
    (void)cell;
    (void)glyph;
}
void vfd_set_brightness(uint8_t level) {
    (void)level;
}

serv_result_t serv_check(void) {
    return SERV_ALLOWED;
}
bool serv_unlock(const char *code, serv_source_t from) {
    (void)code;
    (void)from;
    return true;
}
bool serv_set_password(const char *code) {
    (void)code;
    return true;
}
bool serv_password_is_set(void) {
    return false;
}

bool settings_store(void) {
    return true;
}
void settings_apply(void) {
}
void keys_ref_confirmed(void) {
}
bool main_warmed_up(void) {
    return true;
}
uint16_t main_power_on_hours(void) {
    return 65535u;
}

bool cal_write_user_zero(func_t f, uint8_t range, float value) {
    (void)f;
    (void)range;
    (void)value;
    return true;
}
bool cal_short_all_start(void) {
    return true;
}
bool cal_open_all_start(void) {
    return true;
}
cal_sweep_state_t cal_sweep_state(void) {
    return CAL_SWEEP_RUNNING;
}
void cal_sweep_abort(void) {
}
bool cal_range_start(func_t f, uint8_t range, float reference, bool accept_cold) {
    (void)f;
    (void)range;
    (void)reference;
    (void)accept_cold;
    return true;
}
bool cal_range_pending(float *change_fraction) {
    *change_fraction = 0.0f;
    return true;
}
bool cal_range_confirm(void) {
    return true;
}
bool cal_verify_start(func_t f, uint8_t range, float reference) {
    (void)f;
    (void)range;
    (void)reference;
    return true;
}
bool cal_verify_result(float *deviation, bool *within_spec) {
    *deviation = 0.0f;
    *within_spec = true;
    return true;
}
bool cal_undo(void) {
    return true;
}
uint8_t cal_check_run(void) {
    return 3u;
}
bool cal_check_name(uint8_t index, char name[CAL_CHECK_NAME_LEN + 1u]) {
    (void)index;
    memcpy(name, "dcv 5v lo", CAL_CHECK_NAME_LEN + 1u); /* the longest a record name can be */
    return true;
}
bool cal_date_write(const char date[8]) {
    (void)date;
    return true;
}

bool meas_relay_exercise_start(bool full) {
    (void)full;
    return true;
}
void meas_relay_exercise_abort(void) {
}
uint8_t meas_relay_exercise_seconds_left(void) {
    return 60u;
}
relay_exercise_result_t meas_relay_exercise_result(void) {
    return RLYEX_RUNNING;
}
bool meas_set_line(bool is_50hz) {
    (void)is_50hz;
    return true;
}
bool meas_selfcal_start(void) {
    return true;
}
bool meas_selfcal_done(bool *ok) {
    *ok = true;
    return true;
}
bool meas_zero_capture(func_t f, uint8_t range, float *out) {
    (void)f;
    (void)range;
    *out = 0.0f;
    return true;
}

/* ---- The rule ----------------------------------------------------------------------------------------- */

static unsigned failures;
static unsigned checked;

/* A character the panel can draw: the one rule the BOOT MSG picker enforces, applied to the same tables. */
static bool showable(char c) {
    return picker_sanitise(c) == c;
}

static bool is_alnum(char c) {
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z');
}

typedef enum {
    LINE_STATIC, /* a text as stored: every rule applies */
    LINE_SCREEN, /* a line a screen built; '^' is the blank prefix cell of the number editors */
    LINE_BLANKING /* the off phase of a number editor's blinking cell: width and glyphs only, see check_line() */
} line_kind_t;

/* Measures `s` as the display prints it and reports every way it can fail. */
static void check_line(const char *where, const char *s, line_kind_t kind) {
    uint16_t glyphs[CAPTURE_MAX];
    const uint8_t cells = tbl_text_cells(s, false, TEXT_PRINTER_TEXT, glyphs, CAPTURE_MAX - 1u);

    checked++;
    if (cells > DISPLAY_TEXT_CELLS) {
        printf("FAIL  %s: \"%s\" takes %u cells, the line holds %u\n", where, s, (unsigned)cells,
               (unsigned)DISPLAY_TEXT_CELLS);
        failures++;
    }
    for (size_t i = 0; s[i] != '\0'; i++) {
        const char c = s[i];

        if (c == '.' || c == ':') {
            /* Behind anything but a letter or digit the mark is a blank cell of its own and moves every
             * cell after it. The off phase of a blinking cell is the one place that still does this: a
             * blanked digit followed by the point, so that phase is held to width only. */
            if (kind != LINE_BLANKING && (i == 0 || !is_alnum(s[i - 1]))) {
                printf("FAIL  %s: \"%s\" has '%c' at %zu behind a character that cannot carry it\n", where, s,
                       c, i);
                failures++;
            }
        } else if (!showable(c) && !(c == '^' && kind != LINE_STATIC)) {
            printf("FAIL  %s: \"%s\" has '%c' at %zu, which has no glyph\n", where, s, c, i);
            failures++;
        }
    }
}

static bool listing;

static void check_text(const char *where, uint8_t id, const char *s_flash) {
    char copy[CAPTURE_MAX];

    capture_flash(copy, s_flash);
    if (listing) {
        uint16_t glyphs[CAPTURE_MAX];

        printf("%-14s %3u  %-14s %u\n", where, (unsigned)id, copy,
               (unsigned)tbl_text_cells(copy, false, TEXT_PRINTER_TEXT, glyphs, CAPTURE_MAX - 1u));
    }
    check_line(where, copy, LINE_STATIC);
}

/* ---- The screens --------------------------------------------------------------------------------------- */

static char where_buf[96];

static void check_screen(const char *what, unsigned a, unsigned b, bool blink_on) {
    const line_kind_t kind = (!blink_on && m.state == MS_NUMERIC) ? LINE_BLANKING : LINE_SCREEN;

    shown_main[0] = '\0';
    shown_second[0] = '\0';
    m.blink_on = blink_on;
    render();
    snprintf(where_buf, sizeof where_buf, "%s %u/%u main", what, a, b);
    check_line(where_buf, shown_main, kind);
    snprintf(where_buf, sizeof where_buf, "%s %u/%u second", what, a, b);
    check_line(where_buf, shown_second, LINE_STATIC);
}

static void screens(bool blink_on) {
    static const float samples[] = {1e8f, -1e8f, 99999.0f, 0.0001f, 123.45f, 1e-4f, 999.99f};

    memset(&m, 0, sizeof m);

    for (uint8_t page = 0; page < PAGE_COUNT; page++) {
        const page_def_t def = page_def(page);

        m.state = MS_PAGE;
        m.page = page;
        check_screen("page", page, 0u, blink_on);
        for (uint8_t item = 0; item < def.count; item++) {
            m.state = MS_ITEM;
            m.item[page] = item;
            check_screen("item", page, item, blink_on);
        }
    }
    for (uint8_t id = 0; id < CH_COUNT; id++) {
        const choice_def_t def = choice_def(id);

        for (uint8_t c = 0; c < def.count; c++) {
            m.state = MS_CHOICE;
            m.param = id;
            m.choice = c;
            check_screen("choice", id, c, blink_on);
        }
    }
    for (uint8_t id = 0; id < NUM_COUNT; id++) {
        for (size_t v = 0; v < sizeof samples / sizeof samples[0]; v++) {
            m.state = MS_NUMERIC;
            m.param = id;
            eng_format(m.text, samples[v]);
            for (m.cursor = first_cursor(MS_NUMERIC); m.cursor < EDIT_TEXT_LEN; m.cursor++) {
                check_screen("numeric", id, (unsigned)v * 8u + m.cursor, blink_on);
            }
        }
    }
    for (uint8_t id = 0; id < INT_COUNT; id++) {
        const integer_def_t def = integer_def(id);

        m.state = MS_INTEGER;
        m.param = id;
        digits_from_value(m.digits, def.digits, 9999u);
        for (m.cursor = 0; m.cursor < def.digits; m.cursor++) {
            check_screen("integer", id, m.cursor, blink_on);
        }
    }
    m.state = MS_REVISION;
    check_screen("revision", 0u, 0u, blink_on);
    for (uint8_t sub = 0; sub < SUB_COUNT; sub++) {
        m.state = MS_SUB;
        m.sub = sub;
        check_screen("sub", sub, 0u, blink_on);
    }
    for (uint8_t text = 0; text < LT_COUNT; text++) {
        m.state = MS_PROMPT;
        m.prompt_action = PA_ZERO;
        m.prompt_text = text;
        m.prompt_caption = LT_EXERCISER;
        check_screen("prompt", text, 0u, blink_on);
    }
    m.state = MS_PROMPT;
    m.prompt_action = PA_RANGE_COMMIT;
    for (uint8_t i = 0; i < 4u; i++) {
        static const float changes[] = {0.0f, 0.0099f, -0.1f, 0.0001f};

        m.range_change = changes[i];
        check_screen("gain", i, 0u, blink_on);
    }

    m.state = MS_BUSY;
    m.activity = ACT_EXER;
    for (unsigned shown = 0; shown <= 255u; shown += 51u) {
        m.shown = (uint8_t)shown;
        check_screen("exerciser", shown, 0u, blink_on);
    }
    m.activity = ACT_SWEEP;
    check_screen("sweep", 0u, 0u, blink_on);
    m.activity = ACT_ZERO_CAL;
    check_screen("zero cal", 0u, 0u, blink_on);
    m.activity = ACT_SELF_TEST;
    m.step = ST_SELFCAL;
    check_screen("self test", 0u, 0u, blink_on);
    m.step = ST_KEYS;
    m.last_key = KEY_NONE;
    check_screen("self test", 0u, 0u, blink_on);
    for (unsigned key = KEY_DCV; key <= KEY_SHIFT; key++) {
        m.last_key = (uint8_t)key;
        check_screen("key name", key, 0u, blink_on);
    }

    memset(&fake_ee[EE_ID_SERIAL], 'W', 20u);
    memset(&fake_ee[EE_CAL_DATE], '9', DATE_CHARS);
    memset(&fake_ee[EE_THID_PROD_DATE], '9', DATE_CHARS);
    m.state = MS_VIEW;
    m.view = VIEW_INFO;
    for (uint8_t page = 0; page < INFO_PAGES; page++) {
        m.view_page = page;
        check_screen("info", page, 0u, blink_on);
    }
    m.view = VIEW_CHECK;
    check_screen("cal check", 0u, 0u, blink_on);
    m.view = VIEW_VERIFY;
    m.verify_deviation = 0.5f;
    for (uint8_t page = 0; page < VERIFY_PAGES; page++) {
        m.view_page = page;
        m.verify_pass = (page == 0u);
        check_screen("verify", page, 0u, blink_on);
    }
    m.verify_pass = false;
    check_screen("verify", 9u, 0u, blink_on);
    m.view = VIEW_COUNT;
    check_screen("count", 0u, 0u, blink_on);

    m.state = MS_TEXTEDIT;
    m.text_kind = TEXT_BOOT_MSG;
    m.text_len = EE_BOOT_MSG_LEN;
    memset(m.text, 'W', EE_BOOT_MSG_LEN);
    m.text[EE_BOOT_MSG_LEN] = '\0';
    for (m.cursor = 0; m.cursor < m.text_len; m.cursor++) {
        check_screen("boot msg", m.cursor, 0u, blink_on);
    }
    m.text_kind = TEXT_DATE;
    m.text_len = DATE_CHARS;
    memcpy_P(m.text, date_template, DATE_CHARS + 1u);
    for (m.cursor = 0; m.cursor < m.text_len; m.cursor++) {
        check_screen("date", m.cursor, 0u, blink_on);
    }
}

/* The dB and dBm reference editors: "REF:" in front of the value, nine cells at most, and the dB editor
 * without a sign cell. Cell counts by tbl_text_cells(), the rule the display prints with:
 *   dB REF   "REF:" 3 cells (the colon shares F's) + five digits 5 + prefix 1 = 9, so a unit V would make 10;
 *   dBm REF  "REF:" 3 cells + four digits 4 + the ohm glyph 1 = 8. */
static void expect_reference_row(const char *what, const char *expected_start, uint8_t expected_cells) {
    uint16_t glyphs[CAPTURE_MAX];
    const uint8_t cells = tbl_text_cells(shown_main, false, TEXT_PRINTER_TEXT, glyphs, CAPTURE_MAX - 1u);

    checked++;
    if (strncmp(shown_main, expected_start, strlen(expected_start)) != 0 || cells != expected_cells ||
        cells > DISPLAY_TEXT_CELLS) {
        printf("FAIL  %s: \"%s\" takes %u cells, expected \"%s...\" and %u cells\n", what, shown_main,
               (unsigned)cells, expected_start, (unsigned)expected_cells);
        failures++;
    }
}

static void reference_editors(void) {
    static const float db_refs[] = {1e-4f, 1.0f, 600.0f, 1000.0f};

    for (size_t v = 0; v < sizeof db_refs / sizeof db_refs[0]; v++) {
        memset(&m, 0, sizeof m);
        g_state.math.db_ref = db_refs[v];
        start_numeric(NUM_DB_REF);
        shown_main[0] = '\0';
        render();
        expect_reference_row("dB REF", "REF:", 9u);
        if (strpbrk(shown_main, "+-") != NULL || m.cursor != 1u) {
            printf("FAIL  dB REF: \"%s\" has a sign cell, or the cursor is on index %u\n", shown_main,
                   (unsigned)m.cursor);
            failures++;
        }
        for (uint8_t step = 0; step < 2u * EDIT_TEXT_LEN; step++) {
            m.cursor = text_cursor_step(m.text, first_cursor(MS_NUMERIC), m.cursor, true);
            if (m.cursor == 0u) {
                printf("FAIL  dB REF: the cursor reached the sign cell\n");
                failures++;
            }
        }
        m.cursor = text_cursor_step(m.text, first_cursor(MS_NUMERIC), 1u, false);
        if (m.cursor != EDIT_TEXT_LEN - 1u) {
            printf("FAIL  dB REF: LEFT from the first digit went to index %u\n", (unsigned)m.cursor);
            failures++;
        }
    }

    memset(&m, 0, sizeof m);
    g_state.math.dbm_ref = 75;
    start_integer(INT_DBM_REF);
    shown_main[0] = '\0';
    render();
    expect_reference_row("dBm REF", "REF:0075&", 8u);

    memset(&m, 0, sizeof m);
    g_state.math.percent_ref = 1.0f;
    start_numeric(NUM_PERCENT_REF);
    shown_main[0] = '\0';
    render();
    if (shown_main[0] != '+' || m.cursor != 0u) {
        printf("FAIL  PERC REF lost its sign cell: \"%s\"\n", shown_main);
        failures++;
    }
}

int main(int argc, char **argv) {
    listing = (argc > 1 && strcmp(argv[1], "--list") == 0);

    if (menu_text_count != MT_COUNT) {
        printf("FAIL  menu_text[] has %u entries, menu_text_id_t names %u\n", (unsigned)menu_text_count,
               (unsigned)MT_COUNT);
        failures++;
    }
    for (uint8_t id = 0; id < menu_text_count; id++) {
        check_text("menu_text", id, text_P(id));
    }
    for (uint8_t id = MT_COUNT; id < LT_COUNT; id++) {
        check_text("local text", id, text_P(id));
    }
    for (uint8_t id = 0; id < ERROR_FRAGMENT_COUNT; id++) {
        check_text("error text", id, (const char *)pgm_read_ptr(&error_fragment[id]));
    }
    check_text("boot message", 0u, boot_msg_default);

    screens(true);
    screens(false);
    reference_editors();

    printf("%u lines measured, %u failures\n", checked, failures);
    return failures == 0u ? 0 : 1;
}
