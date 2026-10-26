/* menu.c -- front-panel menu (SHIFT + MENU): MATH MENU, TRIG MENU, SYS MENU.
 *
 * Original: menu_level0_loop 0xE4D6, menu_level1_loop 0xE646, the per-page key
 * handlers 0x8B52 / 0x8BFC / 0x8C7A, the parameter editors 0xE764 / 0xE88C /
 * 0xEA4C, the numeric editors 0x541A-0x63A4 and the digit editors 0x3C02 /
 * 0x3D34 / 0x4E58 / 0x5046. Every one of them was a loop that polled the
 * keyboard with delays until the operator finished (the original's defect). Here
 * the whole menu is one small state machine: the caller feeds it key presses
 * with menu_key() and a 1 ms clock with menu_tick(), and nothing ever waits.
 *
 * Contract with the caller (main loop):
 *  - menu_tick() is called once per millisecond from the main loop, not from
 * interrupt context. It times the cursor blink and the one-second messages.
 *  - While menu_active() is true, display_update() must leave the main display
 * alone and every key press is passed to menu_key() instead of the normal key
 * dispatcher.
 *  - A function key (DCV ... DIODE) pressed inside the menu closes the menu.
 * The menu does not execute it: the caller sees menu_active() fall from true to
 * false across menu_key() and then dispatches that same key normally, with the
 * SHIFT state left exactly as it was (the original ran 0xDF00 / 0xE380 here).
 *
 * Screen text goes through display_show_text() and display_show_second_P(), which blank-fill their line to
 * the nine text cells; nothing here writes display cells except the self-test lamp pattern. A '.' or ':'
 * right behind a digit or letter merges into that cell (tables.h), which is how the original printers drew
 * them. The panel is cleared on entry (display_clear_all, original 0xE31C) and inside the menu only the
 * SHIFT and TRIG lamps are written; release_display() redraws the measurement screen.
 *
 * Every text fits nine cells, and so does every composed screen. Nothing here carries a prefix such as
 * "A:" or "7:": a page, an item or a submenu entry is its name. A name that does not fit one line in
 * full takes both: RELAY on the main line and EXERCISER on the second. An editor shows its value on the
 * main line and the name of what it edits on the second. MENU_TEXTS.md lists every text, and
 * host/test_menu_text.c fails the build check if one grows past its line.
 *
 * SYS items 8 to 18 are the same machine with
 * a few more states: a submenu (CAL), a yes/no prompt, a running activity that is polled from
 * menu_tick() (relay exercise, calibration sweep, self test), a paged read-only view (INFO, CAL
 * CHECK, VERIFY) and a character editor (CAL DATE, BOOT MSG). This file draws those screens and
 * moves between them; every calibration, converter and relay operation is a call into the module
 * that owns it.
 *
 * menu_tick() is called from the main loop, once per elapsed millisecond (main.c), never
 * from an interrupt: it draws, and the PT6311 lines have one writer. It counts time and polls state;
 * everything that starts an operation, writes the EEPROM or can take long runs from menu_key().
 */
#include "menu.h"

#include <avr/pgmspace.h>
#include "../fmathf.h"
#include <stddef.h>
#include <string.h>

#include "../cal/cal.h"
#include "../eeprom_map.h"
#include "../hal/eeprom.h"
#include "../hal/keys.h"
#include "../hal/vfd.h"
#include "../meas/meas.h"
#include "../serv/serv.h"
#include "../state.h"
#include "../tables.h"
#include "../version.h"
#include "display.h"
#include "settings.h"

/* Names for the keys as the menu uses them. The scan codes are CERTAIN; which physical cap each one
 * belongs to is UNVERIFIED -- see the check in keys.h. The menu dispatches on the code. */
#define KEY_UP KEY_RANGE_UP
#define KEY_DOWN KEY_RANGE_DOWN
#define KEY_LEFT KEY_PREV
#define KEY_RIGHT KEY_NEXT
#define KEY_ENTER KEY_AUTO

/* Cursor blink: the original showed the character for 31 loop passes of 10 ms
 * and hid it for 29 (0x3822). */
#define BLINK_PERIOD_MS 600u
#define BLINK_ON_MS 310u

#define MESSAGE_MS 1000u /* SAVED, TOO LARGE, TOO SMALL, KEY ERROR ... */

#define LINE_CAPACITY 24u
#define EDIT_TEXT_LEN                                                                                        \
    8u /* sign, five digits and a point, then the SI prefix                                                  \
        */
#define PREFIX_INDEX 7u
#define MAX_DIGITS 4u

#define LIMIT_ABS_MAX 1e8f /* HI / LO LIMIT and PERC REF accept +-1e8 (flash 0x46B / 0x35D) */
#define DB_REF_MIN 1e-4f   /* dB REF accepts 1e-4 ... 1000 */
#define DB_REF_MAX 1000.0f
#define HOLD_COUNT_MIN 2u
#define HOLD_COUNT_MAX 100u
#define CAL_REF_RESTART 1.0f /* where the CAL reference editor starts, and restarts after a refused zero */
#define DBM_REF_MIN 1u
#define KEY_CODE_DIGITS 4u

/* The character editors (CAL DATE, BOOT MSG) keep their text in menu_ctx_t.text, which must also hold a
 * whole boot message. */
#define DATE_CHARS 8u
#define TEXT_CAPACITY (EE_BOOT_MSG_LEN + 1u)
_Static_assert(TEXT_CAPACITY > EDIT_TEXT_LEN, "the character editors share the numeric editor's buffer");
_Static_assert(TEXT_CAPACITY > DATE_CHARS, "a date must fit the shared text buffer");

#define LABEL_MS 700u            /* the caption shown before a value on a paged view */
#define LAMP_TEST_MS 1500u       /* SELF TEST: every segment and lamp lit */
#define KEY_TEST_EXIT_PRESSES 3u /* SELF TEST key test ends on this many MENU presses in a row */
#define NO_TEXT 0xFFu

/* SYS item 10, LINE. Stored value 0 = 60 Hz and 1 = 50 Hz (eeprom_map.h); an erased byte, or anything
 * else, means 60 Hz, which is the default. The choice index is the stored value. */
#define LINE_STORED_50HZ 1u
#define LINE_HZ_DEFAULT 60u
#define LINE_HZ_ALTERNATE 50u

/* SYS item 9, RELAY POWER ON. The stored values and which one means off are in eeprom_map.h, next to the
 * address, so that this file and main.c cannot read the same byte differently -- a divergence
 * there would be a defect. */

/* SYS item 15, BRIGHT: the PT6311 display-control dimming field is 3 bits, codes 0 to 7, the chip's 8
 * steps of 1, 2, 4, 10, 11, 12, 13, 14 sixteenths (datasheet PT6311B V1.0, page 9, command 4; vfd.h).
 * All 8 are offered and shown as 1 to 8 (the top five alone changed too
 * little). The level is previewed as the key is pressed and only stored on ENTER, and RESET restores the
 * brightest, so a dim choice is always undone from this same menu. */
#define BRIGHT_MIN 0u
#define BRIGHT_MAX VFD_BRIGHT_MAX
#define BRIGHT_DEFAULT BRIGHT_MAX
#define BRIGHT_CHOICES (BRIGHT_MAX - BRIGHT_MIN + 1u)

/* RESET: the factory values of the user settings, from a vendor unit's EEPROM for the five
 * vendor bytes. The three bytes this firmware added reset to "erased", which every reader treats as
 * "use the default". */
#define DEFAULT_BAUD_INDEX 2u    /* 9600 */
#define DEFAULT_TX_TERMINATOR 0u /* LF */
#define DEFAULT_ECHO 1u
#define DEFAULT_BEEPER 1u
#define DEFAULT_KEY_CLICK 1u
#define ERASED_BYTE 0xFFu
#define ERASED_WORD 0xFFFFu

/* The SELF TEST lamp test lights every cell of both controllers: the display memory of the main
 * controller holds 12 cells and that of the secondary one 11 (vfd.c: 0x24 and 0x21 bytes, three per cell). */
#define LAMP_TEST_GLYPH 0xFFFFu
#define LAMP_TEST_MAIN_CELLS 12u
#define LAMP_TEST_SECOND_CELLS 11u

/* The splash text of an unset boot message (display.c prints the same text). */
static const char boot_msg_default[EE_BOOT_MSG_LEN + 1u] PROGMEM = "BK 5491B ";

/* ---- Calls into other modules --------------------------------------------------------------------------
 * These two live in main.c and are not declared by a header this file includes. */
bool main_warmed_up(void);          /* main.c: 30 minutes since power-on */
uint16_t main_power_on_hours(void); /* main.c: the wear-levelled hour meter */
/* Everything else this file needs is declared by meas.h, vfd.h and cal.h, which it includes. Two of
 * the names used below are shorthands for calls that read awkwardly at the call site:
 *
 *   meas_relay_exercise_stop()  ->  meas_relay_exercise_abort()
 *   meas_set_line_hz(hz)        ->  meas_set_line(hz == 50)
 *
 * and the busy test is meas_relay_exercise_result() == RLYEX_RUNNING. They are written out here rather than
 * aliased in meas.c, so that there is exactly one public name per operation. */
#define meas_relay_exercise_stop() meas_relay_exercise_abort()
#define meas_set_line_hz(hz) ((void)meas_set_line((hz) == LINE_HZ_ALTERNATE))
#define meas_relay_exercise_busy() (meas_relay_exercise_result() == RLYEX_RUNNING)

/* ---- Text ----------------------------------------------------------------------------------------------- */

/* Text comes from the shared menu_text[] table. The strings the table does not hold have local ids that
 * continue after its last one. Values are printed without the trailing blanks the table keeps, and
 * display_show_text() blank-fills the rest of the line.
 *
 * The VFD has no glyph for lower-case letters other than m k u n d, nor for ? = , _ and most
 * punctuation, so none of these strings uses one (tables.h says what a character without a glyph costs).
 * Every string is at most DISPLAY_TEXT_CELLS wide, a '.' or ':' sharing the cell before it; the host check
 * (host/test_menu_text.c) holds this list to that, and to the same rule for every screen built from it.
 *
 * The three lists below are derived from one list so that the ids, the strings and the table cannot
 * drift apart. The key names must stay consecutive and in scan-code order: key_name_text() relies on it.
 * The CAL submenu entries are consecutive and in sub_item_t order for the same reason.
 */
#define LOCAL_TEXTS(X)                                                                                       \
    X(KEY_ERROR, "BAD CODE")                                                                                 \
    X(CANNOT, "CANNOT")                                                                                      \
    X(STORE_FAILED, "SAVE FAIL")                                                                             \
    X(RELAY, "RELAY")                                                                                        \
    X(EXERCISER, "EXERCISER")                                                                                \
    X(POWER_ON, "POWER ON")                                                                                  \
    X(RELAY_PWR, "RELAY PWR")                                                                                \
    X(ITEM_LINE, "LINE FREQ")                                                                                \
    X(ITEM_CAL, "CAL")                                                                                       \
    X(ITEM_CAL_CHECK, "CAL CHECK")                                                                           \
    X(ITEM_INFO, "INFO")                                                                                     \
    X(ITEM_SELF_TEST, "SELF TEST")                                                                           \
    X(ITEM_BRIGHT, "BRIGHT")                                                                                 \
    X(ITEM_CODE, "CODE")                                                                                     \
    X(ITEM_RESET, "RESET")                                                                                   \
    X(ITEM_BOOT_MSG, "BOOT MSG")                                                                             \
    X(SUB_ZERO, "ZERO")                                                                                      \
    X(SUB_OPEN, "OPEN")                                                                                      \
    X(SUB_RANGE, "RANGE")                                                                                    \
    X(SUB_VERIFY, "VERIFY")                                                                                  \
    X(SUB_UNDO, "UNDO")                                                                                      \
    X(SUB_DATE, "DATE")                                                                                      \
    X(SUB_COUNT, "COUNT")                                                                                    \
    X(FUNCTION, "FUNCTION")                                                                                  \
    X(REFERENCE, "REFERENCE")                                                                                \
    X(REF_LABEL, "REF:")                                                                                     \
    X(NEW_CODE, "NEW CODE")                                                                                  \
    X(CONFIRM, "CONFIRM")                                                                                    \
    X(LINE_60, "60 HZ")                                                                                      \
    X(LINE_50, "50 HZ")                                                                                      \
    X(FUNC_DCA, "DCA")                                                                                       \
    X(NO_INPUT, "NO INPUT")                                                                                  \
    X(INPUT_ON, "INPUT ON")                                                                                  \
    X(OPEN_IN, "OPEN IN")                                                                                    \
    X(GAIN, "GAIN")                                                                                          \
    X(RUNNING, "RUNNING")                                                                                    \
    X(SELFCAL, "SELF CAL")                                                                                   \
    X(PRESS_KEY, "PRESS KEY")                                                                                \
    X(DONE, "DONE")                                                                                          \
    X(ABORTED, "ABORTED")                                                                                    \
    X(FAILED, "FAILED")                                                                                      \
    X(REFUSED, "REFUSED")                                                                                    \
    X(LOCKED, "LOCKED")                                                                                      \
    X(WARM_UP, "WARM UP")                                                                                    \
    X(RESTORED, "RESTORED")                                                                                  \
    X(MISMATCH, "MISMATCH")                                                                                  \
    X(CAL_OK, "CAL OK")                                                                                      \
    X(PASS, "PASS")                                                                                          \
    X(FAIL, "FAIL")                                                                                          \
    X(FACTORY, "FACTORY")                                                                                    \
    X(CHANGED, "CHANGED")                                                                                    \
    X(NOT_SET, "NOT SET")                                                                                    \
    X(LBL_VERSION, "VERSION")                                                                                \
    X(LBL_SERIAL, "SERIAL")                                                                                  \
    X(LBL_SERIAL_2, "SERIAL 2")                                                                              \
    X(LBL_CAL_DATE, "CAL DATE")                                                                              \
    X(LBL_CAL_COUNT, "CAL COUNT")                                                                            \
    X(LBL_PROD_DATE, "PROD DATE")                                                                            \
    X(LBL_HOURS, "ON HOURS")                                                                                 \
    X(LBL_POWER_UPS, "POWER UPS")                                                                            \
    X(LBL_ERROR, "ERROR %")                                                                                  \
    X(KN_DCV, "DCV")                                                                                         \
    X(KN_ACV, "ACV")                                                                                         \
    X(KN_OHM, "OHM")                                                                                         \
    X(KN_FREQ, "FREQ")                                                                                       \
    X(KN_ACDC, "ACDC")                                                                                       \
    X(KN_DIODE, "DIODE")                                                                                     \
    X(KN_UP, "UP")                                                                                           \
    X(KN_DOWN, "DOWN")                                                                                       \
    X(KN_LEFT, "LEFT")                                                                                       \
    X(KN_RIGHT, "RIGHT")                                                                                     \
    X(KN_ENTER, "ENTER")                                                                                     \
    X(KN_MENU, "MENU")                                                                                       \
    X(KN_MAXMIN, "MAX MIN")                                                                                  \
    X(KN_TRIG, "TRIG")                                                                                       \
    X(KN_REL, "REL")                                                                                         \
    X(KN_RATE, "RATE")                                                                                       \
    X(KN_SHIFT, "SHIFT")

typedef enum {
    LT_BASE = MT_COUNT - 1,
#define X(name, text) LT_##name,
    LOCAL_TEXTS(X)
#undef X
        LT_COUNT
} local_text_id_t;

#define X(name, text) static const char lt_##name[] PROGMEM = text;
LOCAL_TEXTS(X)
#undef X

static const char *const local_texts[LT_COUNT - MT_COUNT] PROGMEM = {
#define X(name, text) lt_##name,
    LOCAL_TEXTS(X)
#undef X
};

static const char *text_P(uint8_t id) {
    if (id < MT_COUNT) {
        return (const char *)pgm_read_ptr(&menu_text[id]);
    }
    return (const char *)pgm_read_ptr(&local_texts[id - MT_COUNT]);
}

/* ---- Menu structure ------------------------------------------------------------------------------------- */

typedef enum {
    KIND_NUMERIC,
    KIND_INTEGER,
    KIND_CHOICE,
    KIND_REVISION,
    KIND_ZERO_CAL,
    KIND_RLY_EXER,
    KIND_CAL_MENU,
    KIND_CAL_CHECK,
    KIND_INFO,
    KIND_SELF_TEST,
    KIND_CODE,
    KIND_RESET,
    KIND_BOOT_MSG
} kind_t;

typedef enum { NUM_LIMIT_HI, NUM_LIMIT_LO, NUM_PERCENT_REF, NUM_DB_REF, NUM_CAL_REF, NUM_COUNT } numeric_id_t;
typedef enum {
    INT_DBM_REF,
    INT_HOLD_COUNT,
    INT_KEY_CODE,
    INT_NEW_CODE,
    INT_NEW_CODE_AGAIN,
    INT_COUNT
} integer_id_t;
/* The three range choices follow CH_CAL_FUNC in the order of cal_funcs[]: the function choice index
 * selects the range choice by addition. */
typedef enum {
    CH_TRIG_MODE,
    CH_HOLD_WINDOW,
    CH_BEEP,
    CH_BAUD,
    CH_TERMINATOR,
    CH_ECHO,
    CH_KEY_CLICK,
    CH_RLY_BOOT,
    CH_LINE,
    CH_BRIGHT,
    CH_CAL_FUNC,
    CH_RANGE_DCV,
    CH_RANGE_DCA,
    CH_RANGE_OHM,
    CH_COUNT
} choice_id_t;

/* The functions the panel can calibrate (DC V, DC A and ohms only; AC stays with
 * BK-Calibrator). The index is the CH_CAL_FUNC choice. */
static const uint8_t cal_funcs[] PROGMEM = {FUNC_DCV, FUNC_DCA, FUNC_OHM};

/* CAL submenu, in the order the screens appear. The screen titles are LT_SUB_ZERO onwards. */
typedef enum { SUB_ZERO, SUB_OPEN, SUB_RANGE, SUB_VERIFY, SUB_UNDO, SUB_DATE, SUB_COUNT } sub_item_t;

/* Items that sit behind the service code. ZERO CAL (item 7) always did. */
static bool item_is_protected(uint8_t kind) {
    return kind == KIND_ZERO_CAL || kind == KIND_CAL_MENU || kind == KIND_CODE || kind == KIND_RESET;
}

typedef struct {
    uint8_t title;  /* text id, the main line */
    uint8_t title2; /* text id of the second line, or NO_TEXT when the name fits one line */
    uint8_t kind;
    uint8_t param;
} item_def_t;

typedef struct {
    uint8_t title; /* text id */
    const item_def_t *items;
    uint8_t count;
} page_def_t;

static const item_def_t math_items[] PROGMEM = {
    {MT_ITEM_HI_LIMIT, NO_TEXT, KIND_NUMERIC, NUM_LIMIT_HI},
    {MT_ITEM_LO_LIMIT, NO_TEXT, KIND_NUMERIC, NUM_LIMIT_LO},
    {MT_ITEM_PERC_REF, NO_TEXT, KIND_NUMERIC, NUM_PERCENT_REF},
    {MT_ITEM_DB_REF, NO_TEXT, KIND_NUMERIC, NUM_DB_REF},
    {MT_ITEM_DBM_REF, NO_TEXT, KIND_INTEGER, INT_DBM_REF},
};

static const item_def_t trig_items[] PROGMEM = {
    {MT_ITEM_TRIG_MODE, NO_TEXT, KIND_CHOICE, CH_TRIG_MODE},
    {MT_ITEM_HOLD_WIN, NO_TEXT, KIND_CHOICE, CH_HOLD_WINDOW},
    {MT_ITEM_HOLD_CNT, NO_TEXT, KIND_INTEGER, INT_HOLD_COUNT},
};

/* The SYS items in the order of the original's numbers: items 1 to 7 are the vendor's, items 8 to 18 are
 * this firmware's. The panel no longer shows the numbers. */
static const item_def_t sys_items[] PROGMEM = {
    {MT_ITEM_BEEPER, NO_TEXT, KIND_CHOICE, CH_BEEP},
    {MT_ITEM_BAUD, NO_TEXT, KIND_CHOICE, CH_BAUD},
    {MT_ITEM_LINE_END, NO_TEXT, KIND_CHOICE, CH_TERMINATOR},
    {MT_ITEM_ECHO, NO_TEXT, KIND_CHOICE, CH_ECHO},
    {MT_ITEM_KEY_SOUND, NO_TEXT, KIND_CHOICE, CH_KEY_CLICK},
    {MT_ITEM_REVISION, NO_TEXT, KIND_REVISION, 0u},
    {MT_ITEM_ZERO_CAL, NO_TEXT, KIND_ZERO_CAL, 0u},
    {LT_RELAY, LT_EXERCISER, KIND_RLY_EXER, 0u},
    {LT_RELAY, LT_POWER_ON, KIND_CHOICE, CH_RLY_BOOT},
    {LT_ITEM_LINE, NO_TEXT, KIND_CHOICE, CH_LINE},
    {LT_ITEM_CAL, NO_TEXT, KIND_CAL_MENU, 0u},
    {LT_ITEM_CAL_CHECK, NO_TEXT, KIND_CAL_CHECK, 0u},
    {LT_ITEM_INFO, NO_TEXT, KIND_INFO, 0u},
    {LT_ITEM_SELF_TEST, NO_TEXT, KIND_SELF_TEST, 0u},
    {LT_ITEM_BRIGHT, NO_TEXT, KIND_CHOICE, CH_BRIGHT},
    {LT_ITEM_CODE, NO_TEXT, KIND_CODE, 0u},
    {LT_ITEM_RESET, NO_TEXT, KIND_RESET, 0u},
    {LT_ITEM_BOOT_MSG, NO_TEXT, KIND_BOOT_MSG, 0u},
};

typedef enum { PAGE_MATH, PAGE_TRIG, PAGE_SYS, PAGE_COUNT } page_t;

#define ARRAY_COUNT(a) ((uint8_t)(sizeof(a) / sizeof((a)[0])))

static const page_def_t pages[PAGE_COUNT] PROGMEM = {
    {MT_MENU_MATH, math_items, ARRAY_COUNT(math_items)},
    {MT_MENU_TRIG, trig_items, ARRAY_COUNT(trig_items)},
    {MT_MENU_SYS, sys_items, ARRAY_COUNT(sys_items)},
};

/* Every editor shows its value on the main line, from cell 0, and says what it is editing on the second
 * line: the `caption` text id. The original put a prompt before the value ("HI:", "R.F:", "RDGS:"), which
 * left the value two to four cells and forced the name down to what fitted beside it. */
typedef struct {
    uint8_t caption; /* text id */
} numeric_def_t;

static const numeric_def_t numeric_defs[NUM_COUNT] PROGMEM = {
    {MT_ITEM_HI_LIMIT}, {MT_ITEM_LO_LIMIT}, {MT_ITEM_PERC_REF},
    {MT_ITEM_DB_REF},   {LT_REFERENCE}, /* NUM_CAL_REF: the value applied to the input during CAL RANGE / VERIFY */
};

typedef struct {
    uint8_t caption; /* text id */
    uint8_t digits;
    char suffix; /* printed after the digits, or 0 */
} integer_def_t;

static const integer_def_t integer_defs[INT_COUNT] PROGMEM = {
    {MT_ITEM_DBM_REF, 4u, '&'}, /* '&' is the ohm glyph */
    {MT_ITEM_HOLD_CNT, 3u, '\0'},         {LT_ITEM_CODE, KEY_CODE_DIGITS, '\0'},
    {LT_NEW_CODE, KEY_CODE_DIGITS, '\0'}, {LT_CONFIRM, KEY_CODE_DIGITS, '\0'},
};

typedef struct {
    uint8_t caption;      /* text id */
    const uint8_t *texts; /* text ids */
    uint8_t count;
} choice_def_t;

static const uint8_t trig_texts[] PROGMEM = {MT_IMM, MT_MAN, MT_BUS};
static const uint8_t window_texts[] PROGMEM = {MT_PCT0_01, MT_PCT0_1, MT_PCT1, MT_PCT10};
static const uint8_t switch_texts[] PROGMEM = {MT_OFF, MT_ON};
static const uint8_t baud_texts[] PROGMEM = {MT_BAUD38K4, MT_BAUD19K2, MT_BAUD9600, MT_BAUD4800,
                                             MT_BAUD2400, MT_BAUD1200, MT_BAUD600};
static const uint8_t terminator_texts[] PROGMEM = {MT_LF, MT_CR};
/* Index 0 is stored value 0 (60 Hz), index 1 is stored value 1 (50 Hz). */
static const uint8_t line_texts[] PROGMEM = {LT_LINE_60, LT_LINE_50};
static const uint8_t func_texts[] PROGMEM = {MT_LBL_DCV, LT_FUNC_DCA, MT_LBL_OHM};
/* Range order is the EEPROM order (eeprom_map.h): highest range first. */
static const uint8_t dcv_range_texts[] PROGMEM = {MT_RANGE_1000V, MT_RANGE_500V, MT_RANGE_50V, MT_RANGE_5V,
                                                  MT_RANGE_500MV};
static const uint8_t dca_range_texts[] PROGMEM = {MT_RANGE_20A, MT_RANGE_5A, MT_RANGE_500MA, MT_RANGE_50MA,
                                                  MT_RANGE_5MA};
static const uint8_t ohm_range_texts[] PROGMEM = {MT_RANGE_50M, MT_RANGE_5M, MT_RANGE_500K,
                                                  MT_RANGE_50K, MT_RANGE_5K, MT_RANGE_500R};

static const choice_def_t choice_defs[CH_COUNT] PROGMEM = {
    {MT_ITEM_TRIG_MODE, trig_texts, ARRAY_COUNT(trig_texts)},
    {MT_ITEM_HOLD_WIN, window_texts, ARRAY_COUNT(window_texts)},
    {MT_ITEM_BEEPER, switch_texts, ARRAY_COUNT(switch_texts)},
    {MT_ITEM_BAUD, baud_texts, ARRAY_COUNT(baud_texts)},
    {MT_ITEM_LINE_END, terminator_texts, ARRAY_COUNT(terminator_texts)},
    {MT_ITEM_ECHO, switch_texts, ARRAY_COUNT(switch_texts)},
    {MT_ITEM_KEY_SOUND, switch_texts, ARRAY_COUNT(switch_texts)},
    {LT_RELAY_PWR, switch_texts, ARRAY_COUNT(switch_texts)},
    {LT_ITEM_LINE, line_texts, ARRAY_COUNT(line_texts)},
    {LT_ITEM_BRIGHT, NULL, BRIGHT_CHOICES}, /* the value is drawn by line_put_choice() */
    {LT_FUNCTION, func_texts, ARRAY_COUNT(func_texts)},
    {LT_SUB_RANGE, dcv_range_texts, ARRAY_COUNT(dcv_range_texts)},
    {LT_SUB_RANGE, dca_range_texts, ARRAY_COUNT(dca_range_texts)},
    {LT_SUB_RANGE, ohm_range_texts, ARRAY_COUNT(ohm_range_texts)},
};
_Static_assert(sizeof cal_funcs == 3u, "CH_RANGE_DCV..CH_RANGE_OHM follow CH_CAL_FUNC, one per function");

/* Hold window fraction for each HOLD WIN choice: 0.01 %, 0.1 %, 1 %, 10 % (RAM
 * 0x0E2D in the original). */
static const float hold_window_values[] PROGMEM = {1e-4f, 1e-3f, 1e-2f, 1e-1f};
#define HOLD_WINDOW_CHOICES ARRAY_COUNT(hold_window_values)

/* ---- State ---------------------------------------------------------------------------------------------- */

typedef enum {
    MS_CLOSED,
    MS_PAGE,     /* level 0: MATH MENU / TRIG MENU / SYS MENU */
    MS_ITEM,     /* level 1: the items of one page */
    MS_CHOICE,   /* pick one of a few texts with LEFT / RIGHT */
    MS_NUMERIC,  /* engineering-notation number with a cursor */
    MS_INTEGER,  /* fixed-width digit field */
    MS_REVISION, /* read-only */
    MS_MESSAGE,  /* timed message, keys ignored */
    MS_SUB,      /* the CAL submenu */
    MS_PROMPT,   /* a blinking prompt: ENTER does it, any other key cancels */
    MS_BUSY,     /* something is running; menu_tick() watches it, a key aborts it */
    MS_VIEW,     /* read-only pages: INFO, CAL CHECK, VERIFY, COUNT */
    MS_TEXTEDIT  /* character editor: CAL DATE, BOOT MSG */
} menu_state_t;

/* What happens when a timed message ends. */
typedef enum {
    NEXT_ITEMS, /* back to the item list (or the CAL submenu when it is open) */
    NEXT_RESTART,
    NEXT_CONFIRM,
    NEXT_VIEW,  /* show the value of the current view page */
    NEXT_PROMPT /* show the prompt that was prepared before the message */
} after_message_t;

typedef enum { ACT_EXER, ACT_SWEEP, ACT_ZERO_CAL, ACT_SELF_TEST } activity_t;
typedef enum { ST_LAMPS, ST_KEYS, ST_SELFCAL } self_test_step_t;

/* Which calibration activity a sweep belongs to; decides what is shown when it ends. */
typedef enum { FLOW_ZERO, FLOW_OPEN, FLOW_RANGE, FLOW_VERIFY } sweep_flow_t;

typedef enum {
    PA_EXER_PROMPT, /* from SELF TEST: go on to the NO INPUT prompt */
    PA_EXER_START,
    PA_WARM_OK, /* the warm-up warning was shown and accepted: go on to choose the range */
    PA_ZERO,
    PA_USER_ZERO, /* SYS item 7: capture and store the user zero of every DC V and ohm range */
    PA_OPEN,
    PA_RANGE_COMMIT, /* the measured CAL RANGE point and its change were shown: ENTER writes it */
    PA_UNDO,
    PA_RESET
} prompt_action_t;

typedef enum { VIEW_INFO, VIEW_CHECK, VIEW_VERIFY, VIEW_COUNT } view_t;
typedef enum { TEXT_DATE, TEXT_BOOT_MSG } text_edit_t;

typedef struct {
    uint8_t func;     /* index into cal_funcs[] */
    uint8_t range;    /* EEPROM range index of that function */
    float applied;    /* the value on the input, in volts, amperes or ohms */
    bool verify;      /* VERIFY rather than RANGE: nothing is written */
    bool accept_cold; /* the warm-up warning was shown and the operator chose to continue */
} cal_target_t;

typedef struct {
    menu_state_t state;
    menu_state_t resume; /* editor state to return to after an error message */
    after_message_t after;
    bool in_sub; /* the CAL submenu is open: "back" means back to it */
    uint8_t sub;
    uint8_t pending_kind; /* the protected item the KEY prompt is unlocking */
    uint8_t activity;
    uint8_t step;
    uint8_t shown; /* the last value drawn by a running activity */
    uint8_t last_key;
    uint8_t menu_presses;
    uint16_t timer_ms;
    uint8_t prompt_text;    /* text id */
    uint8_t prompt_caption; /* text id of the second line, or NO_TEXT */
    uint8_t prompt_action;
    uint8_t flow;
    cal_target_t cal;
    uint8_t view;
    uint8_t view_page;
    uint8_t view_pages;
    float verify_deviation;
    bool verify_pass;
    float range_change; /* the fractional change the measured CAL RANGE point would make */
    uint8_t text_kind;
    uint8_t text_len;
    char code[KEY_CODE_DIGITS + 1u]; /* the first entry of a new service code, until it is confirmed */
    uint16_t message_ms;
    uint16_t blink_ms;
    bool blink_on;
    bool shortcut;       /* a reference editor opened straight from the SHIFT layer */
    uint8_t preset_item; /* MATH item the menu shows next time, set when a shortcut editor is confirmed */
    uint8_t page;
    uint8_t item[PAGE_COUNT]; /* the selected item of each page survives closing
                                 the menu */
    uint8_t param;            /* numeric / integer / choice id of the open editor */
    uint8_t cursor;
    uint8_t choice;
    float value;
    char text[TEXT_CAPACITY]; /* edit text or character-editor text, always NUL terminated */
    char digits[MAX_DIGITS + 1u];
} menu_ctx_t;

static menu_ctx_t m;

/* ---- One line of display text -------------------------------------------------------------------------- */

typedef struct {
    char buf[LINE_CAPACITY];
    uint8_t len;
    uint8_t cells; /* display cells used: '.' and ':' share the cell before them */
} line_t;

static void line_putc(line_t *l, char c) {
    if (l->len + 1u >= LINE_CAPACITY) {
        return;
    }
    const bool shares_cell = (c == '.' || c == ':') && l->cells > 0u;

    l->buf[l->len++] = c;
    l->buf[l->len] = '\0';
    if (!shares_cell) {
        l->cells++;
    }
}

static void line_puts_P(line_t *l, const char *s_flash) {
    for (char c = (char)pgm_read_byte(s_flash); c != '\0'; c = (char)pgm_read_byte(++s_flash)) {
        line_putc(l, c);
    }
}

/* Length of a flash text without its trailing blanks. */
static uint8_t trimmed_length_P(const char *s_flash) {
    uint8_t length = 0u;

    for (uint8_t i = 0u; pgm_read_byte(s_flash + i) != '\0'; i++) {
        if (pgm_read_byte(s_flash + i) != ' ') {
            length = (uint8_t)(i + 1u);
        }
    }
    return length;
}

static void line_puts_trimmed_P(line_t *l, const char *s_flash) {
    const uint8_t length = trimmed_length_P(s_flash);

    for (uint8_t i = 0u; i < length; i++) {
        line_putc(l, (char)pgm_read_byte(s_flash + i));
    }
}

/* The same width as s_flash, drawn blank (the off phase of a blinking value).
 */
static void line_blank_like_P(line_t *l, const char *s_flash) {
    const uint8_t length = trimmed_length_P(s_flash);

    for (uint8_t i = 0u; i < length; i++) {
        const char c = (char)pgm_read_byte(s_flash + i);

        if (c != '.' && c != ':') {
            line_putc(l, ' ');
        }
    }
}

/* ---- Engineering-notation text editing ----------------------------------------------------------------- */

static bool is_prefix(char c) {
    return c == 'M' || c == 'k' || c == '^' || c == 'm';
}

static bool is_digit(char c) {
    return c >= '0' && c <= '9';
}

/* Index of the decimal point in an edit text; the text always has one in
 * positions 2..6. */
static uint8_t point_index(const char *text) {
    for (uint8_t i = 1u; i < PREFIX_INDEX; i++) {
        if (text[i] == '.') {
            return i;
        }
    }
    return PREFIX_INDEX - 1u;
}

/* Text for v: sign, five digits with the point placed so the magnitude fits,
 * and the SI prefix M, k, ^ (none) or m. The thresholds are the original's
 * (flash 0x497, 0x48B, 0x47F and 0x4A7, 0x4A3, 0x4AF). */
static void eng_format(char *text, float v) {
    const float mag = fabsf(v);
    float scaled;
    char prefix;

    if (mag >= 1e6f) {
        scaled = v / 1e6f;
        prefix = 'M';
    } else if (mag >= 1e3f) {
        scaled = v / 1e3f;
        prefix = 'k';
    } else if (mag >= 1.0f) {
        scaled = v;
        prefix = '^';
    } else {
        scaled = v * 1e3f;
        prefix = 'm';
    }

    const float smag = fabsf(scaled);
    uint8_t decimals;
    float factor;

    if (smag >= 999.995f) {
        decimals = 1u;
        factor = 10.0f;
    } else if (smag >= 99.9995f) {
        decimals = 2u;
        factor = 100.0f;
    } else if (smag >= 9.99995f) {
        decimals = 3u;
        factor = 1000.0f;
    } else {
        decimals = 4u;
        factor = 10000.0f;
    }

    uint32_t units = (uint32_t)(smag * factor + 0.5f);
    if (units > 99999UL) {
        units = 99999UL;
    }

    char digits[5];
    for (uint8_t i = 5u; i > 0u; i--) {
        digits[i - 1u] = (char)('0' + (units % 10u));
        units /= 10u;
    }

    const uint8_t integer_digits = (uint8_t)(5u - decimals);
    uint8_t pos = 0u;

    text[pos++] = (v < 0.0f) ? '-' : '+';
    for (uint8_t i = 0u; i < 5u; i++) {
        if (i == integer_digits) {
            text[pos++] = '.';
        }
        text[pos++] = digits[i];
    }
    text[pos++] = prefix;
    text[pos] = '\0';
}

/* The value an edit text stands for. */
static float eng_parse(const char *text) {
    static const float powers[] = {1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f};
    const uint8_t point = point_index(text);
    uint32_t number = 0u;
    uint8_t decimals = 0u;

    for (uint8_t i = 1u; i < PREFIX_INDEX; i++) {
        if (is_digit(text[i])) {
            number = number * 10u + (uint32_t)(text[i] - '0');
            if (i > point) {
                decimals++;
            }
        }
    }

    float v = (float)number;
    switch (text[PREFIX_INDEX]) {
    case 'M':
        v = v / powers[decimals] * 1e6f;
        break;
    case 'k':
        v = v / powers[decimals] * 1e3f;
        break;
    case 'm':
        v = v / (powers[decimals] * 1e3f);
        break;
    default:
        v = v / powers[decimals];
        break;
    }
    return (text[0] == '-') ? -v : v;
}

/* UP on the cursor cell (original 0x3C02). A digit steps up and wraps 9 -> 0
 * without carry; the sign flips; on the prefix the number is multiplied by ten
 * and re-normalised. Returns the new value. */
static float ui_edit_digit_up(char *text, uint8_t cursor, float value) {
    const char c = text[cursor];

    if (is_digit(c)) {
        text[cursor] = (c == '9') ? '0' : (char)(c + 1);
        return eng_parse(text);
    }
    if (c == '+' || c == '-') {
        text[0] = (c == '+') ? '-' : '+';
        return -value;
    }
    if (is_prefix(c) && fabsf(value) < LIMIT_ABS_MAX) {
        /* Moving the point right is only possible while a digit remains behind it.
         * The original swapped the point with the prefix character here once the
         * point reached the last position. */
        const uint8_t point = point_index(text);

        if (point + 1u < PREFIX_INDEX) {
            text[point] = text[point + 1u];
            text[point + 1u] = '.';
            value = eng_parse(text);
            eng_format(text, value);
        }
    }
    return value;
}

/* DOWN on the cursor cell (original 0x3D34): the mirror image. On the prefix
 * the number is divided by ten; nothing happens once it is already in the
 * smallest representable decade (point at position 2 with 'm'). */
static float ui_edit_digit_down(char *text, uint8_t cursor, float value) {
    const char c = text[cursor];

    if (is_digit(c)) {
        text[cursor] = (c == '0') ? '9' : (char)(c - 1);
        return eng_parse(text);
    }
    if (c == '+' || c == '-') {
        text[0] = (c == '+') ? '-' : '+';
        return -value;
    }
    if (!is_prefix(c)) {
        return value;
    }

    const uint8_t point = point_index(text);

    if (point == 2u) {
        char lower;

        if (c == 'M') {
            lower = 'k';
        } else if (c == 'k') {
            lower = '^';
        } else if (c == '^') {
            lower = 'm';
        } else {
            return value;
        }
        /* One decade down within the next lower prefix: x.xxxx -> xxx.xx */
        text[cursor] = lower;
        text[2] = text[3];
        text[3] = text[4];
        text[4] = '.';
    } else {
        text[point] = text[point - 1u];
        text[point - 1u] = '.';
    }
    value = eng_parse(text);
    eng_format(text, value);
    return value;
}

/* The first cell the cursor may stand on. A dB reference voltage is never negative, so its editor has
 * no sign cell: text[0] stays '+' and the cursor starts at the first digit. HI LIMIT, LO LIMIT
 * and PERC REF keep the sign. `state` is the editor the question is about, which is not m.state while a
 * message is up. */
static uint8_t first_cursor(menu_state_t state) {
    return (state == MS_NUMERIC && m.param == NUM_DB_REF) ? 1u : 0u;
}

/* Next cursor position over the editable cells; the point is not one. */
static uint8_t text_cursor_step(const char *text, uint8_t first, uint8_t cursor, bool forward) {
    for (uint8_t i = 0u; i < EDIT_TEXT_LEN; i++) {
        if (forward) {
            cursor = (cursor + 1u >= EDIT_TEXT_LEN) ? first : (uint8_t)(cursor + 1u);
        } else {
            cursor = (cursor <= first) ? (uint8_t)(EDIT_TEXT_LEN - 1u) : (uint8_t)(cursor - 1u);
        }
        if (text[cursor] != '.') {
            break;
        }
    }
    return cursor;
}

/* ---- Fixed-width digit fields -------------------------------------------------------------------------- */

static void digits_from_value(char *digits, uint8_t count, uint16_t value) {
    uint16_t limit = 1u;

    for (uint8_t i = 0u; i < count; i++) {
        limit = (uint16_t)(limit * 10u);
    }
    if (value >= limit) {
        value = (uint16_t)(limit - 1u);
    }
    for (uint8_t i = count; i > 0u; i--) {
        digits[i - 1u] = (char)('0' + (value % 10u));
        value = (uint16_t)(value / 10u);
    }
    digits[count] = '\0';
}

static uint16_t digits_to_value(const char *digits) {
    uint16_t value = 0u;

    for (const char *p = digits; *p != '\0'; p++) {
        value = (uint16_t)(value * 10u + (uint16_t)(*p - '0'));
    }
    return value;
}

/* One digit steps up or down and wraps at the ends without carrying into its
 * neighbours. */
static void digit_step(char *digit, bool up) {
    if (up) {
        *digit = (*digit == '9') ? '0' : (char)(*digit + 1);
    } else {
        *digit = (*digit == '0') ? '9' : (char)(*digit - 1);
    }
}

/* ---- Stored values behind SYS items 9, 10, 13 and 15 ---------------------------------------------------- */

/* EEPROM cells have a limited write life, so a byte that already holds the value is not written again. */
static bool store_setting_byte(uint16_t address, uint8_t value) {
    return ee_read_u8(address) == value || ee_write_u8(address, value);
}

/* Every reader of the bytes this firmware added treats an erased byte as "not set, use the default". */
static uint8_t rly_boot_choice(void) {
    /* Inverted against the others on purpose: this setting defaults ON, so the test names the OFF value.
     * See eeprom_map.h. */
    return (ee_read_u8(EE_SET_RLY_BOOT) == RLY_BOOT_STORED_OFF) ? (uint8_t)0u : (uint8_t)1u;
}

static uint8_t line_choice(void) {
    return ee_read_u8(EE_SET_LINE) == LINE_STORED_50HZ ? 1u : 0u;
}

static uint8_t bright_level(void) {
    const uint8_t stored = ee_read_u8(EE_SET_BRIGHT);

    if (stored > BRIGHT_MAX) { /* erased, or not a level */
        return BRIGHT_DEFAULT;
    }
    return stored;
}

static uint16_t cal_count(void) {
    const uint16_t count = ee_read_u16(EE_SAVE_COUNTER);

    return (count == 0xFFFFu) ? 0u : count;
}

static uint16_t power_up_count(void) {
    const uint16_t count = ee_read_u16(EE_PWRUP_COUNT);

    return (count == 0xFFFFu) ? 0u : count;
}

/* ---- Text fields read from the EEPROM ------------------------------------------------------------------ */

#define FIELD_MAX 20u /* the longest field shown: the serial number */

static char upper_ascii(char c) {
    return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

/* A text field as the panel shows it: printable characters only, in upper case, up to the first byte that
 * is not one (an erased field starts with 0xFF), without trailing blanks. out has room for length + 1.
 * Returns the length. */
static uint8_t read_field(uint16_t address, uint8_t length, char *out) {
    char raw[FIELD_MAX];
    uint8_t n = 0u;

    if (length > FIELD_MAX) {
        length = FIELD_MAX;
    }
    ee_read_block(address, raw, length);
    while (n < length && raw[n] >= ' ' && raw[n] <= '~') {
        out[n] = upper_ascii(raw[n]);
        n++;
    }
    while (n > 0u && out[n - 1u] == ' ') {
        n--;
    }
    out[n] = '\0';
    return n;
}

/* ---- The character picker of BOOT MSG ------------------------------------------------------------------ */

/* The display can show only what its glyph tables hold, and a boot message made of anything else would be
 * a permanent unreadable splash. So the picker is built from those tables: the space (a blank cell, on
 * purpose), each digit and letter whose glyph word is not zero, and each symbol of glyph_symbol[] whose
 * glyph is not zero. The symbol characters are listed in the table's own order (tables.h: m k u @ - + % /
 * & n d). ':' and ';' of glyph_digit[] are not offered: a '.' or ':' in a text merges into the cell
 * before it instead of taking one, so they cannot occupy a cell of a message. */
#define PICKER_DIGITS 10u
#define PICKER_SLOTS (1u + PICKER_DIGITS + GLYPH_ALPHA_COUNT + GLYPH_SYMBOL_COUNT)

static const char picker_symbols[GLYPH_SYMBOL_COUNT + 1u] PROGMEM = "mku@-+%/&nd";

/* The character of a picker slot, or '\0' when the table has no visible glyph for it. */
static char picker_slot_char(uint8_t slot) {
    if (slot == 0u) {
        return ' ';
    }
    slot--;
    if (slot < PICKER_DIGITS) {
        return (tbl_glyph_digit(slot) != 0u) ? (char)('0' + slot) : '\0';
    }
    slot = (uint8_t)(slot - PICKER_DIGITS);
    if (slot < GLYPH_ALPHA_COUNT) {
        return (tbl_glyph_alpha(slot) != 0u) ? (char)('A' + slot) : '\0';
    }
    slot = (uint8_t)(slot - GLYPH_ALPHA_COUNT);
    return (tbl_glyph_symbol(slot) != 0u) ? (char)pgm_read_byte(&picker_symbols[slot]) : '\0';
}

/* The slot of c, or the space's slot when c is not in the picker. */
static uint8_t picker_slot_of(char c) {
    for (uint8_t slot = 0u; slot < PICKER_SLOTS; slot++) {
        if (c != '\0' && picker_slot_char(slot) == c) {
            return slot;
        }
    }
    return 0u;
}

/* A stored character the display cannot show becomes a blank. */
static char picker_sanitise(char c) {
    const char shown = picker_slot_char(picker_slot_of(c));

    return (shown == c) ? c : ' ';
}

/* The next (or previous) character the display can show; wraps. */
static char picker_step(char c, bool up) {
    uint8_t slot = picker_slot_of(c);

    do {
        if (up) {
            slot = (slot + 1u >= PICKER_SLOTS) ? 0u : (uint8_t)(slot + 1u);
        } else {
            slot = (slot == 0u) ? (uint8_t)(PICKER_SLOTS - 1u) : (uint8_t)(slot - 1u);
        }
    } while (picker_slot_char(slot) == '\0');
    return picker_slot_char(slot);
}

/* ---- Names for the key test --------------------------------------------------------------------------- */

_Static_assert(LT_KN_SHIFT - LT_KN_DCV == KEY_SHIFT - KEY_DCV, "key names are in scan-code order");

static bool is_panel_key(uint8_t key) {
    return key >= KEY_DCV && key <= KEY_SHIFT;
}

static uint8_t key_name_text(uint8_t key) {
    return (uint8_t)(LT_KN_DCV + (key - KEY_DCV));
}

/* ---- Screen --------------------------------------------------------------------------------------------- */

static page_def_t page_def(uint8_t page) {
    page_def_t def;

    memcpy_P(&def, &pages[page], sizeof def);
    return def;
}

static item_def_t current_item_def(void) {
    const page_def_t page = page_def(m.page);
    item_def_t def;

    memcpy_P(&def, &page.items[m.item[m.page]], sizeof def);
    return def;
}

static numeric_def_t numeric_def(uint8_t id) {
    numeric_def_t def;

    memcpy_P(&def, &numeric_defs[id], sizeof def);
    return def;
}

static integer_def_t integer_def(uint8_t id) {
    integer_def_t def;

    memcpy_P(&def, &integer_defs[id], sizeof def);
    return def;
}

static choice_def_t choice_def(uint8_t id) {
    choice_def_t def;

    memcpy_P(&def, &choice_defs[id], sizeof def);
    return def;
}

static const char *choice_text(const choice_def_t *def, uint8_t index) {
    return text_P(pgm_read_byte(&def->texts[index]));
}

/* A text id for the second line: NULL, which clears the line, when there is none. */
static const char *caption_P(uint8_t id) {
    return (id == NO_TEXT) ? NULL : text_P(id);
}

static void line_put_text(line_t *l, const char *s) {
    for (; *s != '\0'; s++) {
        line_putc(l, *s);
    }
}

static void line_put_u16(line_t *l, uint16_t value) {
    char digits[5];
    uint8_t n = 0u;

    do {
        digits[n++] = (char)('0' + (value % 10u));
        value = (uint16_t)(value / 10u);
    } while (value != 0u);
    while (n > 0u) {
        line_putc(l, digits[--n]);
    }
}

/* The value of the open choice editor, or blank cells of the same width in the off phase of the blink.
 * BRIGHT shows its step number, 1 (dimmest) to 8 (brightest), one per chip dimming code. */
static void line_put_choice(line_t *l, const choice_def_t *def, bool visible) {
    if (m.param == CH_BRIGHT) {
        line_t value = {.len = 0u, .cells = 0u};

        line_put_u16(&value, (uint16_t)(m.choice + 1u));
        for (uint8_t i = 0u; i < value.len; i++) {
            line_putc(l, visible ? value.buf[i] : ' ');
        }
    } else if (visible) {
        line_puts_trimmed_P(l, choice_text(def, m.choice));
    } else {
        line_blank_like_P(l, choice_text(def, m.choice));
    }
}

/* A field that may be empty or erased. */
static void line_put_field(line_t *l, const char *field, uint8_t length) {
    if (length == 0u) {
        line_puts_P(l, text_P(LT_NOT_SET));
    } else {
        line_put_text(l, field);
    }
}

/* ---- Paged views: INFO, CAL CHECK, VERIFY, COUNT ------------------------------------------------------- */

typedef enum {
    INFO_VERSION,
    INFO_SERIAL,
    INFO_SERIAL_2, /* the rest of a serial number longer than one line */
    INFO_CAL_DATE,
    INFO_CAL_COUNT,
    INFO_PROD_DATE,
    INFO_HOURS,
    INFO_POWER_UPS,
    INFO_PAGES
} info_page_t;

typedef enum { VERIFY_PAGE_RESULT, VERIFY_PAGE_ERROR, VERIFY_PAGES } verify_page_t;

static const char firmware_version[] PROGMEM = FW_VERSION;

/* The firmware release, "VerX.Y.Z.yy.mm.dd" shown as "VX.Y.Z": the word "Ver" has lower-case letters the display
 * cannot draw, and the whole string with its date is ten cells on a nine-cell line, so the page stops at the
 * third '.' (the date stays in *IDN? and :SYST:INFO?). Shown by INFO page 1 and by SYS item 6, REVISION: the
 * original's REVISION screen printed the EEPROM model name and the constant "001" and never a version
 * (0xEDF6-0xEE22, ORIGINAL_BUGS 92). */
static void render_version(line_t *l) {
    const char *version = firmware_version;
    uint8_t dots = 0u;
    char c;

    if (pgm_read_byte(version) == 'V' && pgm_read_byte(version + 1) == 'e' &&
        pgm_read_byte(version + 2) == 'r') {
        version += 3;
    }
    line_putc(l, 'V');
    while ((c = (char)pgm_read_byte(version)) != '\0') {
        if (c == '.' && ++dots == 3u) {
            break;
        }
        line_putc(l, c);
        version++;
    }
}

_Static_assert(LT_LBL_POWER_UPS - LT_LBL_VERSION == INFO_POWER_UPS - INFO_VERSION,
               "the INFO captions follow the INFO pages");

/* The caption shown for 0.7 s before the value of a page, or NO_TEXT when the value explains itself. */
static uint8_t view_label(void) {
    switch (m.view) {
    case VIEW_INFO:
        return (uint8_t)(LT_LBL_VERSION + m.view_page);
    case VIEW_VERIFY:
        return (m.view_page == VERIFY_PAGE_ERROR) ? (uint8_t)LT_LBL_ERROR : NO_TEXT;
    case VIEW_COUNT:
        return LT_LBL_CAL_COUNT;
    default:
        return NO_TEXT;
    }
}

static bool view_page_valid(uint8_t page) {
    char serial[FIELD_MAX + 1u];

    if (m.view == VIEW_INFO && page == INFO_SERIAL_2) {
        return read_field(EE_ID_SERIAL, FIELD_MAX, serial) > VFD_TEXT_CELLS;
    }
    return true;
}

static void render_info(line_t *l) {
    char field[FIELD_MAX + 1u];
    uint8_t length;

    switch (m.view_page) {
    case INFO_VERSION:
        render_version(l);
        break;
    case INFO_SERIAL:
    case INFO_SERIAL_2: {
        const uint8_t first = (m.view_page == INFO_SERIAL_2) ? VFD_TEXT_CELLS : 0u;

        length = read_field(EE_ID_SERIAL, FIELD_MAX, field);
        if (length <= first) {
            line_put_field(l, field, 0u);
        } else {
            for (uint8_t i = first; i < length && i < first + VFD_TEXT_CELLS; i++) {
                line_putc(l, field[i]);
            }
        }
        break;
    }
    case INFO_CAL_DATE:
        length = read_field(EE_CAL_DATE, DATE_CHARS, field);
        line_put_field(l, field, length);
        break;
    case INFO_CAL_COUNT:
        line_put_u16(l, cal_count());
        break;
    case INFO_PROD_DATE:
        length = read_field(EE_THID_PROD_DATE, DATE_CHARS, field);
        line_put_field(l, field, length);
        break;
    case INFO_HOURS:
        line_put_u16(l, main_power_on_hours());
        break;
    default: /* INFO_POWER_UPS */
        line_put_u16(l, power_up_count());
        break;
    }
}

/* "GAIN+0.12%": the change a CAL RANGE point would make to the stored gain, in per cent, two decimals (one
 * from 10 %, the limit cal_range_preview() allows). It is nine cells, the width of the line. */
static void render_range_change(line_t *l) {
    const float percent = m.range_change * 100.0f;
    const uint16_t hundredths = (uint16_t)(fabsf(percent) * 100.0f + 0.5f);

    line_puts_P(l, text_P(LT_GAIN));
    line_putc(l, (percent < 0.0f) ? '-' : '+');
    if (hundredths >= 1000u) {
        const uint16_t tenths = (uint16_t)(hundredths / 10u);

        line_put_u16(l, (uint16_t)(tenths / 10u));
        line_putc(l, '.');
        line_put_u16(l, (uint16_t)(tenths % 10u));
    } else {
        line_put_u16(l, (uint16_t)(hundredths / 100u));
        line_putc(l, '.');
        line_putc(l, (char)('0' + (hundredths / 10u) % 10u));
        line_putc(l, (char)('0' + hundredths % 10u));
    }
    line_putc(l, '%');
}

static void render_view(line_t *l) {
    switch (m.view) {
    case VIEW_INFO:
        render_info(l);
        break;
    case VIEW_CHECK: {
        char name[CAL_CHECK_NAME_LEN + 1u];

        if (cal_check_name(m.view_page, name)) {
            for (const char *p = name; *p != '\0'; p++) {
                line_putc(l, upper_ascii(*p));
            }
        } else {
            line_puts_P(l, text_P(LT_NOT_SET));
        }
        break;
    }
    case VIEW_VERIFY:
        if (m.view_page == VERIFY_PAGE_RESULT) {
            line_puts_P(l, text_P(m.verify_pass ? LT_PASS : LT_FAIL));
        } else {
            /* The deviation as a percentage of the reading, in the editor's notation. */
            eng_format(m.text, m.verify_deviation * 100.0f);
            for (uint8_t i = 0u; i < EDIT_TEXT_LEN; i++) {
                line_putc(l, m.text[i]);
            }
        }
        break;
    default: /* VIEW_COUNT */
        line_put_u16(l, cal_count());
        break;
    }
}

/* ---- A running activity -------------------------------------------------------------------------------- */

static void render_busy(line_t *l) {
    switch (m.activity) {
    case ACT_EXER: /* the seconds that are left, under the name EXERCISER on the second line */
        line_puts_P(l, text_P(LT_RELAY));
        line_putc(l, ' ');
        line_put_u16(l, m.shown);
        break;
    case ACT_SWEEP:
    case ACT_ZERO_CAL:
        line_puts_P(l, text_P(LT_RUNNING));
        break;
    default: /* ACT_SELF_TEST: the lamp test draws its own cells, so there is no text for it */
        if (m.step == ST_KEYS) {
            line_puts_P(l,
                        text_P(m.last_key == KEY_NONE ? (uint8_t)LT_PRESS_KEY : key_name_text(m.last_key)));
        } else if (m.step == ST_SELFCAL) {
            line_puts_P(l, text_P(LT_SELFCAL));
        }
        break;
    }
}

/* The character editors. The cursor cell shows a bar instead of a blank so that a blinking space can be seen. */
static void render_text_edit(line_t *l) {
    for (uint8_t i = 0u; i < m.text_len; i++) {
        char c = m.text[i];

        if (i == m.cursor && !m.blink_on) {
            c = (c == ' ') ? '-' : ' ';
        }
        line_putc(l, c);
    }
}

/* No cell is blanked. Any value outside the nine cells would do; this one says why it is there. */
#define NO_BLANK_CELL 0xFFu

static void render(void) {
    line_t l = {.len = 0u, .cells = 0u};
    uint8_t blank_cell_of = NO_BLANK_CELL;
    const char *second = NULL; /* the second line: the name of what the main line shows */

    switch (m.state) {
    case MS_PAGE:
        line_puts_P(&l, text_P(page_def(m.page).title));
        break;
    case MS_ITEM: {
        const item_def_t item = current_item_def();

        line_puts_P(&l, text_P(item.title));
        second = caption_P(item.title2);
        break;
    }
    case MS_CHOICE: {
        const choice_def_t def = choice_def(m.param);

        line_put_choice(&l, &def, m.blink_on);
        second = caption_P(def.caption);
        break;
    }
    case MS_NUMERIC: {
        const uint8_t first = first_cursor(MS_NUMERIC);
        uint8_t first_char;

        /* The whole text, every phase. The cursor cell is blanked after layout -- substituting
         * a space for the character moved the rest of the line whenever it carried a decimal point. */
        if (m.param == NUM_DB_REF) { /* "REF:" then the digits: no sign cell, and no cell left for a unit */
            line_puts_P(&l, text_P(LT_REF_LABEL));
        }
        first_char = l.len;
        for (uint8_t i = first; i < EDIT_TEXT_LEN; i++) {
            line_putc(&l, m.text[i]);
        }
        blank_cell_of = m.blink_on ? NO_BLANK_CELL : (uint8_t)(first_char + m.cursor - first);
        second = caption_P(numeric_def(m.param).caption);
        break;
    }
    case MS_INTEGER: {
        const integer_def_t def = integer_def(m.param);
        uint8_t first_char;

        if (m.param == INT_DBM_REF) {
            line_puts_P(&l, text_P(LT_REF_LABEL));
        }
        first_char = l.len;
        for (uint8_t i = 0u; i < def.digits; i++) {
            line_putc(&l, m.digits[i]);
        }
        blank_cell_of = m.blink_on ? NO_BLANK_CELL : (uint8_t)(first_char + m.cursor);
        if (def.suffix != '\0') {
            line_putc(&l, def.suffix);
        }
        second = caption_P(def.caption);
        break;
    }
    case MS_REVISION:
        render_version(&l);
        break;
    case MS_SUB:
        line_puts_P(&l, text_P((uint8_t)(LT_SUB_ZERO + m.sub)));
        break;
    case MS_PROMPT:
        if (m.prompt_action == PA_RANGE_COMMIT) {
            render_range_change(&l);
        } else if (m.blink_on) { /* a prompt blinks as a whole: ENTER does what it says */
            line_puts_P(&l, text_P(m.prompt_text));
        } else {
            line_blank_like_P(&l, text_P(m.prompt_text));
        }
        second = caption_P(m.prompt_caption);
        break;
    case MS_BUSY:
        if (m.activity == ACT_SELF_TEST && m.step == ST_LAMPS) {
            return;
        }
        render_busy(&l);
        if (m.activity == ACT_EXER) {
            second = text_P(LT_EXERCISER);
        }
        break;
    case MS_VIEW:
        render_view(&l);
        break;
    case MS_TEXTEDIT:
        render_text_edit(&l);
        break;
    case MS_CLOSED:
    case MS_MESSAGE:
        return;
    }
    if (blank_cell_of == NO_BLANK_CELL) {
        display_show_text(l.buf);
    } else {
        display_show_text_blank_cell(l.buf, blank_cell_of);
    }
    display_show_second_P(second);
}

/* Show the current state with the cursor visible and the blink phase restarted.
 */
static void redraw(void) {
    m.blink_ms = 0u;
    m.blink_on = true;
    render();
}

/* ---- Leaving and messages ------------------------------------------------------------------------------- */

/* Hand the display back. Does not touch g_state.menu_mode: see menu_close(). */
static void release_display(void) {
    if (m.state == MS_CHOICE && m.param == CH_BRIGHT) {
        vfd_set_brightness(bright_level()); /* a level that was only being previewed is dropped */
    }
    m.state = MS_CLOSED;
    m.shortcut = false;
    m.in_sub = false;
    memset(m.code, 0, sizeof m.code);
    display_annunciators();
    display_update();
}

/* Close for every reason except a function key. g_state.menu_mode is RAM 0x0F61 bits 6 and 7 in the
 * original: bit 6 is set when SHIFT+MENU opens the menu (0xE30A) and bit 7 when level 1 is entered
 * (0x16A94, called at 0xE5FA / 0xE618 / 0xE62E). The dB and dBm shortcuts test both (0xDFAC, 0xE0BC), so
 * one flag covers them. Bit 6 is cleared at 0x1671E when the menu loop ends, bit 7 at 0xE364 after the
 * loop has returned. Neither is set by the three shortcut editors, so for those the flag stays false. */
static void menu_close(void) {
    g_state.menu_mode = false;
    release_display();
}

/* Back out of whatever is open: to the CAL submenu when that is where it came from, otherwise to the item
 * list. A half-entered new service code never survives leaving its editor. */
static void back_to_items(void) {
    memset(m.code, 0, sizeof m.code);
    m.state = m.in_sub ? MS_SUB : MS_ITEM;
    redraw();
}

/* A message on the main line and, when it needs two lines, a caption on the second. */
static void show_message_lines_P(const char *text_flash, const char *caption_flash, uint16_t duration_ms,
                                 after_message_t after) {
    m.state = MS_MESSAGE;
    m.after = after;
    m.message_ms = duration_ms;
    display_show_text_P(text_flash);
    display_show_second_P(caption_flash);
}

static void show_message_P(const char *text_flash, uint16_t duration_ms, after_message_t after) {
    show_message_lines_P(text_flash, NULL, duration_ms, after);
}

static void message_done(void) {
    switch (m.after) {
    case NEXT_ITEMS:
        back_to_items();
        break;
    case NEXT_RESTART:
        m.state = m.resume;
        m.cursor = first_cursor(m.state);
        redraw();
        break;
    case NEXT_CONFIRM: {
        /* The original returned to its caller only after the one-second SAVED (0x604E, 0x3594 with
         * 1000), and only then did the dispatcher finish the key (0xE02E, 0xE214). */
        const uint8_t item = m.preset_item;

        menu_close();
        m.item[PAGE_MATH] = item;
        keys_ref_confirmed();
        break;
    }
    case NEXT_VIEW:
        m.state = MS_VIEW;
        redraw();
        break;
    case NEXT_PROMPT:
        m.state = MS_PROMPT;
        redraw();
        break;
    }
}

/* An edit finished: say so for a second, then go back to the item list. */
static void finish_saved(bool stored) {
    show_message_P(text_P(stored ? MT_MSG_SAVED : LT_STORE_FAILED), MESSAGE_MS, NEXT_ITEMS);
}

/* A reference editor was confirmed and its value stored. In the menu this is finish_saved(); an editor
 * opened from the SHIFT layer closes instead and lets the dispatcher finish its key. */
static void finish_reference_saved(void) {
    if (m.shortcut) {
        show_message_P(text_P(MT_MSG_SAVED), MESSAGE_MS, NEXT_CONFIRM);
    } else {
        finish_saved(true);
    }
}

/* An out-of-range entry: show why, put the nearest legal value in the editor,
 * and start it again. */
static void reject_entry(bool too_large) {
    show_message_P(text_P(too_large ? MT_MSG_TOO_LARGE : MT_MSG_TOO_SMALL), MESSAGE_MS, NEXT_RESTART);
}

/* The keys every menu screen shares (original 0xE9E4). Returns true when the
 * key was consumed. */
static bool menu_common_key(uint8_t key) {
    if (key >= KEY_DCV && key <= KEY_DIODE) {
        /* The caller dispatches this key next, and the original did so before it cleared the menu
         * flags (0xE50C-0xE512 against 0x1671E), so a shifted dB / dBm key is still refused. Leaving
         * g_state.menu_mode set here is deliberate; main clears it after that dispatch. */
        release_display();
        return true;
    }
    if (key == KEY_SHIFT) {
        g_state.shift_active = !g_state.shift_active;
        vfd_set_annunciator(ANN_SHIFT,
                            g_state.shift_active); /* 0xEA0C-0xEA22: the lamp byte and nothing else */
        return true;
    }
    if (key == KEY_MENU && g_state.shift_active) {
        g_state.shift_active = false;
        menu_close();
        return true;
    }
    return false;
}

/* Defined with the calibration screens further down. */
static void open_item(uint8_t kind);
static void cal_value_entered(void);
static bool write_allowed(void);

/* ---- Numeric editors: HI LIMIT, LO LIMIT, PERC REF, dB REF, and the CAL reference value ---------------- */

static float numeric_current(uint8_t id) {
    switch (id) {
    case NUM_LIMIT_HI:
        return g_state.math.limit_hi;
    case NUM_LIMIT_LO:
        return g_state.math.limit_lo;
    case NUM_PERCENT_REF:
        return g_state.math.percent_ref;
    case NUM_CAL_REF:
        return m.cal.applied;
    default:
        return g_state.math.db_ref;
    }
}

static void numeric_store(uint8_t id, float v) {
    switch (id) {
    case NUM_LIMIT_HI:
        g_state.math.limit_hi = v;
        break;
    case NUM_LIMIT_LO:
        g_state.math.limit_lo = v;
        break;
    case NUM_PERCENT_REF:
        g_state.math.percent_ref = v;
        break;
    case NUM_CAL_REF:
        m.cal.applied = v;
        break;
    default:
        g_state.math.db_ref = v;
        break;
    }
}

static void start_numeric(uint8_t id) {
    m.state = MS_NUMERIC;
    m.param = id;
    m.cursor = first_cursor(MS_NUMERIC);
    m.value = numeric_current(id);
    eng_format(m.text, m.value);
    redraw();
}

/* ENTER: store the value, or reject it and restart the editor with the nearest
 * legal value. The original (0x590C, 0x63A4, 0x5B5C) did the same: >1e8 TOO
 * LARGE, <-1e8 TOO SMALL, and PERC REF also refuses zero. */
static void numeric_enter(void) {
    const float v = m.value;
    float legal = v;
    bool reject = false;
    bool too_large = false;

    if (m.param == NUM_DB_REF) {
        if (v < DB_REF_MIN) {
            reject = true;
            legal = DB_REF_MIN;
        } else if (v > DB_REF_MAX) {
            reject = true;
            too_large = true;
            legal = DB_REF_MAX;
        }
    } else if (v > LIMIT_ABS_MAX) {
        reject = true;
        too_large = true;
        legal = LIMIT_ABS_MAX;
    } else if (m.param == NUM_CAL_REF && v == 0.0f) {
        /* A calibration against zero would divide by it. DEFAULT: refused, restarting at 1. */
        reject = true;
        legal = CAL_REF_RESTART;
    } else if (v < -LIMIT_ABS_MAX || (m.param == NUM_PERCENT_REF && v == 0.0f)) {
        reject = true;
        legal = -LIMIT_ABS_MAX;
    }

    if (reject) {
        m.value = legal;
        eng_format(m.text, legal);
        m.resume = MS_NUMERIC;
        reject_entry(too_large);
        return;
    }
    numeric_store(m.param, v);
    if (m.param == NUM_CAL_REF) {
        cal_value_entered();
        return;
    }
    finish_reference_saved();
}

/* MENU inside an editor cancels it: back to the item list, or out altogether for a shortcut editor
 * (0x5C24-0x5C54 shows the item title only when the shortcut flag 0x0F5F bit 4 is clear). */
static void leave_editor(void) {
    if (m.shortcut) {
        menu_close();
    } else {
        back_to_items();
    }
}

static void numeric_key(uint8_t key) {
    if (menu_common_key(key)) {
        return;
    }
    switch (key) {
    case KEY_MENU:
        leave_editor();
        break;
    case KEY_UP:
        m.value = ui_edit_digit_up(m.text, m.cursor, m.value);
        redraw();
        break;
    case KEY_DOWN:
        m.value = ui_edit_digit_down(m.text, m.cursor, m.value);
        redraw();
        break;
    case KEY_LEFT:
        m.cursor = text_cursor_step(m.text, first_cursor(MS_NUMERIC), m.cursor, false);
        redraw();
        break;
    case KEY_RIGHT:
        m.cursor = text_cursor_step(m.text, first_cursor(MS_NUMERIC), m.cursor, true);
        redraw();
        break;
    case KEY_ENTER:
        numeric_enter();
        break;
    default:
        break;
    }
}

/* ---- Digit-field editors: dBm REF, HOLD CNT, the KEY prompt, the new service code ---------------------- */

static void start_integer(uint8_t id) {
    const integer_def_t def = integer_def(id);
    uint16_t initial = 0u;

    switch (id) {
    case INT_DBM_REF:
        initial = (g_state.math.dbm_ref < (float)DBM_REF_MIN) ? DBM_REF_MIN : (uint16_t)g_state.math.dbm_ref;
        break;
    case INT_HOLD_COUNT:
        initial = g_state.hold_count;
        break;
    default:
        break;
    }
    m.state = MS_INTEGER;
    m.param = id;
    m.cursor = 0u;
    digits_from_value(m.digits, def.digits, initial);
    redraw();
}

/* The KEY: prompt accepts a 4-digit code and passes it to the service lock. The
 * original compared it with the fixed number 6666 (0x51F6); that code is not
 * reproduced. */
static bool ui_key_code_entry(const char *code) {
    return serv_unlock(code, SERV_FROM_PANEL);
}

static void integer_enter(void) {
    const integer_def_t def = integer_def(m.param);
    const uint16_t v = digits_to_value(m.digits);

    switch (m.param) {
    case INT_DBM_REF:
        if (v < DBM_REF_MIN) {
            digits_from_value(m.digits, def.digits, DBM_REF_MIN);
            m.resume = MS_INTEGER;
            reject_entry(false);
            return;
        }
        g_state.math.dbm_ref = (int16_t)v;
        finish_reference_saved();
        break;
    case INT_HOLD_COUNT:
        if (v < HOLD_COUNT_MIN || v > HOLD_COUNT_MAX) {
            const bool too_large = v > HOLD_COUNT_MAX;

            digits_from_value(m.digits, def.digits, too_large ? HOLD_COUNT_MAX : HOLD_COUNT_MIN);
            m.resume = MS_INTEGER;
            reject_entry(too_large);
            return;
        }
        g_state.hold_count = (uint8_t)v;
        finish_saved(true);
        break;
    case INT_KEY_CODE:
        /* The lock decides, and it is asked again after the code is accepted: the protected item opens
         * only when serv_check() says SERV_ALLOWED, whatever serv_unlock() returned. */
        if (ui_key_code_entry(m.digits) && serv_check() == SERV_ALLOWED) {
            memset(m.digits, 0, sizeof m.digits);
            open_item(m.pending_kind);
        } else {
            digits_from_value(m.digits, def.digits, 0u);
            m.resume = MS_INTEGER;
            show_message_P(text_P(LT_KEY_ERROR), MESSAGE_MS, NEXT_RESTART);
        }
        break;
    case INT_NEW_CODE:
        memcpy(m.code, m.digits, sizeof m.code);
        start_integer(INT_NEW_CODE_AGAIN);
        break;
    default: /* INT_NEW_CODE_AGAIN */
        if (memcmp(m.code, m.digits, KEY_CODE_DIGITS) != 0) {
            show_message_P(text_P(LT_MISMATCH), MESSAGE_MS, NEXT_ITEMS);
        } else if (write_allowed()) {
            finish_saved(serv_set_password(m.code));
        }
        memset(m.code, 0, sizeof m.code);
        memset(m.digits, 0, sizeof m.digits);
        break;
    }
}

static void integer_key(uint8_t key) {
    const integer_def_t def = integer_def(m.param);

    if (menu_common_key(key)) {
        return;
    }
    switch (key) {
    case KEY_MENU:
        leave_editor();
        break;
    case KEY_UP:
    case KEY_DOWN:
        digit_step(&m.digits[m.cursor], key == KEY_UP);
        redraw();
        break;
    case KEY_LEFT:
        m.cursor = (m.cursor == 0u) ? (uint8_t)(def.digits - 1u) : (uint8_t)(m.cursor - 1u);
        redraw();
        break;
    case KEY_RIGHT:
        m.cursor = (m.cursor + 1u >= def.digits) ? 0u : (uint8_t)(m.cursor + 1u);
        redraw();
        break;
    case KEY_ENTER:
        integer_enter();
        break;
    default:
        break;
    }
}

/* ---- Reference editors opened from the SHIFT layer ---------------------------------------------------- */

/* The dispatcher (0xDF00) opens these and, once one returns confirmed, presets the MATH menu cursor:
 * RAM 0x0131 = 0 (the MATH page, which menu_enter() always selects) and RAM 0x0E24 = the MATH item the
 * menu shows next time. 0x0E24 is 3 (dB REF) after SHIFT+ACDC (0xE09E), 4 (dBm REF) after SHIFT+DIODE
 * (0xE1C4) and also 3 after SHIFT+REL (0xE22A, an item 3 although PERC REF is item 2: kept as found).
 * A cancelled editor leaves the cursor alone (0xE0A6, 0xE232). */
static void start_shortcut(uint8_t preset_item) {
    if (m.state != MS_CLOSED) {
        return;
    }
    m.shortcut = true;
    m.preset_item = preset_item;
    /* The same clear as menu_enter(): without it the measurement's minus sign and lamps stay lit behind the
     * editor. Closing the editor, confirmed or cancelled, goes through release_display(), which redraws the
     * measurement. */
    display_clear_all();
}

void menu_edit_percent_ref(void) {
    start_shortcut(3u);
    if (m.shortcut) {
        start_numeric(NUM_PERCENT_REF);
    }
}

void menu_edit_db_ref(void) {
    start_shortcut(3u);
    if (m.shortcut) {
        start_numeric(NUM_DB_REF);
    }
}

void menu_edit_dbm_ref(void) {
    start_shortcut(4u);
    if (m.shortcut) {
        start_integer(INT_DBM_REF);
    }
}

/* ---- Choice editors: TRIG MODE, HOLD WIN, and the SYS settings ------------------------------------------ */

static uint8_t hold_window_choice(float window) {
    for (uint8_t i = 0u; i + 1u < HOLD_WINDOW_CHOICES; i++) {
        float nominal;

        memcpy_P(&nominal, &hold_window_values[i], sizeof nominal);
        if (window <= nominal * 1.5f) {
            return i;
        }
    }
    return (uint8_t)(HOLD_WINDOW_CHOICES - 1u);
}

static uint8_t choice_current(uint8_t id) {
    const settings_t *s = &g_state.settings;

    switch (id) {
    case CH_TRIG_MODE:
        return (uint8_t)g_state.trigger;
    case CH_HOLD_WINDOW:
        return hold_window_choice(g_state.hold_window);
    case CH_BEEP:
        return s->beeper ? 1u : 0u;
    case CH_BAUD:
        return s->baud_index;
    case CH_TERMINATOR:
        return s->tx_terminator ? 1u : 0u;
    case CH_ECHO:
        return s->echo ? 1u : 0u;
    case CH_KEY_CLICK:
        return s->key_click ? 1u : 0u;
    case CH_RLY_BOOT:
        return rly_boot_choice();
    case CH_LINE:
        return line_choice();
    case CH_BRIGHT:
        return (uint8_t)(bright_level() - BRIGHT_MIN);
    default: /* the CAL function and range choices start at the first entry */
        return 0u;
    }
}

static void start_choice(uint8_t id) {
    m.state = MS_CHOICE;
    m.param = id;
    m.choice = choice_current(id);
    redraw();
}

/* ENTER on a choice. The SYS settings are applied to the drivers at once and
 * then stored to EEPROM (the original applied them only on the store
 * path and set only the low byte of the baud divisor). */
static void choice_enter(void) {
    settings_t *s = &g_state.settings;
    bool stored = true;

    switch (m.param) {
    case CH_TRIG_MODE:
        g_state.trigger = (trig_t)m.choice;
        vfd_set_annunciator(ANN_TRIG, g_state.trigger != TRIG_IMM); /* 0xE942-0xE950: the TRIG lamp only */
        break;
    case CH_HOLD_WINDOW:
        memcpy_P(&g_state.hold_window, &hold_window_values[m.choice], sizeof g_state.hold_window);
        break;
    case CH_RLY_BOOT:
        stored =
            store_setting_byte(EE_SET_RLY_BOOT, (m.choice != 0u) ? RLY_BOOT_STORED_ON : RLY_BOOT_STORED_OFF);
        break;
    case CH_LINE: {
        /* The stored byte changes first; the converter follows only once it is safely stored, so a
         * failed write cannot leave the meter on a frequency it would forget at the next power-up. */
        const bool changed = line_choice() != m.choice;

        stored = store_setting_byte(EE_SET_LINE, (m.choice != 0u) ? LINE_STORED_50HZ : 0u);
        if (stored && changed) {
            meas_set_line_hz((m.choice != 0u) ? LINE_HZ_ALTERNATE : LINE_HZ_DEFAULT);
        }
        break;
    }
    case CH_BRIGHT:
        stored = store_setting_byte(EE_SET_BRIGHT, (uint8_t)(BRIGHT_MIN + m.choice));
        vfd_set_brightness(bright_level()); /* what is stored, or the old level if the write failed */
        break;
    case CH_CAL_FUNC:
        /* Not a setting: it picks the function CAL RANGE / VERIFY works on, then asks for its range. */
        m.cal.func = m.choice;
        start_choice((uint8_t)(CH_RANGE_DCV + m.choice));
        return;
    case CH_RANGE_DCV:
    case CH_RANGE_DCA:
    case CH_RANGE_OHM:
        m.cal.range = m.choice;
        m.cal.applied = CAL_REF_RESTART;
        start_numeric(NUM_CAL_REF);
        return;
    default:
        switch (m.param) {
        case CH_BEEP:
            s->beeper = m.choice != 0u;
            break;
        case CH_BAUD:
            s->baud_index = m.choice;
            break;
        case CH_TERMINATOR:
            s->tx_terminator = m.choice;
            break;
        case CH_ECHO:
            s->echo = m.choice != 0u;
            break;
        default:
            s->key_click = m.choice != 0u;
            break;
        }
        settings_apply();
        stored = settings_store();
        break;
    }
    finish_saved(stored);
}

/* Moving through the BRIGHT levels shows each one on the panel at once. ENTER keeps it; leaving any other
 * way puts the stored level back (release_display() does the same when a function key closes the menu). */
static void choice_preview(void) {
    if (m.param == CH_BRIGHT) {
        vfd_set_brightness((uint8_t)(BRIGHT_MIN + m.choice));
    }
}

static void choice_key(uint8_t key) {
    const choice_def_t def = choice_def(m.param);

    if (menu_common_key(key)) {
        return;
    }
    switch (key) {
    case KEY_UP:
    case KEY_MENU:
        if (m.param == CH_BRIGHT) {
            vfd_set_brightness(bright_level());
        }
        back_to_items();
        break;
    case KEY_LEFT:
        m.choice = (m.choice == 0u) ? (uint8_t)(def.count - 1u) : (uint8_t)(m.choice - 1u);
        choice_preview();
        redraw();
        break;
    case KEY_RIGHT:
        m.choice = (m.choice + 1u >= def.count) ? 0u : (uint8_t)(m.choice + 1u);
        choice_preview();
        redraw();
        break;
    case KEY_ENTER:
        choice_enter();
        break;
    default:
        break;
    }
}

/* ---- REVISION and ZERO CAL ------------------------------------------------------------------------------ */

static void revision_key(uint8_t key) {
    if (menu_common_key(key)) {
        return;
    }
    if (key == KEY_UP || key == KEY_MENU) {
        back_to_items();
    }
}

/* ---- SYS items 8 to 18: gates, prompts and running activities ---------------------------------------- */

/* The service code is asked for when a protected item is opened (start_editor). It is asked of the lock
 * again at the moment of every write, because the session can end in between (serv.h: ten minutes). A
 * refusal leaves the CAL submenu as well, so the next attempt starts from the item and its KEY prompt. */
static bool write_allowed(void) {
    if (serv_check() == SERV_ALLOWED) {
        return true;
    }
    m.in_sub = false;
    show_message_P(text_P(LT_LOCKED), MESSAGE_MS, NEXT_ITEMS);
    return false;
}

static void show_refused(void) {
    show_message_P(text_P(LT_REFUSED), MESSAGE_MS, NEXT_ITEMS);
}

static void prepare_prompt(uint8_t text, uint8_t caption, prompt_action_t action) {
    m.prompt_text = text;
    m.prompt_caption = caption;
    m.prompt_action = action;
}

static void start_prompt(uint8_t text, uint8_t caption, prompt_action_t action) {
    prepare_prompt(text, caption, action);
    m.state = MS_PROMPT;
    redraw();
}

static void begin_busy(activity_t activity) {
    m.state = MS_BUSY;
    m.activity = activity;
    m.timer_ms = 0u;
}

/* ---- Paged views --------------------------------------------------------------------------------------- */

static void view_show_page(void) {
    const uint8_t label = view_label();

    if (label == NO_TEXT) {
        m.state = MS_VIEW;
        redraw();
    } else {
        show_message_P(text_P(label), LABEL_MS, NEXT_VIEW);
    }
}

static void start_view(view_t view, uint8_t page_count) {
    m.view = view;
    m.view_page = 0u;
    m.view_pages = page_count;
    view_show_page();
}

static void view_move(bool forward) {
    if (m.view_pages <= 1u) {
        return;
    }
    for (uint8_t tries = 0u; tries < m.view_pages; tries++) {
        if (forward) {
            m.view_page = (m.view_page + 1u >= m.view_pages) ? 0u : (uint8_t)(m.view_page + 1u);
        } else {
            m.view_page = (m.view_page == 0u) ? (uint8_t)(m.view_pages - 1u) : (uint8_t)(m.view_page - 1u);
        }
        if (view_page_valid(m.view_page)) {
            break;
        }
    }
    view_show_page();
}

static void view_key(uint8_t key) {
    if (menu_common_key(key)) {
        return;
    }
    switch (key) {
    case KEY_UP:
    case KEY_MENU:
        back_to_items();
        break;
    case KEY_LEFT:
        view_move(false);
        break;
    case KEY_RIGHT:
        view_move(true);
        break;
    default:
        break;
    }
}

/* SYS item 12: scan the records. Nothing bad is one second of CAL OK; otherwise each bad record is named
 * on a page of its own. */
static void start_cal_check(void) {
    const uint8_t bad = cal_check_run();

    if (bad == 0u) {
        show_message_P(text_P(LT_CAL_OK), MESSAGE_MS, NEXT_ITEMS);
    } else {
        start_view(VIEW_CHECK, bad);
    }
}

/* ---- Relay exercise (SYS item 8) ----------------------------------------------------------------------- */

/* The sequencer is meas.c's. It refuses to start when the input is live (more than 2 V), and it counts the
 * 60 s down itself; the menu shows the seconds that are left. */
static void exer_start(void) {
    if (!meas_relay_exercise_start(true)) { /* SYS item 8 is the 60 s sequence with the countdown */
        show_message_P(text_P(LT_INPUT_ON), MESSAGE_MS, NEXT_ITEMS);
        return;
    }
    begin_busy(ACT_EXER);
    m.shown = meas_relay_exercise_seconds_left();
    redraw();
}

static void exer_tick(void) {
    const uint8_t seconds = meas_relay_exercise_seconds_left();

    if (!meas_relay_exercise_busy()) {
        /* It ran its time out, or the input guard stopped it part-way: the seconds left tell which. */
        show_message_P(text_P(seconds == 0u ? LT_DONE : LT_ABORTED), MESSAGE_MS, NEXT_ITEMS);
    } else if (seconds != m.shown) {
        m.shown = seconds;
        render();
    }
}

/* ---- Calibration sweeps and the panel calibration flows (SYS item 11) ---------------------------------- */

static func_t cal_target_func(void) {
    return (func_t)pgm_read_byte(&cal_funcs[m.cal.func]);
}

static void begin_sweep(sweep_flow_t flow) {
    m.flow = flow;
    begin_busy(ACT_SWEEP);
    redraw();
}

/* A sweep that the guard in cal_ops.c refused starts nothing and writes nothing. */
static void sweep_or_refuse(bool started, sweep_flow_t flow) {
    if (started) {
        begin_sweep(flow);
    } else {
        show_refused();
    }
}

/* The main loop advances the sweep (cal_sweep_task); the menu only watches how it ended. */
static void sweep_tick(void) {
    const cal_sweep_state_t state = cal_sweep_state();

    if (state == CAL_SWEEP_RUNNING) {
        return;
    }
    if (state != CAL_SWEEP_DONE) {
        show_message_P(text_P(LT_FAILED), MESSAGE_MS, NEXT_ITEMS);
    } else if (m.flow == FLOW_RANGE) {
        /* Measured and previewed, nothing written. The operator sees the change and decides. */
        if (cal_range_pending(&m.range_change)) {
            start_prompt(LT_GAIN, NO_TEXT, PA_RANGE_COMMIT);
        } else {
            show_message_P(text_P(LT_FAILED), MESSAGE_MS, NEXT_ITEMS);
        }
    } else if (m.flow != FLOW_VERIFY) {
        show_message_P(text_P(MT_MSG_SAVED), MESSAGE_MS, NEXT_ITEMS);
    } else if (cal_verify_result(&m.verify_deviation, &m.verify_pass)) {
        start_view(VIEW_VERIFY, VERIFY_PAGES);
    } else {
        show_message_P(text_P(LT_FAILED), MESSAGE_MS, NEXT_ITEMS);
    }
}

/* SYS item 7: the user zero of the five DC V ranges and then the six ohm ranges, one range per tick (the
 * original's sweep at 0xEF1E). meas_zero_capture() is the port of the capture at 0x1B76 and refuses an
 * implausible zero, which ends the sweep with FAILED: a short that is not a short must not be stored.
 * Neither the displayed function nor its range is touched. */
#define USER_ZERO_STEPS (EE_DCV_RANGES + EE_OHM_RANGES)

static void zero_cal_tick(void) {
    const bool dc = m.step < EE_DCV_RANGES;
    const func_t f = dc ? FUNC_DCV : FUNC_OHM;
    const uint8_t range = dc ? m.step : (uint8_t)(m.step - EE_DCV_RANGES);
    float zero;

    if (!meas_zero_capture(f, range, &zero) || !cal_write_user_zero(f, range, zero)) {
        show_message_P(text_P(LT_FAILED), MESSAGE_MS, NEXT_ITEMS);
        return;
    }
    m.step++;
    if (m.step >= USER_ZERO_STEPS) {
        show_message_P(text_P(MT_MSG_SAVED), MESSAGE_MS, NEXT_ITEMS);
    }
}

/* CAL RANGE and CAL VERIFY share the choice of function, range and applied value. CAL RANGE warns, and
 * lets the operator continue, when the meter has been on for under 30 minutes. */
static void start_cal_flow(bool verify) {
    m.cal.verify = verify;
    m.cal.accept_cold = false;
    if (!verify && !main_warmed_up()) {
        /* A WARNING with a confirm, not a refusal. The 30-minute timer measures time since power-on, which is not the same thing as a
         * cold instrument: a meter that has been on all day and was power-cycled two minutes ago is warm,
         * and the operator knows that and the timer does not.
         *
         * So it is shown before the operator types anything, and ENTER carries the acknowledgement into
         * cal_range_start(). Any other key leaves the submenu with nothing written. */
        start_prompt(LT_WARM_UP, NO_TEXT, PA_WARM_OK);
        return;
    }
    start_choice(CH_CAL_FUNC);
}

/* The applied value was entered. Both flows measure at once and write nothing yet: CAL RANGE shows the
 * change its point would make and writes only on the operator's ENTER (PA_RANGE_COMMIT). */
static void cal_value_entered(void) {
    if (m.cal.verify) {
        sweep_or_refuse(cal_verify_start(cal_target_func(), m.cal.range, m.cal.applied), FLOW_VERIFY);
    } else if (write_allowed()) {
        sweep_or_refuse(cal_range_start(cal_target_func(), m.cal.range, m.cal.applied, m.cal.accept_cold),
                        FLOW_RANGE);
    }
}

/* ---- RESET (SYS item 17) ------------------------------------------------------------------------------- */

typedef struct {
    uint16_t address;
    uint8_t value;
} setting_default_t;

/* The user settings and nothing else: the five vendor bytes at their factory values, then the bytes this
 * firmware added (LINE FREQ, RELAY POWER ON, BRIGHT and the boot message signature) back to "erased", which every
 * reader takes to mean "use the default". Calibration records, the identity block, the calibration date and
 * counter, the production data and the service code are not in this list, and write_user_setting() below
 * refuses any address that is not one of the user-setting bytes. */
static const setting_default_t setting_defaults[] PROGMEM = {
    {EE_SET_BAUD, DEFAULT_BAUD_INDEX}, {EE_SET_KEYCLICK, DEFAULT_KEY_CLICK},
    {EE_SET_BEEPER, DEFAULT_BEEPER},   {EE_SET_TXTERM, DEFAULT_TX_TERMINATOR},
    {EE_SET_ECHO, DEFAULT_ECHO},       {EE_SET_LINE, ERASED_BYTE},
    {EE_SET_RLY_BOOT, ERASED_BYTE},    {EE_SET_BRIGHT, ERASED_BYTE},
    {EE_BOOT_MSG_SIG, ERASED_BYTE},    {EE_BOOT_MSG_SIG + 1u, ERASED_BYTE},
};

/* The windows of the EEPROM that hold user settings. 0x0854-0x0856 and 0x0863 and 0x0865 are single vendor
 * bytes: the calibration date (0x0857) and the save counter (0x0861) sit between them and are outside every
 * window. 0x0A82 up to the power-up counter is the block of bytes this firmware added, after the boot
 * marker at 0x0A80. */
_Static_assert(EE_SET_BAUD + 1u == EE_SET_KEYCLICK && EE_SET_KEYCLICK + 1u == EE_SET_BEEPER,
               "the three vendor setting bytes are adjacent");
_Static_assert(EE_SET_BEEPER < EE_CAL_DATE, "the calibration date is outside the user-setting window");
_Static_assert(EE_SET_LINE >= EE_BOOT_MARK + 2u, "the boot marker is outside the user-setting window");
_Static_assert(EE_BOOT_MSG_SIG + 2u <= EE_PWRUP_COUNT, "the power-up counter is outside the window");

static bool is_user_setting_address(uint16_t address) {
    return (address >= EE_SET_BAUD && address <= EE_SET_BEEPER) || address == EE_SET_TXTERM ||
           address == EE_SET_ECHO || (address >= EE_SET_LINE && address < EE_PWRUP_COUNT);
}

/* The only EEPROM write RESET performs. */
static bool write_user_setting(uint16_t address, uint8_t value) {
    return is_user_setting_address(address) && store_setting_byte(address, value);
}

static bool reset_user_settings(void) {
    bool ok = true;

    for (uint8_t i = 0u; i < ARRAY_COUNT(setting_defaults); i++) {
        setting_default_t entry;

        memcpy_P(&entry, &setting_defaults[i], sizeof entry);
        ok = write_user_setting(entry.address, entry.value) && ok;
    }
    return ok;
}

static void run_reset(void) {
    settings_t *s = &g_state.settings;
    const uint8_t line_before = line_choice();
    const bool ok = reset_user_settings();

    s->baud_index = DEFAULT_BAUD_INDEX;
    s->echo = DEFAULT_ECHO != 0u;
    s->tx_terminator = DEFAULT_TX_TERMINATOR;
    s->beeper = DEFAULT_BEEPER != 0u;
    s->key_click = DEFAULT_KEY_CLICK != 0u;
    settings_apply();
    vfd_set_brightness(bright_level());
    if (line_choice() != line_before) {
        meas_set_line_hz((line_choice() != 0u) ? LINE_HZ_ALTERNATE : LINE_HZ_DEFAULT);
    }
    finish_saved(ok);
}

/* ---- Prompts ------------------------------------------------------------------------------------------- */

static void prompt_run(void) {
    switch (m.prompt_action) {
    case PA_EXER_PROMPT:
        start_prompt(LT_NO_INPUT, LT_EXERCISER, PA_EXER_START);
        return;
    case PA_EXER_START:
        exer_start();
        return;
    case PA_WARM_OK:
        /* Acknowledged. The write itself is still gated: this only gets past the warm-up check, and
         * PA_RANGE_COMMIT below is where write_allowed() and the confirmation apply. */
        m.cal.accept_cold = true;
        start_choice(CH_CAL_FUNC);
        return;
    default:
        break;
    }
    /* Everything below changes something that is kept. */
    if (!write_allowed()) {
        return;
    }
    switch (m.prompt_action) {
    case PA_ZERO:
        sweep_or_refuse(cal_short_all_start(), FLOW_ZERO);
        break;
    case PA_USER_ZERO:
        m.step = 0u;
        begin_busy(ACT_ZERO_CAL);
        redraw();
        break;
    case PA_OPEN:
        sweep_or_refuse(cal_open_all_start(), FLOW_OPEN);
        break;
    case PA_RANGE_COMMIT:
        show_message_P(text_P(cal_range_confirm() ? MT_MSG_SAVED : LT_FAILED), MESSAGE_MS, NEXT_ITEMS);
        break;
    case PA_UNDO:
        if (cal_undo()) {
            show_message_P(text_P(LT_RESTORED), MESSAGE_MS, NEXT_ITEMS);
        } else {
            show_message_lines_P(text_P(LT_CANNOT), text_P(LT_SUB_UNDO), MESSAGE_MS, NEXT_ITEMS);
        }
        break;
    default: /* PA_RESET */
        run_reset();
        break;
    }
}

/* ENTER does it; any other key cancels. A function key closes the menu as everywhere else. */
static void prompt_key(uint8_t key) {
    if (menu_common_key(key)) {
        return;
    }
    if (key == KEY_ENTER) {
        prompt_run();
    } else {
        back_to_items();
    }
}

/* ---- Character editors: CAL DATE and BOOT MSG ---------------------------------------------------------- */

static const char date_template[DATE_CHARS + 1u] PROGMEM = "00/00/00";

/* The cursor of the date editor stops on digits only; the slashes stay where they are. */
static void text_edit_step(bool forward) {
    for (uint8_t i = 0u; i < m.text_len; i++) {
        if (forward) {
            m.cursor = (m.cursor + 1u >= m.text_len) ? 0u : (uint8_t)(m.cursor + 1u);
        } else {
            m.cursor = (m.cursor == 0u) ? (uint8_t)(m.text_len - 1u) : (uint8_t)(m.cursor - 1u);
        }
        if (m.text_kind != TEXT_DATE || is_digit(m.text[m.cursor])) {
            break;
        }
    }
}

/* The stored calibration date if it is a date the editor can work on (digits and slashes, at least one
 * digit), otherwise the template. cal_date_write() accepts exactly those characters. */
static void date_text_load(void) {
    bool usable = false;

    ee_read_block(EE_CAL_DATE, m.text, DATE_CHARS);
    for (uint8_t i = 0u; i < DATE_CHARS; i++) {
        if (is_digit(m.text[i])) {
            usable = true;
        } else if (m.text[i] != '/') {
            usable = false;
            break;
        }
    }
    if (!usable) {
        memcpy_P(m.text, date_template, DATE_CHARS);
    }
}

/* ---- The boot message -------------------------------------------------------------------------
 * Shared, because three callers need the same answer: the text editor on the panel, `display_splash()` at
 * power-up, and `:DISP:BOOT`. It lives here rather than in display.c because validating a character means
 * asking whether the glyph tables can show it, and the picker that knows how is here.
 *
 * `ui_boot_text` always produces EE_BOOT_MSG_LEN characters plus a terminator: the stored message if one
 * was set, otherwise the splash the meter ships with. A stored character the display cannot show becomes a
 * blank, so a message written over USB can never leave the panel showing nonsense. */
void ui_boot_text(char out[EE_BOOT_MSG_LEN + 1u]) {
    if (ee_read_u16(EE_BOOT_MSG_SIG) == EE_SIG_VALID) {
        ee_read_block(EE_BOOT_MSG, out, EE_BOOT_MSG_LEN);
        for (uint8_t i = 0u; i < EE_BOOT_MSG_LEN; i++) {
            out[i] = picker_sanitise(out[i]);
        }
    } else {
        memcpy_P(out, boot_msg_default, EE_BOOT_MSG_LEN);
    }
    out[EE_BOOT_MSG_LEN] = '\0';
}

/* Signature last: a power failure part-way leaves "no message", never half of one. */
static bool boot_text_store_from(const char *text) {
    return ee_write_u16(EE_BOOT_MSG_SIG, ERASED_WORD) && ee_write_block(EE_BOOT_MSG, text, EE_BOOT_MSG_LEN) &&
           ee_write_u16(EE_BOOT_MSG_SIG, EE_SIG_VALID);
}

static bool boot_text_store(void) {
    return boot_text_store_from(m.text);
}

/* `:DISP:BOOT "<text>"`. A short string is padded with blanks and a long one is refused rather than
 * silently truncated -- a caller that sent nine characters and got seven would have no way to tell.
 * Characters the display cannot show are rejected for the same reason: the panel is the only place the
 * result is visible, so a substitution made quietly here is a substitution nobody ever sees reported. */
bool ui_boot_text_set(const char *text) {
    char buf[EE_BOOT_MSG_LEN + 1u];
    uint8_t n = 0u;

    while (text[n] != '\0') {
        if (n >= EE_BOOT_MSG_LEN) {
            return false; /* too long */
        }
        if (picker_sanitise(text[n]) != text[n]) {
            return false; /* a character with no glyph */
        }
        buf[n] = text[n];
        n++;
    }
    while (n < EE_BOOT_MSG_LEN) {
        buf[n++] = ' ';
    }
    buf[EE_BOOT_MSG_LEN] = '\0';
    return boot_text_store_from(buf);
}

static void start_text_edit(text_edit_t kind) {
    m.text_kind = kind;
    m.text_len = (kind == TEXT_DATE) ? DATE_CHARS : EE_BOOT_MSG_LEN;
    if (kind == TEXT_DATE) {
        date_text_load();
    } else {
        ui_boot_text(m.text);
    }
    m.text[m.text_len] = '\0';
    m.cursor = (uint8_t)(m.text_len - 1u); /* so that the first step lands on the first editable cell */
    text_edit_step(true);
    m.state = MS_TEXTEDIT;
    redraw();
}

static void text_edit_enter(void) {
    if (m.text_kind == TEXT_BOOT_MSG) {
        finish_saved(boot_text_store());
    } else if (write_allowed()) {
        finish_saved(cal_date_write(m.text));
    }
}

static void text_edit_key(uint8_t key) {
    if (menu_common_key(key)) {
        return;
    }
    switch (key) {
    case KEY_MENU:
        back_to_items();
        break;
    case KEY_UP:
    case KEY_DOWN:
        if (m.text_kind == TEXT_DATE) {
            digit_step(&m.text[m.cursor], key == KEY_UP);
        } else {
            m.text[m.cursor] = picker_step(m.text[m.cursor], key == KEY_UP);
        }
        redraw();
        break;
    case KEY_LEFT:
    case KEY_RIGHT:
        text_edit_step(key == KEY_RIGHT);
        redraw();
        break;
    case KEY_ENTER:
        text_edit_enter();
        break;
    default:
        break;
    }
}

/* ---- SELF TEST (SYS item 14) --------------------------------------------------------------------------- */

/* Four parts, in order: every lamp and segment lit for LAMP_TEST_MS; a key test that names each key as it
 * is pressed (MENU three times in a row ends it, so every other key can be tried, SHIFT and the function
 * keys included); the converter self-calibration; then an offer to run the relay exerciser. */
static void lamp_test_on(void) {
    for (uint8_t id = 0u; id < ANNUN_COUNT; id++) {
        vfd_set_annunciator(id, true);
    }
    for (uint8_t cell = 0u; cell < LAMP_TEST_MAIN_CELLS; cell++) {
        vfd_write_cell(VFD_MAIN, cell, LAMP_TEST_GLYPH);
    }
    for (uint8_t cell = 0u; cell < LAMP_TEST_SECOND_CELLS; cell++) {
        vfd_write_cell(VFD_SECOND, cell, LAMP_TEST_GLYPH);
    }
}

static void start_self_test(void) {
    begin_busy(ACT_SELF_TEST);
    m.step = ST_LAMPS;
    lamp_test_on();
}

/* End of the lamp test: clear what it lit, as the menu's panel is while it is open (only SHIFT and TRIG are
 * ever lit there), and wait for keys. */
static void start_key_test(void) {
    display_clear_all();
    vfd_set_annunciator(ANN_SHIFT, g_state.shift_active);
    vfd_set_annunciator(ANN_TRIG, g_state.trigger != TRIG_IMM);
    m.step = ST_KEYS;
    m.last_key = KEY_NONE;
    m.menu_presses = 0u;
    redraw();
}

static void self_test_result(bool ok) {
    prepare_prompt(LT_RELAY, LT_EXERCISER, PA_EXER_PROMPT);
    show_message_P(text_P(ok ? LT_PASS : LT_FAIL), MESSAGE_MS, NEXT_PROMPT);
}

static void start_self_cal(void) {
    m.step = ST_SELFCAL;
    if (meas_selfcal_start()) {
        redraw();
    } else {
        self_test_result(false);
    }
}

static void self_test_key(uint8_t key) {
    switch (m.step) {
    case ST_LAMPS:
        start_key_test();
        break;
    case ST_KEYS:
        if (!is_panel_key(key)) {
            break;
        }
        m.last_key = key;
        m.menu_presses = (key == KEY_MENU) ? (uint8_t)(m.menu_presses + 1u) : 0u;
        if (m.menu_presses >= KEY_TEST_EXIT_PRESSES) {
            start_self_cal();
        } else {
            redraw();
        }
        break;
    default: /* ST_SELFCAL: the converter is busy; keys wait */
        break;
    }
}

static void self_test_tick(void) {
    bool ok = false;

    if (m.step == ST_LAMPS) {
        if (++m.timer_ms >= LAMP_TEST_MS) {
            start_key_test();
        }
    } else if (m.step == ST_SELFCAL && meas_selfcal_done(&ok)) {
        self_test_result(ok);
    }
}

/* ---- Running activities: keys and time ----------------------------------------------------------------- */

/* Any key stops an exercise or a sweep at once, function keys included: they do not close the menu from
 * here, because the activity has to be stopped first. */
static void busy_key(uint8_t key) {
    switch (m.activity) {
    case ACT_EXER:
        meas_relay_exercise_stop();
        show_message_P(text_P(LT_ABORTED), MESSAGE_MS, NEXT_ITEMS);
        break;
    case ACT_SWEEP:
        cal_sweep_abort();
        show_message_P(text_P(LT_ABORTED), MESSAGE_MS, NEXT_ITEMS);
        break;
    case ACT_ZERO_CAL:
        /* The ranges already stored stay: each is one complete four-byte write. */
        show_message_P(text_P(LT_ABORTED), MESSAGE_MS, NEXT_ITEMS);
        break;
    default:
        self_test_key(key);
        break;
    }
}

/* Returns true only when this tick BLOCKED on the converter, which is the question the main loop is
 * asking. The first version returned true for every busy state, and that was
 * over-broad: `sweep_tick()` and `exer_tick()` only read a state variable and return -- the sweep's real
 * work happens in `cal_sweep_task()` from the main loop, and the relay exercise's in `meas_task()`. The
 * only branch here that holds the CPU is ZERO CAL, whose `meas_zero_capture()` waits on conversions.
 *
 * Being over-broad was not harmless. The main loop stops catching up a backlog on a true, so the SELF TEST
 * lamp phase -- which counts 1500 ticks of `m.timer_ms` -- advanced once per loop pass instead of once per
 * millisecond, stretching a 1.5 s lamp test by however far behind the menu clock had fallen. */
static bool busy_tick(void) {
    switch (m.activity) {
    case ACT_EXER:
        exer_tick();
        break;
    case ACT_SWEEP:
        sweep_tick();
        break;
    case ACT_ZERO_CAL:
        zero_cal_tick();
        return true; /* blocked on the converter: the caller must not run another this pass */
    default:
        self_test_tick();
        break;
    }
    return false;
}

/* ---- The CAL submenu (SYS item 11) --------------------------------------------------------------------- */

static void sub_enter(void) {
    switch (m.sub) {
    case SUB_ZERO:
        start_prompt(MT_PROMPT_CAL_SHORT, NO_TEXT, PA_ZERO);
        break;
    case SUB_OPEN:
        start_prompt(LT_OPEN_IN, NO_TEXT, PA_OPEN);
        break;
    case SUB_RANGE:
        start_cal_flow(false);
        break;
    case SUB_VERIFY:
        start_cal_flow(true);
        break;
    case SUB_UNDO:
        start_prompt(LT_SUB_UNDO, NO_TEXT, PA_UNDO);
        break;
    case SUB_DATE:
        start_text_edit(TEXT_DATE);
        break;
    default: /* SUB_COUNT */
        start_view(VIEW_COUNT, 1u);
        break;
    }
}

static void sub_key(uint8_t key) {
    if (menu_common_key(key)) {
        return;
    }
    switch (key) {
    case KEY_UP:
    case KEY_MENU:
        m.in_sub = false;
        m.state = MS_ITEM;
        redraw();
        break;
    case KEY_DOWN:
        sub_enter();
        break;
    case KEY_LEFT:
        m.sub = (m.sub == 0u) ? (uint8_t)(SUB_COUNT - 1u) : (uint8_t)(m.sub - 1u);
        redraw();
        break;
    case KEY_RIGHT:
        m.sub = (m.sub + 1u >= SUB_COUNT) ? 0u : (uint8_t)(m.sub + 1u);
        redraw();
        break;
    default:
        break;
    }
}

/* ---- Levels 0 and 1 ------------------------------------------------------------------------------------- */

/* Opens a SYS item that is not a plain editor. Protected items arrive here only after the lock has said
 * SERV_ALLOWED. */
static void open_item(uint8_t kind) {
    switch (kind) {
    case KIND_REVISION:
        m.state = MS_REVISION;
        redraw();
        break;
    case KIND_ZERO_CAL:
        /* Item 7, as the original does it (0xEA4C): CAL:SHORT, then ENTER captures every DC V and ohm range.
         * CAL > ZERO (item 11) is a different operation: the short-circuit sweep of the calibration
         * pairs. */
        start_prompt(MT_PROMPT_CAL_SHORT, NO_TEXT, PA_USER_ZERO);
        break;
    case KIND_RLY_EXER:
        start_prompt(LT_NO_INPUT, LT_EXERCISER, PA_EXER_START);
        break;
    case KIND_CAL_MENU:
        m.in_sub = true;
        m.sub = SUB_ZERO;
        m.state = MS_SUB;
        redraw();
        break;
    case KIND_CAL_CHECK:
        start_cal_check();
        break;
    case KIND_INFO:
        start_view(VIEW_INFO, INFO_PAGES);
        break;
    case KIND_SELF_TEST:
        start_self_test();
        break;
    case KIND_CODE:
        /* Says first whether the service code is still the factory 6666 or one the user stored, then the
         * editor for the new one. */
        start_integer(INT_NEW_CODE);
        m.resume = MS_INTEGER;
        show_message_P(text_P(serv_password_is_set() ? LT_CHANGED : LT_FACTORY), MESSAGE_MS, NEXT_RESTART);
        break;
    case KIND_RESET:
        start_prompt(LT_ITEM_RESET, NO_TEXT, PA_RESET);
        break;
    case KIND_BOOT_MSG:
        start_text_edit(TEXT_BOOT_MSG);
        break;
    default:
        break;
    }
}

static void start_editor(void) {
    const item_def_t item = current_item_def();

    switch (item.kind) {
    case KIND_NUMERIC:
        start_numeric(item.param);
        return;
    case KIND_INTEGER:
        start_integer(item.param);
        return;
    case KIND_CHOICE:
        start_choice(item.param);
        return;
    default:
        break;
    }
    if (item_is_protected(item.kind) && serv_check() != SERV_ALLOWED) {
        m.pending_kind = item.kind;
        start_integer(INT_KEY_CODE);
        return;
    }
    open_item(item.kind);
}

/* Level 0 (original 0xE4D6): pick the page. */
static void page_key(uint8_t key) {
    if (menu_common_key(key)) {
        return;
    }
    switch (key) {
    case KEY_LEFT:
        m.page = (m.page == 0u) ? (uint8_t)(PAGE_COUNT - 1u) : (uint8_t)(m.page - 1u);
        redraw();
        break;
    case KEY_RIGHT:
        m.page = (m.page + 1u >= PAGE_COUNT) ? 0u : (uint8_t)(m.page + 1u);
        redraw();
        break;
    case KEY_DOWN:
        back_to_items();
        break;
    case KEY_MENU:
        menu_close(); /* decision 2026-10-26: plain ESC leaves the menu; the original needed SHIFT+ESC */
        break;
    default:
        break;
    }
}

/* Level 1 (original 0xE646 with the page handlers 0x8B52, 0x8BFC, 0x8C7A): pick
 * the item. LEFT and RIGHT step through the items of every page with wrap-around. The original's TRIG
 * handler stored item 0 for both keys (0x8C4A, 0x8C60), so HOLD WIN and HOLD CNT could not be reached
 * from the panel, which ORIGINAL_BUGS 33 fixes. */
static void item_key(uint8_t key) {
    const page_def_t page = page_def(m.page);
    uint8_t *item = &m.item[m.page];

    if (menu_common_key(key)) {
        return;
    }
    switch (key) {
    case KEY_UP:
        m.state = MS_PAGE;
        redraw();
        break;
    case KEY_DOWN:
        start_editor();
        break;
    case KEY_LEFT:
        *item = (*item == 0u) ? (uint8_t)(page.count - 1u) : (uint8_t)(*item - 1u);
        redraw();
        break;
    case KEY_RIGHT:
        *item = (*item + 1u >= page.count) ? 0u : (uint8_t)(*item + 1u);
        redraw();
        break;
    case KEY_MENU:
        menu_close(); /* plain ESC leaves the menu, as on the page level; inside an editor ESC cancels the edit */
        break;
    default:
        break;
    }
}

/* ---- Public interface ----------------------------------------------------------------------------------- */

/* Besides clearing the menu state this applies the stored BRIGHT level: the controller powers up at 1/16 and
 * vfd_init() turns it on at the driver's default, so without this a level chosen in SYS item 15 would last
 * only until the next power cycle. main.c runs it after vfd_init() and before the splash, so the splash is
 * shown at the stored level. */
void menu_init(void) {
    memset(&m, 0, sizeof m);
    g_state.menu_mode = false;
    vfd_set_brightness(bright_level());
}

bool menu_active(void) {
    return m.state != MS_CLOSED;
}

void menu_enter(void) {
    if (m.state != MS_CLOSED) {
        return;
    }
    g_state.menu_mode = true;
    m.page = PAGE_MATH;
    m.state = MS_PAGE;
    display_clear_all(); /* 0xE31C: no lamp, minus sign or secondary text of the measurement stays lit */
    redraw();
}

void menu_key(uint8_t key) {
    switch (m.state) {
    case MS_PAGE:
        page_key(key);
        break;
    case MS_ITEM:
        item_key(key);
        break;
    case MS_CHOICE:
        choice_key(key);
        break;
    case MS_NUMERIC:
        numeric_key(key);
        break;
    case MS_INTEGER:
        integer_key(key);
        break;
    case MS_REVISION:
        revision_key(key);
        break;
    case MS_SUB:
        sub_key(key);
        break;
    case MS_PROMPT:
        prompt_key(key);
        break;
    case MS_BUSY:
        busy_key(key);
        break;
    case MS_VIEW:
        view_key(key);
        break;
    case MS_TEXTEDIT:
        text_edit_key(key);
        break;
    case MS_CLOSED:
    case MS_MESSAGE:
        break;
    }
}

bool menu_tick(void) {
    if (m.state == MS_MESSAGE) {
        if (m.message_ms > 0u && --m.message_ms == 0u) {
            message_done();
        }
        return false;
    }
    if (m.state == MS_BUSY) {
        return busy_tick();
    }
    /* Nothing blinks on a title or on a read-only screen. */
    if (m.state == MS_CLOSED || m.state == MS_REVISION || m.state == MS_VIEW || m.state == MS_PAGE ||
        m.state == MS_ITEM || m.state == MS_SUB) {
        return false;
    }
    if (++m.blink_ms >= BLINK_PERIOD_MS) {
        m.blink_ms = 0u;
    }

    const bool on = m.blink_ms < BLINK_ON_MS;

    if (on != m.blink_on) {
        m.blink_on = on;
        render();
    }
    return false;
}
