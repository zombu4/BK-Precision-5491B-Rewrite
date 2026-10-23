/* tables.c -- hand-written flash tables of the 5491B application (literal area
 * 0x00B6-0x1B2F).
 *
 * Every table lives in flash (TBL_FLASH); nothing here is copied to RAM
 * Types and accessors are declared in tables.h.
 * tools/check_tables.py compares each table below with the bytes of the
 * original image at the address named in its heading.
 *
 * Conventions: text is stored in named flash strings so the pointer tables can
 * reference them; a table that the original terminated with a sentinel record
 * keeps that record after the last real entry, and the matching *_count
 * excludes it.
 */
#include "tables.h"

/* ---- Display glyphs: flash 0x00F5-0x0156 --------------------------------------------------------------- */

/* '0'..'9', ':' and ';' (0x00F5). */
const uint16_t glyph_digit[GLYPH_DIGIT_COUNT] TBL_FLASH = {
    0x2362, 0x0808, 0x8361, 0x0343, 0xA203, 0xA143, 0xA163, 0x0508, 0xA363, 0xA343, 0x8001, 0x0040,
};

/* 'A'..'Z' (0x010D); 'J' is blank. */
const uint16_t glyph_alpha[GLYPH_ALPHA_COUNT] TBL_FLASH = {
    0xA323, 0x0B4B, 0x2160, 0x0B4A, 0xA161, 0xA121, 0x2163, 0xA223, 0x0948, 0x0000, 0x0C0C, 0x2060, 0x3622,
    0x3226, 0x2362, 0xA321, 0x2366, 0xA325, 0xA143, 0x0908, 0x2262, 0x2430, 0x2236, 0x1414, 0x1408, 0x0550,
};

/* 'm' 'k' 'u' '@' '-' '+' '%' '/' '&' 'n' 'd' (0x0141). */
const uint16_t glyph_symbol[GLYPH_SYMBOL_COUNT] TBL_FLASH = {
    0x802B, 0x0C0C, 0x0A03, 0x8001, 0x8001, 0x8809, 0xAC1B, 0x0410, 0xA315, 0x8028, 0x8263,
};

/* ---- Panel text: the rule that turns a string into cells ------------------------------------------------ */

#define GLYPH_DP 0x0080u
#define GLYPH_COLON 0x4080u

/* Symbol glyph indexes in tables.h order: m k u @ - + % / & n d. */
enum { SYM_M, SYM_K, SYM_U, SYM_AT, SYM_MINUS, SYM_PLUS, SYM_PERCENT, SYM_SLASH, SYM_AMP, SYM_N, SYM_D };

/* Glyph for one character. *takes_marks is set when a following '.' or ':' joins the cell. A character with
 * no glyph is a blank cell that still advances: the original skipped it without advancing, so every cell
 * after it moved one place left (ORIGINAL_BUGS 91; a menu's '^' for "no prefix" is the case that occurs).
 *
 * The two printers differ in one rule: the number printer (original 0x213A, 0x2354) lets a '-' take a
 * following '.' or ':' into its cell (0x2186-0x21AE); the simple text printer (0x2D50) does not -- there
 * ' ', '-' and '+' go straight to the cell write (0x2D70-0x2D9C) and only digits and letters look at the
 * next character. */
static uint16_t cell_glyph(char c, text_printer_t printer, bool *takes_marks) {
    *takes_marks = false;
    if (c == '-') {
        *takes_marks = (printer == TEXT_PRINTER_NUMBER);
        return tbl_glyph_symbol(SYM_MINUS);
    }
    if (c >= '0' && c <= '9') {
        *takes_marks = true;
        return tbl_glyph_digit((uint8_t)(c - '0'));
    }
    if (c >= 'A' && c <= 'Z') {
        *takes_marks = true;
        return tbl_glyph_alpha((uint8_t)(c - 'A'));
    }
    switch (c) {
    case 'm':
        return tbl_glyph_symbol(SYM_M);
    case 'k':
        return tbl_glyph_symbol(SYM_K);
    case 'u':
        return tbl_glyph_symbol(SYM_U);
    case '@':
        return tbl_glyph_symbol(SYM_AT);
    case '+':
        return tbl_glyph_symbol(SYM_PLUS);
    case '%':
        return tbl_glyph_symbol(SYM_PERCENT);
    case '/':
        return tbl_glyph_symbol(SYM_SLASH);
    case '&':
        return tbl_glyph_symbol(SYM_AMP);
    case 'n':
        return tbl_glyph_symbol(SYM_N);
    case 'd':
        return tbl_glyph_symbol(SYM_D);
    default: /* ' ' and everything without a glyph */
        return 0u;
    }
}

/* The cell that character `char_index` of `s` lands in, under exactly the rule tbl_text_cells uses.
 *
 * It exists so that a blinking cursor can blank a CELL instead of substituting a space for a character.
 * Substituting the character moves everything after it whenever the character was carrying a '.' or ':'
 * mark, because the space cannot carry the mark and the orphan then takes a cell of its own. Blanking the
 * cell cannot move anything: the glyphs are already laid out. */
uint8_t tbl_cell_of_char(const char *s, bool in_flash, text_printer_t printer, uint8_t char_index) {
    uint8_t cell = 0u;
    uint8_t i = 0u;

    while (true) {
        const char c = in_flash ? (char)pgm_read_byte(s + i) : s[i];
        bool takes_marks;

        if (c == '\0') {
            return cell; /* past the end: the caller asked about a character that is not there */
        }
        if (i == char_index) {
            return cell;
        }
        (void)cell_glyph(c, printer, &takes_marks);
        i++;
        if (takes_marks) {
            const char next = in_flash ? (char)pgm_read_byte(s + i) : s[i];

            if (next == '.' || next == ':') {
                if (i == char_index) {
                    return cell; /* the mark shares the cell of the character before it */
                }
                i++;
            }
        }
        cell++;
    }
}

uint8_t tbl_text_cells(const char *s, bool in_flash, text_printer_t printer, uint16_t *out,
                       uint8_t max_cells) {
    uint8_t n = 0;

    while (n < max_cells) {
        const char c = in_flash ? (char)pgm_read_byte(s) : *s;
        bool takes_marks;

        if (c == '\0') {
            break;
        }
        s++;
        uint16_t glyph = cell_glyph(c, printer, &takes_marks);

        if (takes_marks) {
            const char next = in_flash ? (char)pgm_read_byte(s) : *s;

            if (next == '.') {
                glyph |= GLYPH_DP;
                s++;
            } else if (next == ':') {
                glyph |= GLYPH_COLON;
                s++;
            }
        }
        out[n++] = glyph;
    }
    return n;
}

/* ---- Annunciators: flash 0x050A-0x05B1, 24 records {legend[6], VFD address} ---------------------------- */

const annun_t annun_table[ANNUN_COUNT] TBL_FLASH = {
    {"Fast", 0x1D}, {"Dc", 0x1D},   {"Ac", 0x1D},  {"Med", 0x02},  {"Slow", 0x05}, {"Trig", 0x08},
    {"Hold", 0x0B}, {"Auto", 0x0E}, {"Rel", 0x11}, {"Math", 0x14}, {"Beep", 0x17}, {"Diod", 0x1A},
    {"Comp", 0x1D}, {"Dc", 0x1D},   {"Ac", 0x1D},  {"Hi", 0x02},   {"In", 0x05},   {"Lo", 0x08},
    {"Rmt", 0x0B},  {"Auto", 0x0E}, {"Max", 0x11}, {"Min", 0x14},  {"Err", 0x17},  {"Shift", 0x1A},
};

/* ---- Range limits: flash 0x05B9-0x0783 ----------------------------------------------------------------- */

/* Highest valid range index per function (0x05B9, stored as 16-bit words in the
 * original). */
const uint8_t range_max_index[FUNC_COUNT] TBL_FLASH = {4, 4, 4, 4, 5, 4, 4, 5, 0, 4, 4};

/* Volts-at-ADC to input-unit scale factors per function and range
 * (0x071B-0x0782). DCV 0x071B, ACV 0x072F, ACA 0x0743, DCA 0x0757, resistance
 * 0x076B (also used by continuity). Functions without a per-range scale
 * (frequency, period, diode) and the two combined AC+DC functions, which load
 * both an AC and a DC scale, have no row here.
 */
/* Samples averaged per reading, by rate (state.h declares it; original RAM 0x0106, written at 0x0F046,
 * 0x0F072 and 0x0F096). The per-function exceptions are --
 * ohm range 0 uses 10/20/40, continuity 3, the diode function 6 -- and those live in meas.c because they
 * are measurement policy rather than a flash table.
 *
 * In RAM rather than flash on purpose: it is three bytes read on the hot path of every reading, and
 * reaching through pgm_read_byte for three bytes costs more than it saves. */
const uint8_t rate_sample_count[RATE_COUNT] = {3u, 6u, 11u};

const float range_scale[FUNC_COUNT][RANGE_MAX_INDEX + 1u] TBL_FLASH = {
    [FUNC_DCV] = {495.55f, 247.775f, 25.0025f, 2.5002f, 0.25f, 0.0f},
    [FUNC_ACV] = {497.018f, 263.792f, 30.902f, 2.924f, 0.24829f, 0.0f},
    [FUNC_DCA] = {12.5f, 3.125f, 0.25f, 0.03125f, 0.0025f, 0.0f},
    [FUNC_ACA] = {24.829f, 4.176f, 0.24829f, 0.04176f, 0.0024829f, 0.0f},
    [FUNC_OHM] = {24593496.0f, 2705284.5f, 245934.9f, 24593.4f, 2459.3f, 307.41f},
    [FUNC_CONT] = {24593496.0f, 2705284.5f, 245934.9f, 24593.4f, 2459.3f, 307.41f},
};

/* Overload limit per function (0x05EF-0x061A, 11 floats): a reading whose
 * magnitude exceeds the limit of the current function sets the overload flag.
 * One value per function, not per range.
 */
const float overload_limit[FUNC_COUNT] TBL_FLASH = {
    [FUNC_DCV] = 1010.0f, [FUNC_ACV] = 757.5f,    [FUNC_DCA] = 21.0f,   [FUNC_ACA] = 21.0f,
    [FUNC_OHM] = 51e6f,   [FUNC_FREQ] = 500e3f,   [FUNC_PERIOD] = 0.2f, [FUNC_CONT] = 5100.0f,
    [FUNC_DIODE] = 2.3f,  [FUNC_ACDCV] = 1010.0f, [FUNC_ACDCI] = 21.0f,
};

/* Diode range limit (0x06FF) and the six continuity thresholds, highest range
 * first (0x0703-0x071A). */
const float diode_limit TBL_FLASH = 2.3f;
const float continuity_threshold[CONTINUITY_RANGE_COUNT] TBL_FLASH = {1e6f, 1e5f, 1e4f, 1e3f, 100.0f, 10.0f};

/* Range-up thresholds (0x061B-0x06FE). The AC+DC functions use the DC voltage
 * and DC current rows. */
const float autorange_up[FUNC_COUNT][RANGE_MAX_INDEX + 1u] TBL_FLASH = {
    [FUNC_DCV] = {1010.0f, 510.0f, 51.0f, 5.1f, 0.51f, 0.0f},
    [FUNC_ACV] = {757.5f, 510.0f, 51.0f, 5.1f, 0.51f, 0.0f},
    [FUNC_DCA] = {20.0f, 5.1f, 0.51f, 0.051f, 0.0051f, 0.0f},
    [FUNC_ACA] = {20.0f, 5.1f, 0.51f, 0.051f, 0.0051f, 0.0f},
    [FUNC_OHM] = {51e6f, 5.1e6f, 510e3f, 51e3f, 5100.0f, 510.0f},
    [FUNC_ACDCV] = {1010.0f, 510.0f, 51.0f, 5.1f, 0.51f, 0.0f},
    [FUNC_ACDCI] = {20.0f, 5.1f, 0.51f, 0.051f, 0.0051f, 0.0f},
};

/* Range-down thresholds (0x062F-0x06FE). A zero entry ends the search. */
const float autorange_down[FUNC_COUNT][RANGE_MAX_INDEX + 1u] TBL_FLASH = {
    [FUNC_DCV] = {475.0f, 47.5f, 4.75f, 0.475f, 0.0f, 0.0f},
    [FUNC_ACV] = {475.0f, 47.5f, 4.75f, 0.475f, 0.0f, 0.0f},
    [FUNC_DCA] = {4.75f, 0.475f, 0.0475f, 0.00475f, 0.0f, 0.0f},
    [FUNC_ACA] = {4.75f, 0.475f, 0.0475f, 0.00475f, 0.0f, 0.0f},
    [FUNC_OHM] = {4.75e6f, 475e3f, 47.5e3f, 4750.0f, 475.0f, 0.0f},
    [FUNC_ACDCV] = {475.0f, 47.5f, 4.75f, 0.475f, 0.0f, 0.0f},
    [FUNC_ACDCI] = {4.75f, 0.475f, 0.0475f, 0.00475f, 0.0f, 0.0f},
};

/* ---- SI prefixes: flash 0x05D1-0x05EE ------------------------------------------------------------------ */

const float si_multiplier[SI_PREFIX_COUNT] TBL_FLASH = {1e6f, 1e3f, 1.0f, 1e-3f, 1e-6f, 1e-9f};
const char si_character[SI_PREFIX_COUNT] TBL_FLASH = {'M', 'k', ' ', 'm', 'u', 'n'};

/* ---- NPLC values for fast, medium, slow (original name records "0.1", "1", "10" at 0x1546) ------------- */

const float nplc_value[NPLC_COUNT] TBL_FLASH = {0.1f, 1.0f, 10.0f};

/* ---- Root keyword table: flash 0x138C-0x1545, 33 records {name[12], code} + terminator ----------------- */

static const char k_function[] TBL_FLASH = "function";
static const char k_func[] TBL_FLASH = "func";
static const char k_function2[] TBL_FLASH = "function2";
static const char k_func2[] TBL_FLASH = "func2";
static const char k_calculate[] TBL_FLASH = "calculate";
static const char k_calc[] TBL_FLASH = "calc";
static const char k_hold[] TBL_FLASH = "hold";
static const char k_current[] TBL_FLASH = "current";
static const char k_curr[] TBL_FLASH = "curr";
static const char k_voltage[] TBL_FLASH = "voltage";
static const char k_volt[] TBL_FLASH = "volt";
static const char k_resistance[] TBL_FLASH = "resistance";
static const char k_res[] TBL_FLASH = "res";
static const char k_frequency[] TBL_FLASH = "frequency";
static const char k_freq[] TBL_FLASH = "freq";
static const char k_period[] TBL_FLASH = "period";
static const char k_per[] TBL_FLASH = "per";
static const char k_diode[] TBL_FLASH = "diode";
static const char k_diod[] TBL_FLASH = "diod";
static const char k_continuity[] TBL_FLASH = "continuity";
static const char k_cont[] TBL_FLASH = "cont";
static const char k_fetch[] TBL_FLASH = "fetch";
static const char k_fetc[] TBL_FLASH = "fetc";
static const char k_display[] TBL_FLASH = "display";
static const char k_disp[] TBL_FLASH = "disp";
static const char k_unit[] TBL_FLASH = "unit";
static const char k_trigger[] TBL_FLASH = "trigger";
static const char k_trig[] TBL_FLASH = "trig";
static const char k_scal[] TBL_FLASH = "scal";
static const char k_scalibration[] TBL_FLASH = "scalibration";
static const char k_thid[] TBL_FLASH = "thid";
static const char k_system[] TBL_FLASH = "system";
static const char k_syst[] TBL_FLASH = "syst";
static const char k_none[] TBL_FLASH = "";

/* Keywords that exist in one spelling only (hold, unit, thid) repeat it as the
 * short form. */
const keyword_t root_keyword[] TBL_FLASH = {
    {k_function, k_func, 1},      {k_function2, k_func2, 2},  {k_calculate, k_calc, 3},
    {k_hold, k_hold, 4},          {k_current, k_curr, 5},     {k_voltage, k_volt, 6},
    {k_resistance, k_res, 7},     {k_frequency, k_freq, 8},   {k_period, k_per, 9},
    {k_diode, k_diod, 10},        {k_continuity, k_cont, 11}, {k_fetch, k_fetc, 12},
    {k_display, k_disp, 13},      {k_unit, k_unit, 14},       {k_trigger, k_trig, 15},
    {k_scalibration, k_scal, 16}, {k_thid, k_thid, 17},       {k_system, k_syst, 18},
    {k_none, k_none, 0x13},
};
const uint8_t root_keyword_count = 18;

/* ---- Range spellings: flash 0x156A-0x1B2B -------------------------------------------------------------- */

static const char w_1000[] TBL_FLASH = "1000";
static const char w_750[] TBL_FLASH = "750";
static const char w_500[] TBL_FLASH = "500";
static const char w_50[] TBL_FLASH = "50";
static const char w_5[] TBL_FLASH = "5";
static const char w_0_5[] TBL_FLASH = "0.5";
static const char w_1000v[] TBL_FLASH = "1000v";
static const char w_750v[] TBL_FLASH = "750v";
static const char w_500v[] TBL_FLASH = "500v";
static const char w_50v[] TBL_FLASH = "50v";
static const char w_5v[] TBL_FLASH = "5v";
static const char w_500mv[] TBL_FLASH = "500mv";
static const char w_20[] TBL_FLASH = "20";
static const char w_0_05[] TBL_FLASH = "0.05";
static const char w_0_005[] TBL_FLASH = "0.005";
static const char w_20a[] TBL_FLASH = "20a";
static const char w_5a[] TBL_FLASH = "5a";
static const char w_500ma[] TBL_FLASH = "500ma";
static const char w_50ma[] TBL_FLASH = "50ma";
static const char w_5ma[] TBL_FLASH = "5ma";
static const char w_50000000[] TBL_FLASH = "50000000";
static const char w_5000000[] TBL_FLASH = "5000000";
static const char w_500000[] TBL_FLASH = "500000";
static const char w_50000[] TBL_FLASH = "50000";
static const char w_5000[] TBL_FLASH = "5000";
static const char w_50mohm[] TBL_FLASH = "50mohm";
static const char w_5mohm[] TBL_FLASH = "5mohm";
static const char w_500kohm[] TBL_FLASH = "500kohm";
static const char w_50kohm[] TBL_FLASH = "50kohm";
static const char w_5kohm[] TBL_FLASH = "5kohm";
static const char w_500ohm[] TBL_FLASH = "500ohm";
static const char w_10[] TBL_FLASH = "10";
static const char w_100[] TBL_FLASH = "100";
static const char w_1[] TBL_FLASH = "1";
static const char w_10ua[] TBL_FLASH = "10ua";
static const char w_100ua[] TBL_FLASH = "100ua";
static const char w_1ma[] TBL_FLASH = "1ma";
static const char w_def[] TBL_FLASH = "def";
static const char w_default[] TBL_FLASH = "default";
static const char w_min[] TBL_FLASH = "min";
static const char w_minimum[] TBL_FLASH = "minimum";
static const char w_max[] TBL_FLASH = "max";
static const char w_maximum[] TBL_FLASH = "maximum";

/* The original keeps seven near-identical tables selected by function; here
 * every entry carries its function. Order within a function is the original
 * order, so the first match wins as before. DC volts 0x156A; AC volts 0x16E0
 * (also frequency, period); DC current 0x179B; AC current 0x1856; resistance
 * 0x1911 (also continuity); diode 0x1A9D; AC+DC volts 0x19E2; AC+DC current
 * 0x1625. The original ends each table with an all-zero record; it is not kept.
 */
const range_word_t range_word[] TBL_FLASH = {
    {w_1000, FUNC_DCV, 0},       {w_500, FUNC_DCV, 1},        {w_50, FUNC_DCV, 2},
    {w_5, FUNC_DCV, 3},          {w_0_5, FUNC_DCV, 4},        {w_1000v, FUNC_DCV, 0},
    {w_500v, FUNC_DCV, 1},       {w_50v, FUNC_DCV, 2},        {w_5v, FUNC_DCV, 3},
    {w_500mv, FUNC_DCV, 4},      {w_def, FUNC_DCV, 0},        {w_default, FUNC_DCV, 0},
    {w_min, FUNC_DCV, 4},        {w_minimum, FUNC_DCV, 4},    {w_max, FUNC_DCV, 0},
    {w_maximum, FUNC_DCV, 0},

    {w_750, FUNC_ACV, 0},        {w_500, FUNC_ACV, 1},        {w_50, FUNC_ACV, 2},
    {w_5, FUNC_ACV, 3},          {w_0_5, FUNC_ACV, 4},        {w_750v, FUNC_ACV, 0},
    {w_500v, FUNC_ACV, 1},       {w_50v, FUNC_ACV, 2},        {w_5v, FUNC_ACV, 3},
    {w_500mv, FUNC_ACV, 4},      {w_def, FUNC_ACV, 0},        {w_default, FUNC_ACV, 0},
    {w_min, FUNC_ACV, 4},        {w_minimum, FUNC_ACV, 4},    {w_max, FUNC_ACV, 0},
    {w_maximum, FUNC_ACV, 0},

    {w_20, FUNC_DCA, 0},         {w_5, FUNC_DCA, 1},          {w_0_5, FUNC_DCA, 2},
    {w_0_05, FUNC_DCA, 3},       {w_0_005, FUNC_DCA, 4},      {w_20a, FUNC_DCA, 0},
    {w_5a, FUNC_DCA, 1},         {w_500ma, FUNC_DCA, 2},      {w_50ma, FUNC_DCA, 3},
    {w_5ma, FUNC_DCA, 4},        {w_def, FUNC_DCA, 0},        {w_default, FUNC_DCA, 0},
    {w_min, FUNC_DCA, 4},        {w_minimum, FUNC_DCA, 4},    {w_max, FUNC_DCA, 0},
    {w_maximum, FUNC_DCA, 0},

    {w_20, FUNC_ACA, 0},         {w_5, FUNC_ACA, 1},          {w_0_5, FUNC_ACA, 2},
    {w_0_05, FUNC_ACA, 3},       {w_0_005, FUNC_ACA, 4},      {w_20a, FUNC_ACA, 0},
    {w_5a, FUNC_ACA, 1},         {w_500ma, FUNC_ACA, 2},      {w_50ma, FUNC_ACA, 3},
    {w_5ma, FUNC_ACA, 4},        {w_def, FUNC_ACA, 0},        {w_default, FUNC_ACA, 0},
    {w_min, FUNC_ACA, 4},        {w_minimum, FUNC_ACA, 4},    {w_max, FUNC_ACA, 0},
    {w_maximum, FUNC_ACA, 0},

    {w_50000000, FUNC_OHM, 0},   {w_5000000, FUNC_OHM, 1},    {w_500000, FUNC_OHM, 2},
    {w_50000, FUNC_OHM, 3},      {w_5000, FUNC_OHM, 4},       {w_500, FUNC_OHM, 5},
    {w_50mohm, FUNC_OHM, 0},     {w_5mohm, FUNC_OHM, 1},      {w_500kohm, FUNC_OHM, 2},
    {w_50kohm, FUNC_OHM, 3},     {w_5kohm, FUNC_OHM, 4},      {w_500ohm, FUNC_OHM, 5},
    {w_def, FUNC_OHM, 0},        {w_default, FUNC_OHM, 0},    {w_min, FUNC_OHM, 5},
    {w_minimum, FUNC_OHM, 5},    {w_max, FUNC_OHM, 0},        {w_maximum, FUNC_OHM, 0},

    {w_750, FUNC_FREQ, 0},       {w_500, FUNC_FREQ, 1},       {w_50, FUNC_FREQ, 2},
    {w_5, FUNC_FREQ, 3},         {w_0_5, FUNC_FREQ, 4},       {w_750v, FUNC_FREQ, 0},
    {w_500v, FUNC_FREQ, 1},      {w_50v, FUNC_FREQ, 2},       {w_5v, FUNC_FREQ, 3},
    {w_500mv, FUNC_FREQ, 4},     {w_def, FUNC_FREQ, 0},       {w_default, FUNC_FREQ, 0},
    {w_min, FUNC_FREQ, 4},       {w_minimum, FUNC_FREQ, 4},   {w_max, FUNC_FREQ, 0},
    {w_maximum, FUNC_FREQ, 0},

    {w_750, FUNC_PERIOD, 0},     {w_500, FUNC_PERIOD, 1},     {w_50, FUNC_PERIOD, 2},
    {w_5, FUNC_PERIOD, 3},       {w_0_5, FUNC_PERIOD, 4},     {w_750v, FUNC_PERIOD, 0},
    {w_500v, FUNC_PERIOD, 1},    {w_50v, FUNC_PERIOD, 2},     {w_5v, FUNC_PERIOD, 3},
    {w_500mv, FUNC_PERIOD, 4},   {w_def, FUNC_PERIOD, 0},     {w_default, FUNC_PERIOD, 0},
    {w_min, FUNC_PERIOD, 4},     {w_minimum, FUNC_PERIOD, 4}, {w_max, FUNC_PERIOD, 0},
    {w_maximum, FUNC_PERIOD, 0},

    {w_50000000, FUNC_CONT, 0},  {w_5000000, FUNC_CONT, 1},   {w_500000, FUNC_CONT, 2},
    {w_50000, FUNC_CONT, 3},     {w_5000, FUNC_CONT, 4},      {w_500, FUNC_CONT, 5},
    {w_50mohm, FUNC_CONT, 0},    {w_5mohm, FUNC_CONT, 1},     {w_500kohm, FUNC_CONT, 2},
    {w_50kohm, FUNC_CONT, 3},    {w_5kohm, FUNC_CONT, 4},     {w_500ohm, FUNC_CONT, 5},
    {w_def, FUNC_CONT, 0},       {w_default, FUNC_CONT, 0},   {w_min, FUNC_CONT, 5},
    {w_minimum, FUNC_CONT, 5},   {w_max, FUNC_CONT, 0},       {w_maximum, FUNC_CONT, 0},

    {w_10, FUNC_DIODE, 0},       {w_100, FUNC_DIODE, 1},      {w_1, FUNC_DIODE, 2},
    {w_10ua, FUNC_DIODE, 0},     {w_100ua, FUNC_DIODE, 1},    {w_1ma, FUNC_DIODE, 2},
    {w_def, FUNC_DIODE, 2},      {w_default, FUNC_DIODE, 2},  {w_min, FUNC_DIODE, 0},
    {w_minimum, FUNC_DIODE, 0},  {w_max, FUNC_DIODE, 2},      {w_maximum, FUNC_DIODE, 2},

    {w_750, FUNC_ACDCV, 0},      {w_500, FUNC_ACDCV, 1},      {w_50, FUNC_ACDCV, 2},
    {w_5, FUNC_ACDCV, 3},        {w_0_5, FUNC_ACDCV, 4},      {w_750v, FUNC_ACDCV, 0},
    {w_500v, FUNC_ACDCV, 1},     {w_50v, FUNC_ACDCV, 2},      {w_5v, FUNC_ACDCV, 3},
    {w_500mv, FUNC_ACDCV, 4},    {w_def, FUNC_ACDCV, 0},      {w_default, FUNC_ACDCV, 0},
    {w_min, FUNC_ACDCV, 4},      {w_minimum, FUNC_ACDCV, 4},  {w_max, FUNC_ACDCV, 0},
    {w_maximum, FUNC_ACDCV, 0},

    {w_20, FUNC_ACDCI, 0},       {w_5, FUNC_ACDCI, 1},        {w_0_5, FUNC_ACDCI, 2},
    {w_0_05, FUNC_ACDCI, 3},     {w_0_005, FUNC_ACDCI, 4},    {w_20a, FUNC_ACDCI, 0},
    {w_5a, FUNC_ACDCI, 1},       {w_500ma, FUNC_ACDCI, 2},    {w_50ma, FUNC_ACDCI, 3},
    {w_5ma, FUNC_ACDCI, 4},      {w_def, FUNC_ACDCI, 0},      {w_default, FUNC_ACDCI, 0},
    {w_min, FUNC_ACDCI, 4},      {w_minimum, FUNC_ACDCI, 4},  {w_max, FUNC_ACDCI, 0},
    {w_maximum, FUNC_ACDCI, 0},
};
const uint8_t range_word_count = 176;

/* ---- Function and second-display name records: flash 0x0787-0x0933, 11-byte {name[10], index} ---------- */

/* 0x0787: primary function keywords; the sentinel (index 0x13) follows the nine
 * real records. */
const name_record_t func_name[] TBL_FLASH = {
    {"volt:dc", 0}, {"volt:ac", 1}, {"curr:dc", 2}, {"curr:ac", 3}, {"res", 4},
    {"freq", 5},    {"per", 6},     {"cont", 7},    {"diod", 8},    {"", 0x13},
};
const uint8_t func_name_count = 9;

/* 0x07F5-0x0933: the seven second-display choice lists, each ended by a
 * sentinel record (index 0x13), in this order: DC volts (5 records), DC current
 * (3), AC volts (5), AC current (3), AC+DC volts (6), AC+DC current (4),
 * frequency/period input source (3). The count includes the sentinels.
 */
const name_record_t second_display_name[] TBL_FLASH = {
    {"volt:ac", 0}, {"freq", 1},    {"db", 2},      {"dbm", 3},     {"", 0x13},  {"curr:ac", 0},
    {"freq", 1},    {"", 0x13},     {"volt:dc", 0}, {"freq", 1},    {"db", 2},   {"dbm", 3},
    {"", 0x13},     {"curr:dc", 0}, {"freq", 1},    {"", 0x13},     {"freq", 0}, {"db", 1},
    {"dbm", 2},     {"volt:dc", 3}, {"volt:ac", 4}, {"", 0x13},     {"freq", 0}, {"curr:ac", 1},
    {"curr:dc", 2}, {"", 0x13},     {"volt:ac", 0}, {"curr:ac", 1}, {"", 0x13},
};
const uint8_t second_display_name_count = 29;

/* ---- Error fragments: flash 0x0B36-0x0BCA, indexed by the original message number 0..13 ---------------- */

static const char ef_blank[] TBL_FLASH = "         ";
static const char ef_bus_bad[] TBL_FLASH = " BUS: BAD ";
static const char ef_command[] TBL_FLASH = " COMMAND. ";
static const char ef_bus_syntax[] TBL_FLASH = "BUS:SYNTAX";
static const char ef_error[] TBL_FLASH = " ERROR   ";
static const char ef_bus_data[] TBL_FLASH = "BUS: DATA ";
static const char ef_too_long[] TBL_FLASH = "TOO LONG ";
static const char ef_bus_cannot[] TBL_FLASH = "BUS:CANNOT";
static const char ef_executed[] TBL_FLASH = " EXECUTED.";
static const char ef_bus_error[] TBL_FLASH = "BUS:ERROR ";
static const char ef_suffix[] TBL_FLASH = " SUFFIX.  ";
static const char ef_parameter[] TBL_FLASH = "PARAMETER.";
static const char ef_bus_record[] TBL_FLASH = "BUS:RECORD";
static const char ef_not_exist[] TBL_FLASH = "NOT EXIST";

/* Message text is built from two fragment indexes; index 0 is nine blanks (no
 * second part). Flash 0x135E. */
const char *const error_fragment[ERROR_FRAGMENT_COUNT] TBL_FLASH = {
    ef_blank,      ef_bus_bad,  ef_command,   ef_bus_syntax, ef_error,     ef_bus_data,   ef_too_long,
    ef_bus_cannot, ef_executed, ef_bus_error, ef_suffix,     ef_parameter, ef_bus_record, ef_not_exist,
};

/* ---- Menu strings, prompts, range texts and status messages -------------------------------------------- */

/* The vendor's table (flash 0x0BCC-0x0EE6, 95 strings) with these changes: no "A:" or
 * "1:" prefixes, words spelled out, the vendor's "MEU" spelt MENU, and every text within the nine cells of
 * a line. 37 vendor strings that nothing shows any more are gone. tools/check_tables.py names each string
 * that differs from the image, with the vendor's text beside ours, and each one that was dropped; docs/MENU_TEXTS.md
 * lists every text with its cell count.
 *
 * The choice and range values are shown on their own, with the name of the setting on the second line,
 * so none of them carries a prompt or padding of its own. Strings kept from the vendor keep their trailing
 * blanks; the menu trims them. */

static const char mt_cr[] TBL_FLASH = "RETURN";
static const char mt_lf[] TBL_FLASH = "LINE FEED";
static const char mt_pct10[] TBL_FLASH = "10%  ";
static const char mt_pct1[] TBL_FLASH = "1%   ";
static const char mt_pct0_1[] TBL_FLASH = "0.1% ";
static const char mt_pct0_01[] TBL_FLASH = "0.01%";
static const char mt_baud600[] TBL_FLASH = "600 ";
static const char mt_baud1200[] TBL_FLASH = "1200";
static const char mt_baud2400[] TBL_FLASH = "2400";
static const char mt_baud4800[] TBL_FLASH = "4800";
static const char mt_baud9600[] TBL_FLASH = "9600";
static const char mt_baud19k2[] TBL_FLASH = "19.2K";
static const char mt_baud38k4[] TBL_FLASH = "38.4K";
static const char mt_on[] TBL_FLASH = "ON    ";
static const char mt_off[] TBL_FLASH = "OFF   ";
static const char mt_bus[] TBL_FLASH = "BUS";
static const char mt_man[] TBL_FLASH = "MANUAL";
static const char mt_imm[] TBL_FLASH = "IMMEDIATE";
static const char mt_item_zero_cal[] TBL_FLASH = "ZERO CAL";
static const char mt_item_revision[] TBL_FLASH = "REVISION";
static const char mt_item_key_sound[] TBL_FLASH = "KEY SOUND";
static const char mt_item_echo[] TBL_FLASH = "ECHO";
static const char mt_item_line_end[] TBL_FLASH = "LINE END";
static const char mt_item_baud[] TBL_FLASH = "BAUD RATE";
static const char mt_item_beeper[] TBL_FLASH = "BEEPER";
static const char mt_item_hold_cnt[] TBL_FLASH = "HOLD CNT";
static const char mt_item_hold_win[] TBL_FLASH = "HOLD WIN";
static const char mt_item_trig_mode[] TBL_FLASH = "TRIG MODE";
static const char mt_item_dbm_ref[] TBL_FLASH = "dBm REF";
static const char mt_item_db_ref[] TBL_FLASH = "dB REF";
static const char mt_item_perc_ref[] TBL_FLASH = "PERC REF";
static const char mt_item_lo_limit[] TBL_FLASH = "LO LIMIT";
static const char mt_item_hi_limit[] TBL_FLASH = "HI LIMIT";
static const char mt_menu_sys[] TBL_FLASH = "SYS MENU";
static const char mt_menu_trig[] TBL_FLASH = "TRIG MENU";
static const char mt_menu_math[] TBL_FLASH = "MATH MENU";
static const char mt_prompt_cal_short[] TBL_FLASH = "CAL:SHORT ";
static const char mt_range_500r[] TBL_FLASH = "500&";
static const char mt_range_5k[] TBL_FLASH = "5k&";
static const char mt_range_50k[] TBL_FLASH = "50k&";
static const char mt_range_500k[] TBL_FLASH = "500k&";
static const char mt_range_5m[] TBL_FLASH = "5M&";
static const char mt_range_50m[] TBL_FLASH = "50M&";
static const char mt_range_5ma[] TBL_FLASH = "5mA";
static const char mt_range_50ma[] TBL_FLASH = "50mA";
static const char mt_range_500ma[] TBL_FLASH = "500mA";
static const char mt_range_5a[] TBL_FLASH = "5A";
static const char mt_range_20a[] TBL_FLASH = "20A";
static const char mt_range_500mv[] TBL_FLASH = "500mV";
static const char mt_range_5v[] TBL_FLASH = "5V";
static const char mt_range_50v[] TBL_FLASH = "50V";
static const char mt_range_500v[] TBL_FLASH = "500V";
static const char mt_range_1000v[] TBL_FLASH = "1000V";
static const char mt_lbl_dcv[] TBL_FLASH = "DCV";
static const char mt_lbl_ohm[] TBL_FLASH = "OHM";
static const char mt_msg_saved[] TBL_FLASH = "  SAVED  ";
static const char mt_msg_too_small[] TBL_FLASH = "TOO SMALL";
static const char mt_msg_too_large[] TBL_FLASH = "TOO LARGE";

/* In the order of menu_text_id_t. */
const char *const menu_text[] TBL_FLASH = {
    mt_cr,
    mt_lf,
    mt_pct10,
    mt_pct1,
    mt_pct0_1,
    mt_pct0_01,
    mt_baud600,
    mt_baud1200,
    mt_baud2400,
    mt_baud4800,
    mt_baud9600,
    mt_baud19k2,
    mt_baud38k4,
    mt_on,
    mt_off,
    mt_bus,
    mt_man,
    mt_imm,
    mt_item_zero_cal,
    mt_item_revision,
    mt_item_key_sound,
    mt_item_echo,
    mt_item_line_end,
    mt_item_baud,
    mt_item_beeper,
    mt_item_hold_cnt,
    mt_item_hold_win,
    mt_item_trig_mode,
    mt_item_dbm_ref,
    mt_item_db_ref,
    mt_item_perc_ref,
    mt_item_lo_limit,
    mt_item_hi_limit,
    mt_menu_sys,
    mt_menu_trig,
    mt_menu_math,
    mt_prompt_cal_short,
    mt_range_500r,
    mt_range_5k,
    mt_range_50k,
    mt_range_500k,
    mt_range_5m,
    mt_range_50m,
    mt_range_5ma,
    mt_range_50ma,
    mt_range_500ma,
    mt_range_5a,
    mt_range_20a,
    mt_range_500mv,
    mt_range_5v,
    mt_range_50v,
    mt_range_500v,
    mt_range_1000v,
    mt_lbl_dcv,
    mt_lbl_ohm,
    mt_msg_saved,
    mt_msg_too_small,
    mt_msg_too_large,
};
const uint8_t menu_text_count = 58;
