/* fmt.c -- number formatting and scanning.
 *
 * Replaces the ICCAVR runtime's ftoa (0x13FA4), printf float conversion (0x14D14), strtod (0x13DC0),
 * atoi (0x136B0), the itoa/ltoa/utoa/ultoa family (0x13732/0x137FA/0x13D74/0x13D0A) and the number part
 * of the scanf engine (0x14A52/0x149B2).
 *
 * The behaviour of the original runtime was recovered from its code and checked in a simulator, including
 * the exact %e trailing-zero rule (0x14D14).
 *
 * WHAT IS PRESERVED (rule: reply LAYOUT is preserved)
 *   - Field widths, separators, padding and justification.
 *   - Exponent style: 'e' or 'E', a '-' only when negative, no '+', no zero padding, no minimum width.
 *     So "e0", "e5", "e-6", "e37" -- never "e+05".
 *   - The trailing-zero trim of the %e mantissa, and the "at least one digit after the point" rule.
 *   - Rounding direction: half away from zero, as the original's "+half" trick and its "next digit >= 5"
 *     test both did. A standard C printf rounds halfway cases to even instead, so this module deliberately
 *     differs from printf on exact ties such as 0.03125 at 4 decimals (here "0.0313", printf "0.0312").
 *
 * WHAT IS FIXED (every pre-existing bug is fixed; see ORIGINAL_BUGS.md and PROTOCOL_CHANGES.md)
 *   - Digits are now exact. The original produced them from a 24-bit fixed-point fraction and truncated,
 *     so 1e-5 printed as "9.9e-6" and 0.0012345 as "1.2344e-3". Both are now correctly rounded.
 *   - %e and %E of 0.0 printed NOTHING (the code read the byte before its own buffer). Now "0.0e0", which
 *     is what the surviving layout rules produce for zero.
 *   - |x| >= 2^31 or 0 < |x| < 1.19e-7 made ftoa fail, and the conversion then emitted the literal text
 *     "ftoa error: number too big" / "... too small" in place of the number. A reply can no longer contain
 *     words where a number belongs: the full binary32 range prints.
 *   - A mantissa carry that turned 9.99.. into 1.0 left the exponent text as "-0". Now "e0".
 *   - A left-justified %e field was silently one character short of the width.
 *   - The '0' flag padded zeros in front of the minus sign.
 *   - %f with precision >= 7 printed the "too small" error text for x = 0.
 *   - Input: a token of 20 or more characters overflowed a 20-byte stack buffer. The token
 *     limit is now FMT_NUM_TOKEN_MAX and is enforced.
 *   - Input: an out-of-range exponent silently produced a wrong value ("1e39" became 1.1755e-38, which
 *     could be written into a calibration coefficient). It is now FMT_ERR_RANGE. "1e60000" used to loop a
 *     multiply 60000 times; it now fails immediately.
 *   - Input: a numeric parameter of exactly 0 was rejected with error (9,10). Now 0 is accepted
 *     wherever the parameter's range allows it, so FMT_ERR_ZERO is never returned (the enumerator is kept
 *     so existing call sites still compile).
 *   - Input: integer parameters went through a 16-bit atoi, so "99999" silently became 34463.
 *     fmt_parse_long is 32-bit and reports FMT_ERR_RANGE instead of wrapping.
 *   - Input: hexadecimal, "inf" and "nan" produced 0.0, which the caller then rejected as a zero
 *     parameter. They are now FMT_ERR_SYNTAX, which is what the caller should report.
 *   - fmt_pow10f replaces the library pow() that the panel digit scaling used (0x146D8, called from
 *     0x01DBA and 0x0541A). That pow was accurate to about 5 digits -- pow(10,2) returned 99.9999.
 *
 * ACCURACY
 *   - Output: exact. The decimal digits are produced from the binary32 value with integer arithmetic
 *     only, so the printed digits are those of the stored value, correctly rounded at the cut.
 *   - Input: within 1.5 ulp. The significant digits go into a uint32 exactly and are then scaled by an
 *     exact power of ten, which is one rounding of the digits plus one of the product. The original was
 *     1 to 3 ulp off, so this is strictly closer without claiming to be exact.
 *
 * STACK
 *   fdigits() uses about 200 bytes of stack (a 29-limb big integer and a 116-character digit string).
 *   It is called from the display path and the reply path, never from an interrupt.
 */
#include "fmt.h"

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_float(p) (*(const float *)(p))
#endif

/* ---- Bounded output ------------------------------------------------------------------------------------ */

/* Characters past dst_size are dropped, and the return value counts only what was stored, so a caller that
 * adds the return to an offset can never walk past its own buffer. */
typedef struct {
    char *dst;
    size_t size;
    size_t len;
} out_t;

static void put(out_t *o, char c) {
    if (o->len < o->size) {
        o->dst[o->len++] = c;
    }
}

static void put_rep(out_t *o, char c, uint8_t n) {
    while (n-- != 0u) {
        put(o, c);
    }
}

/* ---- Exact decimal digits of a binary32 --------------------------------------------------------------- */

/* The value is mant * 2^exp2 with mant < 2^24. For exp2 >= 0 that is an integer; for exp2 < 0 it is
 * mant * 5^-exp2 * 10^exp2, and mant * 5^149 has at most 112 digits. A base-10000 big integer of 29 limbs
 * holds 116 digits, which covers both. Every digit is then exact, with no floating point anywhere. */
#define BIG_LIMBS 29u
#define BIG_TEXT_MAX (BIG_LIMBS * 4u + 2u)

typedef struct {
    uint16_t l[BIG_LIMBS]; /* l[0] is the least significant group of four digits */
    uint8_t n;
} big_t;

static void big_set(big_t *b, uint32_t v) {
    b->n = 0u;
    do {
        b->l[b->n++] = (uint16_t)(v % 10000u);
        v /= 10000u;
    } while (v != 0u);
}

/* b *= m. m is at most 625, so a limb product is at most 9999 * 625 + 624 and fits in 32 bits. */
static void big_mul(big_t *b, uint16_t m) {
    uint16_t carry = 0u;
    uint8_t i;

    for (i = 0u; i < b->n; i++) {
        const uint32_t p = (uint32_t)b->l[i] * m + carry;

        b->l[i] = (uint16_t)(p % 10000u);
        carry = (uint16_t)(p / 10000u);
    }
    while (carry != 0u && b->n < BIG_LIMBS) {
        b->l[b->n++] = (uint16_t)(carry % 10000u);
        carry /= 10000u;
    }
}

static void big_mul_pow2(big_t *b, int16_t k) {
    while (k >= 9) {
        big_mul(b, 512u);
        k -= 9;
    }
    if (k > 0) {
        big_mul(b, (uint16_t)(1u << (uint8_t)k));
    }
}

static void big_mul_pow5(big_t *b, int16_t k) {
    static const uint16_t small[4] = {5u, 25u, 125u, 625u};

    while (k >= 4) {
        big_mul(b, 625u);
        k -= 4;
    }
    if (k > 0) {
        big_mul(b, small[k - 1]);
    }
}

static uint8_t u16_text(char *out, uint16_t v) {
    char tmp[5];
    uint8_t n = 0u;
    uint8_t i;

    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u);
    for (i = 0u; i < n; i++) {
        out[i] = tmp[n - 1u - i];
    }
    return n;
}

/* Decimal text of b, most significant digit first, with no leading zero. Returns the length. */
static uint8_t big_text(const big_t *b, char *out) {
    uint8_t len;
    uint8_t i = b->n;

    i--;
    len = u16_text(out, b->l[i]);
    while (i != 0u) {
        uint16_t v;

        i--;
        v = b->l[i];
        out[len++] = (char)('0' + (v / 1000u));
        out[len++] = (char)('0' + ((v / 100u) % 10u));
        out[len++] = (char)('0' + ((v / 10u) % 10u));
        out[len++] = (char)('0' + (v % 10u));
    }
    return len;
}

/* Nine digits round-trip a binary32, but a fixed-point field must print the INTEGER part exactly, and the
 * exact integer value of a large float runs to 39 digits (2^128 is 3.4e38). The original did print it
 * exactly: its ftoa shifted the 24-bit mantissa up into a 32-bit integer and ran ltoa on the result, so
 * 2147483520 printed as all ten of its digits. Capping at nine would turn 2147483648 into 2147483650,
 * which is a wrong number, not a layout difference. So the digit buffer holds the widest integer part plus
 * the original's own precision clamp of 20 fraction digits. */
#define FDIG_MAX 60u

/* The scientific and general forms stay at nine significant digits. One digit sits before the point there,
 * so no exact integer part is at stake, and beyond nine digits a binary32 carries no information. The
 * original's widest such format was "%.8e", which is exactly nine. */
#define FDIG_SCI_MAX 9u

/* The original clamped a printf precision to 20. */
#define FDIG_DECIMALS_MAX 20u

/* Significant digits the scanner keeps. Nine is the most a uint32 accumulator holds without overflowing
 * and the most a binary32 can distinguish; further digits only move the decimal exponent. */
#define FMT_PARSE_SIG 9u

/* NaN and infinity cannot come out of the original's arithmetic but can come out of this port's, and a
 * reply must stay parseable. Mapping them here keeps every entry point consistent about it. */
static float finite_of(float v) {
    union {
        float f;
        uint32_t u;
    } b;

    b.f = v;
    if (((b.u >> 23) & 0xFFu) != 0xFFu) {
        return v;
    }
    if ((b.u & 0x7FFFFFuL) != 0u) {
        return 0.0f; /* not a number */
    }
    b.u = (b.u & 0x80000000uL) | 0x7F7FFFFFuL; /* infinity -> the largest finite float */
    return b.f;
}

typedef struct {
    char d[FDIG_MAX + 1u]; /* d[0] is the first significant digit */
    uint8_t count;         /* how many of d[] are filled, always the requested count */
    int16_t exp10;         /* the value is d[0] . d[1] d[2] ... times 10^exp10 */
    bool negative;
    bool zero;
} fdig_t;

/* `sig` significant digits of v, rounded half away from zero at the cut.
 *
 * Infinity and not-a-number cannot come out of the original's arithmetic (its exponent-255 patterns were
 * ordinary large numbers), but they can come out of this port's real IEEE arithmetic. A
 * reply must stay parseable, so an infinity formats as the largest finite float and a NaN formats as zero.
 * Neither should ever reach here; the measurement path is responsible for not producing them. */
static void fdigits(float v, uint8_t sig, fdig_t *o) {
    union {
        float f;
        uint32_t u;
    } bits;
    uint32_t mant;
    int16_t exp2;
    uint8_t biased;
    big_t big;
    char text[BIG_TEXT_MAX];
    uint8_t len;
    int16_t dec;
    uint8_t i;
    bool round_up;

    if (sig < 1u) {
        sig = 1u;
    }
    if (sig > FDIG_MAX) {
        sig = FDIG_MAX;
    }

    bits.f = finite_of(v);
    o->negative = (bits.u >> 31) != 0u;
    o->zero = false;
    o->count = sig;
    o->exp10 = 0;

    biased = (uint8_t)((bits.u >> 23) & 0xFFu);
    mant = bits.u & 0x7FFFFFuL;

    if (biased == 0u && mant == 0u) {
        o->zero = true;
        o->negative = false; /* a zero prints unsigned, as the original's arithmetic never made a -0 */
    }
    if (o->zero) {
        for (i = 0u; i < sig; i++) {
            o->d[i] = '0';
        }
        return;
    }

    if (biased == 0u) { /* subnormal: no hidden bit */
        exp2 = -149;
    } else {
        mant |= 0x800000uL;
        exp2 = (int16_t)biased - 150;
    }

    big_set(&big, mant);
    if (exp2 >= 0) {
        big_mul_pow2(&big, exp2);
        dec = 0;
    } else {
        big_mul_pow5(&big, (int16_t)-exp2);
        dec = exp2;
    }

    len = big_text(&big, text);
    o->exp10 = (int16_t)((int16_t)(len - 1u) + dec);

    if (len <= sig) {
        for (i = 0u; i < len; i++) {
            o->d[i] = text[i];
        }
        for (i = len; i < sig; i++) {
            o->d[i] = '0';
        }
        return;
    }

    for (i = 0u; i < sig; i++) {
        o->d[i] = text[i];
    }

    /* Half away from zero: the first dropped digit decides on its own, exactly as the original's
     * "next digit >= 5" test did. Digits beyond it can only strengthen a round up, never reverse one. */
    round_up = text[sig] >= '5';
    if (round_up) {
        i = sig;
        while (i-- != 0u) {
            if (o->d[i] != '9') {
                o->d[i]++;
                return;
            }
            o->d[i] = '0';
        }
        /* every digit was a 9: 9.99 becomes 1.00 one decade up */
        o->d[0] = '1';
        o->exp10++;
    }
}

/* ---- Exact powers of ten ------------------------------------------------------------------------------- */

static const float pow10_tab[39] PROGMEM = {
    1e0f,  1e1f,  1e2f,  1e3f,  1e4f,  1e5f,  1e6f,  1e7f,  1e8f,  1e9f,  1e10f, 1e11f, 1e12f,
    1e13f, 1e14f, 1e15f, 1e16f, 1e17f, 1e18f, 1e19f, 1e20f, 1e21f, 1e22f, 1e23f, 1e24f, 1e25f,
    1e26f, 1e27f, 1e28f, 1e29f, 1e30f, 1e31f, 1e32f, 1e33f, 1e34f, 1e35f, 1e36f, 1e37f, 1e38f};

float fmt_pow10f(int8_t n) {
    if (n > 38) {
        n = 38;
    }
    if (n < -38) {
        n = -38;
    }
    if (n >= 0) {
        return pgm_read_float(&pow10_tab[n]);
    }
    return 1.0f / pgm_read_float(&pow10_tab[-n]);
}

/* ---- Integers ------------------------------------------------------------------------------------------ */

size_t fmt_int(char *dst, size_t dst_size, int32_t v) {
    out_t o = {dst, dst_size, 0u};
    char tmp[10];
    uint8_t n = 0u;
    uint32_t u;

    if (v < 0) {
        put(&o, '-');
        u = (uint32_t)(-(v + 1)) + 1uL; /* INT32_MIN without signed overflow */
    } else {
        u = (uint32_t)v;
    }
    do {
        tmp[n++] = (char)('0' + (uint8_t)(u % 10uL));
        u /= 10uL;
    } while (u != 0uL);
    while (n-- != 0u) {
        put(&o, tmp[n]);
    }
    return o.len;
}

/* ---- Fixed point ("%#<width>.<decimals>f") -------------------------------------------------------------- */

/* The decimal point is always printed and the fraction is always `decimals` long, which is what the
 * original's '#' flag did for the panel formats %+#7.4f and %#7.4f. The field is right justified in
 * `width` with spaces; content longer than the width is not cut (the original also raised the width to the
 * content length). A negative value that rounds to zero keeps its sign, as it did. */
static size_t fixed_core(out_t *o, float v, uint8_t width, uint8_t decimals) {
    fdig_t g;
    int16_t needed;
    uint8_t int_len;
    uint8_t body;
    uint8_t j;
    bool show_sign;

    if (decimals > FDIG_DECIMALS_MAX) {
        decimals = FDIG_DECIMALS_MAX;
    }

    fdigits(v, FDIG_MAX, &g);
    show_sign = g.negative && !g.zero; /* an exact zero prints unsigned */
    if (g.zero) {
        g.exp10 = 0;
        needed = 1;
    } else {
        needed = (int16_t)(g.exp10 + 1 + (int16_t)decimals);
        if (needed <= 0) {
            /* The value sits entirely below the last printed place. Only the first significant digit can
             * still carry into it. The original kept the sign of a negative value that rounded away. */
            const bool carry_only = (needed == 0) && (g.d[0] >= '5');

            g.exp10 = -(int16_t)decimals;
            for (j = 0u; j < FDIG_MAX; j++) {
                g.d[j] = '0';
            }
            if (carry_only) {
                g.d[0] = '1';
            }
            g.count = FDIG_MAX;
            needed = 1;
        } else if (needed < (int16_t)FDIG_MAX) {
            fdigits(v, (uint8_t)needed, &g); /* round once, at the place that is actually printed */
        }
    }

    int_len = (g.exp10 >= 0) ? (uint8_t)(g.exp10 + 1) : 1u; /* "0" when |v| < 1 */

    body = (uint8_t)(int_len + 1u + decimals);
    if (show_sign) {
        body++;
    }
    if (width > body) {
        put_rep(o, ' ', (uint8_t)(width - body));
    }

    if (show_sign) {
        put(o, '-');
    }

    if (g.exp10 < 0) {
        put(o, '0');
    } else {
        for (j = 0u; j < int_len; j++) {
            put(o, (j < g.count) ? g.d[j] : '0');
        }
    }

    put(o, '.');
    for (j = 1u; j <= decimals; j++) {
        const int16_t i = (int16_t)(g.exp10 + (int16_t)j);

        put(o, (i >= 0 && i < (int16_t)g.count) ? g.d[i] : '0');
    }
    return o->len;
}

size_t fmt_fixed(char *dst, size_t dst_size, float v, uint8_t width, uint8_t decimals) {
    out_t o = {dst, dst_size, 0u};

    return fixed_core(&o, v, width, decimals);
}

/* ---- Scientific ("%.<decimals>e") ---------------------------------------------------------------------- */

/* The trailing-zero trim of the mantissa, (recovered from 0x14ECC-0x14F8C and
 * executed in an interpreter against the real image).
 *
 * The original built the mantissa out of an intermediate decimal text and computed an end index E: the
 * position just past the last non-zero FRACTION digit of that text, or just past the decimal point when
 * the fraction was all zeros. Digits before E were copied whatever their value, so zeros belonging to the
 * integer part were never dropped; past E a zero was appended only while nothing had been emitted yet.
 * That is why "%.8e" of 1.0 is "1.0e0", of 100.0 is "1.00e2" and of 1e6 is "1.000000e6".
 *
 * Here `from_int` is the count of fraction-position digits that are really integer digits of the value,
 * which is exp10 when exp10 is positive. Stopping the strip at `from_int`, and at one digit when there is
 * no integer part, reproduces E exactly: the original's "pad one zero when none was emitted" and this
 * function's "keep one digit" put the same byte in the same place. */
static uint8_t trim_fraction(const char *frac, uint8_t kept, uint8_t from_int) {
    while (kept > from_int && kept > 1u && frac[kept - 1u] == '0') {
        kept--;
    }
    return kept;
}

static size_t sci_core(out_t *o, float v, uint8_t decimals, char echar) {
    fdig_t g;
    char frac[FDIG_SCI_MAX];
    uint8_t kept;
    uint8_t from_int;
    int16_t e;
    uint8_t j;

    if (decimals > FDIG_SCI_MAX - 1u) {
        decimals = (uint8_t)(FDIG_SCI_MAX - 1u);
    }

    fdigits(v, (uint8_t)(decimals + 1u), &g);

    if (g.negative && !g.zero) {
        put(o, '-');
    }
    put(o, g.d[0]);

    if (decimals != 0u) {
        for (j = 0u; j < decimals; j++) {
            frac[j] = g.d[j + 1u];
        }
        kept = decimals;
        /* The original's mantissa digits came from a decimal text whose integer part was exp10 + 1 digits
         * long, so that many of them are never trimmed. A value below 1 contributes none. */
        from_int = (g.exp10 > 0) ? (uint8_t)g.exp10 : 0u;
        if (from_int > decimals) {
            from_int = decimals;
        }
        kept = trim_fraction(frac, kept, from_int);
        put(o, '.');
        for (j = 0u; j < kept; j++) {
            put(o, frac[j]);
        }
    }

    put(o, echar);
    e = g.zero ? 0 : g.exp10;
    if (e < 0) {
        put(o, '-');
        e = -e;
    }
    o->len = fmt_int(o->dst + o->len, o->size - o->len, (int32_t)e) + o->len;
    return o->len;
}

size_t fmt_float_e_prec(char *dst, size_t dst_size, float v, uint8_t decimals) {
    out_t o = {dst, dst_size, 0u};

    return sci_core(&o, v, decimals, 'e');
}

size_t fmt_float_E_prec(char *dst, size_t dst_size, float v, uint8_t decimals) {
    out_t o = {dst, dst_size, 0u};

    return sci_core(&o, v, decimals, 'E');
}

/* ---- Scanning ------------------------------------------------------------------------------------------ */

/* The original skipped the same six characters. */
static bool is_space(char c) {
    return c == ' ' || (c >= '\t' && c <= '\r');
}

static bool is_digit(char c) {
    return c >= '0' && c <= '9';
}

fmt_status_t fmt_parse_long(const char *s, const char **end, int32_t *out) {
    const char *p = s;
    bool negative = false;
    uint32_t acc = 0uL;
    uint8_t digits = 0u;

    if (end != NULL) {
        *end = s;
    }
    while (is_space(*p)) {
        p++;
    }
    if (*p == '-' || *p == '+') {
        negative = *p == '-';
        p++;
    }
    while (is_digit(*p)) {
        const uint8_t d = (uint8_t)(*p - '0');

        if (++digits > FMT_NUM_TOKEN_MAX) {
            return FMT_ERR_OVERFLOW;
        }
        /* The original wrapped at 16 bits here, so "99999" became 34463. Report the overflow instead.
         * The negative limit is one larger because INT32_MIN has no positive counterpart. */
        if (acc > 214748364uL || (acc == 214748364uL && d > (negative ? 8u : 7u))) {
            return FMT_ERR_RANGE;
        }
        acc = (uint32_t)(acc * 10uL + d);
        p++;
    }
    if (digits == 0u) {
        return FMT_ERR_SYNTAX;
    }
    if (end != NULL) {
        *end = p;
    }
    if (acc == 0uL) {
        *out = 0;
    } else if (negative) {
        *out = -(int32_t)(acc - 1uL) - 1; /* reaches INT32_MIN without signed overflow */
    } else {
        *out = (int32_t)acc;
    }
    return FMT_OK;
}

/* The accepted grammar is exactly the original's:
 *   [space]* [sign] digit* [ '.' digit* ] [ ('e'|'E') [sign] digit* ]
 * with at least one mantissa digit required. A second sign, a lone '.', a lone sign, hexadecimal, "inf"
 * and "nan" are all rejected -- the original returned 0.0 for each, which its caller then reported as a
 * zero-parameter error, so a syntax error is the same refusal with the right reason. An exponent letter
 * with no digits after it still means exponent 0, which the original accepted and this port keeps. */
fmt_status_t fmt_parse_float(const char *s, const char **end, float *out) {
    const char *p = s;
    bool negative = false;
    uint32_t acc = 0uL;
    uint8_t sig = 0u;        /* significant digits taken into acc */
    int16_t scale = 0;       /* decimal places acc must still be shifted by */
    uint16_t mant_digits = 0u;
    bool seen_digit = false;
    bool seen_nonzero = false;
    int32_t exponent = 0;
    float value;

    if (end != NULL) {
        *end = s;
    }
    while (is_space(*p)) {
        p++;
    }
    if (*p == '-' || *p == '+') {
        negative = *p == '-';
        p++;
    }

    while (is_digit(*p)) {
        seen_digit = true;
        if (++mant_digits > FMT_NUM_TOKEN_MAX) {
            return FMT_ERR_OVERFLOW;
        }
        if (*p != '0') {
            seen_nonzero = true;
        }
        if (seen_nonzero) {
            if (sig < FMT_PARSE_SIG) {
                acc = (uint32_t)(acc * 10uL + (uint32_t)(*p - '0'));
                sig++;
            } else {
                scale++; /* beyond float precision: keep the magnitude, drop the digit */
            }
        }
        p++;
    }

    if (*p == '.') {
        p++;
        while (is_digit(*p)) {
            seen_digit = true;
            if (++mant_digits > FMT_NUM_TOKEN_MAX) {
                return FMT_ERR_OVERFLOW;
            }
            if (*p != '0') {
                seen_nonzero = true;
            }
            if (seen_nonzero && sig < FMT_PARSE_SIG) {
                acc = (uint32_t)(acc * 10uL + (uint32_t)(*p - '0'));
                sig++;
                scale--;
            } else if (!seen_nonzero) {
                scale--; /* a leading zero of the fraction only moves the point */
            }
            p++;
        }
    }

    if (!seen_digit) {
        return FMT_ERR_SYNTAX;
    }

    if (*p == 'e' || *p == 'E') {
        const char *mark = p;
        bool exp_negative = false;
        uint8_t exp_digits = 0u;

        p++;
        if (*p == '-' || *p == '+') {
            exp_negative = *p == '-';
            p++;
        }
        while (is_digit(*p)) {
            if (exp_digits < 5u) {
                exponent = exponent * 10 + (int32_t)(*p - '0');
                exp_digits++;
            }
            p++;
        }
        if (exp_digits == 0u) {
            /* "1e", "1e+" and "1e-" meant exponent 0 and consumed the letter. That is kept. */
            p = mark + 1;
            if (*p == '-' || *p == '+') {
                p++;
            }
        } else if (exp_digits >= 5u) {
            /* The original looped a multiply once per unit of exponent, so "1e60000" ran for a second and
             * returned nonsense. Decide it here instead. */
            if (!exp_negative) {
                return FMT_ERR_RANGE;
            }
            exponent = 0;
            acc = 0uL;
            scale = 0;
        }
        if (exp_negative) {
            exponent = -exponent;
        }
    }

    if (end != NULL) {
        *end = p;
    }

    /* "-0" and "-0.0" gave +0 in the original, because its negate skipped zero. Kept, so a
     * negative zero can never reach a coefficient. */
    if (acc == 0uL) {
        *out = 0.0f;
        return FMT_OK;
    }

    exponent += scale;
    if (exponent > 38) {
        return FMT_ERR_RANGE;
    }
    if (exponent < -60) { /* below the smallest subnormal whatever the nine digits are */
        *out = 0.0f;
        return FMT_OK;
    }

    /* acc holds up to nine digits exactly, so (float)acc is one rounding. The scale is then applied in at
     * most two steps, chosen so that only the last one can land in the subnormal range: acc is at least 1
     * and below 1e9, so value stays normal until the final operation. */
    value = (float)acc;
    if (exponent > 19) {
        value *= fmt_pow10f(19);
        exponent -= 19;
    }
    if (exponent < -30) {
        value /= fmt_pow10f(30);
        exponent += 30;
    }
    if (exponent > 0) {
        value *= fmt_pow10f((int8_t)exponent);
    } else if (exponent < 0) {
        value /= fmt_pow10f((int8_t)-exponent);
    }

    if (value != finite_of(value)) { /* the scaling overflowed to infinity */
        return FMT_ERR_RANGE;
    }
    *out = negative ? -value : value;
    return FMT_OK;
}
