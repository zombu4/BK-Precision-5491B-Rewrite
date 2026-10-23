/* fmt.h -- number formatting and scanning. PROTOCOL-VISIBLE. FROZEN.
 *
 * Why this module exists: the original runtime cannot simply be replaced by avr-libc. The
 * original parser used ICC atof (0x13DC0) and sscanf (0x14C86); avr-libc strtod behaves differently
 * (1e39 becomes inf, not 1.2e-38). So the accept/reject set is reproduced here by hand.
 *
 * OUTPUT: field layout, width, exponent style and separators are reproduced BYTE FOR BYTE so that
 * both BK-Calibrator and the vendor tools keep parsing. Only defects that produce a wrong number or no
 * output at all are fixed: %e of 0.0 printing nothing, zero-padding before the minus sign, and the
 * 24-bit fraction error (1e-5 printing as 9.9e-6).
 *
 * INPUT: the accepted grammar is kept. Two deliberate differences: an out-of-range exponent is
 * a NORMAL ERROR instead of silently yielding a wrong value (the original stored 1e39 as 1.2e-38, which
 * can corrupt a :SCAL:COEF value), and text that is not a number at all is a syntax error instead of the
 * value 0.0 that the caller then reported as a zero parameter.
 *
 * A numeric parameter of ZERO IS ACCEPTED wherever the
 * parameter's range allows it, because every pre-existing defect is fixed. So
 * FMT_ERR_ZERO is never returned. The enumerator stays so that existing call sites keep compiling; a
 * caller that must refuse zero checks the value itself.
 */
#ifndef BK_FMT_H
#define BK_FMT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    FMT_OK = 0,
    FMT_ERR_SYNTAX,   /* not a number at all */
    FMT_ERR_RANGE,    /* exponent out of range -- was silently wrong */
    FMT_ERR_ZERO,     /* never returned; see the amendment note above */
    FMT_ERR_OVERFLOW  /* token longer than the buffer -- the overflow fix */
} fmt_status_t;

/* The original overflowed a 20-byte local on tokens of 20+ characters. */
#define FMT_NUM_TOKEN_MAX 32u

/* Byte-exact reply formatting. Returns the length written; never overruns dst_size. */
size_t fmt_int(char *dst, size_t dst_size, int32_t v);

/* Scanning, with the input rules described above. */
fmt_status_t fmt_parse_float(const char *s, const char **end, float *out);
fmt_status_t fmt_parse_long(const char *s, const char **end, int32_t *out);

/* There is no fmt_eng. The original's engineering-notation pair fmt_float_eng 0x03968 and fmt_parse_eng
 * 0x03AAA is only called by the front-panel Math number editors, and ui/menu.c ports both from the
 * disassembly as eng_format and eng_parse, including the '^' prefix character the original stored for
 * "no prefix". The display path formats through fmt_fixed. */

/* The original uses %e (6 decimals) for the CALC/UNIT/FREQ:REF/HOLD replies but %.8e for
 * FETC? and REF?; the other replies use 6. */
size_t fmt_float_e_prec(char *dst, size_t dst_size, float v, uint8_t decimals);

/* Fixed-point formatting ("%#7.nf"), with the digit logic in one place.
 *
 * The decimal point is always written and the fraction is always `decimals` long, which is what the
 * original's '#' flag did for its two panel formats. The field is right justified in `width` with spaces,
 * and content longer than `width` is not cut. Pass width 0 for no padding. */
size_t fmt_fixed(char *dst, size_t dst_size, float v, uint8_t width, uint8_t decimals);

/* The original's "%+#7.nf" numeric-editor formats are drawn by ui/menu.c (eng_format), and its "%g" had no
 * user in the port; neither has a formatter here. */

/* The original had "%.8E" and "%E,%d" format strings as well as the lower-case forms,
 * and only the exponent letter differs. */
size_t fmt_float_E_prec(char *dst, size_t dst_size, float v, uint8_t decimals);

/* An exact power of ten, for -38 to 38; values outside that are clamped.
 *
 * This replaces the library pow() that the panel digit scaling called (0x146D8 from 0x01DBA and 0x0541A).
 * That pow was built from exp(y * log(x)) and was accurate to about five digits, so pow(10, 2) returned
 * 99.9999 and pow(10, -2) returned 0.0100000082. Every caller that wants a power of ten uses this instead;
 * nothing in the port calls pow(). */
float fmt_pow10f(int8_t n);

#endif /* BK_FMT_H */
