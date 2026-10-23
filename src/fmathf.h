/* fmathf.h -- the float math functions this firmware uses, actually typed as float.
 *
 * WHY THIS FILE EXISTS. avr-libc does not provide float versions of the math functions. It provides
 * MACROS that alias them to the double ones:
 *
 *     avr-libc 2.0.0 math.h:146  extern double fabs(double __x) __ATTR_CONST__;
 *     avr-libc 2.0.0 math.h:147  #define fabsf fabs
 *     avr-libc 2.0.0 math.h:312  #define log10f log10
 *
 * So `fabsf(x) > 0.5f`, which reads as float arithmetic, is a double call whose result is compared against
 * a promoted float -- and -Wdouble-promotion is right to say so. There were 21 such lines in this port.
 *
 * On THIS toolchain it makes no difference to the emitted code: avr-gcc 7.3 has no -mdouble=64, so
 * `double` is 32 bits and is the same representation as `float`. The promotion is a no-op.
 *
 * It is still worth fixing properly rather than silencing, for two reasons. The code should mean what it
 * says -- a reader who sees `fabsf` is entitled to conclude the arithmetic is single precision. And the
 * day this is built with a toolchain where `double` is 64 bits (avr-gcc 10 and later, with -mdouble=64),
 * every one of those lines becomes genuine double arithmetic: slower, larger, and in the calibration maths
 * a different answer from the one the coefficients were computed against. -Wdouble-promotion is the thing
 * that would catch that, so it stays on, and these wrappers make it mean something.
 *
 * Include this INSTEAD OF <math.h>. It includes <math.h> itself, then replaces the aliases with real
 * float-typed inline functions. Nothing in src/ includes <math.h> directly, so no file can miss this.
 *
 * Only on an avr-libc that actually has the problem. Two other C libraries compile these same sources and
 * both declare the float functions correctly, so redefining them there is a hard error:
 *
 *   - glibc, for the host tests under host/.
 *   - avr-libc 2.2.0 and later, which replaced the macro aliases with real declarations:
 *     `static inline float fabsf(float)` returning `__builtin_fabsf` (math.h:186) and
 *     `extern float log10f(float)` (math.h:427) in the 14.1.0 toolchain's copy.
 *
 * The test is therefore the avr-libc VERSION and not `__AVR__`. Measured in this workspace:
 * avr-gcc 7.3.0 ships avr-libc 2.0.0 (`__AVR_LIBC_VERSION__ == 20000UL`), which aliases; avr-gcc 14.1.0
 * ships 2.2.0 (`20200UL`), which does not. The boundary is written as `< 20200UL` because 2.2.0 is the
 * first version measured to be correct, not because 2.1.x was checked -- there was no 2.1 release to check.
 *
 * The casts inside are explicit and deliberate: with a 32-bit double they are identity, and with a 64-bit
 * double they say exactly where the single-precision rounding happens, which is what single-precision
 * arithmetic means. */
#ifndef FMATHF_H
#define FMATHF_H

#include <math.h>

/* Only the ones this firmware actually calls. Adding an unused wrapper would be flash spent on nothing --
 * though in practice each of these inlines away to the same instructions as the double call.
 *
 * -Wshadow is switched off for exactly these five declarations, and nowhere else. It fires because gcc
 * knows `fabsf` and the rest as built-ins with the correct float signatures, and it is right that a
 * definition here hides them -- that is the entire purpose of this file. gcc's built-in declaration is not
 * what the code was getting; avr-libc's `#define fabsf fabs` got there first and replaced the name before
 * the compiler ever saw it. The definitions below are what the built-in would have been. */
#ifdef __AVR__
#include <avr/version.h>
#endif

#if defined(__AVR__) && defined(__AVR_LIBC_VERSION__) && __AVR_LIBC_VERSION__ < 20200UL

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"

#undef fabsf
#undef floorf
#undef log10f
#undef powf
#undef truncf

static inline float fabsf(float x) {
    return (float)fabs((double)x);
}

static inline float floorf(float x) {
    return (float)floor((double)x);
}

static inline float log10f(float x) {
    return (float)log10((double)x);
}

static inline float powf(float x, float y) {
    return (float)pow((double)x, (double)y);
}

static inline float truncf(float x) {
    return (float)trunc((double)x);
}

#pragma GCC diagnostic pop

#endif /* avr-libc older than 2.2.0 */

#endif /* FMATHF_H */
