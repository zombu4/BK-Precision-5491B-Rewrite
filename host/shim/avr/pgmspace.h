/* Host shim for <avr/pgmspace.h>. On a host there is no separate program space, so PROGMEM is nothing and
 * the pgm_read_* accessors are plain reads. Only what this port actually uses is defined -- a shim that
 * guesses at the rest would hide a real dependency. */
#ifndef SHIM_PGMSPACE_H
#define SHIM_PGMSPACE_H
#include <stdint.h>
#include <string.h>
#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(p) (*(const uint8_t *)(p))
#define pgm_read_word(p) (*(const uint16_t *)(p))
#define pgm_read_dword(p) (*(const uint32_t *)(p))
#define pgm_read_float(p) (*(const float *)(p))
#define pgm_read_ptr(p) (*(void *const *)(p))
#define memcpy_P memcpy
#define strlen_P strlen
#define strcpy_P strcpy
#define strncpy_P strncpy
#define strcmp_P strcmp
#define snprintf_P snprintf
#endif
