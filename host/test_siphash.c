/* test_siphash.c -- checks the service lock's hash against the published SipHash-2-4 vectors.
 *
 * This comes first in the test pass. The implementation in `serv.c` was written from
 * the definition. A wrong hash would not fail loudly: it would store something
 * that the same wrong code verifies consistently, so the lock would appear to work perfectly -- until the
 * day the implementation is corrected, at which point the user is locked out of their own meter by their
 * own correct code. That is the failure mode this file exists to prevent.
 *
 * The reference is the SipHash paper's own test data: key 00 01 02 ... 0F with messages 00, 00 01, and so
 * on. The empty-message answer, 0x726FDB47DD0E0E31, is the one quoted in `serv.c`'s comment.
 *
 * Built as one translation unit: it includes just enough of serv.c to reach `siphash24`, with the EEPROM
 * and the rest stubbed, so the function under test is the shipped one and not a copy.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- Stubs, so serv.c compiles on the host -------------------------------------------------------- */
static uint8_t fake_ee[4096];

uint8_t ee_read_u8(uint16_t a) { return a < sizeof fake_ee ? fake_ee[a] : 0xFFu; }
uint16_t ee_read_u16(uint16_t a) { return (uint16_t)(ee_read_u8(a) | ((uint16_t)ee_read_u8((uint16_t)(a + 1u)) << 8)); }
float ee_read_f32(uint16_t a) { (void)a; return 0.0f; }
void ee_read_block(uint16_t a, void *d, uint16_t n) { for (uint16_t i = 0; i < n; i++) ((uint8_t *)d)[i] = ee_read_u8((uint16_t)(a + i)); }
bool ee_write_u8(uint16_t a, uint8_t v) { if (a >= sizeof fake_ee) return false; fake_ee[a] = v; return true; }
bool ee_write_u16(uint16_t a, uint16_t v) { return ee_write_u8(a, (uint8_t)v) && ee_write_u8((uint16_t)(a + 1u), (uint8_t)(v >> 8)); }
bool ee_write_f32(uint16_t a, float v) { (void)a; (void)v; return true; }
bool ee_write_block(uint16_t a, const void *s, uint16_t n) { for (uint16_t i = 0; i < n; i++) if (!ee_write_u8((uint16_t)(a + i), ((const uint8_t *)s)[i])) return false; return true; }

#include "../src/state.h"
instrument_t g_state;

/* The code under test. */
#include "../src/serv/serv.c"

/* ---- The published vectors --------------------------------------------------------------------------
 * From the SipHash reference implementation's `vectors.h`, for SipHash-2-4 with the 64-bit output. The
 * first entry is the empty message. Only the first eight are needed to be convincing: a byte-order error,
 * a wrong round count or a wrong finalisation constant all show up within the first two.
 */
static const uint64_t expected[8] = {
    0x726FDB47DD0E0E31ULL, 0x74F839C593DC67FDULL, 0x0D6C8009D9A94F5AULL, 0x85676696D7FB7E2DULL,
    0xCF2794E0277187B7ULL, 0x18765564CD99A68DULL, 0xCBC9466E58FEE3CEULL, 0xAB0200F58B01D137ULL,
};

int main(void) {
    uint8_t key[16];
    uint8_t message[8];
    int failures = 0;

    for (uint8_t i = 0u; i < 16u; i++) {
        key[i] = i;
    }

    printf("SipHash-2-4 against the published vectors\n\n");
    for (uint8_t n = 0u; n < 8u; n++) {
        uint64_t got;

        for (uint8_t i = 0u; i < n; i++) {
            message[i] = i;
        }
        got = siphash24(key, message, n);
        printf("  message of %u byte(s): got 0x%016llX want 0x%016llX  %s\n", (unsigned)n,
               (unsigned long long)got, (unsigned long long)expected[n],
               got == expected[n] ? "ok" : "FAIL");
        if (got != expected[n]) {
            failures++;
        }
    }

    /* A property the lock depends on and that a vector test does not cover: the same code and salt must
     * give the same answer twice. If the derivation ever read uninitialised memory, every unlock after the
     * first would fail and it would look like the user mistyping. */
    {
        uint8_t salt[SERV_SALT_BYTES];
        uint8_t a[SERV_HASH_BYTES];
        uint8_t b[SERV_HASH_BYTES];

        for (uint8_t i = 0u; i < SERV_SALT_BYTES; i++) {
            salt[i] = (uint8_t)(0xA0u + i);
        }
        derive_hash(salt, "6666", 4u, a);
        derive_hash(salt, "6666", 4u, b);
        printf("\n  derive_hash is deterministic: %s\n", memcmp(a, b, sizeof a) == 0 ? "ok" : "FAIL");
        if (memcmp(a, b, sizeof a) != 0) {
            failures++;
        }

        derive_hash(salt, "6667", 4u, b);
        printf("  a different code gives a different hash: %s\n",
               memcmp(a, b, sizeof a) != 0 ? "ok" : "FAIL");
        if (memcmp(a, b, sizeof a) == 0) {
            failures++;
        }
    }

    printf("\n%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
