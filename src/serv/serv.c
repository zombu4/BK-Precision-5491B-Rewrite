/* serv.c -- the service lock (see serv.h).
 *
 * WHAT THIS IS, AND WHAT IT IS NOT
 * The serial link is plain text. Anyone who can put bytes on the cable can also read every byte of an unlock,
 * and anyone with a programmer on the ISP header can read or rewrite the EEPROM, including this block. The
 * lock therefore protects against a stray, mistaken or badly written command: a script that was pointed at
 * the wrong meter, a typo, a client that sends :SCAL:COEF when it meant to read. It is NOT a security
 * boundary and must not be described as one.
 *
 * THE CODE: FOUR DIGITS, DEFAULT 6666
 * One code of exactly SERV_CODE_DIGITS digits serves the front panel and the remote link alike. While no code
 * has been stored the default SERV_CODE_DEFAULT applies, so a fresh meter works out of the box like the
 * vendor's. 6666 IS THE VENDOR'S PUBLICLY KNOWN CODE: a meter left on the default is protected against an
 * accident and against nothing else. A four digit code has only 10000 values in any case, so even a changed
 * code is only a speed bump for someone who can read the EEPROM; see the hash notes below.
 *
 * NOTHING SECRET IN THE SOURCE
 * The default is the vendor's public code, not a secret, and no other code, backdoor or recovery command
 * exists. A changed code belongs to one meter and is chosen by its user; only a salt and a hash of it are
 * stored, in the EEPROM block at EE_SERV_BASE (eeprom_map.h). The default is never stored anywhere: with no
 * valid block the candidate is compared with the constant directly.
 * A FORGOTTEN CODE IS RECOVERED BY AN ISP WRITE OF THAT BLOCK (0x0A40), which returns the meter to 6666.
 * Nothing in the firmware implements recovery and nothing must be added: a "recovery command" over the
 * plain text link would be a backdoor.
 *
 * THE HASH, AND ITS LIMITS
 * An ATmega128A has 4 KB of RAM and 12 MHz of 8-bit arithmetic. A memory-hard password hash does not fit and
 * a slow one would stall the instrument, so this is a compromise and is stated as one:
 *   - the primitive is SipHash-2-4 (a short-input keyed function, small and easy to get right), keyed with
 *     the 16 byte per-meter salt;
 *   - the password is run through SERV_HASH_ROUNDS chained calls, then four more calls with a counter fill
 *     the 32 byte field;
 *   - the four output words are derived from the same inputs. The 32 bytes are the width of the field, not
 *     256 bits of strength: the strength of one 64 bit output is what exists.
 * What that buys: two meters set to the same password hold different hashes, and the stored bytes are not
 * the password. What it does NOT buy: resistance to anyone who holds the EEPROM contents. A short password
 * falls to an offline search on a PC in moments, because SipHash is fast and SERV_HASH_ROUNDS multiplies the
 * cost by a few hundred, not by millions. The real protection against guessing over the link is the
 * attempt limit below, and that protects only against a client that tries a few passwords, not against an
 * attacker who reads the EEPROM.
 *
 * ATTEMPT LIMIT
 * The count of wrong attempts lives in EEPROM (EE_SERV_FAILS), so removing power does not reset it. It is
 * ONE counter for the panel and the link, and it counts whether or not a code is stored (an erased byte,
 * 0xFF, means zero). After SERV_MAX_FAILS wrong attempts an unlock is refused for SERV_LOCKOUT_MS, and every
 * further wrong attempt after that starts the wait again. ONLY A CORRECT UNLOCK (or choosing a new code while
 * unlocked, which needs one) CLEARS THE COUNTER. A power cycle while the counter is at its limit starts the
 * wait over; it does not skip it.
 * The wait is a countdown in serv_tick_1ms(). Nothing busy-waits for it (the original stalled a
 * second on every error with interrupts off and lost the host's input).
 *
 * SESSION
 * An unlock opens a session of SERV_SESSION_TIMEOUT_MS. serv_check() must not extend it (serv.h: no side
 * effects); the executor extends it with serv_note_activity() after a protected command succeeds. The
 * session also ends on serv_lock() and on any reset, and -- only when it was opened over the link -- on a
 * front-panel key press. See serv.h for why that condition is there and what it cost to leave out.
 *
 * UNVERIFIED: SipHash-2-4 below is written from the published definition and has not been run. Check it
 * against the reference vector: key 00 01 ... 0F and an empty message give 0x726FDB47DD0E0E31.
 * UNVERIFIED: the time of serv_unlock() with SERV_HASH_ROUNDS = 256. The estimate is about 0.25 s at
 * 12 MHz (a 64 bit SipHash call of about 11 000 cycles, 261 calls). Measure it; the rounds constant is the
 * only thing to change.
 * UNVERIFIED: whether serv_tick_1ms() is called from an interrupt or from the main loop. The shared counters
 * are accessed atomically, so either is safe; the caller's choice does not matter to this file.
 */
#include <avr/wdt.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <util/atomic.h>

#include "../eeprom_map.h"
#include "../hal/eeprom.h"
#include "serv.h"

#define SERV_BLOCK_VERSION 1u
#define SERV_SALT_BYTES 16u
#define SERV_HASH_BYTES 32u
#define SERV_HASH_ROUNDS 256u
#define SERV_FAILS_ERASED 0xFFu /* an erased EEPROM byte: no attempt has been counted */
#define SERV_FAILS_LIMIT 254u

#define SERIAL_BYTES 20u /* the meter serial at EE_ID_SERIAL, used only to make salts differ between meters */

/* The chained value, one counter byte and the code: the message of one hash call. */
#define CHAIN_BYTES 8u
#define MESSAGE_MAX (CHAIN_BYTES + 1u + SERV_CODE_DIGITS)
#define COUNTER_ROUND 0xFFu /* the counter byte of the chained rounds; the output words use 0..3 */

/* ---- State --------------------------------------------------------------------------------------------- */

static volatile bool unlocked;
static volatile serv_source_t session_from; /* only meaningful while `unlocked` */
static volatile uint32_t session_left_ms;
static volatile uint32_t lockout_left_ms;
static volatile uint32_t uptime_ms; /* only feeds the salt; wraps harmlessly */
static bool password_present;       /* a code has been stored; cached, serv_check() must not touch the EEPROM */

/* ---- SipHash-2-4 --------------------------------------------------------------------------------------- */

static uint64_t rotate_left(uint64_t x, uint8_t bits) {
    return (x << bits) | (x >> (64u - bits));
}

static void sip_round(uint64_t v[4]) {
    v[0] += v[1];
    v[1] = rotate_left(v[1], 13u);
    v[1] ^= v[0];
    v[0] = rotate_left(v[0], 32u);
    v[2] += v[3];
    v[3] = rotate_left(v[3], 16u);
    v[3] ^= v[2];
    v[0] += v[3];
    v[3] = rotate_left(v[3], 21u);
    v[3] ^= v[0];
    v[2] += v[1];
    v[1] = rotate_left(v[1], 17u);
    v[1] ^= v[2];
    v[2] = rotate_left(v[2], 32u);
}

/* Up to eight bytes as a little-endian number. */
static uint64_t load_le(const uint8_t *bytes, size_t count) {
    uint64_t value = 0u;

    for (size_t i = count; i > 0u; i--) {
        value = (value << 8) | bytes[i - 1u];
    }
    return value;
}

static void store_le(uint8_t *bytes, uint64_t value) {
    for (uint8_t i = 0u; i < 8u; i++) {
        bytes[i] = (uint8_t)(value >> (8u * i));
    }
}

static uint64_t siphash24(const uint8_t key[SERV_SALT_BYTES], const uint8_t *message, size_t length) {
    const uint64_t k0 = load_le(key, 8u);
    const uint64_t k1 = load_le(key + 8u, 8u);
    uint64_t v[4] = {k0 ^ 0x736F6D6570736575ULL, k1 ^ 0x646F72616E646F6DULL, k0 ^ 0x6C7967656E657261ULL,
                     k1 ^ 0x7465646279746573ULL};
    const size_t whole = length & ~(size_t)7u;
    uint64_t last;

    for (size_t i = 0u; i < whole; i += 8u) {
        const uint64_t m = load_le(message + i, 8u);

        v[3] ^= m;
        sip_round(v);
        sip_round(v);
        v[0] ^= m;
    }
    last = ((uint64_t)(length & 0xFFu) << 56) | load_le(message + whole, length - whole);
    v[3] ^= last;
    sip_round(v);
    sip_round(v);
    v[0] ^= last;
    v[2] ^= 0xFFu;
    for (uint8_t i = 0u; i < 4u; i++) {
        sip_round(v);
    }
    return v[0] ^ v[1] ^ v[2] ^ v[3];
}

/* ---- Password hash and salt ---------------------------------------------------------------------------- */

static void derive_hash(const uint8_t salt[SERV_SALT_BYTES], const char *password, size_t length,
                        uint8_t hash[SERV_HASH_BYTES]) {
    uint8_t message[MESSAGE_MAX];
    uint64_t chain = 0u;

    memcpy(message + CHAIN_BYTES + 1u, password, length);
    message[CHAIN_BYTES] = COUNTER_ROUND;
    for (uint16_t iteration = 0u; iteration < SERV_HASH_ROUNDS; iteration++) {
        store_le(message, chain);
        chain = siphash24(salt, message, CHAIN_BYTES + 1u + length);
        wdt_reset(); /* bounded work: at most SERV_HASH_ROUNDS calls, kept short of any watchdog timeout */
    }
    store_le(message, chain);
    for (uint8_t word = 0u; word < SERV_HASH_BYTES / 8u; word++) {
        message[CHAIN_BYTES] = word;
        store_le(hash + 8u * word, siphash24(salt, message, CHAIN_BYTES + 1u + length));
    }
}

/* A fresh salt. A salt has to differ between meters and between password changes; it does not have to be
 * secret or unpredictable, and nothing here claims it is. It is mixed from the meter serial, the time since
 * power-up and the previous salt (erased bytes if there was none). */
static void make_salt(uint8_t salt[SERV_SALT_BYTES]) {
    uint8_t previous[SERV_SALT_BYTES];
    uint8_t message[SERIAL_BYTES + 4u + 1u];
    uint32_t now;

    ee_read_block(EE_SERV_SALT, previous, SERV_SALT_BYTES);
    ee_read_block(EE_ID_SERIAL, message, SERIAL_BYTES);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        now = uptime_ms;
    }
    for (uint8_t i = 0u; i < 4u; i++) {
        message[SERIAL_BYTES + i] = (uint8_t)(now >> (8u * i));
    }
    for (uint8_t half = 0u; half < SERV_SALT_BYTES / 8u; half++) {
        message[SERIAL_BYTES + 4u] = half;
        store_le(salt + 8u * half, siphash24(previous, message, sizeof message));
    }
}

/* Equal contents, in time that does not depend on where they first differ. */
static bool same_bytes(const uint8_t *a, const uint8_t *b, size_t count) {
    uint8_t difference = 0u;

    for (size_t i = 0u; i < count; i++) {
        difference |= (uint8_t)(a[i] ^ b[i]);
    }
    return difference == 0u;
}

/* ---- The block in EEPROM ------------------------------------------------------------------------------- */

static bool block_is_valid(void) {
    return ee_read_u16(EE_SERV_MAGIC) == EE_SERV_MAGIC_VALUE &&
           ee_read_u8(EE_SERV_VERSION) == SERV_BLOCK_VERSION;
}

/* The block is written magic last, so a power loss part-way leaves it invalid, never half valid. An invalid
 * block means no code is stored, so the default applies: a failed change falls back to the code the meter
 * shipped with, which the user can enter. The magic is cleared first for the same reason when an existing
 * code is replaced. */
static bool block_store(const uint8_t salt[SERV_SALT_BYTES], const uint8_t hash[SERV_HASH_BYTES]) {
    return ee_write_u16(EE_SERV_MAGIC, 0xFFFFu) && ee_write_u8(EE_SERV_VERSION, SERV_BLOCK_VERSION) &&
           ee_write_block(EE_SERV_SALT, salt, SERV_SALT_BYTES) &&
           ee_write_block(EE_SERV_HASH, hash, SERV_HASH_BYTES) && ee_write_u8(EE_SERV_FAILS, 0u) &&
           ee_write_u16(EE_SERV_MAGIC, EE_SERV_MAGIC_VALUE);
}

/* ---- Interface ----------------------------------------------------------------------------------------- */

/* The wrong-attempt count; an erased byte is zero. */
static uint8_t read_fails(void) {
    const uint8_t fails = ee_read_u8(EE_SERV_FAILS);

    return fails == SERV_FAILS_ERASED ? 0u : fails;
}

/* Exactly SERV_CODE_DIGITS characters, each 0-9, then the terminator. Reads no further than the first byte
 * that is not a digit, so an unterminated buffer is never overrun. */
static bool code_is_well_formed(const char *code) {
    for (size_t i = 0u; i < SERV_CODE_DIGITS; i++) {
        if (code[i] < '0' || code[i] > '9') {
            return false;
        }
    }
    return code[SERV_CODE_DIGITS] == '\0';
}

void serv_init(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        unlocked = false;
        session_left_ms = 0u;
        lockout_left_ms = 0u;
    }
    password_present = block_is_valid();
    if (read_fails() >= SERV_MAX_FAILS) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            lockout_left_ms = SERV_LOCKOUT_MS;
        }
    }
}

bool serv_password_is_set(void) {
    return password_present;
}

/* Never SERV_DENIED_NO_PASSWORD: with no stored code the default applies, so a protected command is refused
 * only because the session is locked. */
serv_result_t serv_check(void) {
    return unlocked ? SERV_ALLOWED : SERV_DENIED_LOCKED;
}

static bool lockout_active(void) {
    uint32_t left;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        left = lockout_left_ms;
    }
    return left != 0u;
}

static void record_wrong_attempt(void) {
    uint8_t fails = read_fails();

    if (fails < SERV_FAILS_LIMIT) {
        fails++;
    }
    (void)ee_write_u8(EE_SERV_FAILS, fails);
    if (fails >= SERV_MAX_FAILS) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            lockout_left_ms = SERV_LOCKOUT_MS;
        }
    }
}

/* Accepts the stored hash when a code has been stored, otherwise the default. A malformed code is a wrong
 * attempt like any other. The default is compared in place, never copied into the EEPROM or hashed. */
bool serv_unlock(const char *code, serv_source_t from) {
    uint8_t salt[SERV_SALT_BYTES];
    uint8_t stored[SERV_HASH_BYTES];
    uint8_t candidate[SERV_HASH_BYTES];
    bool match = false;

    if (code == NULL || lockout_active()) {
        return false; /* during the wait an attempt is neither examined nor counted */
    }
    if (code_is_well_formed(code)) {
        if (password_present) {
            ee_read_block(EE_SERV_SALT, salt, SERV_SALT_BYTES);
            ee_read_block(EE_SERV_HASH, stored, SERV_HASH_BYTES);
            derive_hash(salt, code, SERV_CODE_DIGITS, candidate);
            match = same_bytes(stored, candidate, SERV_HASH_BYTES);
        } else {
            match = same_bytes((const uint8_t *)code, (const uint8_t *)SERV_CODE_DEFAULT, SERV_CODE_DIGITS);
        }
    }
    if (!match) {
        record_wrong_attempt();
        return false;
    }
    if (read_fails() != 0u) {
        (void)ee_write_u8(EE_SERV_FAILS, 0u);
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        session_left_ms = SERV_SESSION_TIMEOUT_MS;
        unlocked = true;
        session_from = from;
    }
    return true;
}

void serv_lock(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        unlocked = false;
        session_left_ms = 0u;
    }
}

bool serv_is_unlocked(void) {
    return unlocked;
}

/* Restarts the session clock. It resolves the tension between the ten-minute session rule ("ten
 * minutes with no protected command") and this module's rule that serv_check() has no
 * side effects. Without it the ten minutes ran from the unlock, so a long calibration had to unlock again
 * part-way through -- and a lock that obstructs the work it protects is a lock people switch off.
 *
 * The executor calls this after a protected command SUCCEEDS. Deliberately not on a mere attempt: a client
 * that kept failing at one could otherwise hold a session open indefinitely. It does nothing when no
 * session is open, so it can never create one. */
void serv_note_activity(void) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (unlocked) {
            session_left_ms = SERV_SESSION_TIMEOUT_MS;
        }
    }
}

/* Allowed only while unlocked. Since the default always unlocks, the code can always be changed. The new code
 * must be exactly SERV_CODE_DIGITS digits and is refused, not adjusted, otherwise. A failed write leaves no
 * code stored, so the default applies again, and ends the session (see block_store). */
bool serv_set_password(const char *code) {
    uint8_t salt[SERV_SALT_BYTES];
    uint8_t hash[SERV_HASH_BYTES];

    if (code == NULL || !unlocked || !code_is_well_formed(code)) {
        return false;
    }
    make_salt(salt);
    derive_hash(salt, code, SERV_CODE_DIGITS, hash);
    password_present = false;
    if (!block_store(salt, hash)) {
        serv_lock();
        return false;
    }
    password_present = true;
    return true;
}

void serv_tick_1ms(void) {
    uptime_ms++;
    if (session_left_ms != 0u && --session_left_ms == 0u) {
        unlocked = false;
    }
    if (lockout_left_ms != 0u) {
        lockout_left_ms--;
    }
}

void serv_note_front_panel_key(void) {
    /* Only a session opened over the link. A panel session is being used BY the person pressing the key. */
    if (unlocked && session_from == SERV_FROM_REMOTE) {
        serv_lock();
    }
}
