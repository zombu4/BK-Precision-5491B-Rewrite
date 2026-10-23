/* thid.c -- the :THID subsystem and the * common commands.
 *
 * Original: 0x04DBC (dispatcher), 0x04816 (PRODuct), 0x04934 (CALibrate), 0x04B22 (SERVice), 0x04D14 (DEBug)
 * and 0x089BE (the * commands).
 *
 * :THID is the identity and history block at EEPROM 0x0A00-0x0A3F (eeprom_map.h). Every query is open. Every
 * write asks the service lock first (serv.h) and, if it refuses, reports the lock error
 * and changes nothing. That lock stops stray or mistaken commands on a plain-text link; it is not security.
 *
 * Like the original, the dispatcher consumes the whole header path itself (":THID:CALibrate:DATE <date>"),
 * so the executor calls it once, right after the root keyword has been read.
 *
 * Differences from the original, all listed in ORIGINAL_BUGS.md and PROTOCOL_CHANGES.md:
 *   #13  text fields are zero padded; an over-long serial is an error before anything is written
 *        (the original copied a fixed block and left stale bytes behind the text);
 *   #61  a count is replied in full (the original sent only its first two characters);
 *   #62  INCR at 65535 and DECR at 0 are data errors and change nothing (the original wrapped);
 *   #63  every field is validated BEFORE it is written (the original wrote after reporting an error), and a
 *        production date after the write-once lock is an error instead of being ignored in silence;
 *   the unbounded copy of the token into the global scratch buffer at 0x0F29 does not exist here.
 *
 * *FAC, *PRO and *WSN rewrite the model name and the serial number. They are on the transport denylist and
 * are REFUSED, not protected: no password unlocks them. The recovery path for those fields is an ISP write.
 */
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../eeprom_map.h"
#include "../main.h"
#include "../hal/eeprom.h"
#include "../remote/scpi.h"
#include "serv.h"

#include "../version.h"

#define KW(text) scpi_keyword_match(PSTR(text))

#define DATE_LENGTH 8u
#define MEMBER_LENGTH 2u
#define SERIAL_LENGTH 16u

#define IDN_MODEL_BYTES 28u  /* EE_ID_MODEL */
#define IDN_SERIAL_BYTES 20u /* EE_ID_SERIAL */

/* The firmware version that *IDN? reports: "Ver2.0.<yy>.<mm>.<dd>", distinct from the vendor's
 * "Ver1.4.14.06.18". It is a constant of the firmware, not read from EEPROM 0x0830 (ORIGINAL_BUGS #12), so
 * an edit of that text has no effect. The release stamp is set by the build (-DFW_VERSION=...).
 * UNVERIFIED: the date below is the date this file was written, not a release date. No header declares the
 * version for REVISION in the menu either; the release process must set it and decide where it lives. */

/* FW_VERSION comes from ../version.h, so the *IDN? reply and the SYS REVISION page cannot
 * drift apart and a release can stamp it from the build. */
static const char firmware_version[] PROGMEM = FW_VERSION;

/* ---- Shared pieces ------------------------------------------------------------------------------------- */

static bool is_digit(char c) {
    return c >= '0' && c <= '9';
}

/* A write needs the lock open. An error already reported on this line (an unterminated string, an over-long
 * token) ends it before anything is written. */
static bool write_permitted(void) {
    if (scpi_line_failed()) {
        return false;
    }
    if (serv_check() != SERV_ALLOWED) {
        scpi_fail_locked();
        return false;
    }
    return true;
}

/* Replies the stored text of a field: up to `length` bytes, ended at the first zero byte as the original's
 * reply routine (0x1E48) does. */
static void reply_field(uint16_t address, uint8_t length) {
    char text[SERIAL_LENGTH + 1u];

    ee_read_block(address, text, length);
    text[length] = '\0';
    remote_reply(text);
}

static void fail_write(void) {
    scpi_fail_cannot_execute();
}

/* A date is exactly eight characters, each a digit or a slash (loop at 0x04986-0x049AE). */
static bool date_text_valid(const char *text) {
    if (strlen(text) != DATE_LENGTH) {
        return false;
    }
    for (uint8_t i = 0u; i < DATE_LENGTH; i++) {
        if (!is_digit(text[i]) && text[i] != '/') {
            return false;
        }
    }
    return true;
}

/* DATE <text> | DATE?. The production date is write-once: the original guards it with the signature word at
 * 0x0A00 (read at 0x048D2, compared with 0x5554 at 0x048D8-0x048DE) and, only after writing the date, stamps
 * that word (0x048F4). The date field is never examined. So the lock is closed by the SIGNATURE, whatever
 * the date field holds */
static void date_command(uint16_t address, bool write_once) {
    scpi_next_token();
    if (scpi_token_is_query()) {
        reply_field(address, DATE_LENGTH);
        return;
    }
    if (scpi_token_type() != SCPI_TOK_NUMBER) {
        scpi_fail_syntax();
        return;
    }
    if (!write_permitted()) {
        return;
    }
    if (!date_text_valid(scpi_token())) {
        scpi_fail_bad_command();
        return;
    }
    if (write_once && ee_read_u16(EE_THID_PRODDATE_LOCK) == EE_SIG_VALID) {
        fail_write();
        return;
    }
    /* Data first, then the signature (the order the EEPROM layer uses everywhere), so an interrupted write cannot close the window on a
     * date that was never stored. */
    if (!ee_write_block(address, scpi_token(), DATE_LENGTH) ||
        (write_once && !ee_write_u16(EE_THID_PRODDATE_LOCK, EE_SIG_VALID))) {
        fail_write();
    }
}

/* COUNT INCRement -- a u16 that never wraps (ORIGINAL_BUGS #62).
 *
 * THERE IS NO DECREMENT. The original has one and this firmware refuses it . Counter decrements are among the operations that must never be
 * possible, next to memory wipes and identity rewrites, and for the same reason: the calibration and
 * service counts are the only record of how often this instrument has been adjusted, and nothing else on
 * the meter can contradict a count that has been walked backwards. A meter whose count says 3 when it has
 * been calibrated eleven times is not a meter whose history can be audited.
 *
 * It is refused rather than dropped from the keyword table, so a client sending it is told what happened
 * instead of getting a syntax error that reads like a typo. Recorded in PROTOCOL_CHANGES.md. */
static void count_increment(uint16_t address) {
    uint16_t count;

    if (!write_permitted()) {
        return;
    }
    count = ee_read_u16(address);
    if (count == UINT16_MAX) {
        scpi_fail_data();
        return;
    }
    if (!ee_write_u16(address, (uint16_t)(count + 1u))) {
        fail_write();
    }
}

static void count_command(uint16_t address) {
    scpi_next_token();
    if (KW("increment") || KW("incr")) {
        count_increment(address);
    } else if (KW("decrease") || KW("decr")) {
        /* Recognised and refused. Nothing is read and nothing is written. */
        scpi_fail_cannot_execute();
    } else if (scpi_token_is_query()) {
        remote_reply_int((int32_t)ee_read_u16(address));
    } else {
        scpi_fail_syntax();
    }
}

/* MEMBer <dd> | MEMBer? -- two ASCII digits. */
static void member_command(uint16_t address) {
    const char *text;

    scpi_next_token();
    if (scpi_token_is_query()) {
        reply_field(address, MEMBER_LENGTH);
        return;
    }
    if (scpi_token_type() != SCPI_TOK_NUMBER) {
        scpi_fail_syntax();
        return;
    }
    if (!write_permitted()) {
        return;
    }
    text = scpi_token();
    if (strlen(text) != MEMBER_LENGTH || !is_digit(text[0]) || !is_digit(text[1])) {
        scpi_fail_bad_command();
        return;
    }
    if (!ee_write_block(address, text, MEMBER_LENGTH)) {
        fail_write();
    }
}

/* The calibration and service histories have the same three fields. */
static void history_handler(uint16_t date_address, uint16_t count_address, uint16_t member_address) {
    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    scpi_next_token();
    if (KW("date") || KW("dat")) {
        date_command(date_address, false);
    } else if (KW("count") || KW("coun")) {
        count_command(count_address);
    } else if (KW("member") || KW("mem")) {
        member_command(member_address);
    } else {
        scpi_fail_bad_command();
    }
}

/* ---- The four sub-handlers ----------------------------------------------------------------------------- */

/* SNUM "<serial>" | SNUM? -- 16 bytes, zero padded (ORIGINAL_BUGS #13). */
static void serial_command(void) {
    scpi_next_token();
    if (scpi_token_type() == SCPI_TOK_STRING) {
        const char *text = scpi_token();
        uint8_t field[SERIAL_LENGTH] = {0};
        size_t length;

        if (!write_permitted()) {
            return;
        }
        length = strlen(text);
        if (length > SERIAL_LENGTH) {
            scpi_fail_too_long();
            return;
        }
        memcpy(field, text, length);
        if (!ee_write_block(EE_THID_SERIAL, field, SERIAL_LENGTH)) {
            fail_write();
        }
    } else if (scpi_token_is_query()) {
        reply_field(EE_THID_SERIAL, SERIAL_LENGTH);
    } else {
        scpi_fail_syntax();
    }
}

/* 0x04816: :THID:PRODuct:SNUM | :DATE. Only the long spelling "snum" exists (flash 0x0466). */
static void thid_product_handler(void) {
    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    scpi_next_token();
    if (KW("snum")) {
        serial_command();
    } else if (KW("date") || KW("dat")) {
        date_command(EE_THID_PROD_DATE, true);
    } else {
        scpi_fail_bad_command();
    }
}

/* 0x04934: :THID:CALibrate:DATE | :COUNt | :MEMBer. */
static void thid_calibrate_handler(void) {
    history_handler(EE_THID_CAL_DATE, EE_THID_CAL_COUNT, EE_THID_CAL_MEMBER);
}

/* 0x04B22: :THID:SERVice:DATE | :COUNt | :MEMBer. */
static void thid_service_handler(void) {
    history_handler(EE_THID_SERV_DATE, EE_THID_SERV_COUNT, EE_THID_SERV_MEMBER);
}

/* 0x04D14: :THID:DEBug:MEMBer. */
static void thid_debug_handler(void) {
    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    scpi_next_token();
    if (KW("member") || KW("mem")) {
        member_command(EE_THID_DEBUG_MEMBER);
    } else {
        scpi_fail_bad_command();
    }
}

/* 0x04DBC: the :THID root. Called once, right after the root keyword. A bare ":THID" is a syntax error, as in
 * the original (0x04DC0, 0x04E4E). */
void scpi_cmd_thid(void) {
    if (!scpi_next_is_colon()) {
        scpi_fail_syntax();
        return;
    }
    scpi_next_token();
    if (KW("prod") || KW("product")) {
        thid_product_handler();
    } else if (KW("cal") || KW("calibrate")) {
        thid_calibrate_handler();
    } else if (KW("serv") || KW("service")) {
        thid_service_handler();
    } else if (KW("deb") || KW("debug")) {
        thid_debug_handler();
    } else {
        scpi_fail_bad_command();
    }
}

/* ---- * common commands --------------------------------------------------------------------------------- */

/* *RST restarts through `main_soft_reset()` in main.c. There used to be a second, identical
 * implementation here called `restart()`, and `main_soft_reset()` -- the one declared in main.h as the
 * public way to do this -- was the copy nothing called.
 *
 * The UNVERIFIED note that stood here is now settled and can be stated as fact: `hardware_init()` clears
 * WDRF in MCUCSR and calls `wdt_disable()` as its very first action, before the ports, so a watchdog
 * reset does not turn into a reset loop. That is main.c's start-up, read directly. */

/* Appends the text of an EEPROM field, up to its first zero byte or its size, whichever comes first. The
 * original joined three fields with strcat and so ran on past a field that had no zero byte (ORIGINAL_BUGS
 * #14); this is bounded by construction. */
static size_t append_eeprom_text(char *line, size_t length, uint16_t address, uint8_t size) {
    char field[IDN_MODEL_BYTES];

    ee_read_block(address, field, size);
    for (uint8_t i = 0u; i < size && field[i] != '\0'; i++) {
        line[length++] = field[i];
    }
    return length;
}

/* *IDN? -- "<model>,<version>,<serial>". The model and the serial are the EEPROM fields (0x0810, 0x0867);
 * the version is the firmware constant. Needs the query mark (0x089FC-0x08A02). */
static void identity_query(void) {
    char line[IDN_MODEL_BYTES + 1u + sizeof firmware_version + 1u + IDN_SERIAL_BYTES];
    size_t length;

    scpi_next_token();
    if (!scpi_token_is_query()) {
        scpi_fail_bad_command();
        return;
    }
    length = append_eeprom_text(line, 0u, EE_ID_MODEL, IDN_MODEL_BYTES);
    line[length++] = ',';
    memcpy_P(line + length, firmware_version, sizeof firmware_version - 1u);
    length += sizeof firmware_version - 1u;
    line[length++] = ',';
    length = append_eeprom_text(line, length, EE_ID_SERIAL, IDN_SERIAL_BYTES);
    line[length] = '\0';
    remote_reply(line);
}

/* *RSN -- the meter serial. The original needs no query mark here (0x08AF2-0x08B16); kept. */
static void serial_number_query(void) {
    char line[IDN_SERIAL_BYTES + 1u];
    size_t length = append_eeprom_text(line, 0u, EE_ID_SERIAL, IDN_SERIAL_BYTES);

    line[length] = '\0';
    remote_reply(line);
}

/* The keyword after '*' has been consumed by the caller (parser.c); this reads it. The trailing check the
 * original makes at 0x08B1C (the next token must be ';' or the end of the line) is scpi_parse_separator().
 * Unknown words are a bad command, as at 0x08B18. */
void scpi_cmd_common_star(void) {
    scpi_next_token();
    if (KW("rst")) {
        main_soft_reset();
    } else if (KW("trg")) {
        scpi_bus_trigger();
    } else if (KW("idn")) {
        identity_query();
    } else if (KW("rsn")) {
        serial_number_query();
    } else if (KW("pro") || KW("fac") || KW("wsn")) {
        scpi_fail_cannot_execute();
    } else {
        scpi_fail_bad_command();
    }
}
