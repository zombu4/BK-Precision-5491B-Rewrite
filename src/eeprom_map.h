/* eeprom_map.h -- EEPROM layout and record types. COMPATIBILITY CONTRACT.
 *
 * FROZEN FOUNDATION HEADER. Modules code against this; changing it changes every module.
 *
 * Source: the original firmware's EEPROM accesses, with all 4096 bytes accounted for.
 * Every offset and record layout here MUST match the original byte for byte: a meter's existing
 * calibration data has to load unchanged. Nothing in this file may be "tidied".
 *
 * Endianness: little endian. Floats: IEEE-754 single.
 * Validity is a 16-bit signature word, never a checksum (there is no checksum anywhere in theoriginal layout).
 */
#ifndef BK_EEPROM_MAP_H
#define BK_EEPROM_MAP_H

#include <stdint.h>

#include "state.h"

#define EE_SIZE 4096u

/* Record signatures. Anything that is neither of these is invalid and loads identity defaults.
 * NOTE: a pair (0,0) WITH a valid signature is accepted and yields a zero reading -- reproduce that. */
#define EE_SIG_VALID 0x5554u
#define EE_SIG_CLEARED 0x5161u

/* ---- Record types ---------------------------------------------------------------------------------
 * Both are written little-endian with the signature word LAST (data, verify, then signature).
 * sizeof() must stay 10 and 14 -- asserted in cal/record.c.
 */
/* f0 is NOT a "gain" and f1 is NOT an "offset copy". That is only what the vendor tool happens to store
 * in DC V pairs, where f1 does equal the zero value -- on a calibrated meter DCV positive range 0 rate 0 is (1.02423, -0.181163) and its zero record is -0.181163.
 *
 * For every other function the firmware reads the pair as the coefficients of a QUADRATIC:
 *     y = m * (f0*m + f1)
 * so f0 is the quadratic coefficient and f1 the LINEAR one, with the identity (0, 1). Dumps of a
 * calibrated meter bear it out: DC A reads (3.5e-5, 1.106) and AC V (1.9e-6, 0.934) -- an "offset" of 1.1 would be absurd, a linear
 * coefficient of 1.1 is exactly right. Only DC V and AC+DC V use f0 alone, as a plain gain, and on that
 * path f1 is never read at all. */
typedef struct {
    float f0; /* the quadratic coefficient; the gain in the DC V linear form */
    float f1; /* the linear coefficient; equal to the zero in vendor-written DC V pairs, unread there */
    uint16_t sig;
} ee_pair_t; /* 10 bytes */

typedef struct {
    float z[RATE_COUNT]; /* fast, medium, slow */
    uint16_t sig;
} ee_zero_t; /* 14 bytes */

#define EE_PAIR_SIZE 10u
#define EE_ZERO_SIZE 14u

/* ---- Gain/pair tables ------------------------------------------------------------------------------
 * DC voltage 0x0000-0x012B: positive blocks per rate, then negative blocks per rate; range 0..4 at +10R.
 * Range order 0..4 = 1000 V, 500 V, 50 V, 5 V, 500 mV.
 */
#define EE_DCV_POS_FAST 0x0000u
#define EE_DCV_NEG_FAST 0x0032u
#define EE_DCV_POS_MED 0x0064u
#define EE_DCV_NEG_MED 0x0096u
#define EE_DCV_POS_SLOW 0x00C8u
#define EE_DCV_NEG_SLOW 0x00FAu
#define EE_DCV_RANGES 5u

/* DC current 0x012C-0x0257. Range order 0..4 = 20 A, 5 A, 500 mA, 50 mA, 5 mA. */
#define EE_DCA_POS_FAST 0x012Cu
#define EE_DCA_NEG_FAST 0x015Eu
#define EE_DCA_POS_MED 0x0190u
#define EE_DCA_NEG_MED 0x01C2u
#define EE_DCA_POS_SLOW 0x01F4u
#define EE_DCA_NEG_SLOW 0x0226u
#define EE_DCA_RANGES 5u

/* Ohm pair set X 0x0258-0x030B: rate step 60, 6 ranges (50 M, 5 M, 500 k, 50 k, 5 k, 500). */
#define EE_OHM_X_BASE 0x0258u
#define EE_OHM_RATE_STEP 60u
#define EE_OHM_RANGES 6u

/* Ohm pair set Y 0x0DC0-0x0E73 (same shape as set X). */
#define EE_OHM_Y_BASE 0x0DC0u

/* AC voltage / AC current pairs: rate step 50, 5 ranges. */
#define EE_ACV_BASE 0x030Cu
#define EE_ACA_BASE 0x03A2u
#define EE_AC_RATE_STEP 50u
#define EE_AC_RANGES 5u

/* ---- Zero records (14-byte), written by the SHOR/OPEN sweeps and by COEF (AC) --------------------- */
#define EE_ZERO_OHM 0x0438u /* 6 ranges */
#define EE_ZERO_DCV 0x048Cu /* 5 */
#define EE_ZERO_DCA 0x04D2u /* 5 */
#define EE_ZERO_ACA 0x0518u /* 5 */
#define EE_ZERO_ACV 0x055Eu /* 5 */

/* ---- AC low-range correction sets (A,B pairs), rate step 50 --------------------------------------- */
#define EE_ACLOW_V_SET1 0x05A4u
#define EE_ACLOW_A_SET1 0x063Au
#define EE_ACLOW_V_SET3 0x06D2u
#define EE_ACLOW_A_SET3 0x0768u
#define EE_ACLOW_V_SET2 0x0C90u
#define EE_ACLOW_A_SET2 0x0D28u

/* ---- AC auxiliary zero areas ----------------------------------------------------------------------
 * These are not "written by the sweeps": NO loader
 * ever reads them, and the AC V and AC A zero records the loaders DO read are the ordinary ones at 0x055E
 * and 0x0518, written only by :SCAL:COEF. These two areas are therefore write-only as far as the firmware
 * is concerned.
 * UNVERIFIED: that nothing reads them at all. A cross-reference scan found no consumer, but a table-driven
 * read computed at run time would not show up in one. To settle it, watch for any EEPROM read in
 * 0x0C00-0x0C8F while stepping the loaders. Until then the port writes them exactly where the original
 * does and reads them nowhere, which is faithful either way. */
#define EE_ACZERO_V 0x0C00u /* only the 500 mV range is used */
#define EE_ACZERO_A 0x0C48u

/* ---- Identity and settings block 0x0800-0x0897 ---------------------------------------------------- */
#define EE_ID_SIG 0x0800u       /* 0x5554 when initialised */
#define EE_ID_DESIGN 0x0802u    /* "Design By LJL" */
#define EE_ID_MODEL 0x0810u     /* 28 B, model name used by *IDN? */
#define EE_ID_VERSION 0x0830u   /* 16 B, version text used by *IDN? and REVISION */
#define EE_ID_VENDOR 0x0844u
#define EE_SET_BAUD 0x0854u
#define EE_SET_KEYCLICK 0x0855u
#define EE_SET_BEEPER 0x0856u
#define EE_CAL_DATE 0x0857u     /* 8 B */
#define EE_SAVE_COUNTER 0x0861u /* u16; :SCAL:SAVE increments this and nothing else */
#define EE_SET_TXTERM 0x0863u   /* 0 = LF, else CR */
#define EE_SET_ECHO 0x0865u
#define EE_ID_SERIAL 0x0867u    /* 20 B */
#define EE_UZERO_DCV 0x0880u    /* 5 floats */
#define EE_UZERO_DCV_SIG 0x0894u
#define EE_UZERO_OHM 0x0898u    /* 6 floats */

/* ---- :THID block 0x0A00-0x0A3F -------------------------------------------------------------------- */
#define EE_THID_PRODDATE_LOCK 0x0A00u
#define EE_THID_SERIAL 0x0A02u /* 16 B */
#define EE_THID_PROD_DATE 0x0A12u
#define EE_THID_CAL_DATE 0x0A1Cu
#define EE_THID_CAL_COUNT 0x0A26u /* u16 */
#define EE_THID_CAL_MEMBER 0x0A28u
#define EE_THID_SERV_DATE 0x0A2Du
#define EE_THID_SERV_COUNT 0x0A37u
#define EE_THID_SERV_MEMBER 0x0A39u
#define EE_THID_DEBUG_MEMBER 0x0A3Eu

/* ---- Service credential block (NEW) --------------------------------------------------
 * Sited inside 0x0A40-0x0BFF, which is unused in the original (erased 0xFF, not referenced
 * by any firmware path). It is NOT inside any existing record, so no vendor data is disturbed.
 * Per-meter, user-set. Salted hash only -- no secret exists in any source file.
 */
#define EE_SERV_BASE 0x0A40u
#define EE_SERV_MAGIC 0x0A40u   /* u16, 0x5356 ("SV") when a password has been set */
#define EE_SERV_VERSION 0x0A42u /* u8 */
#define EE_SERV_SALT 0x0A44u    /* 16 B */
#define EE_SERV_HASH 0x0A54u    /* 32 B */
#define EE_SERV_FAILS 0x0A74u   /* u8, wrong-unlock counter */
#define EE_SERV_END 0x0A80u     /* exclusive */
#define EE_SERV_MAGIC_VALUE 0x5356u

/* ---- Boot-initialiser marker (NEW, 2026-10-22) -------------------------------------------------------
 * The boot initialiser must record that it has run WITHOUT touching the signature at 0x0A00, and this is
 * where it does it.
 *
 * Why it cannot use 0x0A00, which is what a first fix of ORIGINAL_BUGS #108 would use: that word is the WRITE-ONCE
 * GUARD of the production-date handler. Traced to the instruction: 0x048D2 reads it, 0x048D8-0x048DE
 * compares it with 0x5554 and branches away when it matches, and only the writing path stamps it at
 * 0x048F4 after storing the date. So a boot initialiser that stamps 0x0A00 would permanently prevent the
 * production date from ever being set on a unit that has not had one -- it would close the window before
 * anyone reached it, and there is no command to reopen it.
 *
 * The marker therefore lives in 0x0A80-0x0BFF, which is unused and unreferenced
 * by any original firmware path. 0x0A00 is left strictly alone, and the production-date handler keeps its window. */
#define EE_BOOT_MARK 0x0A80u /* u16 */
#define EE_BOOT_MARK_VALUE 0x4249u /* "BI" */

/* ---- Settings and counters added by this firmware -----------------------------
 * All of it sits in 0x0A82 onwards, inside the span the original leaves unused and unreferenced. No
 * byte here is inside a vendor record, inside the service-code block at 0x0A40, or on the boot marker at
 * 0x0A80 -- the rule for new settings, and the reason the layout is laid out explicitly rather
 * than grown as each feature lands.
 *
 * Every one of these reads as 0xFF on a meter that has not had this firmware before, so EVERY reader must
 * treat 0xFF as "not set, use the default". That is the lesson of the boot initialiser, written into the map: a new byte
 * always starts erased, and code that forgets it will either misbehave or overwrite something real. */
#define EE_SET_LINE 0x0A82u      /* u8: 0 = 60 Hz, 1 = 50 Hz, 0xFF = unset -> default 60 */
#define EE_SET_RLY_BOOT 0x0A83u /* u8: see RLY_BOOT_* below */

/* RLY BOOT. Only an explicit stored 0 turns it off; an erased byte, or any other value, means ON.
 *
 * The default is ON. An erased byte is exactly the state of a meter on the first boot of this firmware,
 * so reading an erased byte as OFF would make the default unreachable rather than merely different.
 *
 * The sense is deliberately inverted relative to the other settings here: those default off or low, so
 * `0xFF means unset means default` lands on a safe value by itself. This one defaults ON, so the test has
 * to name the OFF value rather than the ON value. Both constants live here so that no reader of this byte
 * can invent its own meaning. */
#define RLY_BOOT_STORED_OFF 0u
#define RLY_BOOT_STORED_ON 1u
#define EE_SET_BRIGHT 0x0A84u    /* u8: display brightness level, 0xFF = unset -> brightest */

/* The power-up splash text. VFD_TEXT_CELLS characters plus a signature, because an erased span
 * would otherwise read as a line of 0xFF glyphs rather than as "no text set". */
#define EE_BOOT_MSG 0x0A85u      /* 9 characters, not NUL-terminated */
#define EE_BOOT_MSG_LEN 9u
#define EE_BOOT_MSG_SIG 0x0A8Eu  /* u16, EE_SIG_VALID when the text has been set */

/* The power-up counter. */
#define EE_PWRUP_COUNT 0x0A90u   /* u16; 0xFFFF = unset -> start from 0 */

/* The power-on hour meter, WEAR LEVELLED over 8 slots of 4 bytes (u16 hours, u16 sequence).
 * The arithmetic, because "wear levelled" with no numbers is just a word: the cell is rated 100,000
 * erase/write cycles, and ten years of continuous running is 87,600 hours. Written to one slot per hour
 * with the slot chosen round-robin, each slot takes 87,600 / 8 = 10,950 writes over that decade -- under
 * 11 % of the rating, with the other 89 % as margin for a meter that outlives the estimate. A single slot
 * would have taken 87,600 writes, which is inside the rating but has essentially no margin, and the
 * failure mode would be a silently stuck hour count. The highest sequence number wins on read. */
#define EE_HOURS_BASE 0x0A94u
#define EE_HOURS_SLOTS 8u
#define EE_HOURS_SLOT_SIZE 4u
#define EE_HOURS_END 0x0AB4u /* exclusive */

/* The UNDO slot. One record, because UNDO restores the LAST write: the
 * address it came from, the widest record (14 bytes), and a signature. Written before every panel or
 * remote calibration write, through the same invalidate-write-verify-revalidate path as the records
 * themselves -- an undo slot that can be half-written is worse than none, because it would restore
 * rubbish over something that was merely wrong. */
#define EE_UNDO_ADDR 0x0AB4u /* u16: the EEPROM address the saved record belongs to */
#define EE_UNDO_DATA 0x0AB6u /* 14 bytes: the widest record (ee_zero_t) */
#define EE_UNDO_LEN 14u
#define EE_UNDO_SIG 0x0AC4u  /* u16, EE_SIG_VALID when a record is saved */

/* ---- The boot loader's transfer-in-progress flag ---------------------------------------
 * Set before the first flash page of a download is written, cleared only after the whole image has been
 * received and verified. The boot loader reads it at reset BEFORE it decides whether to start the
 * application.
 *
 * Why it has to exist. The loader writes each page as it arrives, beginning with page 0, which holds the
 * reset vector. If the transfer then fails, the loader stays resident and offers a retry -- which is fine
 * while the power stays on. But the application area now holds the new image's first N pages and the old
 * image's remainder, and on the next POWER-UP the loader sees no keys held and starts that mixture
 * immediately. Nobody chooses to run it. On a meter whose only other recovery is an ISP programmer, that is
 * the difference between a retry over the serial port and opening the case.
 *
 * With this flag, an interrupted transfer lands in the loader at the next power-up, every time, however the
 * power went away. That is recoverable with the cable already attached.
 *
 * It lives at the start of a proven-unused span, deliberately far from every record and from the block the
 * application writes, because the one thing that must never corrupt it is the application. The loader
 * carries its own EEPROM read and write for it: a boot loader that called into application modules would
 * stop being a stand-alone program. */
#define EE_BOOT_XFER 0x0E74u        /* u8 */
#define EE_BOOT_XFER_ACTIVE 0x5Au   /* a transfer is in flight; do not start the application */
/* Anything else, 0xFF included, means no transfer is in progress -- which is the right default for a meter
 * that has never run this loader, and the reason the test is for one specific value rather than against
 * one. An erased byte must mean "safe to boot", or the first meter to see this firmware would refuse to
 * start its own application. */

/* ---- Proven-unused spans (update this list when using one) ------------------------- */
/* 0x0AC6-0x0BFF, 0x08B0-0x09FF, 0x0E75-0x0FFF.
 * (0x0A80-0x0A81 is the boot marker; 0x0A82-0x0AC5 is the block above; 0x0E74 is EE_BOOT_XFER.) */

#endif /* BK_EEPROM_MAP_H */
