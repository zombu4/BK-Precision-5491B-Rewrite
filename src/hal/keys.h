/* keys.h -- front-panel keyboard (read through VFD controller 1).
 * FROZEN.
 *
 * Scan codes are CERTAIN -- they are read from the original's key dispatch table. The key NAMES are
 * UNVERIFIED: which physical cap each scan code belongs to -- check: press each key on a unit
 * and record what the original firmware does. The names are for reading the code; the dispatch is by
 * code, so a wrong name costs nothing but a confusing comment.
 *
 * Debounce: the same code twice, 50 ms apart.
 * There is NO auto-repeat in the original -- do not add one.
 */
#ifndef BK_KEYS_H
#define BK_KEYS_H
#include <stdbool.h>
#include <stdint.h>
#define KEY_NONE 0x00u
#define KEY_DCV 0x11u
#define KEY_ACV 0x12u
#define KEY_OHM 0x13u
#define KEY_FREQ 0x14u
#define KEY_ACDC 0x15u
#define KEY_DIODE 0x16u
#define KEY_RANGE_UP 0x17u
#define KEY_RANGE_DOWN 0x18u
#define KEY_PREV 0x19u
#define KEY_NEXT 0x1Au
#define KEY_AUTO 0x1Bu
#define KEY_MENU 0x1Cu
#define KEY_MAXMIN 0x1Du
#define KEY_TRIG 0x1Eu
#define KEY_REL 0x1Fu
#define KEY_RATE 0x20u
#define KEY_SHIFT 0x21u
#define KEY_DEBOUNCE_MS 50u
void keys_init(void);
uint8_t keys_poll(void);       /* KEY_NONE, or a debounced code, once per press */

/* Amendment 2026-10-22. The three original key handlers
 * (key_main_handler 0x0DB4E, key_dispatch_shifted 0x0DF00, key_dispatch_unshifted 0x0E380) merge into
 * keys_dispatch(); they were blocked for want of this declaration and the state fields now in state.h.
 * keys_tick_1ms() is declared HERE so that buzzer.c stops privately declaring it: main.c calls the
 * buzzer tick and the key tick independently from its 1 ms tick. */
void keys_dispatch(uint8_t key);
void keys_sub_next(void);
void keys_sub_prev(void);

/* Called once per tick of 1.00267 ms (Timer0 CTC, prescaler 64, OCR0 = 187: (187 + 1) * 64 / 12 MHz = 12032
 * cycles). Every millisecond figure in this module is therefore 0.27 % long. NOTE: the original instrument has NO periodic
 * interrupt -- five of its 35 vectors are populated and Timer0 is unused -- so its key debounce was a count
 * of software loop iterations rather than a duration. main.c installs a 1 ms Timer0 tick and calls this from
 * it, which is a deliberate addition. Without that tick the debounce never
 * expires. */
void keys_tick_1ms(void);

/* Amendment 2026-10-22, from the key-dispatcher port. The three reference editors the SHIFT layer opens
 * directly (menu.h: menu_edit_percent_ref, menu_edit_db_ref, menu_edit_dbm_ref) are blocking in the
 * original. In the port they return at once, so the dispatcher cannot finish the key's work inline: the
 * editor calls this on a confirmed ENTER and the dispatcher then does what the key was for -- entering dB
 * or dBm mode, or applying the percent reference. A cancelled editor calls nothing. */
void keys_ref_confirmed(void);
#endif
