/* serv.h -- service lock. FROZEN.
 * The remote code calls serv_check() before every protected command and must not implement its own gate.
 *
 * The serial link is PLAIN TEXT, so this protects against
 * stray, accidental or mistaken commands -- NOT against someone with the cable. Stated plainly on purpose.
 *
 * The password is PER-METER and set by the user; only a salted hash is stored, at EEPROM 0x0A40 in a span
 * that is unused and unreferenced by the original. NO SECRET EXISTS IN ANY SOURCE FILE, because
 * this source is public and nothing may be compiled in.
 *
 * *FAC, *PRO and *WSN are NOT protected -- they are REFUSED OUTRIGHT, whatever the
 * lock state, like the CLEAR commands. They rewrite the identity and the serial number, there is no
 * legitimate use for them in this firmware, and they can
 * never be sent. "Protected" would imply an unlock makes them available, and nothing should.
 *
 * Protected (an unlock makes them available): every :THID write; and
 * all calibration writes -- :SCAL:COEF, :SCAL:SHOR[:ONCE|:ALL], :SCAL:OPEN[:ONCE|:ALL], :SCAL:DATE,
 * :SCAL:SAVE, :SCAL:COEF? and the front-panel ZERO CAL.
 * Open: every :THID query, :SCAL:STD, :SCAL:STD:AC, :SCAL:TEST, :SCAL:DCV:ZERO, :SCAL:ACV:ZERO.
 */
#ifndef BK_SERV_H
#define BK_SERV_H

#include <stdbool.h>
#include <stdint.h>

#define SERV_SESSION_TIMEOUT_MS 600000UL /* 10 minutes with no protected command */
#define SERV_MAX_FAILS 3u
#define SERV_LOCKOUT_MS 60000UL          /* after 3 wrong attempts; never busy-waits */

typedef enum {
    SERV_ALLOWED = 0,
    SERV_DENIED_LOCKED,
    /* RETAINED FOR THE CALLERS, BUT NO LONGER REACHABLE from serv_check(): an absent stored code means the DEFAULT 6666 applies, so there is no state in which a
     * protected command is refused for want of a code. Kept so existing call sites keep compiling and so
     * that the reason a caller might once have printed is still nameable. */
    SERV_DENIED_NO_PASSWORD
} serv_result_t;

void serv_init(void);
/* True when a code has been STORED. False means the default 6666 is in force -- not that the meter is
 * unprotected-and-unusable, which is what it used to mean. The menu uses it to show whether the code
 * is still the factory one. */
bool serv_password_is_set(void);
serv_result_t serv_check(void); /* call before any protected command; no side effects */
/* ---- Where a session was opened -------------------------------------------------------------------
 * A front-panel key press ends a session that was opened OVER THE LINK, and does not touch one opened at
 * the panel. The distinction is the whole point: the security property worth having is that a remote
 * session does not survive somebody walking up to the instrument. Ending a PANEL session on a key press
 * ends it on the first keystroke after the code is typed -- which is every panel operation the lock exists
 * to permit, so ZERO CAL, the CAL submenu and NEW CODE could never complete.
 *
 * The source is therefore recorded at unlock time and `serv_unlock` takes it. There is no default: a
 * caller has to say which it is, because a wrong guess here is silent and the symptom appears three
 * keystrokes later. */
typedef enum { SERV_FROM_PANEL = 0, SERV_FROM_REMOTE } serv_source_t;

/* The code as four digit characters. Accepts the default 6666 when none is stored. Counts a wrong attempt
 * against the one shared counter whichever route it came in by. */
bool serv_unlock(const char *code, serv_source_t from);
void serv_lock(void);
bool serv_is_unlocked(void);

/* Changes the code. Exactly SERV_CODE_DIGITS digits, 0-9 only, and only while unlocked -- which, since the
 * default always unlocks, means there is no state where the code cannot be changed. */
bool serv_set_password(const char *code);

void serv_tick_1ms(void); /* drives the session timeout and the lockout */

void serv_note_front_panel_key(void); /* ends a REMOTE session; leaves a panel session alone */

/* The session times out after ten minutes WITH NO PROTECTED COMMAND. The obvious place to
 * refresh it is `serv_check()` -- and that is exactly what this header forbids, because `serv_check()` is
 * called before every protected command including inside loops and must have no side effects. Left as it
 * was, the ten minutes ran from the UNLOCK instead, so a long calibration would have had to unlock again
 * part-way through: a lock that gets in the way of the work it is protecting is a lock people disable.
 *
 * So the refresh is explicit. The executor calls this after a protected command SUCCEEDS -- not when it is
 * merely attempted, or a client could hold a session open forever by failing at it. `serv_check()` stays
 * pure. */
void serv_note_activity(void);

/* ---- The service code -----------------------------------------------------------------------------
 *
 * ONE code, EXACTLY FOUR DIGITS, used by the front panel and the remote link alike. The vendor meter works
 * this way and the panel's KEY prompt only enters four digits, so anything else makes panel unlock
 * impossible for most codes.
 *
 * **The default is 6666 and it applies whenever no code has been stored.** A meter therefore works out of
 * the box exactly like the vendor's, with no setup step. A fresh meter is usable, not bricked-by-policy.
 *
 * SAY THIS PLAINLY WHEREVER IT IS DOCUMENTED: 6666 is the VENDOR'S PUBLICLY KNOWN CODE. A meter left on
 * the default is protected against an accident and against nothing else. That is the whole intent -- this
 * lock exists so a stray or mistaken command cannot reach a calibration write, and `serv.h`'s threat model
 * has never claimed more. The link is plain text.
 *
 * A changed code is stored as a salted hash in the 0x0A40 block, exactly as before. A FORGOTTEN code is
 * recovered by an ISP write of that block, which returns the meter to 6666 -- so the user can always get
 * back in with a programmer, and nobody is locked out of their own instrument by this feature.
 *
 * The three-attempt lockout is shared: the panel and the remote link count against ONE counter, because
 * two separate counters would let an attacker have six tries, and because an accident at the panel should
 * be visible from the link. */
#define SERV_CODE_DIGITS 4u
#define SERV_CODE_DEFAULT "6666"

#endif /* BK_SERV_H */
