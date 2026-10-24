# PROTOCOL_CHANGES.md -- every protocol-visible difference from the vendor firmware

BK-Calibrator's support for this firmware is built from this file.

**Preserved deliberately:** the reply LAYOUT -- field order, separators, the `!` terminators of the
`:SCAL:SHOR[:ONCE]` / `:SCAL:OPEN[:ONCE]` sweeps, and the LF (or CR when TX TERM is set) line terminator.
That is format, not a bug, so it does not change.

**Preserved because they are not bugs:** `:SCAL:SAVE` still only increments the save counter at EEPROM
0x861; calibration writes are still persistent the moment they are sent. EEPROM record formats
and offsets are unchanged, so existing calibration data loads as-is.

Status: every row was checked against the source on 2026-10-23 ("code audit" and "code reconcile", with file:line). DONE = implemented. REMOVED = a command deliberately not implemented, with the rule that forbids it. HARDWARE = open until the named bench check is done. UNVERIFIED = one claim with the check that settles it. No row is left partial or not done.

## A. New commands

| Command | Behaviour | Reason | Status |
|---|---|---|---|
| `:SERV:UNLOCK "<password>"` | Opens a service session. Wrong password counts toward a 3-attempt lockout of 60 s. | Service lock (the original has no authentication) | done |
| `:SERV:LOCK` | Ends the session immediately. | Service lock | done |
| `:SERV:STATe?` | Replies LOCKED or UNLOCKED. Never gated. | Service lock | done |
| `:SERV:PASSword:SET "<new>"` | Sets the per-meter password. Accepted only while unlocked, or when none is set. | Service lock | done |
| `:SCAL:COEF?` | Replies gain,offset for medium/fast/slow of the active function and range, as eight-decimal `%e` numbers. Protected. | Coefficient read-back; the vendor firmware has none | done |

| `:SYSTem:KCLick ON\|OFF` and `:SYSTem:KCLick?` | NEW. The key click had a front-panel menu item (SYS, KEY SONG) and no remote command at all, so a client could read and set every other SYS setting but not this one. Stored like the rest (EEPROM 0x855) and NOT protected. ORIGINAL_BUGS #42 |

Further new commands, listed here because this file is the client's whole command set:

| Command | Behaviour | Reason | Status |
|---|---|---|---|
| `:SCAL:UNDO` | PROTECTED. Restores the record the last calibration write saved (panel or remote) and replies `!`. An empty undo slot is a cannot-execute error, not a silent success, because coefficients cannot be read back to tell the two apart. | Panel calibration | done (remote) |
| `:SCAL:CHECK?` | Open, read-only. Replies the number of calibration records that fail the signature or plausibility test, then one line `<name>,<eeprom address>` per bad record, one line per main-loop pass. It must be the LAST statement of its line (anything after it is a syntax error and nothing is sent), and no further line is executed until the list is complete. | Panel calibration | done (remote) |
| `:DISPlay:BOOT "<text>"` and `:DISPlay:BOOT?` | Open. Sets or reads the power-up message (EEPROM 0x0A85-0x0A8F). A text that is too long or holds a character with no glyph is a parameter error, never a substitution. | Boot message | done (remote) |
| `:SYSTem:LFRequency 50\|60` and `:SYSTem:LFRequency?` | Open. Line frequency used by the converter's decimation (EEPROM 0x0A82). A change is refused (cannot execute) while a sweep owns the converter. | Line frequency | done (remote) |
| `:SYSTem:INFO?` | Open. One line of seven comma separated fields: firmware version, serial, calibration date, calibration count, production date, power-on hours, power-up count. The date and count are the `:THID:CAL` pair. A date field whose stored 8 bytes are not `dd/mm/yy` (digits, slashes in cells 2 and 5) is sent empty: a vendor unit stores `5///5/5/` in both :THID dates (EEPROM 0x0A1C, 0x0A2D), and an erased field is `FF`. `:THID:CAL:DATE?` is unchanged and still sends the stored bytes. The version field is `Ver<X>.<Y>.<Z>.<yy>.<mm>.<dd>`, up to 23 characters (X.Y.Z is the release tag; `*IDN?` keeps its three fields and reports the same version string, so two releases made on one day differ). | System information | done (remote) |
| `:SYSTem:RELay:STATistics?` | Open (not service-locked). Replies `<patterns>,<elapsed_ms>`: two unsigned integers, the relay patterns written and the milliseconds elapsed in the last relay exercise, or in the running one; `0,0` before any exercise. Normal reply terminator. | Relay exercise report | done (remote) |

Differences kept deliberately, listed so that a client built from this file is not surprised:
`:SCAL:TEST` replies `<value>:` with nothing after the colon (the original appends the main display text; the
vendor utility parses `<value>:<text>` and uses only the value). `*<cmd>;<cmd>` as the
first statement of a line runs both statements (the original answered the `*` command and then raised error
(1,2)). A line of blanks only is ignored without an error (see section K; the original
raised (1,2)).

## B. Commands that become protected (refused unless unlocked)

Every `:THID` **write** (date, member and serial writes and the count increment, src/serv/thid.c:68-76, :120, :153, :193, :234);
`:SCAL:COEF`, `:SCAL:SHOR[:ONCE|:ALL]`, `:SCAL:OPEN[:ONCE|:ALL]` (one gate for both, src/remote/exec.c:349), `:SCAL:DATE`, `:SCAL:SAVE`,
`:SCAL:COEF?`, `:SCAL:UNDO`, and the front-panel ZERO CAL (src/ui/menu.c:2788, with the CAL, CODE and RESET items). The gate is protected_begin() in src/remote/exec.c:125-131, called at :349, :630, :683, :719 and :733; `:SCAL:COEF?` asks serv_check() itself, src/remote/handlers.c:1632; the calibration entry points ask again, src/cal/cal_ops.c:295, :709, :802.

When locked they are refused through the existing error queue with **no state change and no EEPROM
write**.

**Not protected, refused in every state.** `*FAC`, `*PRO` and `*WSN` (identity rewrites) answer a cannot-execute error whether the
meter is locked or unlocked, src/serv/thid.c:380-381, and the `:THID` count decrement (DECRease) likewise, thid.c:170-172.
`:SCAL:CLEAR_DATA|CLEAR_ALL|CLEAR_INIT|CLEAR_REL` do not exist and give the bad-command error (section F). All of these follow the
rule that memory wipes, identity rewrites and counter decrements are never available, so no session reaches them.

**The service code.** Until a code is stored the default is the vendor's public 6666 (a design decision), src/serv/serv.c:10-22,
src/serv/serv.h:92-109. A meter on the default is protected against an accident and against nothing else. A stored code is a salted hash
and a forgotten one is recovered only by an ISP write of its EEPROM block. This is a recorded decision, not an open defect (ORIGINAL_BUGS #60).

Still open and ungated: every `:THID` query, `:SCAL:STD`, `:SCAL:STD:AC`, `:SCAL:TEST`,
`:SCAL:DCV:ZERO`, `:SCAL:ACV:ZERO`, and all measurement and configuration commands.

**Consequence:** the vendor calibration software can no longer calibrate this
firmware. Status: DONE (code reconcile 2026-10-23: every command listed above reaches serv_check()).

## C. Bug fixes that change observable behaviour

**Writers validate the whole statement before they act.** `:SCAL:SAVE`, `:SCAL:UNDO`, `:SCAL:DATE`,
`:SCAL:COEF "..."` and `:SCAL:SHOR` / `:SCAL:OPEN` (all forms) check that the statement ends where it should
before anything is changed, and so do `:SCAL:TEST` and the two `:ZERO` queries. `:SCAL:SAVE?`,
`:SCAL:UNDO?` and `:SCAL:SHOR:ALL x` are syntax errors with no state change; the original ran the action
first (0x07052 increments the counter, the separator check follows). A `;*cmd` after a failed statement no
longer runs (`*BAD;*RST` does not restart the meter): the line ends at the first error.

Every pre-existing bug is fixed. Each row is a behaviour difference a client can see.

| Area | Vendor behaviour (bug) | New behaviour | Status |
|---|---|---|---|
| Number input | A numeric parameter of `0` is rejected | `0` accepted wherever the range allows it | DONE (code audit 2026-10-23: src/fmt/fmt.c:37-39,577-582,710-713; src/fmt/fmt.h:19-20; src/remote/parser.c:305-328) |
| Number input | Root keyword `scalibration` never matches | Accepted | done (remote) |
| Number input | Out-of-range exponent silently yields a wrong value (`1e39` stored as `1.2e-38`) | A normal error; nothing is stored | DONE (code audit 2026-10-23: src/fmt/fmt.c:689-693,715-718,742-744; src/remote/parser.c:317-319) |
| Number output | `+` or space printed before a negative number; zero padding before the minus sign | Fixed; the rest of the layout unchanged | DONE (code audit 2026-10-23: src/fmt/fmt.c:380-430,491-493) |
| Number output | `%e` of 0.0 prints nothing; 1e-5 prints as 9.9e-6 (24-bit fraction) | Correct output | DONE (code audit 2026-10-23: src/fmt/fmt.c:268-276,286-296,515) |
| `:CALC:LIM:FAIL?` | Answers 1 for *in limits* (inverted) | Answers 1 on FAIL | done (remote) |
| `:CALC:LIM:UPP\|LOW` | A non-number argument is treated as a query | Error | done (remote) |
| `:FUNC2 CURR:DC` | On AC+DC current, selects the AC entry | Selects the DC entry | done (remote) |
| `:UNIT ... DB` | Uses the dBm formula, and DB->DBM is converted twice | `20*log10(V/Vref)` for dB; no double conversion | done (remote) |
| NPLC `MIN`/`MAX` | Inverted | Correct | done (remote) |
| NPLC argument | `1.0` and `1e0` rejected | Accepted (a correct fix, kept) | done |
| `:RANGe?` | Replies silently for a range outside the function's family | Error | done (remote): with G.2 #25 every range query names a function of its own family |
| `:VOLT:ACDC:RANGe?` | NUL byte / truncated reply | Correct reply | done |
| `FETCh?;<cmd>` | Syntax error | Accepted | done (remote) |
| Trailing blank before end of line | Syntax error | Accepted | done (remote) |
| `HOLD:WINDow?` | Replies a float built from a 16-bit integer (garbage) | Replies the hold window | done |
| REVISION / `*IDN?` | REVISION shows the model name and "001"; version text comes from EEPROM 0x830 | REVISION shows our version string; `*IDN?` keeps model `5491B` with our version | DONE (code audit 2026-10-23: src/serv/thid.c:58,337-357; version.h:25-27; src/ui/menu.c:1135-1149,1361-1362) |
| Error handling | A command error stalls the CPU about 1 s with interrupts off, losing every byte that arrives | Errors are queued and reported without blocking; no bytes lost | done |
| Bad arguments | A bad `:SCAL:COEF`, `:SCAL:DATE`, range or NPLC argument still takes effect | Validated first; an error changes nothing | DONE (code audit 2026-10-23: src/cal/cal_ops.c:292-345; src/remote/exec.c:629-665; src/remote/exec.c:546-570) |
| `:SCAL:COEF` | No field-count or range checks; index overflow into neighbouring records | Strict per-function field count and bounds-checked index | DONE (code audit 2026-10-23: src/cal/cal_ops.c:173-281; src/cal/cal.c:81-185; src/remote/exec.c:560-562) |

## D. Timing-visible differences

- The command path no longer runs with interrupts disabled, so long sweeps do not make the meter deaf
 . A client that relied on the 1 s stall after an error will see the next reply sooner.
- The bit-banged converter bus keeps the original timing (`15n + 18` cycles per unit). No protocol effect.
- The relay exercise pauses readings while it runs; `reading.valid` is false and the display
  shows the exercise. If a remote command is added for it, it is listed in section A.

## E. Not yet decided
- Whether a relay break-before-make ordering changes any settling time a client could
  observe. HARDWARE (ORIGINAL_BUGS #101 and #102), open until the bench check: with the inputs open, step the ranges and trace each PA..PG bit with a
  scope or continuity meter, and step a known input across a range change to time the settling. The settle itself is already a duration
  (100 ms for DC V and AC V, 60 ms for the rest, src/meas/meas.c:112-117, :436-468); the bench check settles whether those values are enough.

## F. Added from the ORIGINAL_BUGS sweep (2026-10-22)

The sweep found 105 distinct original bugs. These are the ones a remote
client can observe. The rest are internal and do not belong in this file.

| Area | Vendor behaviour (bug) | New behaviour | Status |
|---|---|---|---|
| `:HOLD:STATe ON` | **Locks the remote port.** With the display enabled, the hold refresh loop (0x0AEB4) services the keyboard (0x0DB4E) but never the command executor (0x0FBEA), which has only two callers. The port stays deaf until a front-panel key is pressed. | Hold redraws once; the main loop keeps running and `remote_poll()` is always reached. | **done** (the port removed the nested loop; verified by cross-reference) |
| Baud 38400 | The divisor actually yields **39474 baud, +2.8%** | Correct divisor (with the full 12-bit UBRR1H/L fix) | DONE (code audit 2026-10-23: src/hal/uart1.c:411-426) |
| `:CALC:PERCent` ACQuire and value | Acquiring a percent reference from a **zero** reading, or setting `:CALC:KMATh:PERCent 0`, stored a zero reference, and percent mode then returned +FLT_MAX (#20, #78) | Both are rejected with a data error; no reference stored. Remote `:CALC:KMATh:PERCent <value>` and `:ACQuire` refuse zero at src/remote/handlers.c:1395-1417 (:1403, :1414); the front-panel editor refuses a zero reference at src/ui/menu.c:1634; no other writer of the reference exists | DONE (code reconcile 2026-10-23) |
| `HOLD:WINDow` / `HOLD:COUNt` (#32) | The values are stored but **never read anywhere in the image**, so hold does nothing but freeze the display, and 0x0E2D is zero at power-on while the menu shows "1%" | Field initialised. Capture rule **APPROVED as proposed**: once HOLD is armed, the reading latches when COUNt consecutive readings stay within WINDow (a fraction of the first reading); then the display freezes and the meter beeps if the beeper is on | DONE (code audit 2026-10-23: src/meas/meas.c:1289-1333,2197,2353-2356; main.c:237-238; src/remote/handlers.c:761-773,781-796,805-809) |
| `:SCAL:CLEAR_REL` | Wrote the CLEARED signature 0x5161 to EEPROM 0x0894. The boot initialiser reads that word at 0x0532A and, when it is not 0x5554, writes 0.0 over all five DC V and all six OHM user zeros, so the command wiped every user zero ONCE, at the next power-up (ORIGINAL_BUGS #59) | **Removed by design, not fixed.** The command does not exist: it is an unknown keyword and gives the normal bad-command error. A calibration memory wipe is forbidden by the project safety rule (the project's safety rule names `:scal:clear_data` among the commands that must never be sent), so implementing it would be the wrong repair. The boot initialiser no longer zeroes a user-zero table on one signature word (src/cal/cal_ops.c:1178-1206) | REMOVED BY DESIGN (src/remote/exec.c:305-309, no keyword; src/cal/cal.h:430) |
| `:SCAL:CLEAR_DATA` | Stamps only 19 signature sets; the sweep-written AC zero records at 0x0C38 and 0x0C80 **survive a clear** | **Removed by design, not fixed**, for the same reason as `:SCAL:CLEAR_REL`: a calibration memory wipe is forbidden by the project's safety rule (`:scal:clear_data` is named there). `:SCAL:CLEAR_ALL` and `:SCAL:CLEAR_INIT` are removed with it. All four give the bad-command error | REMOVED BY DESIGN (src/remote/exec.c:305-309; src/cal/cal.h:430) |
| `:SCAL:COEF` | The existing valid signature stays in place while the data underneath is rewritten, so a power loss leaves a record **marked valid over half-old, half-new data** | Signature invalidated first, then data written and verified, then revalidated | DONE (code audit 2026-10-23: src/cal/cal.c:243-263; src/cal/cal.c:287-302; src/cal/cal_ops.c:336-343) |
| `:SCAL:STD` (#55) | Prints the same medium-rate reading three times in the `%.8e:%.8E,%.8E` reply | **APPROVED:** measures at fast, medium and slow and prints them in the field order **medium, fast, slow**: the number before the colon is the MEDIUM reading, field 2 the fast one, field 3 the slow one, which is how the vendor utility parses the reply (ReadStd @51e36c, the 5492B source does the same). Layout unchanged; the command takes longer, so a client needs a longer timeout | done (remote, exec.c); APPROVED 2026-10-22, including the field order |
| `:DIODe` | An error stub with an orphan range table | HARDWARE (ORIGINAL_BUGS #44): the keyword is a valid root with no command behind it (src/remote/exec.c:290, :904). Bench check: scope the diode test current at the 2.3 V diode range while stepping the diode range; if no current select exists the range table (src/tables.c:402-405) is dead and is deleted | HARDWARE |

### Not fixable in firmware
**Overload / blown-fuse detection.** No pin the firmware reads can sense either: the
only input reads are PB3, PB4 and PC2, with no ADC, comparator or external-interrupt use. PD6 and PD7 are
inputs with pull-ups that are never read, and whether they are wired to anything is **unverified**.
Until they are traced this is the one acceptable non-fix under the every-bug rule: HARDWARE (ORIGINAL_BUGS #106). Bench check: trace ADS1216 pins 37-44 (D0-D7); if one reaches a fuse or comparator net, add a DIO register read and keep its DIR bit as an input. Existing
mitigations: overload is caught by the limit-table compare (the converter saturates at about 1.2x range),
and a blown current fuse reads as zero current.

## G. Full ORIGINAL_BUGS review (2026-10-22): remaining client-visible changes

Every row below is keyed by its ORIGINAL_BUGS number. Each row carries
its own status, read from the source on 2026-10-23. Reply layouts stay as in the vendor firmware.

### G.1 Serial link and line handling

| Area | Vendor behaviour (bug) | New behaviour | Status |
|---|---|---|---|
| Early next command (#3) | Bytes received while a line is still pending are dropped silently | RX ring buffer; if it overflows the whole line is discarded and an error is queued | DONE (code audit 2026-10-23: src/hal/uart1.c:233-272; src/hal/uart1.c:323-376; src/remote/exec.c:953-955) |
| Line errors (#4) | Framing, overrun and parity errors are accepted as data | The frame is dropped and an error is queued; no garbled command runs | DONE (code audit 2026-10-23: src/hal/uart1.c:236-246; src/remote/exec.c:956-958) |
| Empty lines, CR LF (#6) | A bare CR or LF, or the LF of CR LF, runs an empty command and lights REM | Empty lines are ignored; CR LF is one terminator; REM lights only for a non-empty command | DONE (code audit 2026-10-23: src/hal/uart1.c:344-363,377-381) |
| Overlong line (#7) | At 255 bytes the line is truncated and the truncated text executes (a cut `:SCAL:COEF` or `:SCAL:DATE` would run) | The whole line is discarded, an error is queued, nothing executes | DONE (code audit 2026-10-23: src/hal/uart1.c:364-375; src/remote/exec.c:995-1000) |
| Reply pacing (#8, #93) | A 1 ms pause follows every reply character; a long `%.8e` reply blocks the meter for 15 ms or more and baud rates above 9600 are throttled | No pause. Replies and the echo go into a 128-byte transmit ring emptied by the UDRE interrupt, so a reply of up to 113 characters never makes the main loop wait. A host now sees replies at line speed, one terminator byte per reply as before | DONE (src/hal/uart1.c:116-250) |
| Baud 2400/1200/600 (#89) | Only the low divisor byte is used: the real rates are about 13393, 6637 and 3318 baud | Full 12-bit divisor; the rates are the nominal ones | DONE (code audit 2026-10-23: src/hal/uart1.c:411-426) |
| Statement limit (#23) | No limit on `;`-chained commands in one line; each level uses stack | Not needed: the executor is a loop over the statements of one line (exec.c remote_execute_line), not a recursion, so a chain uses no stack per statement, and a line is bounded at 255 characters by the receive path | not needed (remote) |
| Lock-ups and traps (#72, #94, #95) | No watchdog, so any lock-up is permanent; a spurious interrupt runs into an unrelated ISR; stack overflow is never detected | A watchdog (2 s, fed every loop pass, src/main.c:442-445) and two traps that reset the meter through it, so the client sees a restart to power-up state instead of a hang. (1) A `.noinit` stack guard filled with 0xA5 and checked every loop pass, src/main.c:354-386, :446-448. (2) `ISR(BADISR_vect)`, main.c:336-352, which routes every unhandled vector through the watchdog instead of avr-libc's default, a `jmp 0` (a `jmp 0`, the jump-to-zero of the original's default handler). A build must show no `__bad_interrupt` in the vector table | DONE (code reconcile 2026-10-23) |
| Command delays (#93) | Beeps, key waits, the lamp test, "SAVED" and the 1/2/3/5 s sweep settles are CPU busy-waits that starve the UART and measurement | Tick-based timers; the meter keeps receiving and measuring during them. The sweep settle times themselves are unchanged. The busy-waits that remain are closed with a reason each: the lamp test (src/hal/vfd.c:124-129, :166) runs in hardware_init() before uart1_init and remote_init (src/main.c:404, :414, :415), so no command can be pending; the splash (src/ui/display.c:848, :853, 1.22 s) runs after them, from main() (main.c:433); the zero capture (src/meas/meas.c:2447-2495, panel ZERO CAL behind the service code, one range per tick) and the sweep averaging (meas.c:2403) hold the CPU for their conversions. In all of those the receiver is interrupt-driven into a 256-byte ring (src/hal/uart1.c:92-94, :253): a command waits, none is dropped until the ring is full, and a full ring sets rx_overrun (:281), which is reported as a data error through uart1_rx_overrun_take() (:309, src/remote/exec.c:1003). That is reply latency with the loss detected and reported, not the silent deafness of the original | DONE (code reconcile 2026-10-23: src/ui/menu.c:78 and menu_tick; src/hal/buzzer.c; src/main.c:404-433) |

### G.2 Parsing and arguments

| Area | Vendor behaviour (bug) | New behaviour | Status |
|---|---|---|---|
| Overlong numeric token (#16) | 20 or more characters raise an error and are still copied into a 20-byte buffer (stack overflow) | Length check first; error and no copy | DONE (code audit 2026-10-23: src/remote/parser.c:141-158; src/fmt/fmt.c:624-626,645-647) |
| Unterminated quote (#17) | Syntax error, but the next token is read from stale buffer bytes | Syntax error, no token, the line ends | DONE (code audit 2026-10-23: src/remote/parser.c:161-178; src/remote/reply.c:79-86) |
| Character that starts no token (#18) | Characters such as `+`, `.`, `#`, control codes and bytes of 0x80 and above are not consumed; only the error flag stops a loop | Always consumed or the line ends; one syntax error | DONE (code audit 2026-10-23: src/remote/parser.c:227,254-259; src/remote/exec.c:863-892; src/remote/reply.c:79-86) |
| Number scanning (#21) | Trailing junk is ignored (`12abc` = 12, `1,5` = 1); `1e`, `1e+` and `1e-` are accepted as exponent 0; results are 1-3 ulp low; a leading blank fails | Round to nearest; trailing junk, empty exponent and an out-of-range exponent are errors (unit suffixes only where the range table lists them) | DONE: `1e`, `1e+` and `1e-` are now a syntax error (src/fmt/fmt.c:683-687); the remainder of the row is as audited 2026-10-23 (src/remote/parser.c:298-314; src/fmt/fmt.c:47-52) |
| Integer arguments (#22) | Wrap at 16 bits (`99999` = 34463); `5.7` becomes 5 for `HOLD:COUNt`; `+` and blanks not accepted | Range-checked; overflow and a fractional value are errors | DONE (code audit 2026-10-23: src/fmt/fmt.c:547-591; src/remote/parser.c:351-361; src/remote/handlers.c:786-791,1139-1145,1288-1294) |
| Node selects the function (#25) | `:AC`, `:DC`, `:ACDC` (and VOLT/CURR/RES) are ignored; every command edits the ACTIVE function, so `VOLT:DC:RANGe 5` in a current function changes the current range | The named node selects the function whose RANGe/REFerence state is edited (`VOLT:DC`, `VOLT:AC`, `VOLT:ACDC`, `CURR:...`, `RES`, and the FREQ and PER roots for their own functions). The hardware is touched only if that function is active. `REFerence:ACQuire` of a function that is not active is a cannot-execute error, because the last reading belongs to the active function | done (remote) |
| Keyword parents (#26) | AUTO, UPPer, STATe and ACQuire are accepted under both RANGe and REFerence; `VOLT:DC:RANGe:STATe ON` switches REL on | AUTO and UPPer are accepted only under RANGe; STATe and ACQuire only under REFerence (for FREQ and PER: VOLTage only under THReshold, STATe and ACQuire only under REFerence); elsewhere a syntax error | done (remote) |
| Threshold range arguments (#28) | `FREQ`/`PER` `:THReshold:VOLTage:RANGe` takes only numbers | DEF, MIN and MAX are accepted, as for VOLT/CURR/RES | done (remote) |
| `CURR:ACDC:RANGe:UPPer?` (#29) | Reply truncated (`0.05` becomes `0`). The `VOLT:ACDC:RANGe?` fix in section C is the same defect | Full spelling of the selected entry | DONE (code audit 2026-10-23: src/remote/tree.c:114-129,159-179) |
| `:FUNC?` on AC+DC (#30) | Reads past the name table | Correct name for all 11 functions: `volt:acdc` and `curr:acdc` for the last two | done (remote) |
| `FETC?` count (#40) | Two `FETC?` before the next conversion give one reply | Each `FETC?` gets its own reply, one per conversion (up to 255 outstanding; a request beyond that is a cannot-execute error). Each reply is sent when the next reading completes (ordering in main.c) | done (remote) |
| Tiny values (#41) | `FETC?`, `REF?` and `FREQ:REF?` print `0.000` below 1e-7 (a 20 MHz period of 5e-8 s; currents under 100 nA) | The true value in the unchanged `%.8e` layout; an exact zero prints `0.0e0` like every other zero in a reply (section H), where the original printed `0.000` | done (remote) |
| `:SYSTem:BEEPer` (#42) | Writes the key-click setting, not the beeper of menu BEEP STA (error, limit, hold) | `:SYST:BEEPer` controls the BEEP STA beeper. **Proposed new command `:SYSTem:KCLick ON\|OFF` (and `?`)** for key click, stored where the old BEEPer value was (EEPROM 0x855). A client that used BEEPer for key click must switch to KCLick | APPROVED 2026-10-22; see section A |
| `:SCAL:STD?` (#43) | No reply and no error, so the host waits forever | A normal error (the command is not a query) | DONE (code audit 2026-10-23: src/remote/exec.c:319-343) |
| `*IDN?` version (#12) | The version text is read from EEPROM 0x830, not from the running firmware (the repaired unit shows Ver1.4.14.06.18 only because that EEPROM text was edited) | The version is a firmware constant; EEPROM 0x830 is no longer used (an edit of it has no effect) | DONE (code audit 2026-10-23: src/serv/thid.c:58,348-353; eeprom_map.h:120) |
| Identity fields (#13) | `*PRO`, `*WSN` and `THID` serial writes copy a fixed block including stale bytes after the NUL; an over-long token is cut silently | Zero-padded fields; a too-long argument is rejected with an error before any write | DONE (code audit 2026-10-23: src/serv/thid.c:227-250; src/serv/thid.c:381-382) |
| `*IDN?` join (#14) | UNVERIFIED: three EEPROM blocks are joined without a bound, so a field with no NUL could run on | Every block copy and the join are bounded; a reply never overruns | DONE (code audit 2026-10-23: src/serv/thid.c:278-357) |

### G.3 Errors and calibration commands

| Area | Vendor behaviour (bug) | New behaviour | Status |
|---|---|---|---|
| Error queue (#10) | Only the last error is kept; `:SYST:ERR?` returns and clears it | 8-deep queue; `:SYST:ERR?` pops the oldest; `NO ERROR!` when empty. A ninth error while eight are queued is dropped, so the eight kept are the first eight | done (remote) |
| `:SCAL:COEF` non-string (#47) | A non-string argument raises an error and the parser still writes EEPROM | Return after the error; nothing written | DONE (code audit 2026-10-23: src/remote/exec.c:604-615) |
| EEPROM write check (#50) | No read-back; an interrupt in the write window can drop a byte | Writes are read back and compared; a mismatch is an error reply to the command | DONE (code audit 2026-10-23: src/hal/eeprom.c:60-81; src/remote/exec.c:672-674,660-662) |
| `:SCAL:SHOR:ONCE`, `:SCAL:OPEN:ONCE` (#52) | No function guard: other functions write the ACV/ACA zero area; the range index can be out of range | A function and range guard; a mismatch is an error and nothing is written | DONE (code audit 2026-10-23: src/cal/cal_ops.c:377-391,686-688; src/remote/exec.c:187-191) |
| After `:SCAL:SHOR`/`OPEN` sweeps (#53) | The function is left at VDC and every swept range index is left past the last valid value | The saved function and range indices are restored | DONE (code audit 2026-10-23: src/cal/cal_ops.c:473-489,564-571) |
| `:SCAL:ACV:ZERO`, `:SCAL:DCV:ZERO` (#54) | Use an uninitialised base address outside their functions; change and restore the rate during the query | Function guard (error outside the function); the rate is not touched | DONE (code audit 2026-10-23: src/remote/exec.c:389-414; src/cal/cal_ops.c:982-1017) |
| `:SCAL:TEST` (#56) | For functions 5-10 no measurement is taken; the reply carries the previous value | A real measurement is taken, or an error | DONE (code audit 2026-10-23: src/cal/cal_ops.c:940-969; src/remote/exec.c:358-373) |
| `:SCAL:DATE` (#57) | Writes 8 bytes even if a character failed validation or the text is shorter than 8 | Validated first; exactly the validated text is written, padded; an invalid date is an error and changes nothing | DONE (code audit 2026-10-23: src/remote/exec.c:629-665; src/cal/cal_ops.c:1023-1037) |
| `THID` counts (#61, #62) | Counts are printed with only their first two characters (100 and above cut); INCR/DECR have no bounds, so DECR from 0 gives 65535 | The full number is sent; counts clamp at 0 and 65535 with an error | DONE (code audit 2026-10-23: src/serv/thid.c:150-178) |
| `THID` writes (#63) | Date and member validation errors do not stop the write; a production-date write after the lock is ignored with no error | Validate, then write; a locked write is an error | DONE (code audit 2026-10-23: src/serv/thid.c:110-137,181-204,227-244) |

### G.4 Measurement, readings and frequency

| Area | Vendor behaviour (bug) | New behaviour | Status |
|---|---|---|---|
| ZERO CAL capture (#65, #104) | The first conversion (possibly stale) is kept, the value is stored with no limit or read-back, and the sweep sets PA5 on every DCV range | The first conversion is discarded; a zero above the dead-band threshold is refused and the old table kept; read-back verify; the capture runs in the normal DCV PA5 state (PA5 = 0). #104 is FIXED-BY-DESIGN | DONE (code audit 2026-10-23: src/meas/meas.c:2422-2431,2442-2493; src/meas/meas.c:549-565; src/ui/menu.c:2181-2196) |
| DC V negative readings (#66, #67) | The negative gain pair is overwritten by the positive pair for the primary path; the secondary path (ACV dual display) uses the cleared negative record with gain 1.0, so negative DC V reads about 2.4% short | Negative pair used only when its signature is valid, else the positive pair, on both paths | DONE (code audit 2026-10-23: src/cal/cal.c:428-442; src/meas/meas.c:1029,1508,1771-1773) |
| Zero-valued coefficients (#68) | A valid-signature pair (0.0, 0.0) makes the range read 0 | A zero or non-finite coefficient is treated as invalid and the identity is used. Two records in an EEPROM dump of a calibrated meter read UNCALIBRATED instead of 0: AC V 50 V slow (0x0384) and AC A 500 mA medium (0x03E8) | DONE (code reconcile 2026-10-23: cal_pair_usable(), src/cal/cal.c:234-238, used at cal.c:365 and src/cal/cal_panel.c:235, :495) |
| ACV secondary DC user zero (#69) | The user-zero array is loaded only by the DCV loader, so the secondary DC path uses stale values when DCV was never selected | The ACV loader loads the DC user zeros when the secondary DC path is active | DONE (code audit 2026-10-23: src/cal/cal.c:467-470; src/cal/cal.c:556-559; src/meas/meas.c:1508,1771-1773) |
| Autorange (#70) | No iteration limit; an input on a threshold can alternate ranges forever | At most 2 x the number of ranges per pass, then the last range is held | DONE (code audit 2026-10-23: src/meas/meas.c:148-149,1647,1721-1730) |
| Converter timeout (#71) | /DRDY is polled with no timeout; a missing converter hangs the meter | Bounded wait; the reading is invalid and an error is queued | DONE (code audit 2026-10-23: src/hal/adcbus.c:200-208; src/hal/adcbus.h:16; src/meas/meas.c:898-925,1700-1704) |
| Frequency scale (#73) | Timer overflow counts weighted 65535 instead of 65536 (15 ppm) | 65536 | DONE (code audit 2026-10-23: src/hal/freq.h:12; src/hal/freq.c:314-316) |
| Frequency timeout (#74) | The wait for the last edge ends after 3000 loop passes, so a slow signal can time out and read 0 | Time-based timeout of the gate time plus two signal periods, armed in three stages (with the gate, at the first edge, at the second edge) and enforced by the Timer1 overflow interrupt | DONE (code reconcile 2026-10-23: src/hal/freq.c:171, :236, :247-248, :297; the fixed 3000 ms test in src/meas/meas.c is gone, step_gate() at :1826-1846 only polls) |
| Period of zero frequency (#75) | 1/f with no guard returns +FLT_MAX | An explicit no-signal reading | DONE (code audit 2026-10-23: src/hal/freq.c:341-351; src/meas/meas.c:171-172,1857-1862) |
| Frequency capture race (#76) | A capture arriving with the overflow flag pending is stamped one overflow wrong (up to 1.1% of a 0.5 s gate) | The pending overflow is counted first | DONE (code audit 2026-10-23: src/hal/freq.c:182-196) |
| Zero frequency display (#77) | Exact 0 Hz keeps the milli prefix | Base unit (display text) | DONE (code audit 2026-10-23: src/ui/display.c:298-312) |
| AC+DC low-range correction (#79) | The vendor compares the SIGNED corrected value with 0.025 times a per-range table entry (0x0AA1A-0x0AA42) and `brlt` (0x0AA46, signed) forces it to 0.0 (flash 0x0B1C, stored at 0x0AA4E-0x0AA52), so every negative value on range indexes 0 to 3 became zero | A negative AC+DC value is not forced to zero: the AC+DC dead-band compares a magnitude, the sign-forcing low-range stage is AC V and AC A only, and AC+DC is sqrtf(dc*dc + ac*ac), which cannot be negative | DONE (code reconcile 2026-10-23: src/cal/cal.c:577-583, :595; src/meas/meas.c:1787; vendor constants read from the firmware image, 0x0B04 = 0.025 and 0x0B1C = 0.0) |
| Number output, rounding and errors (#85) | A carry in `%e` leaves `e-0`; a left-justified `%e` is one character short; some tiny and huge values emit the text `ftoa error: ...` into the reply | Correct rounding, exponent and width; never error text in a reply | DONE (code audit 2026-10-23: src/fmt/fmt.c:312-327 , 324-326 and 480-522 , 380-447 , 21-31) |
| Float arithmetic (#86) | Results are 1-3 ulp low; overflow returns odd constants (`0x007FFFFF`, -1.17549e-38, +FLT_MAX for division by 0); no NaN/Inf; add clamps at 1.7e38 | IEEE binary32 round-to-nearest from avr-gcc; the vendor routines are not ported (`grep -rnE 'udivmod|__div|soft_?float|fp_div|long_div' src/` returns nothing). Where real IEEE produces NaN or infinity, finite_of() maps NaN to 0.0 and +-infinity to +-the largest finite float before any reply. Last-digit differences in readings occur | DONE (code reconcile 2026-10-23: src/fmt/fmt.c:206-221) |
| dB, dBm and power math (#87) | `pow`, `log10` and `sqrt` carry errors (`pow(10,2)` = 99.9999, `log10(10)` = 0.9999998) that reach dB/dBm results and displayed digits | `pow` is an exact table, fmt_pow10f (src/fmt/fmt.c:329-350). `log10f` and `sqrtf` are avr-libm's (src/hal/keys.c:265; src/meas/meas.c:1179, :1192, :1787) | UNVERIFIED: the error bound of avr-libm log10f/sqrtf -- check: run both on an AVR simulator over the dB/dBm and AC+DC input ranges against Python's decimal module and record the maximum error in ulp. The math.h of both toolchains (avr-gcc 7.3 and 14.1) only declare the functions and state no bound. The `pow` part is DONE |
| 32-bit divide (#88) | Loses a carry for divisors of 2^31 and above | Correct divide: the vendor routine is not ported and avr-gcc's own divide is used (the grep in the row above returns nothing) | DONE (code reconcile 2026-10-23: structural; the toolchain is Makefile:17-18) |
| Relay settle time (#102) | Settling is counted in ADC conversions (5 or 3), so it is shortest at the fast rate | Settling is a duration: the vendor count at 20 ms with floors of 100 ms (DC V, AC V) and 60 ms (others), converted to conversions at the line frequency (src/meas/meas.c:112-117, :436-468). The relay's own settling time is unmeasured. HARDWARE (ORIGINAL_BUGS #102), bench check: step a known input across a range change and note the settling time; scope PB4 to confirm the 20.0 ms /DRDY period | HARDWARE |

### G.5 Boot loader (XMODEM download over the serial port)

| Area | Vendor behaviour (bug) | New behaviour | Status |
|---|---|---|---|
| Download limits (#96, #97, #99) | No address limit (an image over 0x1F800 bytes overwrites the loader); no image check or verify; a partial last page is padded with stale data | Never writes at or above 0x1F800; every page is read back and compared; the first block must carry a JMP to a plausible application address before any flash is touched; the last page is padded with 0xFF; an interrupted or failed download leaves a flag in EEPROM (`EE_BOOT_XFER`) that keeps the loader from starting the application until a download completes. Whole-image check: see the next row | FIXED: per-frame CRC16, page read-back, reset-vector check, and the whole-image CRC below (src/boot/boot.c, `image_finish`, `transfer`) |
| Whole-image CRC (#96) | XMODEM carries no image length and no checksum, and an early EOT is a legal end of transfer, so a host that died mid-file or a truncated file passed every per-frame and per-page check and was started | The image sent is the application bytes, 0xFF-padded to a multiple of 128, followed by ONE trailer block (below). The loader holds the latest frame back by one, so at EOT the held frame is the trailer and is never written to flash. At EOT it requires at least two frames, magic `0x42 0x4B`, a non-zero length no greater than 0x1F800 and no greater than the application bytes received; writes the last page; computes CRC16-XMODEM over flash bytes [0, length) and compares it with the trailer. A mismatch ends the transfer with `FAIL C` (a page read-back failure is `FAIL V`, a missing or malformed trailer `FAIL I`). `EE_BOOT_XFER` is cleared and the application started only after the CRC matches; on any failure the flag stays set. The file a host sends is `app_xmodem.bin`, produced by `build.py` and released as `5491B-rewrite-app-xmodem.bin`; an image without the trailer is refused. The trailer's CRC is big-endian, the byte order of the frame CRC. Trailer: offset 0 = 0x42; 1 = 0x4B; 2-5 = application length, uint32 little-endian, excluding trailer and padding; 6 = CRC high byte; 7 = CRC low byte; 8-127 = 0xFF | FIXED (src/boot/boot.c `image_finish`, `transfer`; build.py `write_xmodem_image`) |
| XMODEM receive (#98) | No timeout after the first SOH; after a NAK one byte is discarded and the rest is mis-parsed | A per-byte and per-block timeout; resynchronises on SOH | DONE (code audit 2026-10-23: src/boot/boot.c:183-194,385-392,399-429,441-454,178-181) |
| Boot link (#100) | 2 stop bits at 39474 baud (+2.8%); a banner "Type d download" that does not match the real trigger; debug echoes `o`, `@`, `n` | 1 stop bit at 38461.5 baud (U2X, UBRR1 = 38: 12 MHz / (8 x 39) = 38461.5, +0.16 %); banner text matching the real trigger (both arrow keys at power-up, then ENTER); no debug echoes. A downloader should use 38400 baud, 8 data bits, no parity, and may use 1 or 2 stop bits (the receiver checks only the first). The loader no longer sends `o`, `@` or `n` when a key is pressed; a host must not wait for them. It still sends `C` about every 1.36 s, ACK/NAK, `OK` CR LF at hand-off, and `FAIL x` after CAN CAN on error | DONE in source (src/boot/boot.c:62-65, :132-138, :117, :268-275). The boot loader can be changed only by ISP flashing |

Trailer block (the last 128-byte XMODEM frame of `app_xmodem.bin`):

| Offset | Size | Contents |
|---|---|---|
| 0 | 1 | 0x42 |
| 1 | 1 | 0x4B |
| 2-5 | 4 | application length in bytes, uint32 little-endian, excluding the trailer and the 0xFF padding |
| 6 | 1 | CRC16-XMODEM over application bytes [0, length), high byte |
| 7 | 1 | the same CRC, low byte |
| 8-127 | 120 | 0xFF |

### G.6 Rows not added in section G

Already listed in sections A to F: #1, #2, #9, #15, #19, #20, #24, #27, #31, #32, #34-#39, #44-#46, #48,
#55, #58-#60, #78, #82-#84, #90, #92, #101, #106.
Not client-visible (panel only, internal or information): #5, #11, #33, #49, #51, #64, #80, #81, #91,
#103, #105.

Checked: all 105 ORIGINAL_BUGS rows reviewed; 63 added here.

## H. Number formatting and number parsing

Every float in every reply goes through this module, so these are the changes a client sees most often. The
LAYOUT is unchanged: field order, widths, separators, the `!` terminators, the trailing-zero trim of the
mantissa, and the exponent style -- `e` or `E`, a `-` only when negative, never a `+`, never zero padded.
What changed is that the digits are now correct.

Evidence: the rules were recovered from the printf float conversion at 0x14D14 and executed against the real
flash image. Of the 16 `%.8e` reference values executed on the original, 14 are byte-for-byte
identical in the port. The module is checked on the development machine against an exact oracle built from
Python's decimal module: 900207 cases, 0 failures (`host/check_fmt.py`).

| Area | Vendor behaviour | New behaviour | Status |
|---|---|---|---|
| Small values in `%e` replies (`FETC?`, `REF?`, `:SCAL:STD`) | Digits came from a 24-bit fixed-point fraction and were truncated, so `1e-5` replied `9.9e-6` and `0.0012345` replied `1.2344e-3` | Correctly rounded digits of the stored value: `1.0e-5` and `1.23449997e-3` | FIXED (#7 class) |
| Large and small magnitudes | `\|x\| >= 2^31` or `0 < \|x\| < 1.19e-7` made the formatter fail, and the reply then contained the words `ftoa error: number too big` or `... too small` where a number belonged | The whole binary32 range formats. A reply can no longer contain words in a numeric field | FIXED |
| Zero in `%e`, `%E`, `%g` | Printed NOTHING -- the code read the byte before its own output buffer -- so the field was empty | `0.0e0` for the `e` forms, `0` for `%g` | FIXED |
| Exponent after a rounding carry | A mantissa carry from 9.99.. to 1.0 left the exponent text as `-0`, e.g. `1.00e-0` | `1.00e0` | FIXED |
| Left-justified `%e` field | The field was silently one character shorter than the requested width | The width is honoured | FIXED |
| `'0'` flag with a negative value | Zeros were padded in front of the minus sign | The sign comes first | FIXED |
| `%f` of 0 at precision 7 or more | Printed the `ftoa error: number too small` text | `0.0000000` | FIXED |
| Panel fixed-point values | Rounded by adding half of the last place, in truncating float arithmetic, so 1.005 showed as 1 and 2.675 as 2.67 at two decimals | Rounded half away from zero on the exact value, which is the same rule without the truncation error | FIXED |
| Rounding convention | Half away from zero | Unchanged, deliberately. It differs from a standard C printf only on exact ties such as 0.03125 at four decimals, where this firmware gives `0.0313` and a PC printf gives `0.0312` | KEPT |
| Numeric parameter of `0` | Rejected with error (9,10) | Accepted wherever the parameter's range allows it | FIXED |
| Out-of-range exponent on input | `1e39` was silently stored as 1.1755e-38, which could become a calibration coefficient | A data error. Within about two ulp of FLT_MAX, saturation and overflow are indistinguishable and an error is reported; the original returned nonsense for anything that large | FIXED (R-A3) |
| `1e60000` on input | Ran a multiply loop once per unit of exponent, about a second of dead time | Rejected immediately | FIXED |
| Hexadecimal, `inf`, `nan` on input | Returned 0.0, which the caller then reported as a zero-parameter error (9,10) | A syntax error, which is the same refusal with the right reason | FIXED |
| Long numeric token on input | 20 or more characters overflowed a 20-byte stack buffer | Tokens up to 32 characters; longer is an error | FIXED |
| Integer parameters | Parsed by a 16-bit `atoi`, so `99999` silently became 34463 | 32-bit, with a range error instead of a wrap | FIXED |
| Input accuracy | 1 to 3 ulp out | Within 1.5 ulp | IMPROVED |
| Accepted number grammar | `[space]* [sign] digits [ . digits ] [ (e\|E) [sign] digits ]`, with `1e`, `1e+` and `1e-` meaning exponent 0 | The same, except that an exponent marker needs at least one digit: `1e`, `1e+` and `1e-` are a syntax error, so a mistyped parameter is not taken as a different value (#21) | FIXED (#21; src/fmt/fmt.c:683-687) |
| Text after a number | Ignored, so `"1,5"` was read as 1, `"12abc"` as 12 and `"1 5"` as 1 -- a mistyped parameter was silently accepted as a different value | A suffix error. The original had no unit-suffix table, so no legitimate parameter is refused | FIXED (#21) |
| Parameter magnitude | `scpi_parse_float_checked` 0x07D10 enforced an inclusive `-1e8 .. 1e8` with error (5,4); the port had lost that check and accepted up to 3.4e38 | The check is restored at the commands that had it | FIXED (port defect) |

## I. The serial link and the frequency counter

| Area | Vendor behaviour | New behaviour | Status |
|---|---|---|---|
| Input during a command error | The error routine showed its message for 1000 ms in a busy wait with interrupts DISABLED, so the 2-byte UART FIFO overflowed and everything sent during that second was lost. This is the ~1 s input lockout measured on the real instrument | The receive interrupt only echoes and enqueues into a 256-byte ring; the line is executed in the main loop. Nothing is lost | FIXED |
| Input during a calibration sweep | Same, for tens of seconds | Same fix: the ring keeps filling throughout | FIXED |
| Pipelined commands | A byte arriving while a completed line was still waiting to run was DISCARDED (RAM 0x0F62 bit 1), so a host that sent two commands back to back lost the second | The second line queues. A host may pipeline up to 256 bytes | FIXED |
| A byte with a framing or parity error | Read with the error flags unexamined and appended to the command line as data, so a line corrupted on the wire became a command, or an unexplained syntax error | Discarded, and reported through the error queue so the cause is visible | FIXED |
| 2400, 1200 and 600 baud | Only the low byte of the divisor was written (UBRR1H forced to 0), so the three lowest rates actually ran at about 13393, 6637 and 3319 baud -- unusable, and silently so | Correct rates. The full 12-bit divisor is written | FIXED (#89) |
| 38400 baud | Divisor 18, truncated from 18.53, giving 39474 baud: +2.8 %, outside the usual 2 % tolerance, so the fastest rate was unreliable rather than merely imprecise | 38461.5 baud, +0.16 %, using the double-speed mode. The same accuracy as every other rate | FIXED (#90) |
| 19200 / 9600 / 4800 baud | +0.16 % | Unchanged, deliberately. These are the rates that work today and that the vendor software is known to talk to; the divisor is still truncated rather than rounded, because changing a working rate by one count has no upside | KEPT |
| Reply pacing (#8, #93) | One terminator byte per reply, and a 1 ms pause after each transmitted character | The terminator rule is unchanged (one byte, LF or CR). The pause is gone: replies and echo go through a 128-byte transmit ring emptied by the UDRE interrupt, so the main loop does not wait for the wire. | FIXED (#8, #93; src/hal/uart1.c:116-250) |
| Echo | Applied inside the receive interrupt, immediately, including for CR and LF | Still queued by the receive interrupt, in order, including CR and LF, ahead of any reply to that line; the UDRE interrupt sends it. When the transmit ring is full the echo of that byte is dropped instead of stalling the receiver | KEPT (queued instead of written directly) |
| `CR LF` as a line ending | Produced one command and then an EMPTY command, because an empty line still counted as a line | ONE command. The CR ends the line; the LF after it is an empty line, which is ignored and lights no REM (see section K, #6). The earlier "Unchanged" entry did not match the code | FIXED (#6; src/hal/uart1.c:344-363) |
| A line of 256 characters or more | The 256th byte was dropped and the truncated line executed | A line of up to 255 characters runs. When a 256th character arrives the line is refused whole: an error is queued, nothing runs, and the rest of the line up to its terminator is discarded. The earlier "Unchanged" entry did not match the code (ORIGINAL_BUGS #7) | FIXED (#7; src/hal/uart1.c:364-375, src/remote/exec.c:995-1000) |
| Frequency and period readings | The counter weighted each 16-bit timer overflow as 65535 instead of 65536, losing one count per overflow. Over the 1.005 s slow gate that made the measured interval 15.25 ppm short, so **every reading was about 15 ppm high**. Above roughly 65 kHz the error partly cancels | Correct weight, so about **15 ppm lower and correct**. A deliberate, approved change | FIXED |
| Waiting for the frequency counter | The wait for the final capture was bounded by 3000 software loop iterations, so how long the instrument waited depended on the compiler and on what else the loop was doing | A real timeout, counted in 5.4613 ms timer overflows | FIXED |
| `:PERiod?` with no input | Computed 1 / f with no guard. A zero frequency returned +FLT_MAX in the original's arithmetic, which printed as a believable large number instead of an error | No reading, reported as such | FIXED |
| Internal timing (not visible on the wire) | The instrument had NO periodic interrupt: five of 35 vectors are populated and Timer0 is unused, so the key debounce, the buzzer pulse and the 1000 ms message were busy-waits or loop counts | A 1 ms Timer0 tick drives them, so each is a real duration. An addition the original did not have, recorded here | ADDED |

## K. The empty-line rule (decision 2026-10-22)

| Area | Vendor behaviour | New behaviour | Status |
|---|---|---|---|
| A bare CR or LF | Counted as a line: an empty command ran and the remote annunciator lit. `CR LF` therefore produced one command and then an extra empty one | Ignored: no command runs and REM is not lit. `CR LF` counts as ONE terminator, because the LF after the CR is itself an empty line. A deliberate blank line is ignored too | FIXED (#6) |

An empty command produced no reply and no error, which could be taken for harmless behaviour. The decision is FIX.

## L. Front-panel menu texts (2026-10-23)

Not a change to the remote protocol. Recorded here because `tools/check_tables.py` verifies the vendor's flash
tables byte for byte, and the menu strings stop matching them on purpose. Every text with its cell count is in
`MENU_TEXTS.md` (docs/).

| Area | Vendor behaviour | New behaviour | Status |
|---|---|---|---|
| Page, item and submenu titles | `A:MATH MEU`, `B:TRIG MEU`, `C:SYS  MEU`; items numbered `1:BEEP STA` ... `7:ZERO CAL`; abbreviated (`BAUD RAT`, `TX TERM`, `KEY SONG`) | `MATH MENU`, `TRIG MENU`, `SYS MENU`; items by name (`BEEPER`, `BAUD RATE`, `LINE END`, `KEY SOUND` ...); no number or letter prefix anywhere | CHANGED |
| "MEU" | The vendor's text for MENU | `MENU` | FIXED |
| Item 4 | `4:RETURN` (the echo setting) | `ECHO` | CHANGED |
| Titles longer than a line | Not present: the vendor's fit nine cells by sharing the `:` | A name takes both lines where it does not fit one: `RELAY` / `EXERCISER`, `RELAY` / `POWER ON`, `CANNOT` / `UNDO` | ADDED |
| Editors | A prompt and a padded value on one line (`HI:`, `R.F:`, `RDGS:`, `RANG: 5k&`) | The value on the main line, the name of the setting on the second | CHANGED |
| Choice values | `IMM`, `MAN`, `CR`, `LF`, `RANG:...` | `IMMEDIATE`, `MANUAL`, `RETURN`, `LINE FEED`, the range alone | CHANGED |
| Title blink | The first character (the page letter or item number) blinked | Titles are static; a prompt that waits for ENTER blinks as a whole | CHANGED |
| `menu_text[]` | 95 vendor strings, flash 0x0BCC-0x0EE6 | 58. 37 strings that nothing shows were dropped (old prompts, label copies kept by display.c, `GOING`, `CLEAR`, `004`, `001`) | CHANGED |

`check_tables.py` keeps walking the original's 95 strings in order. A kept string is compared with the image as
before; each changed one is a row with the vendor's text beside ours (`MENU_TEXT_CHANGED`); each dropped one is a
row with its image position and the text that must still be there (`MENU_TEXT_DROPPED`). A string that is none
of the three fails.
