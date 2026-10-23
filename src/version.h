/* version.h -- the firmware version string, in one place.
 *
 * Two subsystems need it: the `*IDN?`
 * reply built by serv/thid.c, and the SYS menu's REVISION page. The second of those is ORIGINAL_BUGS #92:
 * the original's REVISION page shows the model name and the constant "001" and never the version at all,
 * so there was no existing constant to reuse -- which is why it had to be invented rather than ported.
 *
 * FORMAT. The vendor's string is "Ver<major>.<minor>.<yy>.<mm>.<dd>" -- a vendor unit reads
 * Ver1.4.14.06.18. The shape is kept so that a client parsing `*IDN?` does not have to learn
 * a second format, and because the date in it is genuinely useful on a bench.
 *
 * HOW THE DATE GETS SET. `build.py --release` stamps `FW_VERSION` from the build date
 * and passes it as a -D, so a release never depends on anyone remembering to edit this file. The constant
 * below is only the fallback for an ordinary development build, and the build prints which of the two it
 * used so that a version string is never a surprise.
 *
 * A hand-edited version is the failure this is avoiding: the number that tells a bench which firmware is
 * in front of them is exactly the number nobody notices is stale. The major number says 2 only to be
 * unmistakably not the vendor's 1.x -- an instrument reporting a vendor version while running this
 * firmware would be a trap for whoever reads it next. */
#ifndef BK_VERSION_H
#define BK_VERSION_H

#ifndef FW_VERSION
#define FW_VERSION "Ver2.0.00.00.00" /* development build; --release stamps the real date */
#endif

/* The model name in `*IDN?` comes from the EEPROM identity block, not from here: it is per-meter data and
 * one unit may legitimately differ from another. See eeprom_map.h EE_ID_MODEL. */

#endif /* BK_VERSION_H */
