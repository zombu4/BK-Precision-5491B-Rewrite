/* version.h -- the firmware version string, in one place.
 *
 * Two subsystems need it: the `*IDN?`
 * reply built by serv/thid.c, and the SYS menu's REVISION page. The second of those is ORIGINAL_BUGS #92:
 * the original's REVISION page shows the model name and the constant "001" and never the version at all,
 * so there was no existing constant to reuse -- which is why it had to be invented rather than ported.
 *
 * FORMAT. The vendor's string is "Ver<major>.<minor>.<yy>.<mm>.<dd>" -- a vendor unit reads
 * Ver1.4.14.06.18; an earlier vendor release read Ver1.2.13.11.20. This firmware's string is "Ver<X>.<Y>.<Z>.<yy>.<mm>.<dd>": the release tag
 * vX.Y.Z followed by the build date, for example Ver2.0.2.26.10.24. It starts with "Ver2." for every 2.x
 * release, so a client can tell it from the vendor's 1.x string, and the tag tells two releases made on
 * one day apart (v2.0.0 and v2.0.1 both reported Ver2.0.26.10.23 under the earlier date-only form).
 *
 * HOW THE STRING GETS SET. `build.py --release --release-tag vX.Y.Z` stamps
 * `FW_VERSION` from the tag and the build date and passes it as a -D, so a release never depends on anyone
 * remembering to edit this file; CI passes the git tag. A release build without --release-tag refuses to
 * run. The constant below is only the value of an ordinary development build, and the build prints which of
 * the two it used so that a version string is never a surprise.
 *
 * LENGTH. The tag allows up to three digits per component, so the longest string is
 * "Ver255.255.255.26.10.24", 23 characters. Every buffer that holds it is sized from sizeof FW_VERSION
 * (thid.c *IDN? line, handlers.c :SYSTem:INFO? field). The SYS REVISION page is limited to the nine display
 * cells (ui/menu.c render_version()).
 *
 * A hand-edited version is the failure this is avoiding: the number that tells a bench which firmware is
 * in front of them is exactly the number nobody notices is stale. The major number says 2 only to be
 * unmistakably not the vendor's 1.x -- an instrument reporting a vendor version while running this
 * firmware would be a trap for whoever reads it next. */
#ifndef BK_VERSION_H
#define BK_VERSION_H

#ifndef FW_VERSION
#define FW_VERSION "Ver2.0.0.00.00.00" /* development build; --release --release-tag stamps tag and date */
#endif

/* The model name in `*IDN?` comes from the EEPROM identity block, not from here: it is per-meter data and
 * one unit may legitimately differ from another. See eeprom_map.h EE_ID_MODEL. */

#endif /* BK_VERSION_H */
