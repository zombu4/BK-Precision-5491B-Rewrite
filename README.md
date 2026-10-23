# BK-Precision-5491B-Rewrite

A complete, hand-written replacement firmware for the B&K Precision **5491B** 50000-count bench multimeter
(ATmega128A main controller). It keeps the original remote-command reply layout, so existing scripts keep
working, and fixes the defects found in the original firmware.

**Source-available, not open source.** See [LICENSE](LICENSE).

> Not affiliated with, endorsed by or sponsored by B&K Precision. All product names and trademarks belong to
> their owners. Replacing the firmware of an instrument can erase its calibration or leave it unusable until it
> is reprogrammed. **You flash it entirely at your own risk.**

## What is different from the original firmware

- **Original defects fixed** -- the list, with what changed: [docs/ORIGINAL_BUGS.md](docs/ORIGINAL_BUGS.md).
- **Remote interface changes** (new commands, protected commands, fixed replies):
  [docs/PROTOCOL_CHANGES.md](docs/PROTOCOL_CHANGES.md). Calibration, identity and memory-clearing commands are
  refused until unlocked with the 4-digit service code (default `6666`), on the front panel and over the
  remote port. The vendor's calibration software therefore cannot calibrate a meter running this firmware.
- **Front-panel texts** rewritten so every line fits the 9 character cells, spelled out, without the `A:` /
  `1:` prefixes: [docs/MENU_TEXTS.md](docs/MENU_TEXTS.md).
- New SYS settings, among them **LINE 50/60 Hz** (the converter integrates over one mains cycle; default 60 Hz)
  and a relay exerciser.
- The calibration data in EEPROM is read in the same format as the original, so an existing calibration keeps
  working.

## Status

Runs on one 5491B, the developer's unit, with that unit's original calibration data; bench checks of the
front panel and the remote interface on it are ongoing. A simulator suite (24 tests, `sim/`, including the boot
loader) runs on every build. No other 5491B has been tested.

## Release files

Each release (GitHub **Releases**) is built by CI from the tagged source:

| File | Use |
|---|---|
| `5491B-rewrite-full.hex` | application + this project's boot loader, for ISP flashing (the normal way) |
| `5491B-rewrite-full.bin` | the same image as raw bytes (131072 bytes) |
| `5491B-rewrite-app.hex` | application only |
| `5491B-rewrite-app-xmodem.bin` | the application plus a 128-byte trailer (length and CRC-16), for updating over the serial port with this project's boot loader (see below) |
| `SHA256SUMS` | hashes of the four images |

## Flashing with an ISP programmer

You need an AVR ISP programmer (for example a USBasp) connected to the meter's 6-pin ISP header (X101 on the
unit tested), and [avrdude](https://github.com/avrdudes/avrdude). The meter is powered from its own supply
during programming; do not let the programmer power it. Nothing may be connected to the input jacks.

In the commands below `AVRDUDE` stands for `avrdude -c usbasp -p m128a` (use your programmer's `-c` name).

**1. Back up everything first.** Keep these files; they are the only way back to the original firmware and
your calibration.

```
AVRDUDE -n -U flash:r:backup_flash.bin:r -U eeprom:r:backup_eeprom.bin:r -U hfuse:r:hfuse.hex:h -U lfuse:r:lfuse.hex:h -U efuse:r:efuse.hex:h -U lock:r:lock.hex:h
```

Read it twice and compare the two copies before going on.

**2. Check that the EEPROM survives the chip erase.** Writing flash erases the whole chip. The EEPROM, which
holds the calibration, survives only when the EESAVE fuse is programmed: bit 3 of `hfuse` must be **0**. The
unit tested reads `hfuse = 0x14` (EESAVE programmed). If bit 3 is 1 on yours, the calibration is erased by step
3 and you must write `backup_eeprom.bin` back afterwards (step 4).

**3. Write and verify the firmware.**

```
AVRDUDE -U flash:w:5491B-rewrite-full.hex:i
AVRDUDE -U flash:v:5491B-rewrite-full.hex:i
```

If verification reports a mismatch, write it again; do not power the meter off with a failed image in it.

**4. Only if EESAVE was not programmed:** `AVRDUDE -U eeprom:w:backup_eeprom.bin:r`

Do not change the fuses. Disconnect the programmer and power-cycle the meter; `*IDN?` reports `Ver2.0...`.

**Back to the original firmware:** `AVRDUDE -U flash:w:backup_flash.bin:r` with your own backup.

## Updating over the serial port

Once this project's boot loader is in the meter (it is part of `5491B-rewrite-full.hex`), the application can be
replaced over the remote serial port (the USB port on the unit tested) without a programmer. The download itself
has so far been tested in the simulator only (tests T02-T07, T22-T24); on the meter, the banner and the start of
the application have been checked. Hold both horizontal arrow keys while switching the meter on;
the loader shows a banner and waits about 4 s for ENTER. After ENTER it sends `C` about every 1.36 s: start an
XMODEM-CRC transfer of `5491B-rewrite-app-xmodem.bin`. The loader's serial link runs at **38461.5 baud, 8 data
bits, no parity, 1 stop bit** (double speed, UBRR1 = 38); a host set to 38400 baud, 8N1 is within 0.16 % of it.

The loader refuses an image without the trailer, reads every page back after writing it, and starts the new
application only when the CRC of the flash contents matches the trailer. An interrupted or failed download leaves
a flag set, and the loader refuses to start the application while it is set. See
[docs/PROTOCOL_CHANGES.md](docs/PROTOCOL_CHANGES.md) (boot loader rows) for the byte-level details.

## Building

Requirements: avr-gcc 14 with AVR-LibC 2.2 (CI uses avr-gcc 14.1.0 from
[ZakKemble/avr-gcc-build](https://github.com/ZakKemble/avr-gcc-build)), Python 3, and a host C compiler for
the host tests.

```
export AVR_GCC_BIN=/path/to/avr-gcc/bin     # or put avr-gcc on PATH
python3 -I build.py image                   # application, boot loader, size gates, images in build/
python3 -I build.py hosttests               # host-side tests
SIMAVR_DIR=/path/to/simavr/simavr bash sim/run_sim.sh   # simulator suite
```

`--release` stamps the firmware version with the build date. The build fails on any compiler warning, on any
function nothing calls, and on exceeding the flash or RAM budget.

## Third-party components

Firmware images contain parts of AVR-LibC and the GCC runtime library; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
