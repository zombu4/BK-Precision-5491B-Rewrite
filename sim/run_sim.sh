#!/bin/bash
# Builds the simavr harness and runs all 25 tests. Exit status is non-zero if any test fails. Extra
# arguments go to the harness (for example --only 8).
#
# Prerequisites:
#   - the firmware is built:  python -I build.py   (writes build/app.elf and build/boot.elf)
#   - simavr is built, and SIMAVR_DIR names its source directory (the one holding sim/ and obj-*/libsimavr.a).
#     SIMAVR_DIR is required; there is no default.
#   - gcc, libelf and pthreads for the harness itself.
#
# Optional environment:
#   EEPROM_IMAGE  the EEPROM image the simulated meter starts with (default: tests/fixtures/eeprom_sanitized.bin)
#   BUILD_TREE    the build directory to simulate (default: build). Relative to the repository root, or an
#                 absolute directory holding app.elf and boot.elf -- which is how a deliberately broken copy of
#                 the firmware is run to show that a test can fail.
#   SIM_OBJ_DIR   where the harness binary is built (default: a directory under ${TMPDIR:-/tmp})
set -u

: "${SIMAVR_DIR:?set SIMAVR_DIR to the simavr source directory (holding sim/ and obj-*/libsimavr.a)}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$(dirname "$HERE")"
OBJ="${SIM_OBJ_DIR:-${TMPDIR:-/tmp}/sim5491b-build}"
EEPROM="${EEPROM_IMAGE:-$FW/tests/fixtures/eeprom_sanitized.bin}"

LIBSIMAVR="$(ls -d "$SIMAVR_DIR"/obj-*/libsimavr.a 2>/dev/null | head -n 1)"
if [ -z "$LIBSIMAVR" ]; then
    echo "no libsimavr.a under $SIMAVR_DIR/obj-*/ -- build simavr first" >&2
    exit 2
fi

mkdir -p "$OBJ"
gcc -std=gnu11 -O2 -Wall -Wextra -Wno-unused-parameter \
    -I"$SIMAVR_DIR/sim" -I"$SIMAVR_DIR/sim/avr" \
    "$HERE/sim_5491b.c" -o "$OBJ/sim_5491b" \
    "$LIBSIMAVR" -lelf -lpthread -lm || exit 2

TREE="${BUILD_TREE:-build}"
case "$TREE" in
    /*) TREE_DIR="$TREE" ;;
    *)  TREE_DIR="$FW/$TREE" ;;
esac
if [ ! -f "$TREE_DIR/app.elf" ]; then
    echo "no $TREE_DIR/app.elf -- build it first:  python -I build.py" >&2
    exit 2
fi
echo "simulating $TREE_DIR/ (set BUILD_TREE to change)"

# WHAT WAS ACTUALLY TESTED, by hash. A log that says "passed" is only worth something if it names the
# bytes it passed. These lines are the answer to "is this image the one that passed?" -- file times are a
# weak proxy (a rebuild that produces identical bytes, a file copied with its timestamp, or a clock that
# moves all change the answer without the binary changing). The hex is listed as well as the ELF, because
# the image tools package the HEX while the simulator loads the ELF.
echo "app.elf  sha256 $(sha256sum "$TREE_DIR/app.elf"  | cut -d' ' -f1)"
echo "boot.elf sha256 $(sha256sum "$TREE_DIR/boot.elf" | cut -d' ' -f1)"
echo "app.hex  sha256 $(sha256sum "$TREE_DIR/app.hex"  | cut -d' ' -f1)"

exec "$OBJ/sim_5491b" --app "$TREE_DIR/app.elf" --boot "$TREE_DIR/boot.elf" --eeprom "$EEPROM" "$@"
