#!/bin/bash
# Rebuild and run the harness against mathcheck.elf.
# Usage:  SIMAVR_DIR=<simavr source tree, built> bash sim/mathcheck/run.sh
# Needs the AVR image mathcheck.elf built first (see README.md, step 2).
set -e
: "${SIMAVR_DIR:?set SIMAVR_DIR to the simavr source directory (holding sim/ and obj-*/libsimavr.a)}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OBJ="${MATHCHECK_OBJ_DIR:-${TMPDIR:-/tmp}/mathcheck-build}"; mkdir -p "$OBJ"
LIBSIMAVR="$(ls -d "$SIMAVR_DIR"/obj-*/libsimavr.a 2>/dev/null | head -n 1)"
if [ -z "$LIBSIMAVR" ]; then
    echo "no libsimavr.a under $SIMAVR_DIR/obj-*/ -- build simavr first" >&2
    exit 2
fi
gcc -std=gnu11 -O2 -Wall -I"$SIMAVR_DIR/sim" -I"$SIMAVR_DIR/sim/avr" "$HERE/harness.c" -o "$OBJ/harness" \
    "$LIBSIMAVR" -lelf -lpthread -lm
"$OBJ/harness" "$HERE/mathcheck.elf" "$HERE/out.bin"
