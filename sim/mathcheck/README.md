# mathcheck: measured error of avr-libc 2.2.0 log10f and sqrtf (ORIGINAL_BUGS #87)

Toolchain: avr-gcc 14.1.0, avr-libc 2.2.0, `-mmcu=atmega128 -Os`, simavr (under WSL; `SIMAVR_DIR` names the simavr source tree).
Reference: Python `decimal`, 60 digits, from the exact binary32 input, rounded to nearest-even binary32.

## Call sites and input ranges
| Call | Where | Argument | Range used |
|---|---|---|---|
| log10f (dB) | src/meas/meas.c:1179 | abs(c / db_ref) | 1e-9 .. 1e7: c 1e-6 .. 1000 V; db_ref 1e-4 .. 1000 (ui/menu.c:88-89, remote/handlers.c:50-51) |
| log10f (dBm) | src/meas/meas.c:1192 | c*c / dbm_ref / 0.001f | 1e-13 .. 1e9: c 1e-6 .. 1000 V; dbm_ref 1 .. 9999 ohm (src/state.h:93, ui/menu.c:93) |
| log10f (REL ref) | src/hal/keys.c:265, src/remote/handlers.c:1159, :1166 | same two formulas | same two ranges |
| sqrtf (AC+DC) | src/meas/meas.c:1787 | dc*dc + ac*ac | 1e-12 .. 2e6 (each part 1e-6 .. 1000) |

Measured table covers log10f over 1e-13 .. 1e9 (a superset of both log ranges).

## Inputs
`gen_inputs.py` writes `inputs.h`: log10f 2100 inputs = 2000 logarithmically spaced 1e-13 .. 1e9, plus edges (1e-13, 1e9,
1e-9, 1e7, 1e-6, 1e3, 1e-4, every power of ten 1e-13 .. 1e9, 0.5, 2, 0.1, 1e-3, 1.0001, 0.9999, 1 +- 2^-k, 1 +- 1..8 ulp, 1.0).
sqrtf 2042 inputs = 2000 logarithmically spaced 1e-12 .. 2e6, plus edges (0, 1e-12, 2e6, powers of ten, perfect squares, 1 +- 1 ulp).
The tables are in PROGMEM; `analyze.py` checks that the inputs echoed by the AVR equal the table.

## Rebuild and rerun
1. `python -I gen_inputs.py`
2. `avr-gcc.exe -mmcu=atmega128 -Os -std=gnu11 -Wall -o mathcheck.elf mathcheck.c -lm`
   (avr-gcc 14.1.0; on Linux the command is `avr-gcc`)
3. In WSL or Linux: `SIMAVR_DIR=<simavr source tree> bash sim/mathcheck/run.sh`
   (builds `harness.c` against libsimavr, runs the ELF until the 0xDEADBEEF trailer on UART0, writes `out.bin`; 33140 bytes, 9,468,598 cycles)
4. `python -I analyze.py` (prints the results below; copy in `analyze_output.txt`)

## Results (2100 + 2042 inputs)
| | log10f | sqrtf |
|---|---|---|
| inputs equal to correctly rounded result | 1342 | 2042 |
| off by 1 ulp | 742 | 0 |
| off by 2 ulp | 15 | 0 |
| off by 3 ulp | 1 | 0 |
| max abs ulp distance to correctly rounded | 3, at x = 0.7523816823959351 (got -0.1235618069767952, correct -0.1235617846250534) | 0 (all 2042 correctly rounded) |
| max error vs exact value, in ulp | 2.762, same x | 0.5 (rounding of the exact value) |
| max absolute error | 1.091457e-6 in log10, at x = 1.1508812791660716e-9 (result -8.93897) | n/a |
| max relative error | 1.665672e-7, at x = 0.7523816823959351 | 5.960464e-8 (= 2^-24) |

In the firmware's terms (error of the library call alone; the multiplication by 20 or 10, the division c/ref and the
display rounding are not included):
- dB = 20 * log10f: max absolute error 2.182913e-5 dB.
- dBm = 10 * log10f: max absolute error 1.091457e-5 dB.
- AC+DC = sqrtf(dc^2 + ac^2): max relative error 5.960464e-8 (the result is correctly rounded binary32 for all 2042 inputs).
