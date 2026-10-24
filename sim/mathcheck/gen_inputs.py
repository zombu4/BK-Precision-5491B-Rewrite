"""Generates inputs.h (binary32 input tables for mathcheck.c). Run: python -I gen_inputs.py"""
import struct, math, sys, pathlib

def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]

def bits(x):
    return struct.unpack("<I", struct.pack("<f", x))[0]

def from_bits(b):
    return struct.unpack("<f", struct.pack("<I", b))[0]

def next_after(x, up):
    b = bits(x)
    return from_bits(b + 1 if up else b - 1)

def log_inputs():
    xs = [f32(10.0 ** (-13.0 + 22.0 * i / 1999.0)) for i in range(2000)]   # 1e-13 .. 1e9
    ed = [1e-13, 1e9, 1e-9, 1e7, 1e-6, 1e3, 1e-4, 0.5, 2.0, 0.1, 0.001, 1.0001, 0.9999]
    ed += [10.0 ** k for k in range(-13, 10)]
    ed += [1.0 + 2.0 ** -k for k in range(1, 24)] + [1.0 - 2.0 ** -k for k in range(1, 25)]
    ed += [next_after(1.0, True)] * 0
    b1 = 1.0
    for k in range(1, 9):
        ed.append(from_bits(bits(1.0) + k))
        ed.append(from_bits(bits(1.0) - k))
    ed.append(1.0)
    return xs + [f32(e) for e in ed]

def sqrt_inputs():
    lo, hi = -12.0, math.log10(2.0e6)
    xs = [f32(10.0 ** (lo + (hi - lo) * i / 1999.0)) for i in range(2000)]
    ed = [0.0, 1e-12, 2e6, 1.0, 4.0, 2.0, 0.25, 1e-6, 1e6, 1e3, 1e-3, 1e-9]
    ed += [10.0 ** k for k in range(-12, 7)]
    ed += [float(k * k) for k in (1, 2, 3, 5, 7, 10, 100, 1000, 1414)]
    ed += [from_bits(bits(1.0) + 1), from_bits(bits(1.0) - 1)]
    return xs + [f32(e) for e in ed]

def main():
    lg, sq = log_inputs(), sqrt_inputs()
    out = pathlib.Path(__file__).with_name("inputs.h")
    with open(out, "w", newline="\n") as f:
        f.write("/* written by gen_inputs.py */\n#include <stdint.h>\n#include <avr/pgmspace.h>\n")
        for name, xs in (("LOG", lg), ("SQRT", sq)):
            f.write(f"#define N_{name} {len(xs)}u\n")
            f.write(f"static const uint32_t in_{name.lower()}[N_{name}] PROGMEM = {{\n")
            for i in range(0, len(xs), 6):
                f.write("  " + ",".join(f"0x{bits(x):08X}UL" for x in xs[i:i+6]) + ",\n")
            f.write("};\n")
    print(len(lg), len(sq))

main()
