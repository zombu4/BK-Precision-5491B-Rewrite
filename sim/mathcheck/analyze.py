"""Compares out.bin (AVR results) with exact decimal references. Run: python -I analyze.py"""
import struct, pathlib
from decimal import Decimal, getcontext
getcontext().prec = 60
HERE = pathlib.Path(__file__).parent
N_LOG, N_SQRT = 2100, 2042

def f32(b): return struct.unpack("<f", struct.pack("<I", b))[0]

def round_f32(d):
    """Correctly rounded (nearest-even) binary32 of Decimal d; returns (value as Decimal, ulp as Decimal, bits)."""
    if d == 0: return Decimal(0), Decimal(0), 0
    neg = d < 0
    a = -d if neg else d
    e = 0
    two = Decimal(2)
    while a >= two ** (e + 1): e += 1
    while a < two ** e: e -= 1
    q = two ** (e - 23)
    n = (a / q).to_integral_value(rounding="ROUND_HALF_EVEN")
    if n == 2 ** 24: n //= 2; e += 1; q = two ** (e - 23)
    val = n * q
    bits = ((e + 127) << 23) | (int(n) - 2 ** 23)
    return (-val if neg else val), q, bits | (0x80000000 if neg else 0)

def dec(x): return Decimal(x)   # exact: Decimal(float) is exact

data = (HERE / "out.bin").read_bytes()
assert struct.unpack_from("<I", data, len(data) - 4)[0] == 0xDEADBEEF
recs = [struct.unpack_from("<II", data, 8 * i) for i in range((len(data) - 4) // 8)]
assert len(recs) == N_LOG + N_SQRT
# inputs must match the generator's table
import re
hdr = (HERE / "inputs.h").read_text()
tabs = {n: [int(h, 16) for h in re.findall(r"0x([0-9A-F]{8})UL", t)] for n, t in
        re.findall(r"in_(log|sqrt)\[.*?\{(.*?)\};", hdr, re.S)}
assert [r[0] for r in recs[:N_LOG]] == tabs["log"] and [r[0] for r in recs[N_LOG:]] == tabs["sqrt"]

def study(name, rs, ref):
    worst_ulp = (0, None); worst_real = (Decimal(0), None); worst_abs = (Decimal(0), None); worst_rel = (Decimal(0), None)
    hist = {}
    exact_ct = 0
    for xb, rb in rs:
        x = dec(f32(xb)); got = f32(rb)
        ex = ref(x)
        rv, q, rbits = round_f32(ex)
        g = dec(got)
        if ex == 0:
            ulp_int = 0 if g == 0 else None
            if ulp_int is None: raise SystemExit(f"{name}: exact 0 but got {got}")
            real = Decimal(0)
        else:
            ulp_int = int(((g - rv) / q).to_integral_value())   # integer ulp distance to the correctly rounded value
            real = (g - ex) / q                                  # error against the exact value, in ulp of the rounded result
        hist[abs(ulp_int)] = hist.get(abs(ulp_int), 0) + 1
        if abs(ulp_int) > worst_ulp[0] or worst_ulp[1] is None: worst_ulp = (abs(ulp_int), (f32(xb), got, float(rv)))
        if abs(real) > worst_real[0]: worst_real = (abs(real), (f32(xb), got, float(rv)))
        ab = abs(g - ex)
        if ab > worst_abs[0]: worst_abs = (ab, (f32(xb), got))
        if ex != 0:
            rel = ab / abs(ex)
            if rel > worst_rel[0]: worst_rel = (rel, (f32(xb), got))
    return dict(n=len(rs), hist=dict(sorted(hist.items())), ulp=worst_ulp, real=worst_real, abs=worst_abs, rel=worst_rel)

L = study("log10f", recs[:N_LOG], lambda x: x.log10())
S = study("sqrtf", recs[N_LOG:], lambda x: x.sqrt())
for n, r in (("log10f", L), ("sqrtf", S)):
    print(n, "inputs", r["n"])
    print("  |ulp| distance histogram (value: count):", r["hist"])
    print("  max |ulp| to correctly rounded:", r["ulp"][0], "at x=%r got=%r correct=%r" % r["ulp"][1])
    print("  max |err| vs exact in ulp:", format(r["real"][0], ".4f"), "at x=%r got=%r correct=%r" % r["real"][1])
    print("  max abs err:", format(r["abs"][0], ".6e"), "at x=%r got=%r" % r["abs"][1])
    print("  max rel err:", format(r["rel"][0], ".6e"), "at x=%r got=%r" % r["rel"][1])
print("dB  (20*log10)  max abs error dB :", format(20 * L["abs"][0], ".6e"))
print("dBm (10*log10)  max abs error dB :", format(10 * L["abs"][0], ".6e"))
print("AC+DC sqrt      max rel error    :", format(S["rel"][0], ".6e"))
