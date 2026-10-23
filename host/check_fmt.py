#!/usr/bin/env python3
"""Verify src/fmt/fmt.c against an independent exact oracle.

Not part of the firmware image. It builds host/check_fmt.c with the host compiler, feeds it requests, and
compares every answer with a model written here from Python's decimal module, which represents a float
exactly. The two implementations share no code: fmt.c produces digits with a base-10000 big integer, the
model asks decimal to quantize.

The layout rules the model applies were recovered from the original's formatter at 0x14D14 and executed against
the real flash image. The digits are the correctly rounded ones, which
is the deliberate fix; the layout around them is the original's.

Usage:  python -I check_fmt.py [--cases N]
Exit status 0 means every case matched.
"""

import argparse
import math
import random
import struct
import shutil
import subprocess
import sys
from decimal import Decimal, ROUND_HALF_UP, getcontext, localcontext
from pathlib import Path

getcontext().prec = 400

HERE = Path(__file__).resolve().parent
SRC = HERE / "check_fmt.c"
FLT_MAX = 3.4028234663852886e38
SCI_MAX = 9  # significant digits in the e and g forms
DECIMALS_MAX = 20

# ---------------------------------------------------------------------------- oracle


def bits(x: float) -> int:
    return struct.unpack("<I", struct.pack("<f", x))[0]


def unbits(u: int) -> float:
    return struct.unpack("<f", struct.pack("<I", u & 0xFFFFFFFF))[0]


def f32(x: float) -> float:
    """Round a Python double to binary32, which is what the firmware works in."""
    try:
        return unbits(bits(x))
    except OverflowError:
        return math.copysign(math.inf, x)


def substitute(x: float):
    """fmt.c maps the values the original's arithmetic could not make onto ones it could."""
    if math.isnan(x):
        return 0.0
    if math.isinf(x):
        return math.copysign(FLT_MAX, x)
    return f32(x)


def sig_digits(x: float, sig: int):
    """`sig` significant digits of |x|, rounded half away from zero, with the decimal exponent.

    Returns (digit string of length sig, exp10, negative). An exact zero gives all zeros and exp10 0.
    """
    negative = math.copysign(1.0, x) < 0.0
    d = Decimal(x)
    if d == 0:
        return "0" * sig, 0, False
    a = abs(d)
    exp10 = a.adjusted()
    r = a
    for _ in range(2):  # a round up can carry into the next decade; settle it
        q = Decimal(1).scaleb(exp10 - sig + 1)
        with localcontext() as ctx:
            ctx.prec = 400
            r = a.quantize(q, rounding=ROUND_HALF_UP)
        if r.adjusted() == exp10:
            break
        exp10 = r.adjusted()
    text = "".join(c for c in format(r, "f") if c.isdigit()).lstrip("0")
    text = (text + "0" * sig)[:sig]
    return text, exp10, negative


def oracle_sci(x: float, decimals: int, echar: str) -> str:
    decimals = min(decimals, SCI_MAX - 1)
    x = substitute(x)
    digits, exp10, negative = sig_digits(x, decimals + 1)
    zero = Decimal(x) == 0

    out = "-" if negative and not zero else ""
    out += digits[0]
    if decimals > 0:
        frac = digits[1 : decimals + 1]
        from_int = min(max(exp10, 0), decimals)
        kept = len(frac)
        while kept > from_int and kept > 1 and frac[kept - 1] == "0":
            kept -= 1
        out += "." + frac[:kept]
    e = 0 if zero else exp10
    out += echar + ("-" + str(-e) if e < 0 else str(e))
    return out


def oracle_fixed(x: float, width: int, decimals: int) -> str:
    decimals = min(decimals, DECIMALS_MAX)
    x = substitute(x)
    d = Decimal(x)
    negative = math.copysign(1.0, x) < 0.0
    show_sign = negative and d != 0

    with localcontext() as ctx:
        ctx.prec = 400
        r = abs(d).quantize(Decimal(1).scaleb(-decimals), rounding=ROUND_HALF_UP)
    body = format(r, "f")
    if decimals > 0 and "." not in body:
        body += "." + "0" * decimals
    if decimals == 0:
        body += "."

    sign = "-" if show_sign else ""
    text = sign + body
    return text.rjust(width) if len(text) < width else text


def ten_pow(n: int) -> float:
    """What fmt_pow10f returns. The reciprocal is computed in binary32, as the C does."""
    n = max(-38, min(38, n))
    if n >= 0:
        return f32(float(f"1e{n}"))
    return f32(1.0 / f32(float(f"1e{-n}")))


def oracle_parse_float(text: str):
    """Returns (status, value) or (status, None) when no value is defined.

    Status codes match fmt_status_t: 0 OK, 1 SYNTAX, 2 RANGE, 3 ZERO (never), 4 OVERFLOW.
    """
    i, n = 0, len(text)
    while i < n and (text[i] == " " or "\t" <= text[i] <= "\r"):
        i += 1
    neg = False
    if i < n and text[i] in "+-":
        neg = text[i] == "-"
        i += 1
    mant = ""
    mant_digits = 0
    while i < n and text[i].isdigit():
        mant_digits += 1
        if mant_digits > 32:
            return 4, None
        mant += text[i]
        i += 1
    frac = ""
    if i < n and text[i] == ".":
        i += 1
        while i < n and text[i].isdigit():
            mant_digits += 1
            if mant_digits > 32:
                return 4, None
            frac += text[i]
            i += 1
    if not mant and not frac:
        return 1, None
    exponent = 0
    if i < n and text[i] in "eE":
        j = i + 1
        exp_neg = False
        if j < n and text[j] in "+-":
            exp_neg = text[j] == "-"
            j += 1
        ds = ""
        while j < n and text[j].isdigit():
            ds += text[j]
            j += 1
        if ds:
            if len(ds) >= 5:
                if not exp_neg:
                    return 2, None
                return 0, 0.0
            exponent = -int(ds) if exp_neg else int(ds)
            i = j
        else:
            # ORIGINAL_BUGS #21, fixed at src/fmt/fmt.c:683-687: an exponent marker with no digits after it
            # ("1e", "1e+", "1e-") is a syntax error. The oracle used to skip the marker and the sign and
            # carry on, which made it agree with the defect rather than with the fix.
            return 1, None
    value = Decimal(mant + frac or "0").scaleb(exponent - len(frac))
    if value == 0:
        return 0, 0.0
    as_float = f32(float(value)) if abs(value) < Decimal("1e400") else math.inf
    # Within a couple of ulp of FLT_MAX the module cannot tell saturation from overflow: it scales the
    # nine digits with two binary32 operations, and one ulp there is 2e31 while the gap between FLT_MAX
    # and the overflow threshold is only 1e31. Both answers are accepted, and fmt.c states the same limit.
    # An error is the safe answer of the two, and the original produced nonsense for anything this large.
    near_max = Decimal(FLT_MAX) * Decimal("0.99999996") <= abs(value) <= Decimal(FLT_MAX) * Decimal("1.00000004")
    if near_max:
        return -1, None
    if math.isinf(as_float):
        return 2, None
    return 0, math.copysign(as_float, -1.0 if neg else 1.0)


def oracle_parse_long(text: str):
    i, n = 0, len(text)
    while i < n and (text[i] == " " or "\t" <= text[i] <= "\r"):
        i += 1
    neg = False
    if i < n and text[i] in "+-":
        neg = text[i] == "-"
        i += 1
    ds = ""
    while i < n and text[i].isdigit():
        ds += text[i]
        if len(ds) > 32:
            return 4, None
        i += 1
    if not ds:
        return 1, None
    v = int(ds)
    v = -v if neg else v
    if v < -(2**31) or v > 2**31 - 1:
        return 2, None
    return 0, v


# ---------------------------------------------------------------------------- cases


def float_cases(count: int):
    fixed = [
        0.0, -0.0, 1.0, -1.0, 0.5, 0.25, 1.5, 2.0, 3.0, 7.0, 10.0, 100.0, 1000.0, 1e6, 1e7, 1e9,
        12.3456, 123.456, 1234.5678, 0.0012345, -0.0001, 1e-5, 9.99995, 0.1, 0.3, 0.9999999,
        1.005, 2.675, 0.03125, 123456789.0, 2147483520.0, 2147483648.0, 4294967296.0,
        16777215.0, 16777216.0, 16777217.0, FLT_MAX, -FLT_MAX, 1.1754944e-38, 1e-45, -1e-45,
        88.7228, -87.3365, 1e-7, -1e-7, 1e38, -1e38, 0.001, 1e-4, 1e-3, 999.995, 99.9995, 9.99995,
        float("inf"), float("-inf"), float("nan"),
    ]
    out = list(fixed)
    rng = random.Random(0x5491B)
    while len(out) < count:
        kind = rng.randrange(4)
        if kind == 0:
            out.append(unbits(rng.randrange(1 << 32)))
        elif kind == 1:  # the meter's own range
            out.append(rng.uniform(-1200.0, 1200.0))
        elif kind == 2:
            out.append(rng.uniform(-1.0, 1.0) * 10.0 ** rng.randrange(-9, 10))
        else:
            out.append(float(rng.randrange(-2_000_000, 2_000_000)) / 10.0 ** rng.randrange(0, 6))
    return out


def text_cases():
    base = [
        "0", "1", "-1", "+1", "1.0", ".5", "5.", "-.5", "0.1", "3.14159265358979", "1234.5678",
        "1e3", "1E3", "1e+3", "1e-3", "1e", "1e+", "1e-", "1.2.3", "12abc", "1,5", "1 5", "1_000",
        "1e3.5", "--1", "+-1", "-+1", ".", "-", "+", "", " ", "   42", "\t7", "0x10", "inf", "nan",
        "1e39", "1e38", "9e38", "3.4028235e38", "1e-39", "1e-38", "1e-45", "1e-46", "1e60000",
        "1e-60000", "99999", "2147483647", "2147483648", "-2147483648", "-2147483649", "4294967295",
        "00000000000000000000000000000000001", "123456789012345678901234567890", "-0", "-0.0",
        "0.000000000000000000000000000000000000000001", "1.000000000000000000000000000001",
    ]
    return base


# ---------------------------------------------------------------------------- driver


def _wsl_path(p: Path) -> str:
    """A Windows path as WSL sees it: C:\\x\\y -> /mnt/c/x/y."""
    s = str(Path(p).resolve())
    return "/mnt/" + s[0].lower() + s[2:].replace("\\", "/")


def build(exe: Path) -> list[str]:
    """Builds the oracle's counterpart and returns the argv that runs it.

    A native compiler is preferred. Failing that, WSL's gcc: this workstation has avr-gcc, which cannot
    produce something that runs here, and a clang with no C library headers behind it, so WSL is the only
    host compiler actually available. It compiles the same source through /mnt/c.
    """
    for cc in ("clang", "gcc", "cc"):
        try:
            subprocess.run(
                [cc, "-std=c17", "-O1", "-Wall", "-Wextra", "-o", str(exe), str(SRC)],
                check=True,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError:
            continue
        except subprocess.CalledProcessError as exc:
            sys.stderr.write(f"{cc} did not build it:\n{exc.stdout}{exc.stderr}\n")
            continue
        print(f"built with {cc}")
        return [str(exe)]

    if shutil.which("wsl") is not None:
        wsl_exe = _wsl_path(exe.with_suffix(".bin"))
        cmd = (f"gcc -std=c17 -O1 -Wall -Wextra -o {wsl_exe} {_wsl_path(SRC)}")
        r = subprocess.run(["wsl", "-e", "bash", "-lc", cmd], capture_output=True, text=True)
        if r.returncode != 0:
            sys.stderr.write(f"wsl gcc failed:\n{r.stdout}{r.stderr}\n")
            raise SystemExit(2)
        print("built with wsl gcc")
        return ["wsl", "-e", wsl_exe]

    raise SystemExit("no host C compiler found (tried clang, gcc, cc, then WSL gcc)")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", type=int, default=4000)
    args = ap.parse_args()

    exe = HERE / ("check_fmt.exe" if sys.platform == "win32" else "check_fmt.bin")
    argv = build(exe)

    requests = []
    expected = []

    for x in float_cases(args.cases):
        u = bits(x)
        for dec in (0, 1, 2, 4, 6, 8, 12):
            requests.append(f"E {u:08x} {dec}")
            expected.append(("|" + oracle_sci(x, dec, "e") + "|", f"fmt_float_e_prec({x!r}, {dec})"))
        requests.append(f"X {u:08x} 8")
        expected.append(("|" + oracle_sci(x, 8, "E") + "|", f"fmt_float_E_prec({x!r}, 8)"))
        for width, dec in ((0, 4), (7, 4), (7, 0), (12, 6), (0, 1)):
            requests.append(f"F {u:08x} {width} {dec}")
            expected.append(("|" + oracle_fixed(x, width, dec) + "|",
                             f"fmt_fixed({x!r}, {width}, {dec})"))

    for v in (0, 1, -1, 9, 10, -10, 99999, -99999, 2147483647, -2147483648, 1000000, -7):
        requests.append(f"I {v}")
        expected.append((f"|{v}|", f"fmt_int({v})"))

    for n in range(-38, 39):
        requests.append(f"W {n}")
        expected.append((f"{bits(ten_pow(n)):08x}", f"fmt_pow10f({n})"))

    for text in text_cases():
        status, value = oracle_parse_float(text)
        requests.append(f"P {text}")
        expected.append((("float", status, value), f"fmt_parse_float({text!r})"))
        status, ivalue = oracle_parse_long(text)
        requests.append(f"L {text}")
        expected.append((("long", status, ivalue), f"fmt_parse_long({text!r})"))

    proc = subprocess.run(
        argv, input="\n".join(requests) + "\n", capture_output=True, text=True
    )
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr)
        return 2
    answers = proc.stdout.splitlines()
    if len(answers) != len(requests):
        print(f"FAIL: {len(requests)} requests but {len(answers)} answers")
        return 2

    failures = []
    for req, ans, (want, what) in zip(requests, answers, expected):
        if isinstance(want, tuple):
            kind, status, value = want
            parts = ans.split()
            got_status = int(parts[0])
            if status == -1:  # either answer is acceptable; see oracle_parse_float
                if got_status not in (0, 2):
                    failures.append(f"{what}: status {got_status}, expected 0 or 2")
                continue
            if got_status != status:
                failures.append(f"{what}: status {got_status}, expected {status}")
                continue
            if status != 0 or value is None:
                continue
            if kind == "long":
                if int(parts[1]) != value:
                    failures.append(f"{what}: {parts[1]}, expected {value}")
            else:
                got = unbits(int(parts[1], 16))
                if got != value and not within_ulp(got, value, 2):
                    failures.append(f"{what}: {got!r}, expected {value!r}")
        elif ans != want:
            failures.append(f"{what}: {ans}, expected {want}   [{req}]")

    print(f"{len(requests)} cases, {len(failures)} failures")
    for line in failures[:40]:
        print("  " + line)
    if len(failures) > 40:
        print(f"  ... and {len(failures) - 40} more")
    return 1 if failures else 0


def within_ulp(a: float, b: float, n: int) -> bool:
    """The input path is specified to 1.5 ulp, not to exactness, so parsing is compared with a tolerance."""
    if a == b:
        return True
    if math.isnan(a) or math.isnan(b) or math.copysign(1, a) != math.copysign(1, b):
        return False
    return abs(bits(abs(a)) - bits(abs(b))) <= n


if __name__ == "__main__":
    raise SystemExit(main())
