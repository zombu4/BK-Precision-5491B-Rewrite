#!/usr/bin/env python3
"""Build driver for the 5491B replacement firmware -- the same thing the Makefile does, for machines
without `make`, plus the flashable image step. The Makefile stays the reference for the compile and link
flags; if they ever disagree the Makefile wins.

    python -I build.py            the application, the boot loader and the size report
    python -I build.py image      the same, then the flashable images (see below)
    python -I build.py size       the size report alone
    python -I build.py hosttests  the host-side tests
    python -I build.py clean

    --release                     stamp FW_VERSION with today's date (needs avr-gcc 14 or later)

Toolchain. The AVR GNU toolchain is found in the directory named by the environment variable AVR_GCC_BIN,
or else wherever `avr-gcc` is on PATH. Nothing about its location is built into this file.

The image step writes into build/:

    5491B-rewrite-full.hex   application plus this project's boot loader (at 0x1F800), merged, for ISP flashing
    5491B-rewrite-app.hex    the application alone, for ISP flashing without the boot loader
    5491B-rewrite-app-xmodem.bin
                             the file to send to the serial boot loader with XMODEM: the application,
                             padded to 128 bytes, plus a 128-byte trailer with its length and CRC-16
    5491B-rewrite-full.bin   the full image as raw bytes: 131072 bytes, unwritten flash is 0xFF
    SHA256SUMS               the SHA-256 of each of the four files above, in `sha256sum` format

No third-party image goes into any of them: they are made from the two ELF files this script links.
"""

import hashlib
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / "build"

EXE = ".exe" if os.name == "nt" else ""

AVR_BIN = None
CC = OBJCOPY = SIZE = None


def select_toolchain():
    """Locate the AVR toolchain and point the module globals at it. Returns an error text, or None."""
    global AVR_BIN, CC, OBJCOPY, SIZE
    env = os.environ.get("AVR_GCC_BIN")
    if env:
        AVR_BIN = Path(env)
    else:
        found = shutil.which("avr-gcc")
        if found is None:
            return "avr-gcc not found: set AVR_GCC_BIN to the toolchain's bin directory, or put avr-gcc on PATH"
        AVR_BIN = Path(found).parent
    CC = AVR_BIN / ("avr-gcc" + EXE)
    OBJCOPY = AVR_BIN / ("avr-objcopy" + EXE)
    SIZE = AVR_BIN / ("avr-size" + EXE)
    if not CC.exists():
        return f"avr-gcc not found at {CC}"
    return None


def gcc_major():
    """The major version of the selected avr-gcc."""
    r = subprocess.run([str(CC), "-dumpversion"], capture_output=True, text=True)
    m = re.match(r"(\d+)", r.stdout.strip())
    return int(m.group(1)) if m else 0


MCU = "atmega128"
F_CPU = "12000000UL"
BOOT_START = 0x1F800
FLASH_BYTES = 0x20000

FLASH_BUDGET = 110_000
RAM_BUDGET = 2_400

WARN = [
    "-Wall", "-Wextra", "-Werror", "-Wshadow", "-Wconversion", "-Wsign-conversion",
    "-Wundef", "-Wdouble-promotion", "-Wstrict-prototypes", "-Wmissing-prototypes",
    "-Wredundant-decls", "-Wpointer-arith",
]
CFLAGS = [
    "-std=gnu11", "-Os", f"-mmcu={MCU}", f"-DF_CPU={F_CPU}", *WARN,
    "-funsigned-char", "-funsigned-bitfields", "-fshort-enums",
    "-ffunction-sections", "-fdata-sections", "-fno-common", "-g",
]


def run(cmd, **kw):
    return subprocess.run([str(c) for c in cmd], cwd=ROOT, text=True,
                          capture_output=True, **kw)


def sources():
    app, boot = [], []
    for path in sorted(ROOT.glob("src/*.c")) + sorted(ROOT.glob("src/*/*.c")):
        rel = path.relative_to(ROOT).as_posix()
        (boot if rel.startswith("src/boot/") else app).append(rel)
    return app, boot


def compile_all(srcs):
    objs, failures = [], []
    for rel in srcs:
        obj = BUILD / (rel[:-2] + ".o")
        obj.parent.mkdir(parents=True, exist_ok=True)
        r = run([CC, *CFLAGS, "-c", "-o", obj, rel])
        if r.returncode != 0:
            failures.append((rel, (r.stderr or r.stdout).strip()))
        else:
            objs.append(obj)
    return objs, failures


# ---- The "called by nobody" gate --------------------------------------------------------------------
#
# A function can be written, compiled, linked, declared in a header, and called by nothing. That went
# unnoticed several times in this port, because the compiler, the linker, -Werror and the test suite never
# ask "did anyone call this?".
#
# The linker already knows. With -ffunction-sections every function is its own section and --gc-sections
# drops the unreachable ones; --print-gc-sections names each one as it goes. So the answer was being
# computed on every build and thrown away.
#
# The gate fails the build on any dropped function not on the allow-list below. The allow-list is meant to
# stay short and every entry carries its reason -- an allow-list nobody can read is a way of turning the
# gate off slowly.
GC_ALLOWED = {
    # avr-libc and gcc bring in their own unreferenced helpers; they are not ours and not our business.
    # Matched by prefix, see gc_is_allowed().
}
GC_ALLOWED_PREFIXES = (
    "__",       # compiler and libc internals (__vector_*, __udivmodsi4, ...)
    "_etext",
)


def gc_is_allowed(name):
    if name in GC_ALLOWED:
        return True
    return any(name.startswith(p) for p in GC_ALLOWED_PREFIXES)


def gc_dropped(stderr):
    """Function names from --print-gc-sections output.

    The linker prints lines like:
        .../ld: removing unused section '.text.foo' in file '.../foo.o'
    Only .text.* sections name a function; .rodata and friends are data the compiler chose not to emit a
    reference to, which is a different question and not one this gate asks.
    """
    names = []
    for line in stderr.splitlines():
        # Some linkers capitalise it ("Removing unused section"), so the match is case-insensitive. A gate
        # that cannot fail is the thing this gate exists to prevent.
        m = re.search(r"removing unused section '\.text\.([A-Za-z_][A-Za-z0-9_]*)'", line, re.IGNORECASE)
        if m:
            names.append(m.group(1))
    return sorted(set(names))


def link(objs, out, extra=()):
    r = run([CC, f"-mmcu={MCU}", "-Wl,--gc-sections", "-Wl,--relax",
             "-Wl,--print-gc-sections",
             f"-Wl,-Map={BUILD / (out + '.map')}", *extra,
             "-o", BUILD / (out + ".elf"), *objs])
    if r.returncode == 0:
        run([OBJCOPY, "-O", "ihex", "-R", ".eeprom",
             BUILD / (out + ".elf"), BUILD / (out + ".hex")])
    return r.returncode, (r.stderr or r.stdout).strip()


def sections(elf):
    r = run([SIZE, "-A", elf])
    out = {}
    for line in r.stdout.splitlines():
        m = re.match(r"^(\.\w+)\s+(\d+)", line.strip())
        if m:
            out[m.group(1)] = int(m.group(2))
    return out


def crc16_xmodem(data):
    """CRC-16/XMODEM (polynomial 0x1021, initial value 0, MSB first), as src/boot/boot.c computes it."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def write_xmodem_image():
    """Write <build>/app_xmodem.bin, the file a host sends to the boot loader.

    The application bytes, 0xFF-padded to a multiple of 128, followed by one 128-byte trailer block:
    0x42 0x4B, the application length (uint32 little-endian, without padding), the CRC16 of the unpadded
    application (big-endian, like the XMODEM frame CRC), then 0xFF. The loader refuses an image without it
    (PROTOCOL_CHANGES G.5). app.hex and app.elf are not changed.
    """
    raw = BUILD / "app.bin"
    r = run([OBJCOPY, "-O", "binary", "-R", ".eeprom", BUILD / "app.elf", raw])
    if r.returncode != 0:
        print("==== objcopy -O binary failed ====")
        print((r.stderr or r.stdout).strip())
        return 1
    app = raw.read_bytes()
    crc = crc16_xmodem(app)
    padded = app + b"\xFF" * (-len(app) % 128)
    trailer = (b"\x42\x4B" + len(app).to_bytes(4, "little") + crc.to_bytes(2, "big")).ljust(128, b"\xFF")
    out = BUILD / "app_xmodem.bin"
    out.write_bytes(padded + trailer)
    print(f"app_xmodem.bin: length {len(app)} bytes, CRC16 0x{crc:04X}, file size {out.stat().st_size} bytes")
    return 0


def report():
    """Print the size report against the budgets. Returns True when everything is within budget."""
    rows = []
    for name in ("app", "boot"):
        elf = BUILD / f"{name}.elf"
        if not elf.exists():
            continue
        s = sections(elf)
        text, data, bss = s.get(".text", 0), s.get(".data", 0), s.get(".bss", 0)
        rows.append((name, text, data, bss))
    within = True
    print("---- against the budgets ----")
    for name, text, data, bss in rows:
        if name == "app":
            flash_ok = text + data <= FLASH_BUDGET
            ram_ok = data + bss <= RAM_BUDGET
            print(f"flash  {text + data:6d} / {FLASH_BUDGET}  "
                  f"{'ok' if flash_ok else 'OVER'}   (.text {text} + .data {data})")
            print(f"ram    {data + bss:6d} / {RAM_BUDGET:6d}  "
                  f"{'ok' if ram_ok else 'OVER'}   (data {data} + bss {bss})")
            within = within and flash_ok and ram_ok
        else:
            boot_ok = text + data <= 2048
            print(f"boot   {text + data:6d} /   2048  {'ok' if boot_ok else 'OVER'}")
            within = within and boot_ok
    return within


# ---- Flashable images ---------------------------------------------------------------------------------

def read_ihex(path):
    """Intel HEX file -> {byte address: value}. Understands data, end-of-file, extended-address and start-address
    records."""
    data = {}
    base = 0
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"{path}:{number}: not an Intel HEX record")
        raw = bytes.fromhex(line[1:])
        count, offset, kind = raw[0], (raw[1] << 8) | raw[2], raw[3]
        payload = raw[4:4 + count]
        if len(raw) != count + 5 or sum(raw) & 0xFF:
            raise ValueError(f"{path}:{number}: bad length or checksum")
        if kind == 0x00:
            for i, b in enumerate(payload):
                data[base + offset + i] = b
        elif kind == 0x01:
            break
        elif kind == 0x02:
            base = ((payload[0] << 8) | payload[1]) << 4
        elif kind == 0x04:
            base = ((payload[0] << 8) | payload[1]) << 16
        elif kind in (0x03, 0x05):
            continue  # start address: where execution begins, not data
        else:
            raise ValueError(f"{path}:{number}: unsupported record type {kind:#04x}")
    return data


def ihex_record(kind, offset, payload=b""):
    body = bytes([len(payload), (offset >> 8) & 0xFF, offset & 0xFF, kind]) + bytes(payload)
    return ":" + (body + bytes([(-sum(body)) & 0xFF])).hex().upper()


def write_ihex(path, data):
    """{byte address: value} -> Intel HEX text, 16 data bytes per record, extended-address records as needed."""
    lines = []
    upper = None
    addr_list = sorted(data)
    i = 0
    while i < len(addr_list):
        start = addr_list[i]
        end = start
        while (end + 1) in data and end + 1 - start < 16 and ((end + 1) >> 16) == (start >> 16):
            end += 1
        if (start >> 16) != upper:
            upper = start >> 16
            lines.append(ihex_record(0x04, 0, bytes([upper >> 8, upper & 0xFF])))
        lines.append(ihex_record(0x00, start & 0xFFFF, bytes(data[a] for a in range(start, end + 1))))
        i += end - start + 1
    lines.append(ihex_record(0x01, 0))
    Path(path).write_text("\n".join(lines) + "\n", newline="")


def sha256_of(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def image():
    """Merge the application and this project's boot loader into the flashable images."""
    app = read_ihex(BUILD / "app.hex")
    boot = read_ihex(BUILD / "boot.hex")
    if not app or not boot:
        print("image: app.hex or boot.hex is empty")
        return 1
    if max(app) >= BOOT_START:
        print(f"image: the application reaches {max(app):#x}, into the boot section at {BOOT_START:#x}")
        return 1
    if min(boot) < BOOT_START or max(boot) >= FLASH_BYTES:
        print(f"image: the boot loader ({min(boot):#x}-{max(boot):#x}) is outside the boot section")
        return 1

    full = {**app, **boot}
    flat = bytearray(b"\xff" * FLASH_BYTES)
    for a, b in full.items():
        flat[a] = b

    names = {
        "full.hex": BUILD / "5491B-rewrite-full.hex",
        "app.hex": BUILD / "5491B-rewrite-app.hex",
        "app-xmodem.bin": BUILD / "5491B-rewrite-app-xmodem.bin",
        "full.bin": BUILD / "5491B-rewrite-full.bin",
    }
    write_ihex(names["full.hex"], full)
    write_ihex(names["app.hex"], app)
    names["app-xmodem.bin"].write_bytes((BUILD / "app_xmodem.bin").read_bytes())
    names["full.bin"].write_bytes(bytes(flat))
    sums = "".join(f"{sha256_of(p)}  {p.name}\n" for p in names.values())
    (BUILD / "SHA256SUMS").write_text(sums, newline="")

    print("---- images ----")
    for p in names.values():
        print(f"{p.name:26s} {p.stat().st_size:7d} bytes")
    print(f"application {min(app):#x}-{max(app):#x}, boot loader {min(boot):#x}-{max(boot):#x}")
    print("SHA256SUMS written")
    return 0


# ---- Host tests ---------------------------------------------------------------------------------------

# Run from the repository root, so the fixture path is relative to it.
FIXTURE = "tests/fixtures/eeprom_sanitized.bin"

HOST_TESTS = (
    ("test_siphash", []),
    # Every text the front panel can show fits its line. Reads the shipped tables.c and menu.c, so a text
    # added or lengthened tomorrow is measured without anyone remembering to list it.
    ("test_menu_text", []),
    # The boot-time EEPROM initialiser must leave an existing meter's image untouched. What it catches --
    # a boot-time initialiser overwriting data a meter in the field already holds -- depends on what the
    # image contains; the fixture is a real meter's image with the serial number blanked.
    ("test_eeprom", [FIXTURE]),
)

HOSTFLAGS = ["-std=gnu11", "-O1", "-g", "-Wall", "-Wextra", "-I", "host/shim"]


def hosttests():
    """The host tests compile the SHIPPED src/ files for this machine and run them against a fake EEPROM.

    The compiler is, in order: $HOSTCC; gcc or cc on PATH (not on Windows); WSL's gcc (Windows). Under WSL
    the same sources are built from the same paths through /mnt, so the only thing that differs from a
    native checkout is where the files happen to live.
    """
    names = sorted({name for name, _ in HOST_TESTS})
    hostcc = os.environ.get("HOSTCC")
    if hostcc is None and os.name != "nt":
        hostcc = shutil.which("gcc") or shutil.which("cc")
    if hostcc is not None:
        return hosttests_native(hostcc, names)
    return hosttests_wsl(names)


def hosttests_native(hostcc, names):
    (ROOT / "build").mkdir(exist_ok=True)
    rc = 0
    suffix = ".exe" if os.name == "nt" else ""
    for name in names:
        r = run([hostcc, *HOSTFLAGS, "-o", f"build/{name}{suffix}", f"host/{name}.c"])
        if r.returncode != 0:
            print(f"---- {name}: did not compile ----")
            print((r.stderr or r.stdout).strip())
            rc = 1
    for name, args in HOST_TESTS:
        print(f"---- {' '.join([name, *args])} ----")
        exe = ROOT / "build" / f"{name}{suffix}"
        if not exe.exists():
            print("NOT BUILT")
            rc = 1
            continue
        r = run([exe, *args])
        print((r.stdout or "").strip())
        if r.stderr.strip():
            print(r.stderr.strip())
        if r.returncode != 0:
            rc = 1
    return rc


def hosttests_wsl(names):
    wsl = shutil.which("wsl")
    if wsl is None:
        print("no host C compiler reachable (set HOSTCC, or install gcc, or WSL on Windows): host tests not run")
        return 1

    posix = str(ROOT).replace("\\", "/")
    cwd = "/mnt/" + posix[0].lower() + posix[2:] if posix[1:2] == ":" else posix
    flags = " ".join(HOSTFLAGS)
    script = [f"cd '{cwd}'", "mkdir -p build", "rc=0"]
    for name in names:
        script.append(f"rm -f build/{name}")
        script.append(f"gcc {flags} -o build/{name} host/{name}.c || rc=1")
    for name, args in HOST_TESTS:
        label = f"{name} {' '.join(args)}".strip()
        script.append(f'echo "---- {label} ----"')
        script.append(f'[ -x build/{name} ] && {{ ./build/{name} {" ".join(args)} || rc=1; }} '
                      f'|| {{ echo "NOT BUILT"; rc=1; }}')
    script.append("exit $rc")

    r = subprocess.run([wsl, "-e", "bash", "-lc", "; ".join(script)],
                       text=True, capture_output=True)
    print((r.stdout or "").strip())
    if r.stderr.strip():
        print(r.stderr.strip())
    return r.returncode


def release_version() -> str:
    """`Ver2.0.<yy>.<mm>.<dd>` from today's date, matching the vendor's shape (see src/version.h).

    Stamped by --release so that a released firmware never reports a version somebody forgot to edit. A
    development build keeps the 00.00.00 fallback in version.h, which is deliberately not a plausible date.
    """
    from datetime import date

    d = date.today()
    return f"Ver2.0.{d.year % 100:02d}.{d.month:02d}.{d.day:02d}"


def main():
    args = [a for a in sys.argv[1:]]
    release = "--release" in args
    args = [a for a in args if a != "--release"]
    what = args[0] if args else "all"

    if what == "hosttests":
        return hosttests()
    if what == "clean":
        shutil.rmtree(BUILD, ignore_errors=True)
        print("cleaned")
        return 0
    if what not in ("all", "image", "size"):
        print(f"unknown command {what!r}; choose all, image, size, hosttests or clean")
        return 2

    error = select_toolchain()
    if error:
        print(error)
        return 2
    first = run([CC, "--version"]).stdout.splitlines()
    print(f"toolchain: {first[0] if first else CC}  ->  {BUILD.name}/")

    if what == "size":
        return 0 if report() else 1

    if release:
        # avr-libc 2.0.0 (shipped with avr-gcc 7.3) aliases the float math functions to the double ones; the
        # release images come from avr-gcc 14 and its avr-libc 2.2.0, which declares them properly.
        if gcc_major() < 14:
            print("a release build needs avr-gcc 14 or later")
            return 2
        version = release_version()
        # The quotes are part of the macro VALUE, so the argument is -DFW_VERSION="Ver..." with the
        # double quotes inside it. No shell is involved -- subprocess is given argv directly -- so there is
        # nothing to escape them from, and escaping them would put backslashes into the version string.
        CFLAGS.append(f'-DFW_VERSION="{version}"')
        print(f"release build: FW_VERSION = {version}")
    else:
        print("development build: FW_VERSION is the fallback in src/version.h (--release stamps the date)")

    app_src, boot_src = sources()
    print(f"compiling {len(app_src)} application and {len(boot_src)} boot-loader files")

    app_obj, app_fail = compile_all(app_src)
    boot_obj, boot_fail = compile_all(boot_src)

    if app_fail or boot_fail:
        print(f"\n==== {len(app_fail) + len(boot_fail)} file(s) did not compile ====")
        for rel, err in app_fail + boot_fail:
            print(f"\n---- {rel} ----")
            print(err)
        return 1

    rc, err = link(app_obj, "app")
    if rc != 0:
        print("==== application did not link ====")
        print(err)
        return 1

    # The gate. `err` holds the linker's stderr, which is where --print-gc-sections writes.
    dropped = [n for n in gc_dropped(err) if not gc_is_allowed(n)]
    if dropped:
        print()
        print(f"==== {len(dropped)} function(s) are called by nobody ====")
        for n in dropped:
            print(f"  {n}")
        print()
        print("The linker garbage-collected these, which means nothing in the application reaches them.")
        print("Wire each one up or delete it. If one is deliberately unreferenced, add it to GC_ALLOWED in")
        print("build.py WITH THE REASON.")
        return 1
    print("called-by-nobody gate: clean")
    if write_xmodem_image() != 0:
        return 1
    rc, err = link(boot_obj, "boot",
                   extra=[f"-Wl,--section-start=.text={BOOT_START:#x}"])
    if rc != 0:
        print("==== boot loader did not link ====")
        print(err)
        return 1

    if not report():
        print("==== over budget ====")
        return 1
    if what == "image":
        return image()
    return 0


if __name__ == "__main__":
    sys.exit(main())
