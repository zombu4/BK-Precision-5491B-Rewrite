"""Source hygiene for the 5491B firmware sources: the failure classes that have actually bitten this port.

Run with `python -I tools/check_hygiene.py`. It is not a lint and has no opinions about style. Each check is
here because the corresponding mistake was made in this project and cost real time:

 1. Non-ASCII bytes. Two CJK characters were once typed into a source file by mistake.
 2. Control bytes other than CR, LF and TAB. A shell heredoc wrote a literal NUL byte into a character
    literal in parser.c; it still compiled to the right value and was unreadable rubbish in the file.
 3. A block-comment opener inside a block comment. state.h once held a path like data/<star>.txt inside a
    comment, and under -Wcomment -Werror that broke the build of EVERY file including it -- with the error
    pointing at the header rather than at the mistake.
 4. An unterminated block comment, which has the same blast radius.
 5. A missing final newline.
 6. CRLF line endings. The tree is LF: the firmware is built by avr-gcc, 24 of the 41 files always were,
    and a mixed tree makes every diff noisy and hides the real change. It drifted when edit tools
    detected CRLF in one file and rejoined with it, converting files that had been LF. A check is cheaper
    than remembering.

Exits 0 when clean and 1 when not, so it can gate a build.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
problems = []
files = sorted(list(ROOT.rglob("*.c")) + list(ROOT.rglob("*.h")))

for f in files:
    raw = f.read_bytes()
    rel = f.relative_to(ROOT).as_posix()

    for i, b in enumerate(raw):
        if b > 0x7F:
            problems.append(f"{rel}:{raw[:i].count(bytes([10])) + 1}: non-ASCII byte 0x{b:02X}")
            break

    for i, b in enumerate(raw):
        if b < 0x20 and b not in (0x09, 0x0A, 0x0D):
            problems.append(f"{rel}:{raw[:i].count(bytes([10])) + 1}: control byte 0x{b:02X}")
            break

    text = raw.decode("ascii", errors="replace")

    depth = 0
    for m in re.finditer(r"/\*|\*/", text):
        if m.group(0) == "/*":
            if depth > 0:
                line = text[: m.start()].count("\n") + 1
                problems.append(f"{rel}:{line}: comment opener inside a block comment (-Wcomment)")
                break
            depth = 1
        else:
            depth = 0

    if depth != 0:
        problems.append(f"{rel}: unterminated block comment")

    if raw and not raw.endswith(b"\n"):
        problems.append(f"{rel}: no newline at end of file")

    if b"\r\n" in raw:
        problems.append(f"{rel}: CRLF line endings ({raw.count(bytes([13, 10]))} lines); this tree is LF")

print(f"checked {len(files)} source files")
if problems:
    print(f"{len(problems)} PROBLEMS:")
    for p in problems:
        print("  " + p)
else:
    print("clean: no non-ASCII, no stray control bytes, no nested or unterminated comments")

raise SystemExit(1 if problems else 0)
