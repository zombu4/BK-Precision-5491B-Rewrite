# Third-party notices

The firmware images are built with avr-gcc 14.1.0 and AVR-LibC 2.2.0, and contain parts of both.

| Component | Version | License | Where |
|---|---|---|---|
| AVR-LibC | 2.2.0 | Modified BSD | `LICENSES/avr-libc-2.2.0-LICENSE.txt` |
| GCC runtime library (libgcc) | 14.1.0 | GPL-3.0 with the GCC Runtime Library Exception | https://www.gnu.org/licenses/gcc-exception-3.1.html |

The GCC Runtime Library Exception allows the compiled firmware to be distributed under the project's own license.

The AVR-LibC license requires its copyright notice, conditions and disclaimer to be reproduced with any binary
distribution. Every release therefore includes `LICENSES/avr-libc-2.2.0-LICENSE.txt` next to the firmware files.
