# EEPROM test fixture

`tests/fixtures/eeprom_sanitized.bin` is a 4096-byte image of the ATmega128A EEPROM of a real, calibrated
5491B. The host tests (`host/test_eeprom.c`) and the simavr harness (`sim/run_sim.sh`) use it as the state of
an existing meter: the firmware must leave every byte of such an image alone at boot, apart from the
power-up counter.

One thing was changed before it was committed. The meter's serial number, the 9-character text stored in the
identity block at EEPROM 0x0867 (`EE_ID_SERIAL`), was replaced by `000000000`. The image holds no other copy
of it. Every other byte, including all calibration records, user zeros, the `:THID` block and the settings,
is unchanged.
