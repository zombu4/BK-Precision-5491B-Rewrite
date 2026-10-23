# Makefile -- the 5491B replacement firmware.
#
# Two separate programs, as on the real instrument: the application at address 0, and the boot loader in
# the boot section the fuses select. They do not share objects and the boot loader must not call into the
# application's modules.
#
# The toolchain is the AVR GNU toolchain in the directory AVR_GCC_BIN names, or else on PATH. Release
# images come from avr-gcc 14; the sources also build with 7.3, which is why fmt.c never uses `double` (7.3
# has no -mdouble=64, so `double` is 32 bits and writing it would have meant something different from what
# it looks like). Code that compiles clean on both does not depend on which it got.
#
#   make            the application and the boot loader, then the size report
#   make size       the size report alone, against the budgets
#   make host       the host-side checks (the formatter oracle; see host/)
#   make clean

AVR_BIN  ?= $(AVR_GCC_BIN)
AVR_DIR   = $(if $(strip $(AVR_BIN)),$(strip $(AVR_BIN))/,)
CC        = $(AVR_DIR)avr-gcc
OBJCOPY   = $(AVR_DIR)avr-objcopy
SIZE      = $(AVR_DIR)avr-size

MCU       = atmega128
F_CPU     = 12000000UL

BUILD     = build

# -Werror is deliberate: this port's whole method is that a surprise is a defect, and a warning is the
# compiler telling us something surprised it. The conversion warnings are on because the firmware moves
# between uint8_t, uint16_t and float constantly and a silent narrowing there is a wrong reading.
WARN      = -Wall -Wextra -Werror -Wshadow -Wconversion -Wsign-conversion -Wundef -Wdouble-promotion \
            -Wstrict-prototypes -Wmissing-prototypes -Wredundant-decls -Wpointer-arith
CFLAGS    = -std=gnu11 -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) $(WARN) \
            -funsigned-char -funsigned-bitfields -fshort-enums \
            -ffunction-sections -fdata-sections -fno-common -g
LDFLAGS   = -mmcu=$(MCU) -Wl,--gc-sections -Wl,--relax -Wl,-Map=$(BUILD)/$(1).map

# The application: everything under src/ except the boot loader, which is its own program.
APP_SRC  := $(filter-out src/boot/%,$(wildcard src/*.c src/*/*.c))
APP_OBJ  := $(APP_SRC:%.c=$(BUILD)/%.o)

BOOT_SRC := $(wildcard src/boot/*.c)
BOOT_OBJ := $(BOOT_SRC:%.c=$(BUILD)/%.o)

# The boot section: 1024 words = 2048 bytes at the top of the 128 KB flash (the fuses select BOOTSZ for 1024
# words with BOOTRST set).
BOOT_START = 0x1F800

.PHONY: all size host clean
all: $(BUILD)/app.elf $(BUILD)/boot.elf size

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/app.elf: $(APP_OBJ)
	$(CC) -mmcu=$(MCU) -Wl,--gc-sections -Wl,--relax -Wl,-Map=$(BUILD)/app.map -o $@ $^
	$(OBJCOPY) -O ihex -R .eeprom $@ $(BUILD)/app.hex

$(BUILD)/boot.elf: $(BOOT_OBJ)
	$(CC) -mmcu=$(MCU) -Wl,--section-start=.text=$(BOOT_START) -Wl,-Map=$(BUILD)/boot.map -o $@ $^
	$(OBJCOPY) -O ihex -R .eeprom $@ $(BUILD)/boot.hex

# Budgets: flash <= 110000 of 129024 (leaving the boot section and headroom), and
# data + bss <= 2400 of 4096 so there is stack left for the deepest path -- which is the formatter's
# big-integer digit generator at about 200 bytes, called from the display path.
size: $(BUILD)/app.elf
	@echo "---- application ----"
	@$(SIZE) --mcu=$(MCU) -C $(BUILD)/app.elf
	@echo "---- boot loader ----"
	@$(SIZE) --mcu=$(MCU) -C $(BUILD)/boot.elf 2>/dev/null || true
	@echo "---- against the budgets ----"
	@$(SIZE) -A $(BUILD)/app.elf | awk '\
		/^\.text/   {t=$$2} /^\.data/ {d=$$2} /^\.bss/ {b=$$2} \
		END { printf "flash  %6d / 110000  %s\n", t, (t<=110000 ? "ok" : "OVER"); \
		      printf "ram    %6d /   2400  %s  (data %d + bss %d)\n", d+b, (d+b<=2400 ? "ok" : "OVER"), d, b }'

host:
	cd host && python -I check_fmt.py --cases 4000

clean:
	rm -rf $(BUILD)

# ---- Host tests ---------------------------------------------------------------------------------------
# These run the SHIPPED code on this machine, with host/shim/ standing in for the avr headers and the
# fake EEPROM in each test standing in for the chip. They are not a substitute for the AVR build; they
# answer questions the AVR build cannot, like "does this leave an existing meter's EEPROM untouched".
HOSTCC   ?= gcc
HOSTFLAGS = -std=gnu11 -O1 -g -Wall -Wextra -I host/shim

.PHONY: hosttests
hosttests: $(BUILD)/test_siphash $(BUILD)/test_eeprom $(BUILD)/test_menu_text
	@echo "---- SipHash vectors ----"
	@$(BUILD)/test_siphash
	@echo "---- boot-time EEPROM initialiser ----"
	@cd $(BUILD) && ./test_eeprom ../tests/fixtures/eeprom_sanitized.bin
	@echo "---- menu text cell counts ----"
	@$(BUILD)/test_menu_text

# Every menu, item, choice, prompt, message and editor line, in both blink phases, measured with the same
# cell rule the display prints with. It compiles the SHIPPED tables.c and menu.c rather than a copy of the
# strings, so it cannot drift out of date and pass for ever.
$(BUILD)/test_menu_text: host/test_menu_text.c
	@mkdir -p $(BUILD)
	$(HOSTCC) $(HOSTFLAGS) -o $@ $<

$(BUILD)/test_siphash: host/test_siphash.c
	@mkdir -p $(BUILD)
	$(HOSTCC) $(HOSTFLAGS) -o $@ $<

$(BUILD)/test_eeprom: host/test_eeprom.c
	@mkdir -p $(BUILD)
	$(HOSTCC) $(HOSTFLAGS) -o $@ $<
