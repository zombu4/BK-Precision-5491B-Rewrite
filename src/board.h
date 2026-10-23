/* board.h -- ATmega128A pin map and port initialisation for the BK 5491B main board.
 *
 * FROZEN FOUNDATION HEADER. Modules code against this; changing it changes every module.
 *
 * Source: the original's port initialisation at 0x1D02 and the per-pin roles read from the firmware.
 *
 * WHAT IS ESTABLISHED AND WHAT IS NOT. The port bit VALUES, and the ORDER the original writes them in,
 * are read directly from the disassembly at 0x1D02 and are reproduced exactly. Those
 * are facts, and they are the only thing the firmware depends on: it replays the original's writes.
 *
 * The ELECTRICAL ROLE of each pin -- what the relay or analogue switch on the other end of it does -- is
 * a different matter. Every role below tagged
 *
 *     UNVERIFIED: this pin's electrical role -- check: trace the net on the board, or obtain a schematic
 *
 * comes from which bits change together across the function and range table, and from no document.
 * There is no 5491B schematic available. The names are there so the code can be read; nothing decides
 * anything from them, so a wrong name is a wrong comment and not a wrong instrument. Each is tagged
 * [UNVERIFIED] on its own line.
 */
#ifndef BK_BOARD_H
#define BK_BOARD_H

#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

#define F_CPU_HZ 12000000UL

/* ---- What the original's software delay actually cost ----------------------------------------------
 * Amendment 2026-10-22. The original has one microsecond delay routine, `delay_us` at 0x3578, and
 * disassembly establishes its exact cost: a body of 13 + 15n cycles, plus 5 cycles of call setup (6 at
 * the two `call` sites rather than `rcall`). At 12 MHz one unit is therefore 1.25 us and the fixed
 * overhead is 1.5 us.
 *
 * This lives here, not in a driver, because it is a property of the ORIGINAL FIRMWARE that more than one
 * driver has to reproduce: the converter bus (0x3458, 0x34BE, 0x3500, 0xF2A0) and the display
 * controller's key-read gap (0x3774). Both must emit at least the gap the original emitted, because
 * neither part has a published figure we can check the shorter gap against. Deriving both from one macro
 * means a correction is made once.
 *
 * Use the 5-cycle form: it is the LOWER bound of the original's cost, so a gap built from it can never
 * claim more margin than the original had.
 *
 * `delay_ms` at 0x3594 has no equivalent here. Its call sites are start-up, relay and display sequences
 * whose margins are millisecond-scale, and avr-libc's `_delay_ms` is accurate to within a cycle, so it is
 * used directly and is strictly better than the original.
 */
#define ORIG_DELAY_BODY_FIXED_CYCLES 13UL
#define ORIG_DELAY_CALL_SETUP_CYCLES 5UL
#define ORIG_DELAY_CYCLES_PER_UNIT 15UL
#define ORIG_DELAY_US_CYCLES(units)                                                                \
    (ORIG_DELAY_CALL_SETUP_CYCLES + ORIG_DELAY_BODY_FIXED_CYCLES +                                 \
     ORIG_DELAY_CYCLES_PER_UNIT * (unsigned long)(units))

/* ---- Reset-time port state (original writes these at 0x1D02) -------------------------------------- */
#define BOARD_DDRA 0xFFu
#define BOARD_PORTA 0x0Fu
#define BOARD_DDRB 0xE7u
#define BOARD_PORTB 0x9Fu
#define BOARD_DDRC 0xFBu
#define BOARD_PORTC 0x3Fu
#define BOARD_DDRD 0x2Fu
#define BOARD_PORTD 0xFFu
#define BOARD_DDRE 0xBFu
#define BOARD_PORTE 0x1Fu
#define BOARD_DDRF 0xFFu
#define BOARD_PORTF 0xFFu
#define BOARD_DDRG 0x1Fu
#define BOARD_PORTG 0x1Bu

/* ---- Analog front end: range/function switching bits ---------------------------------------------
 * Driven only through meas/frontend.c. No other module may touch these ports.
 */
#define FE_A_LADDER0 PA0 /* range ladder bit, pairs with FE_F_LADDER1 */
#define FE_A_HV_PATH PA1 /* [UNVERIFIED] high-voltage path, 1 in DCV ranges 0..3 */
#define FE_A_OHM_SRC_EN PA2 /* [UNVERIFIED] ohm/diode source enable */
#define FE_A_50M_RANGE PA3 /* [UNVERIFIED] 50 Mohm select, 1 only in OHM range 0 */
#define FE_A_MODE_I_OHM PA5 /* [UNVERIFIED] 1 for current/ohm/diode, 0 for ACV */
#define FE_A_VI_SELECT PA6 /* [UNVERIFIED] 1 for V and I, 0 for ohm/diode */
#define FE_A_ATTEN PA7 /* [UNVERIFIED] 1 for DCV 0..3, ACV, DCI, ACI; 0 for DCV 500 mV, ohm, diode */

#define FE_B_AC_PATH PB5 /* [UNVERIFIED] 1 for AC V/I and ohm 0..4/diode */
#define FE_B_OHM_SRC PB6 /* [UNVERIFIED] ohm/diode source; original only ever SETS this bit */
#define FE_B_ACV_ATTEN PB7 /* [UNVERIFIED] ACV high-range attenuator, 1 on 750/500 V */

#define FE_C_ACV_RANGE_HI PC5 /* [UNVERIFIED] 1 for ACV 750/500/50 V */
#define FE_C_ACV_RANGE_5V PC6 /* [UNVERIFIED] 1 for ACV 5 V */
#define FE_C_SHUNT_5MA PC7 /* [UNVERIFIED] 5 mA shunt select, with FE_E_SHUNT_5MA */

#define FE_D_OHM_PATH PD5 /* [UNVERIFIED] ohm path select */
#define FE_E_50M_PATH PE2 /* [UNVERIFIED] 50 Mohm path */
#define FE_E_SHUNT_MA PE3 /* [UNVERIFIED] 500 mA / 50 mA shunt select */
#define FE_E_SHUNT_5MA PE4 /* [UNVERIFIED] 5 mA shunt select */
#define FE_F_I_DC PF1 /* [UNVERIFIED] DC (1) vs AC (0) current path */
#define FE_F_SHUNT_20A PF2 /* [UNVERIFIED] 20 A / 5 A shunt select */
#define FE_F_LADDER1 PF3 /* [UNVERIFIED] range ladder bit, pairs with FE_A_LADDER0 */
#define FE_G_ACV_750 PG3 /* [UNVERIFIED] ACV 750 V stage */
#define FE_G_ACI_GAIN PG4 /* [UNVERIFIED] ACI gain bit */

/* ---- Buzzer (PA4): high = on ---------------------------------------------------------------------- */
#define BUZZ_PORT PORTA
#define BUZZ_BIT PA4

/* ---- Measurement chip bus (bit-banged on PORTB, SPI mode 3 framing) -------------------------------
 * The converter is a TI ADS1216 (datasheet SBAS171D); the bus timings in hal/adcbus.c were checked
 * against the timing table on its page 8.
 *
 * Reproduce byte sequences exactly -- never "improve" the bus. /DRDY is an input and is polled WITH a
 * bounded timeout. */
#define ADC_PORT PORTB
#define ADC_PIN PINB
#define ADC_CS PB0 /* out, active low */
#define ADC_SCLK PB1 /* out, idles high */
#define ADC_DIN PB2 /* out, to chip */
#define ADC_DOUT PB3 /* in, from chip */
#define ADC_DRDY PB4 /* in, low = result ready */

/* ---- VFD controllers (two PT6311-class parts) ----------------------------------------------------- */
#define VFD_PORT PORTC
#define VFD_PIN PINC
#define VFD_CLK PC0 /* out */
#define VFD_DIN PC1 /* out */
#define VFD_DOUT PC2 /* in, also carries key-scan data */
#define VFD_STB2 PC3 /* out, /STB secondary display */
#define VFD_STB1 PC4 /* out, /STB main display + keyboard */

/* ---- Frequency counter inputs (hardware) ----------------------------------------------------------- */
/* PD4 = ICP1 capture input; PE6 = T3 external clock input. Neither is driven. */

/* ---- Host link ------------------------------------------------------------------------------------ */
/* PD2 = USART1 RXD, PD3 = USART1 TXD. USART0 is forced off and unused. */

/* Apply the reset-time port state above. Called once from main() before anything else. */
void board_init_ports(void);

#endif /* BK_BOARD_H */
