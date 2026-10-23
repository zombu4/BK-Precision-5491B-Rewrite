/* Host shim for <avr/wdt.h>. There is no watchdog on the host; the calls are no-ops so that code which
 * correctly pets the watchdog still compiles and can be tested. */
#ifndef SHIM_WDT_H
#define SHIM_WDT_H
#define wdt_reset() ((void)0)
#define wdt_enable(x) ((void)0)
#define wdt_disable() ((void)0)
#define WDTO_15MS 0
#define WDTO_2S 7
#endif
