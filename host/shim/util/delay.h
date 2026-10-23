/* Host shim for <util/delay.h>. The delays are no-ops: a host test is checking logic, not timing, and a
 * real delay would only make the tests slow. Anything whose CORRECTNESS depends on a delay cannot be
 * host-tested and must say so. */
#ifndef SHIM_DELAY_H
#define SHIM_DELAY_H
#define _delay_ms(x) ((void)0)
#define _delay_us(x) ((void)0)
#endif
