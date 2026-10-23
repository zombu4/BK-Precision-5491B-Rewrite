/* Host shim for <util/atomic.h>. Single-threaded on the host, so an atomic block is just a block. It still
 * has to be a statement that takes a body, or every ATOMIC_BLOCK in the port would fail to compile. */
#ifndef SHIM_ATOMIC_H
#define SHIM_ATOMIC_H
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_FORCEON 0
#define ATOMIC_BLOCK(t) for (int _ab = 1; _ab; _ab = 0)
#define NONATOMIC_BLOCK(t) for (int _nb = 1; _nb; _nb = 0)
#endif
