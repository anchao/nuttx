/* SPDX-License-Identifier: Apache-2.0 */
/* Atomic operations fallback using spinlock.
 * This is used only when the architecture doesn't have native support.
 * NuttX provides atomic.h with native support, so this is just a shim.
 */

#include <zephyr/kernel.h>

/* All atomics are implemented as inline in the header, no source needed */