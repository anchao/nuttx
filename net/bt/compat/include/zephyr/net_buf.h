/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_NET_BUF_H_
#define ZEPHYR_INCLUDE_NET_BUF_H_

#include <stddef.h>
#include <nuttx/mm/net_buf.h>
#include <zephyr/sys_clock.h>

/* The NuttX net_buf provides most of the same APIs as Zephyr's net_buf.
 *
 * Key difference: net_buf_alloc and friends take raw int32_t ticks as the
 * timeout parameter, NOT k_timeout_t. Zephyr source files that pass
 * K_NO_WAIT, K_FOREVER, or K_SECONDS() need to be patched accordingly.
 * K_NO_WAIT maps to 0, K_FOREVER maps to -1, K_SECONDS(n) maps to
 * k_sec_to_ticks_ceil32(n).
 */

/* net_buf_alloc_fixed_debug - Zephyr debug wrapper, map to net_buf_alloc */
static inline struct net_buf *net_buf_alloc_fixed_debug(struct net_buf_pool *pool, k_timeout_t timeout, const char *func, int line)
{
    return net_buf_alloc(pool, timeout.ticks);
}

#endif /* ZEPHYR_INCLUDE_NET_BUF_H_ */
