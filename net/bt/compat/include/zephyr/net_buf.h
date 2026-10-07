/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_NET_BUF_H_
#define ZEPHYR_INCLUDE_NET_BUF_H_

#include <stddef.h>
#include <nuttx/mm/net_buf.h>
#include <zephyr/sys_clock.h>

static inline int32_t bt_net_buf_timeout(clock_t timeout)
{
    return timeout < 0 ? -1 : (int32_t)TICK2MSEC(timeout);
}

static inline struct net_buf_pool *net_buf_pool_get(struct net_buf_pool *pool)
{
    return pool;
}

/* net_buf_alloc_fixed_debug - Zephyr debug wrapper, map to net_buf_alloc */
static inline struct net_buf *net_buf_alloc_fixed_debug(struct net_buf_pool *pool, clock_t timeout, const char *func, int line)
{
    return net_buf_alloc(pool, bt_net_buf_timeout(timeout));
}

#endif /* ZEPHYR_INCLUDE_NET_BUF_H_ */
