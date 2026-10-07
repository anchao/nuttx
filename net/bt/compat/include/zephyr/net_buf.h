/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_NET_BUF_H_
#define ZEPHYR_INCLUDE_NET_BUF_H_

#include <stddef.h>
#include <nuttx/mm/net_buf.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys_clock.h>

/* The NuttX net_buf provides most of the same APIs as Zephyr's net_buf.
 *
 * Key difference: net_buf_alloc and friends take raw int32_t ticks as the
 * timeout parameter, NOT k_timeout_t. Zephyr source files that pass
 * K_NO_WAIT, K_FOREVER, or K_SECONDS() need to be patched accordingly.
 * K_NO_WAIT maps to 0, K_FOREVER maps to -1, K_SECONDS(n) maps to
 * k_sec_to_ticks_ceil32(n).
 */

/* Bridge net_buf_slist_put/get which use different list types:
 *   NuttX: struct net_buf **list (uses buf->node pointer directly)
 *   Zephyr: sys_slist_t *list (uses buf->node as sys_snode_t)
 *
 * Since struct net_buf *node and sys_snode_t have the same binary layout
 * (both are a single pointer), we can bridge via inline wrappers.
 */

static inline void z_net_buf_slist_put(sys_slist_t *list, struct net_buf *buf)
{
    buf->node = NULL;
    sys_slist_append(list, (sys_snode_t *)&buf->node);
}

static inline struct net_buf *z_net_buf_slist_get(sys_slist_t *list)
{
    sys_snode_t *snode = sys_slist_get(list);
    if (!snode)
    {
        return NULL;
    }
    return (struct net_buf *)((char *)snode - offsetof(struct net_buf, node));
}

/* Override NuttX declarations with Zephyr-compatible macros for slist API.
 * These are defined after the NuttX header include, so they only affect
 * call sites in Zephyr source files that include this compat header.
 */
#define net_buf_slist_put(list, buf) z_net_buf_slist_put((sys_slist_t *)(list), (buf))
#define net_buf_slist_get(list) z_net_buf_slist_get((sys_slist_t *)(list))

/* net_buf_alloc_fixed_debug - Zephyr debug wrapper, map to net_buf_alloc */
static inline struct net_buf *net_buf_alloc_fixed_debug(struct net_buf_pool *pool, k_timeout_t timeout, const char *func, int line)
{
    return net_buf_alloc(pool, timeout.ticks);
}

#endif /* ZEPHYR_INCLUDE_NET_BUF_H_ */