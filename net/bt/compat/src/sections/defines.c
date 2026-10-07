/* SPDX-License-Identifier: Apache-2.0 */
/* Placeholder for iterable sections arrays used by the port layer. */

#include <zephyr/sys/iterable_sections.h>
#include <zephyr/init.h>

/* Net buf pools are defined by users, this is just a sentinel */
struct net_buf_pool;
const struct net_buf_pool *_net_buf_pool_list[] __attribute__((__unused__)) = { NULL };

/* K_mem_slab list - declared as weak to avoid link issues */
struct k_mem_slab;
const struct k_mem_slab *_k_mem_slab_list[] __attribute__((__unused__)) = { NULL };