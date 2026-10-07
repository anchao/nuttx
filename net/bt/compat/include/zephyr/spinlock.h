/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SPINLOCK_H_
#define ZEPHYR_INCLUDE_SPINLOCK_H_

#include <nuttx/spinlock.h>

typedef spinlock_t struct_spinlock;

#define SPINLOCK_INIT SP_UNLOCKED

static inline void k_spinlock_init_ptr(spinlock_t *l)
{
    *l = SP_UNLOCKED;
}

static inline bool k_spinlock_is_locked_ptr(spinlock_t *l)
{
    return spin_is_locked(l);
}

#endif /* ZEPHYR_INCLUDE_SPINLOCK_H_ */