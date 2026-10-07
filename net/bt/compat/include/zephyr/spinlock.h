/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SPINLOCK_H_
#define ZEPHYR_INCLUDE_SPINLOCK_H_

#include <nuttx/spinlock.h>

typedef spinlock_t struct_spinlock;

#define SPINLOCK_INIT SP_UNLOCKED

#endif /* ZEPHYR_INCLUDE_SPINLOCK_H_ */
