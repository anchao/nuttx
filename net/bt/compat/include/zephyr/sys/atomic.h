/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_ATOMIC_H_
#define ZEPHYR_INCLUDE_SYS_ATOMIC_H_

#include <nuttx/atomic.h>
#include <nuttx/spinlock.h>
#include <zephyr/sys/util.h>

/* NuttX defines atomic_add, atomic_sub, atomic_set, atomic_or, atomic_and,
 * atomic_xor as macros with different signatures. Undefine them so we can
 * provide Zephyr-compatible static inline functions instead.
 */
#undef atomic_add
#undef atomic_sub
#undef atomic_set
#undef atomic_or
#undef atomic_and
#undef atomic_xor
#undef atomic_clear

/* Types */
typedef int z_atomic_t;
typedef z_atomic_t atomic_val_t;
typedef void *atomic_ptr_t;
typedef uintptr_t atomic_ptr_val_t;

/* ATOMIC_DEFINE */
#define ATOMIC_BITS (sizeof(atomic_t) * 8)
#define ATOMIC_BITMAP_SIZE(num_bits) \
    ((num_bits + ATOMIC_BITS - 1) / ATOMIC_BITS)
#define ATOMIC_DEFINE(name, num_bits) atomic_t name[ATOMIC_BITMAP_SIZE(num_bits)]

/* Bit operations - use NuttX's low-level macros directly */
static inline void atomic_set_bit(atomic_t *target, int bit)
{
    atomic_fetch_or_4((uint32_t *)&target[bit / 32], BIT(bit % 32), __ATOMIC_SEQ_CST);
}

static inline void atomic_clear_bit(atomic_t *target, int bit)
{
    atomic_fetch_and_4((uint32_t *)&target[bit / 32], ~BIT(bit % 32), __ATOMIC_SEQ_CST);
}

static inline int atomic_test_bit(const atomic_t *target, int bit)
{
    return (atomic_load_4((uint32_t *)&target[bit / 32], __ATOMIC_RELAXED) & BIT(bit % 32)) != 0;
}

static inline int atomic_test_and_clear_bit(atomic_t *target, int bit)
{
    return (atomic_fetch_and_4((uint32_t *)&target[bit / 32], ~BIT(bit % 32), __ATOMIC_SEQ_CST) & BIT(bit % 32)) != 0;
}

static inline int atomic_test_and_set_bit(atomic_t *target, int bit)
{
    return (atomic_fetch_or_4((uint32_t *)&target[bit / 32], BIT(bit % 32), __ATOMIC_SEQ_CST) & BIT(bit % 32)) != 0;
}

/* atomic_cas */
static inline bool atomic_cas(atomic_t *target, atomic_val_t old_value, atomic_val_t new_value)
{
    uint32_t oldv = old_value;
    return atomic_compare_exchange_4((uint32_t *)target, &oldv, new_value, false, __ATOMIC_SEQ_CST, __ATOMIC_RELAXED);
}

static inline bool atomic_ptr_cas(atomic_ptr_t *target, atomic_ptr_t old_value, atomic_ptr_t new_value)
{
    return __atomic_compare_exchange_n((void **)target, &old_value, new_value, false, __ATOMIC_SEQ_CST, __ATOMIC_RELAXED);
}

/* Use NuttX's atomic wrappers that include memorder */
static inline atomic_val_t atomic_add(atomic_t *target, atomic_val_t value)
{
    return atomic_fetch_add_4((uint32_t *)target, value, __ATOMIC_SEQ_CST);
}

static inline atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value)
{
    return atomic_fetch_sub_4((uint32_t *)target, value, __ATOMIC_SEQ_CST);
}

static inline atomic_val_t atomic_get(const atomic_t *target)
{
    return atomic_load_4((uint32_t *)target, __ATOMIC_RELAXED);
}

static inline atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
    return atomic_exchange_4((uint32_t *)target, value, __ATOMIC_SEQ_CST);
}

static inline atomic_val_t atomic_clear(atomic_t *target)
{
    return atomic_set(target, 0);
}

static inline atomic_ptr_t atomic_ptr_get(const atomic_ptr_t *target)
{
    return __atomic_load_n((void **)target, __ATOMIC_RELAXED);
}

static inline atomic_ptr_t atomic_ptr_set(atomic_ptr_t *target, atomic_ptr_t value)
{
    return __atomic_exchange_n((void **)target, value, __ATOMIC_SEQ_CST);
}

static inline atomic_ptr_t atomic_ptr_clear(atomic_ptr_t *target)
{
    return __atomic_exchange_n((void **)target, NULL, __ATOMIC_SEQ_CST);
}

static inline atomic_val_t atomic_or(atomic_t *target, atomic_val_t value)
{
    return atomic_fetch_or_4((uint32_t *)target, value, __ATOMIC_SEQ_CST);
}

static inline atomic_val_t atomic_and(atomic_t *target, atomic_val_t value)
{
    return atomic_fetch_and_4((uint32_t *)target, value, __ATOMIC_SEQ_CST);
}

static inline atomic_val_t atomic_xor(atomic_t *target, atomic_val_t value)
{
    return atomic_fetch_xor_4((uint32_t *)target, value, __ATOMIC_SEQ_CST);
}

#define atomic_inc(target)     atomic_fetch_add_4((uint32_t *)(target), 1, __ATOMIC_SEQ_CST)
#define atomic_dec(target)     atomic_fetch_sub_4((uint32_t *)(target), 1, __ATOMIC_SEQ_CST)

#define atomic_set_bit_to(target, bit, val) \
    do { if (val) atomic_set_bit(target, bit); else atomic_clear_bit(target, bit); } while (0)

#endif /* ZEPHYR_INCLUDE_SYS_ATOMIC_H_ */
