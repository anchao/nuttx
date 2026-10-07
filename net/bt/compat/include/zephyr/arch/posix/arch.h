/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_ARCH_POSIX_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_POSIX_ARCH_H_

#include <nuttx/spinlock.h>
#include <nuttx/irq.h>
#include <nuttx/compiler.h>
#include <nuttx/sched.h>
#include <stdbool.h>

/* Spinlock */
struct k_spinlock {
    spinlock_t lock;
};
typedef irqstate_t k_spinlock_key_t;

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l)
{
    return spin_lock_irqsave(&l->lock);
}

static inline void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key)
{
    spin_unlock_irqrestore(&l->lock, key);
}

static inline void k_spinlock_init(struct k_spinlock *l)
{
    l->lock = SP_UNLOCKED;
}

static inline bool k_spinlock_is_locked(struct k_spinlock *l)
{
    return spin_is_locked(&l->lock);
}

/* IRQ lock/unlock - NuttX uses critical sections */
static inline unsigned int arch_irq_lock(void)
{
    if (!up_interrupt_context())
        sched_lock();
    return enter_critical_section();
}

static inline void arch_irq_unlock(unsigned int key)
{
    if (!up_interrupt_context())
        sched_unlock();
    leave_critical_section(key);
}

static inline bool arch_irq_unlocked(unsigned int key)
{
    return key == 0;
}

/* Scheduling */
static inline void k_sched_lock(void)
{
    sched_lock();
}

static inline void k_sched_unlock(void)
{
    sched_unlock();
}

/* ISR context check */
static inline bool k_is_in_isr(void)
{
    return up_interrupt_context();
}

static inline bool k_is_preempt_thread(void)
{
    return false;
}

/* Current thread - declared in kernel.h, provide arch-level implementation */
extern struct k_thread *k_current_get(void);

/* Cycle counter - stub */
static inline uint32_t arch_k_cycle_get_32(void)
{
    return 0;
}

static inline uint64_t arch_k_cycle_get_64(void)
{
    return 0;
}

/* Data memory barrier */
static inline void arch_dmb(void)
{
    __asm__ __volatile__("" ::: "memory");
}

#endif /* ZEPHYR_INCLUDE_ARCH_POSIX_ARCH_H_ */