/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_ARCH_POSIX_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_POSIX_ARCH_H_

#include <nuttx/spinlock.h>
#include <nuttx/irq.h>
#include <nuttx/compiler.h>
#include <nuttx/sched.h>
#include <stdbool.h>

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
