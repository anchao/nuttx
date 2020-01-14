#ifndef SUNXI_HAL_SPINLOCK_H
#define SUNXI_HAL_SPINLOCK_H
#include <hal_interrupt.h>
#include <nuttx/spinlock.h>

typedef hal_irq_state_t hal_cpu_cpsr_t;
typedef spinlock_t hal_spinlock_t;

#define HAL_SPIN_LOCK_DEFINE(lock)  hal_spinlock_t lock = SP_UNLOCKED

#define hal_spin_lock_init(lock) \
	spin_initialize(lock, SP_UNLOCKED)

#define hal_spin_lock(lock) \
	spin_lock(lock)

#define hal_spin_unlock(lock) \
	spin_unlock(lock)

#define hal_spin_lock_irqsave(lock, flags) \
	do { \
		hal_local_irq_save(flags); \
		hal_spin_lock(lock); \
	} while (0)

#define hal_spin_unlock_irqrestore(lock, flags) \
	do { \
		hal_spin_unlock(lock); \
		hal_local_irq_restore(flags); \
	} while (0)

#endif /* SUNXI_HAL_SPINLOCK_H */
