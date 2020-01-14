#ifndef SUNXI_HAL_INTERRUPT_H
#define SUNXI_HAL_INTERRUPT_H
#include <arch/irq.h>

typedef irqstate_t hal_irq_state_t;

#define hal_local_irq_enable() 	do { asm volatile ("cpsie i"); } while (0)

#define hal_local_irq_disable() do { asm volatile ("cpsid i"); } while (0)


#define hal_local_irq_save(flags) \
	do { \
		flags = up_irq_save(); \
	} while (0)

#define hal_local_irq_restore(flags) \
	do { \
		up_irq_restore(flags); \
	} while (0)

#endif /* SUNXI_HAL_INTERRUPT_H */
