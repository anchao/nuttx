/*
 * irqflags.h
 *
 * IRQ flags tracing: follow the state of the hardirq and softirq flags and
 * provide callbacks for transitions between ON and OFF states.
 *
 * This file gets included from lowlevel asm headers too, to provide
 * wrapped versions of the local_irq_*() APIs, based on the
 * raw_local_irq_*() macros from the lowlevel headers.
 */
#ifndef _LINUX_TRACE_IRQFLAGS_H
#define _LINUX_TRACE_IRQFLAGS_H

#ifdef __cplusplus
extern "C" {
#endif

#if (defined(__CONFIG_CHIP_XRADIO))

#include "sys/interrupt.h"

#define raw_local_irq_save(flags)	\
	do {				\
		flags = xr_irq_save();	\
	} while (0)

#define raw_local_irq_restore(flags)	\
	do {				\
		xr_irq_restore(flags);	\
	} while (0)

#elif (defined(CONFIG_OS_RTTHREAD))

#include "rtthread.h"
#include "rthw.h"

#define raw_local_irq_save(flags)	\
	do {				\
		flags = rt_hw_interrupt_disable();	\
	} while (0)

#define raw_local_irq_restore(flags)	\
	do {				\
		rt_hw_interrupt_enable(flags);	\
	} while (0)

#elif (defined(CONFIG_OS_YUNOS))

void save_and_cli(void);
void restore_flags(void);

#define raw_local_irq_save(flags)	\
	({				\
		save_and_cli();	\
		flags;	\
	})

#define raw_local_irq_restore(flags)	\
	({				\
		restore_flags();	\
		flags;	\
	})

#elif (defined(CONFIG_OS_NUTTX))

#include <nuttx/irq.h>
#define raw_local_irq_save(flags)		(flags = enter_critical_section())
#define raw_local_irq_restore(flags)	leave_critical_section(flags)
#endif /*__CONFIG_CHIP_XRADIO*/

#define local_irq_save(flags)		raw_local_irq_save(flags)
#define local_irq_restore(flags)	raw_local_irq_restore(flags)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_TRACE_IRQFLAGS_SUPPORT */
