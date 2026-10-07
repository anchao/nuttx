/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_IRQ_H_
#define ZEPHYR_INCLUDE_IRQ_H_

#include <nuttx/irq.h>

/* Simple wrapper */
#define irq_lock()        enter_critical_section()
#define irq_unlock(key)   leave_critical_section(key)

typedef irqstate_t irq_lock_key_t;

#endif /* ZEPHYR_INCLUDE_IRQ_H_ */