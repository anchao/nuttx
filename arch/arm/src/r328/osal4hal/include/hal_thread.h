#ifndef SUNXI_HAL_THREAD_H
#define SUNXI_HAL_THREAD_H

#include <stddef.h>
#include <stdint.h>


typedef void (*hal_kthread_fn_t)(void *data);
typedef pid_t hal_kthread_t;

int hal_kthread_create(hal_kthread_t *thread, hal_kthread_fn_t entry,
			void *data, const char *name,
			int priority, int stack_size);

void hal_kthread_delete(hal_kthread_t *thread);

#endif /* SUNXI_HAL_THREAD_H */
