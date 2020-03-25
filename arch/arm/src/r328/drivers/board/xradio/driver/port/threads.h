#ifndef _LINUX_THREADS_H
#define _LINUX_THREADS_H

#ifdef __cplusplus
extern "C" {
#endif
#ifdef __MELIS__

#elif (defined(__CONFIG_ARCH_STM32))

#include "rtos/xr_rtos/xr_thread.h"

typedef struct xr_thread xr_drv_thread_t;

#define xr_drv_thread_valid(thread) XR_THREAD_IS_VALID(thread)
#define xr_drv_thread_init(thread, func, arg, priority, stack_size, stack_pointer) \
	xr_thread_create(thread, (os_pthread)func, arg, priority, stack_size, stack_pointer)
#define xr_drv_thread_destroy(thread) xr_thread_exit(thread)

#elif (defined(__CONFIG_CHIP_XRADIO))

#include "kernel/os/os_thread.h"

typedef OS_Thread_t xr_drv_thread_t;

#define schedule()              OS_ThreadYield()

#define xr_drv_thread_valid(thread) OS_ThreadIsValid(thread)
#define xr_drv_thread_init(thread, name, entry, arg, priority, stackSize) \
	OS_ThreadCreate(thread, name, (OS_ThreadEntry_t)entry, (void *)arg, (OS_Priority)priority, stackSize)
#define xr_drv_thread_destroy(thread) OS_ThreadDelete(thread)

#elif (defined(CONFIG_OS_RTTHREAD) || defined(CONFIG_OS_YUNOS) || defined(CONFIG_OS_NUTTX))
#include "kernel/os/os_thread.h"

#define XR_OS_INVALID_HANDLE		OS_INVALID_HANDLE

#define XR_DRV_Priority OS_Priority
#define XR_DRV_THREAD_PRIO_DRV_BH       OS_THREAD_PRIO_DRV_BH
#define XR_DRV_THREAD_PRIO_DRV_WORK     OS_THREAD_PRIO_DRV_WORK
#define XR_DRV_THREAD_PRIO_DRV_RX       OS_THREAD_PRIO_DRV_RX

#define xr_drv_thread_t			OS_Thread_t
#define XR_DRV_ThreadEntry_t	OS_ThreadEntry_t

#define schedule()              OS_ThreadYield()

#define xr_drv_thread_valid(thread) OS_ThreadIsValid(thread)
#define xr_drv_thread_init(thread, name, entry, arg, priority, stackSize) \
	OS_ThreadCreate(thread, name, (OS_ThreadEntry_t)entry, (void *)arg, (OS_Priority)priority, stackSize)
#define xr_drv_thread_destroy(thread) OS_ThreadDelete(thread)

#endif /* __MELIS__ */

#ifdef __cplusplus
}
#endif

#endif /* _LINUX_THREADS_H */
