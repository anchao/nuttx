#ifndef __HAL_SDMMC_H__
#define __HAL_SDMMC_H__

#include <chip/osal4hal/include/hal_cache.h>
#include <sdmmc/osal_sdmmc.h>

/* IRQ disable/enable */
#define HAL_DisableIRQ()    arch_irq_disable()
#define HAL_EnableIRQ()     arch_irq_enable()

typedef irqstate_t osal_sdmmc_irqstate_t;

#define HAL_EnterCriticalSection   enter_critical_section
#define HAL_ExitCriticalSection    leave_critical_section

#define HAL_ATMOTIC_SET(a,v)	({osal_sdmmc_irqstate_t flags = HAL_EnterCriticalSection();\
								a = v;HAL_ExitCriticalSection(flags);})

#define HAL_ATMOTIC_READ(a)			({osal_sdmmc_irqstate_t flags=0;int v=0; \
		flags = HAL_EnterCriticalSection();v=a;HAL_ExitCriticalSection(flags);v;})


#define HAL_FlushDcacheRegion(s,len)        (cpu_dcache_clean_invalidate(s,len))
#define HAL_InvalidDcacheRegion(s,len)		(cpu_dcache_invalidate(s, len))

#define HAL_GetTimeMs()				((OSAL_GET_SYS_TICK*1000)/OS_HZ)
#define HAL_GetTimeUs()				((OSAL_GET_SYS_TICK*1000*1000)/OS_HZ)
#define HAL_GetTimeNs()				((OSAL_GET_SYS_TICK*1000*1000)/OS_HZ)

/* Thread */
#define HAL_ThreadEnd(s)				(HAL_ATMOTIC_SET(s,0))
#define HAL_ThreadStop(s)				(HAL_ATMOTIC_SET(s,1))
#define HAL_Thread_Should_Stop(s)		(HAL_ATMOTIC_READ(s))


/* Time */
#define HAL_Ticks()             OSAL_GET_SYS_TICK()
#define HAL_MSleep(msec)        OS_MSleep(msec)
#define HAL_UDelay(us)          OS_Udelay(us)

#define HAL_SecsToTicks(sec)    OS_SecsToTicks(sec)
#define HAL_MSecsToTicks(msec)  OS_MSecsToTicks(msec)
#define HAL_TicksToMSecs(t)     OS_TicksToMSecs(t)
#define HAL_TicksToSecs(t)      OS_TicksToSecs(t)

#define HAL_TimeAfter(a, b)         OS_TimeAfter(a, b)
#define HAL_TimeBefore(a, b)        OS_TimeBefore(a, b)
#define HAL_TimeAfterEqual(a, b)    OS_TimeAfterEqual(a, b)
#define HAL_TimeBeforeEqual(a, b)   OS_TimeBeforeEqual(a, b)

#define OS_CACHE_ALIGN_BYTES  (64)
#if 0
static inline void *malloc_align_buf(size_t size)
{
	void *fake_ptr = NULL;
	void *malloc_ptr = NULL;

	malloc_ptr = osal_malloc(size + OS_CACHE_ALIGN_BYTES);
	if ((uint32_t)malloc_ptr & 0x3) {
		printf("error: krhino_mm_alloc not align to 4 byte\r\n");
	}
	fake_ptr = (uint32_t)(malloc_ptr + OS_CACHE_ALIGN_BYTES) & (~(OS_CACHE_ALIGN_BYTES -1));
	*(uint32_t *)((uint32_t *)fake_ptr - 1) = malloc_ptr;

	return fake_ptr;
}

static inline void free_align_buf(void *addr)
{
	void *malloc_ptr = NULL;
	if (!addr)
		return;
	malloc_ptr = *(uint32_t *)((uint32_t *)addr - 1);
	osal_free(malloc_ptr);
}
#else
static inline void *malloc_align_buf(size_t size)
{
	void *malloc_ptr;

	uint32_t fake_ptr;
	malloc_ptr = malloc(size + OS_CACHE_ALIGN_BYTES);

	if (!malloc_ptr)
		return NULL;
	if ((uint32_t)malloc_ptr & 0x3)
		printf("malloc not align to 4 bytes\n");
	fake_ptr = (uint32_t)(malloc_ptr + OS_CACHE_ALIGN_BYTES);
	fake_ptr &= (~(OS_CACHE_ALIGN_BYTES-1));
	/* save actual pointer */
	*((uint32_t *)(fake_ptr - 4)) = (uint32_t)malloc_ptr;
	return (void *)fake_ptr;
}

static inline void free_align_buf(void *addr)
{
	void *malloc_ptr = NULL;

	if (!addr)
		return;
	/* get actual pointer */
	malloc_ptr = (void *)(*(uint32_t *)(addr - 4));

	free(malloc_ptr);
}
#endif

#define osal_malloc_align(l)      (malloc_align_buf(l))
#define osal_free_align(p)        (free_align_buf(p))


#endif
