#ifndef __OSAL_SDMMC_H__
#define __OSAL_SDMMC_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>
#include <debug.h>
#include <endian.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <semaphore.h>

#define	NBBY	8		/* number of bits in a byte */
#define	NBPW	sizeof(int)	/* number of bytes per word (integer) */

/* Bit map related macros. */
#define	setbit(a,i)	(((unsigned char *)(a))[(i)/NBBY] |= 1<<((i)%NBBY))
#define	clrbit(a,i)	(((unsigned char *)(a))[(i)/NBBY] &= ~(1<<((i)%NBBY)))
#define	isset(a,i)							\
	(((const unsigned char *)(a))[(i)/NBBY] & (1<<((i)%NBBY)))
#define	isclr(a,i)							\
	((((const unsigned char *)(a))[(i)/NBBY] & (1<<((i)%NBBY))) == 0)

/* Macros for counting and rounding. */
#ifndef howmany
#define	howmany(x, y)	(((x)+((y)-1))/(y))
#endif
#define	nitems(x)	(sizeof((x)) / sizeof((x)[0]))
#define	rounddown(x, y)	(((x)/(y))*(y))
#define	rounddown2(x, y) ((x)&(~((y)-1)))          /* if y is power of two */
#define	roundup(x, y)	((((x)+((y)-1))/(y))*(y))  /* to any y */
#define	roundup2(x, y)	(((x)+((y)-1))&(~((y)-1))) /* if y is powers of two */
#define powerof2(x)	((((x)-1)&(x))==0)

/* Macros for min/max. */
#define	MIN(a,b) (((a)<(b))?(a):(b))
#define	MAX(a,b) (((a)>(b))?(a):(b))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define ct_assert(e) extern char (*ct_assert(void)) [sizeof(char[1 - 2*!(e)])]

/**
 * @brief OS status definition
 */
typedef enum {
    OSAL_STATUS_OK           = 0,    /* success */
    OSAL_STATUS_FAIL         = -1,   /* general failure */
    OSAL_E_NOMEM      = -2,   /* out of memory */
    OSAL_E_PARAM      = -3,   /* invalid parameter */
    OSAL_E_TIMEOUT    = -4,   /* operation timeout */
    OSAL_E_ISR        = -5,   /* not allowed in ISR context */
}osal_sdmmc_status_t;


#define OS_WAIT_FOREVER         0xffffffffU /* Wait forever timeout value */
#define OS_SEMAPHORE_MAX_COUNT  0xffffffffU /* Maximum count value for semaphore */
#define OS_INVALID_HANDLE       NULL        /* OS invalid handle */

typedef clock_t os_time_t;

#define OSAL_GET_SYS_TICK() (clock())

#define OS_GetTicks() ((uint32_t)(clock()))

/* Parameters used to convert the time values */
#ifndef OS_MSEC_PER_SEC
#define OS_MSEC_PER_SEC     1000U       /* milliseconds per second */
#endif

#ifndef OS_USEC_PER_MSEC
#define OS_USEC_PER_MSEC    1000U       /* microseconds per millisecond */
#endif

#ifndef OS_USEC_PER_SEC
#define OS_USEC_PER_SEC     1000000U    /* microseconds per second */
#endif

/* system clock's frequency, OS ticks per second */
#define OS_HZ               CONFIG_USEC_PER_TICK

/* microseconds per OS tick (1000000 / OS_HZ) */
#define OS_TICK             (OS_USEC_PER_SEC / OS_HZ)

#ifndef OS_GetTime
/** @brief Get the number of seconds since OS start */
#define OS_GetTime()        (OSAL_GET_SYS_TICK() / OS_HZ)
#endif

/**
 * @brief Macros used to convert various time units to each other
 *     - Secs stand for seconds
 *     - MSecs stand for milliseconds
 *     - Ticks stand for OS ticks
 *     - Jiffies stand for OS jiffies, which is a synonym for OS ticks
 */
#ifndef OS_SecsToTicks
#define OS_SecsToTicks(sec)     ((os_time_t)(sec) * OS_HZ)
#endif

#ifndef OS_MSecsToTicks
#define OS_MSecsToTicks(msec)   ((os_time_t)(msec * (CONFIG_USEC_PER_TICK/1000)))
#endif

#ifndef OS_TicksToMSecs
#define OS_TicksToMSecs(t)      ((uint32_t)(t) / (OS_USEC_PER_MSEC / OS_TICK))
#endif

#ifndef OS_TicksToSecs
#define OS_TicksToSecs(t)       ((uint32_t)(t) / (OS_USEC_PER_SEC / OS_TICK))
#endif

#ifndef OS_GetJiffies
#define OS_GetJiffies()         OSAL_GET_SYS_TICK()
#endif

#ifndef OS_SecsToJiffies
#define OS_SecsToJiffies(sec)   OS_SecsToTicks(sec)
#endif

#ifndef OS_MSecsToJiffies
#define OS_MSecsToJiffies(msec) OS_MSecsToTicks(msec)
#endif

#ifndef OS_JiffiesToMSecs
#define OS_JiffiesToMSecs(j)    OS_TicksToMSecs(j)
#endif

#ifndef OS_JiffiesToSecs
#define OS_JiffiesToSecs(j)     OS_TicksToSecs(j)
#endif

/**
 * @brief Macros used to sleep for the given time (milliseconds or seconds)
 */
/* sleep */
#ifndef OS_MSleep
#define OS_MSleep(msec)			nxsig_usleep(1000*msec)
#endif

#ifndef OS_Sleep
#define OS_Sleep(sec)			nxsig_sleep(sec)
#endif

#ifndef OS_Udelay
#define OS_Udelay(usec)         up_udelay(usec);
#endif

/**
 * @brief Macros used to compare time values
 *
 *  These inlines deal with timer wrapping correctly. You are
 *  strongly encouraged to use them
 *  1. Because people otherwise forget
 *  2. Because if the timer wrap changes in future you won't have to
 *     alter your code.
 *
 * OS_TimeAfter(a,b) returns true if the time a is after time b.
 *
 * Do this with "<0" and ">=0" to only test the sign of the result. A
 * good compiler would generate better code (and a really good compiler
 * wouldn't care). Gcc is currently neither.
 */
#ifndef OS_TimeAfter
#define OS_TimeAfter(a, b)              ((int32_t)(b) - (int32_t)(a) < 0)
#endif

#ifndef OS_TimeBefore
#define OS_TimeBefore(a, b)             OS_TimeAfter(b, a)
#endif

#ifndef OS_TimeAfterEqual
#define OS_TimeAfterEqual(a, b)         ((int32_t)(a) - (int32_t)(b) >= 0)
#endif

#ifndef OS_TimeBeforeEqual
#define OS_TimeBeforeEqual(a, b)        OS_TimeAfterEqual(b, a)
#endif

/** @brief Macros used to generate fake random 32-bit value */
/* The fake random 32-bit value is generated by combining OS ticks and
 * the value of SysTick current value register.
 */

/*
#define OS_Rand32()                                                 \
    ((uint32_t)(((*((volatile uint32_t *)0xE000E018)) & 0xffffff) | \
                (OSAL_GET_SYS_TICK() << 24)))
*/
#define ALIGN(x, a) __ALIGN_KERNEL((x), (a))
#define ALIGN_DOWN(x, a) __ALIGN_KERNEL((x) - ((a)-1), (a))
#define __ALIGN_KERNEL(x, a) __ALIGN_KERNEL_MASK(x, (typeof(x))(a)-1)
#define __ALIGN_KERNEL_MASK(x, mask) (((x) + (mask)) & ~(mask))

/* Memory */
#define osal_malloc(l)           kmm_malloc(l)
#define osal_free(p)             kmm_free(p)
#define osal_memcpy(d, s, l)     memcpy(d, s, l)
#define osal_memset(d, c, l)     memset(d, c, l)
#define osal_memcmp(a, b, l)     memcmp(a, b, l)

typedef struct osal_sdmmc_mutex {
	sem_t	handle;
}osal_sdmmc_mutex_t;

typedef uint32_t osal_sdmmc_wait_timems_t;

osal_sdmmc_status_t osal_sdmmc_mutex_init(osal_sdmmc_mutex_t *mutex);
osal_sdmmc_status_t osal_sdmmc_mutex_delete(osal_sdmmc_mutex_t *mutex);
osal_sdmmc_status_t osal_sdmmc_mutex_lock(osal_sdmmc_mutex_t *mutex, osal_sdmmc_wait_timems_t waitMS);
osal_sdmmc_status_t osal_sdmmc_mutex_unlock(osal_sdmmc_mutex_t *mutex);

typedef sem_t osal_sdmmc_sem_t;

osal_sdmmc_status_t osal_sdmmc_sem_create(osal_sdmmc_sem_t *sem, uint32_t initCount);
osal_sdmmc_status_t osal_sdmmc_sem_delete(osal_sdmmc_sem_t *sem);
osal_sdmmc_status_t osal_sdmmc_sem_wait(osal_sdmmc_sem_t *sem, osal_sdmmc_wait_timems_t waitMS);
osal_sdmmc_status_t osal_sdmmc_sem_give(osal_sdmmc_sem_t *sem);

/** @brief Thread entry definition, which is a pointer to a function */
typedef int (*osal_sdmmc_thread_t)(int argc, char *argv[]);

/** @brief Thread handle definition */
typedef void osal_sdmmc_task_handle_t;
osal_sdmmc_status_t osal_sdmmc_thread_create(osal_sdmmc_task_handle_t *thread, const char *name,
                          osal_sdmmc_thread_t entry, void *arg,
                          int priority, uint32_t stackSize);

osal_sdmmc_status_t osal_sdmmc_thread_delete(osal_sdmmc_task_handle_t *thread);

#ifdef __cplusplus
}
#endif
#endif
