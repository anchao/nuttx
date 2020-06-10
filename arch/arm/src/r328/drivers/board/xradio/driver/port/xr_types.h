#ifndef _WIRELESS_XRADIO_PORT_XR_TYPES_H_
#define _WIRELESS_XRADIO_PORT_XR_TYPES_H_
#ifndef CONFIG_OS_NUTTX
/* types */
#include "types.h"
#endif

typedef void *              HANDLE;

typedef unsigned long long  u64;
typedef unsigned int        u32;
typedef unsigned short      u16;
typedef unsigned char       u8;
typedef signed long long    s64;
typedef signed int          s32;
typedef signed short        s16;
typedef signed char         s8;

typedef u8  __u8;
typedef u16 __u16;
typedef u32 __u32;
typedef u64 __u64;

typedef s8  __s8;
typedef s16 __s16;
typedef s32 __s32;
typedef s64 __s64;

#define XR_INLINE __inline

#ifdef __CHECKER__
#define __bitwise__ __attribute__((bitwise))
#else
#define __bitwise__
#endif
#ifdef __CHECK_ENDIAN__
#define __bitwise __bitwise__
#else
#define __bitwise
#endif

typedef __u16 __bitwise __le16;
typedef __u16 __bitwise __be16;
typedef __u32 __bitwise __le32;
typedef __u32 __bitwise __be32;
typedef __u64 __bitwise __le64;
typedef __u64 __bitwise __be64;

typedef __u16 __bitwise __sum16;
typedef __u32 __bitwise __wsum;

#ifndef __cplusplus
#ifndef bool
#define bool	char
#define true	1
#define false	0
#define __bool_true_false_are_defined 1
#endif
#endif /* !__cplusplus */

#include "typecheck.h"

#define U32_MAX         ((u32)~0U)

#define BIT(nr)                 (1UL << (nr))
#define BIT_MASK(nr)		(1UL << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)		((nr) / BITS_PER_LONG)

#define BITS_PER_LONG 32

//#define schedule() osThreadYield()
//#define random32() (jiffies)

/*
 * ..and if you can't take the strict
 * types, you can specify one yourself.
 *
 * Or not use min/max/clamp at all, of course.
 */
#define min_t(type, x, y) ({			\
	type __min1 = (x);			\
	type __min2 = (y);			\
	__min1 < __min2 ? __min1: __min2; })

#define max_t(type, x, y) ({			\
	type __max1 = (x);			\
	type __max2 = (y);			\
	__max1 > __max2 ? __max1: __max2; })

# define likely(x)	(x)
# define unlikely(x)	(x)

#if defined(__MELIS__)

#elif (defined(__CONFIG_ARCH_STM32))

#define XR_DRV_WAIT_FOREVER         osWaitForever /* Wait forever timeout value */
#define XR_DRV_SEMAPHORE_MAX_COUNT  0xffffffffU /* Maximum count value for semaphore */

#elif (defined(__CONFIG_CHIP_XRADIO))
/* OS */
#include "kernel/os/os_time.h"
#include "kernel/os/os_timer.h"

#define HZ OS_HZ

#define jiffies                 OS_GetJiffies()
#define jiffies_to_msecs(j)     OS_JiffiesToMSecs(j)
#define	msecs_to_jiffies(msec)  OS_MSecsToJiffies(msec)

#define XR_DRV_WAIT_FOREVER         OS_WAIT_FOREVER /* Wait forever timeout value */
#define XR_DRV_SEMAPHORE_MAX_COUNT  0xffffffffU /* Maximum count value for semaphore */

#elif (defined(CONFIG_OS_RTTHREAD) || defined(CONFIG_OS_YUNOS))// || defined(CONFIG_OS_NUTTX))

#include "kernel/os/os_time.h"
#include "kernel/os/os_timer.h"

#define XR_DRV_Time_t OS_Time_t
#define XR_DRV_Status OS_Status

#define XR_DRV_OK           OS_OK    /* success */
#define XR_DRV_FAIL         OS_FAIL   /* general failure */
#define XR_DRV_E_NOMEM      OS_E_NOMEM   /* out of memory */
#define XR_DRV_E_PARAM      OS_E_PARAM   /* invalid parameter */
#define XR_DRV_E_TIMEOUT    OS_E_TIMEOUT   /* operation timeout */
#define XR_DRV_E_ISR        OS_E_ISR   /* not allowed in ISR context */

#define XR_DRV_WAIT_FOREVER         OS_WAIT_FOREVER /* Wait forever timeout value */
#define XR_DRV_SEMAPHORE_MAX_COUNT  OS_SEMAPHORE_MAX_COUNT /* Maximum count value for semaphore */
//#define ENOTSUPP ENOTSUP
#define XR_DRV_INVALID_HANDLE       OS_INVALID_HANDLE        /* OS invalid handle */

/* Parameters used to convert the time values */
#define XR_OS_MSEC_PER_SEC     OS_MSEC_PER_SEC       /* milliseconds per second */
#define XR_OS_USEC_PER_MSEC    OS_USEC_PER_MSEC       /* microseconds per millisecond */
#define XR_OS_USEC_PER_SEC     OS_USEC_PER_SEC    /* microseconds per second */

/* system clock's frequency, OS ticks per second */
#define HZ                     OS_HZ
#define XR_OS_HZ               OS_HZ

/* microseconds per OS tick (1000000 / OS_HZ) */
#define XR_OS_TICK             OS_TICK

#define jiffies                 OS_GetJiffies()
#define jiffies_to_msecs(j)     OS_JiffiesToMSecs(j)
#define	msecs_to_jiffies(msec)  OS_MSecsToJiffies(msec)
#elif (defined(CONFIG_OS_NUTTX))

#define XR_DRV_Time_t clock_t

#define XR_DRV_Status OS_Status

#define XR_DRV_OK        0
#define XR_DRV_FAIL      -1
#define XR_DRV_E_NOMEM	 -2
#define XR_DRV_E_PARAM   -3
#define XR_DRV_E_TIMEOUT -4
#define XR_DRV_E_ISR     -5

#define XR_DRV_WAIT_FOREVER         0xffffffffU /* Wait forever timeout value */
#define XR_DRV_SEMAPHORE_MAX_COUNT  0xffffffffU /* Maximum count value for semaphore */
//#define ENOTSUPP ENOTSUP
#define XR_DRV_INVALID_HANDLE       NULL        /* OS invalid handle */

/* Parameters used to convert the time values */
#define XR_OS_MSEC_PER_SEC     1000U       /* milliseconds per second */
#define XR_OS_USEC_PER_MSEC    1000U       /* microseconds per millisecond */
#define XR_OS_USEC_PER_SEC     1000000U    /* microseconds per second */

/* system clock's frequency, OS ticks per second */
#define HZ                     CONFIG_USEC_PER_TICK
#define XR_OS_HZ               CONFIG_USEC_PER_TICK

/* microseconds per OS tick (1000000 / OS_HZ) */
#define XR_OS_TICK             (OS_USEC_PER_SEC / OS_HZ)

#define jiffies                 clock()
#define jiffies_to_msecs(t)     ((uint32_t)(t) / (XR_OS_USEC_PER_MSEC / XR_OS_TICK))
#define	msecs_to_jiffies(msec)  ((uint32_t)(msec * (CONFIG_USEC_PER_TICK/1000)))
#define jiffies_to_usecs(t)      ((uint32_t)(t) / (XR_OS_USEC_PER_SEC / OS_TICK))
#endif

#ifndef CONFIG_OS_NUTTX
#define USEC_PER_SEC	OS_USEC_PER_SEC

static XR_INLINE unsigned int jiffies_to_usecs(const unsigned long j)
{
#if HZ <= USEC_PER_SEC && !(USEC_PER_SEC % HZ)
	return (USEC_PER_SEC / HZ) * j;
#elif HZ > USEC_PER_SEC && !(HZ % USEC_PER_SEC)
	return (j + (HZ / USEC_PER_SEC) - 1)/(HZ / USEC_PER_SEC);
#else
# if BITS_PER_LONG == 32
	return (HZ_TO_USEC_MUL32 * j) >> HZ_TO_USEC_SHR32;
# else
	return (j * HZ_TO_USEC_NUM) / HZ_TO_USEC_DEN;
# endif
#endif
}
#endif
/* endian */
#include "sys/endian.h"
/* time compare */
/*
 *	These inlines deal with timer wrapping correctly. You are
 *	strongly encouraged to use them
 *	1. Because people otherwise forget
 *	2. Because if the timer wrap changes in future you won't have to
 *	   alter your driver code.
 *
 * time_after(a,b) returns true if the time a is after time b.
 *
 * Do this with "<0" and ">=0" to only test the sign of the result. A
 * good compiler would generate better code (and a really good compiler
 * wouldn't care). Gcc is currently neither.
 */
#ifndef time_after
#define time_after(a, b)		\
	(typecheck(unsigned int, a) && \
	 typecheck(unsigned int, b) && \
	 ((long)(b) - (long)(a) < 0))
#define time_before(a, b)	time_after(b, a)

#define time_after_eq(a, b)	\
	(typecheck(unsigned int, a) && \
	 typecheck(unsigned int, b) && \
	 ((long)(a) - (long)(b) >= 0))
#define time_before_eq(a, b)	time_after_eq(b, a)
#endif
/* bits */
#include "sys/param.h"

/* memory */
#include "xr_drv_mem.h"

/* misc */
#include "kernel.h"
#ifndef min
#define min(a,b)	MIN(a,b)
#endif

#ifndef max
#define max(a,b)	MAX(a,b)
#endif

#define WARN_ON_ONCE(x) (x)

#define num_present_cpus() 1

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define random32() OS_Rand32()

#define XRDRV_UNUSED(v)	(void)(v)

#endif /* _WIRELESS_XRADIO_PORT_XR_TYPES_H_ */
