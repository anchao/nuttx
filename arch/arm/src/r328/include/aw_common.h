#ifndef COMMON_H
#define COMMON_H

#include <nuttx/spinlock.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __LITTLE_ENDIAN
#define __LITTLE_ENDIAN 1234
#endif

#ifndef __BIG_ENDIAN
#define __BIG_ENDIAN 4321
#endif

#ifndef __BYTE_ORDER
#define __BYTE_ORDER __LITTLE_ENDIAN
#endif

#define ARRAY_SIZE(x)       (sizeof(x) / sizeof((x)[0]))

#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#endif

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({			\
	const typeof(((type *)0)->member) * __mptr = (ptr);	\
	(type *)((char *)__mptr - offsetof(type, member)); })

typedef uint32_t cpu_cpsr_t;

static inline int cpu_intrpt_save(void)
{
	return (int)up_irq_save();
}
static inline int cpu_intrpt_restore(int flags)
{
	up_irq_restore((irqstate_t)flags);
	return 0;
}

typedef spinlock_t kspinlock_t;

#define krhino_spin_lock_init(lock) \
do { \
	spin_initialize(lock, SP_UNLOCKED); \
 \
} while (0)

static inline void krhino_spin_lock(kspinlock_t *lock)
{
	spin_lock(lock);
}

static inline void krhino_spin_unlock(kspinlock_t *lock)
{
	spin_unlock(lock);
}
#define KRHINO_SPIN_LOCK_DEFINE(lock) kspinlock_t lock = {0}
#define krhino_spin_lock_irq_save(lock, flags) \
do { \
	flags = cpu_intrpt_save(); \
	krhino_spin_lock(lock);		 \
} while (0)
#define krhino_spin_unlock_irq_restore(lock, flags) \
do { \
	cpu_intrpt_restore(flags); \
	krhino_spin_unlock(lock);		 \
} while (0)


#define xTaskResumeAll()
#define vTaskSuspendAll()

#define __disable_irq()  	do { asm volatile ("cpsie i"); } while (0)
#define __enable_irq() 		do { asm volatile ("cpsid i"); } while (0)

#ifdef __cplusplus
}
#endif

#endif  /*COMMON_H*/
