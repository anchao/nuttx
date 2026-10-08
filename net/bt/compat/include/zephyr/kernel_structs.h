/*
 * Copyright (c) 2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The purpose of this file is to provide essential/minimal kernel structure
 * definitions, so that they can be used without including kernel.h.
 *
 * The following rules must be observed:
 *  1. kernel_structs.h shall not depend on kernel.h both directly and
 *    indirectly (i.e. it shall not include any header files that include
 *    kernel.h in their dependency chain).
 *  2. kernel.h shall imply kernel_structs.h, such that it shall not be
 *    necessary to include kernel_structs.h explicitly when kernel.h is
 *    included.
 */

#ifndef ZEPHYR_KERNEL_INCLUDE_KERNEL_STRUCTS_H_
#define ZEPHYR_KERNEL_INCLUDE_KERNEL_STRUCTS_H_

#if !defined(_ASMLANGUAGE)
#include <sys/types.h>

#include <nuttx/atomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <nuttx/list.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/structs.h>
#endif

#define K_NUM_THREAD_PRIO (CONFIG_NUM_PREEMPT_PRIORITIES + CONFIG_NUM_COOP_PRIORITIES + 1)
#define PRIQ_BITMAP_SIZE  (DIV_ROUND_UP(K_NUM_THREAD_PRIO, BITS_PER_LONG))

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bitmask definitions for Zephyr thread states.
 *
 * Must be before kernel_arch_data.h because it might need them to be already
 * defined.
 */

/* states: common uses low bits, arch-specific use high bits */

/* Not a real thread */
#define _THREAD_DUMMY (BIT(0))

/* Thread is waiting on an object */
#define _THREAD_PENDING (BIT(1))

/* Thread has not yet started */
#define _THREAD_PRESTART (BIT(2))

/* Thread has terminated */
#define _THREAD_DEAD (BIT(3))

/* Thread is suspended */
#define _THREAD_SUSPENDED (BIT(4))

/* Thread is in the process of aborting */
#define _THREAD_ABORTING (BIT(5))

/* Thread is in the process of suspending */
#define _THREAD_SUSPENDING (BIT(6))

/* Thread is present in the ready queue */
#define _THREAD_QUEUED (BIT(7))

/* end - states */

#ifdef CONFIG_STACK_SENTINEL
/* Magic value in lowest bytes of the stack */
#define STACK_SENTINEL 0xF0F0F0F0
#endif

/* lowest value of _thread_base.preempt at which a thread is non-preemptible */
#define _NON_PREEMPT_THRESHOLD 0x0080U

/* highest value of _thread_base.preempt at which a thread is preemptible */
#define _PREEMPT_THRESHOLD (_NON_PREEMPT_THRESHOLD - 1U)

#if !defined(_ASMLANGUAGE)

/* Traditional/textbook "multi-queue" structure.  Separate lists for a
 * small number (max 32 here) of fixed priorities.  This corresponds
 * to the original Zephyr scheduler.  RAM requirements are
 * comparatively high, but performance is very fast.  Won't work with
 * features like deadline scheduling which need large priority spaces
 * to represent their requirements.
 */
struct _priq_mq {
	struct list_node queues[K_NUM_THREAD_PRIO];
	unsigned long bitmask[PRIQ_BITMAP_SIZE];
};

struct _ready_q {
#ifndef CONFIG_SMP
	/* always contains next thread to run: cannot be NULL */
	pid_t cache;
#endif

#if defined(CONFIG_SCHED_DUMB)
	struct list_node runq;
#elif defined(CONFIG_SCHED_MULTIQ)
	struct _priq_mq runq;
#endif
};

typedef struct _ready_q _ready_q_t;

struct _cpu {
	/* nested interrupt count */
	uint32_t nested;

	/* interrupt stack pointer base */
	char *irq_stack;

	/* currently scheduled thread */
	pid_t current;

	/* one assigned idle thread per CPU */
	pid_t idle_thread;

#ifdef CONFIG_SCHED_CPU_MASK_PIN_ONLY
	struct _ready_q ready_q;
#endif

	uint8_t id;

#if defined(CONFIG_FPU_SHARING)
	void *fp_ctx;
#endif

#ifdef CONFIG_SMP
	/* True when _current is allowed to context switch */
	uint8_t swap_ok;
#endif

	/* Per CPU architecture specifics */
	struct _cpu_arch arch;
};

typedef struct _cpu _cpu_t;

struct z_kernel {
	struct _cpu cpus[CONFIG_MP_MAX_NUM_CPUS];

#ifdef CONFIG_PM
	int32_t idle; /* Number of ticks for kernel idling */
#endif

	/*
	 * ready queue: can be big, keep after small fields, since some
	 * assembly (e.g. ARC) are limited in the encoding of the offset
	 */
#ifndef CONFIG_SCHED_CPU_MASK_PIN_ONLY
	struct _ready_q ready_q;
#endif

#ifdef CONFIG_FPU_SHARING
	/*
	 * A 'current_sse' field does not exist in addition to the 'current_fp'
	 * field since it's not possible to divide the IA-32 non-integer
	 * registers into 2 distinct blocks owned by differing threads.  In
	 * other words, given that the 'fxnsave/fxrstor' instructions
	 * save/restore both the X87 FPU and XMM registers, it's not possible
	 * for a thread to only "own" the XMM registers.
	 */

	/* thread that owns the FP regs */
	pid_t current_fp;
#endif

#if defined(CONFIG_THREAD_MONITOR)
	pid_t threads;
#endif
#if defined(CONFIG_SMP) && defined(CONFIG_SCHED_IPI_SUPPORTED)
	/* Identify CPUs to send IPIs to at the next scheduling point */
	int32_t pending_ipi;
#endif
};

typedef struct z_kernel _kernel_t;

extern struct z_kernel _kernel;

extern int32_t _cpus_active;

#ifdef CONFIG_SMP

/* True if the current context can be preempted and migrated to
 * another SMP CPU.
 */
bool z_smp_cpu_mobile(void);

#define _current_cpu ({ DEBUGASSERT(!z_smp_cpu_mobile()); \
			arch_curr_cpu(); })
#define _current k_sched_current_thread_query()

#else
#define _current_cpu (&_kernel.cpus[0])
#define _current k_sched_current_thread_query()
#endif

/* kernel wait queue record */
typedef struct {
	struct list_node waitq;
} _wait_q_t;

#define Z_WAIT_Q_INIT(wait_q) { LIST_INITIAL_VALUE((wait_q)->waitq) }

/* kernel timeout record */
struct _timeout;
typedef void (*_timeout_func_t)(struct _timeout *t);

struct _timeout {
	struct list_node node;
	_timeout_func_t fn;
#ifdef CONFIG_TIMEOUT_64BIT
	/* Can't use clock_t for header dependency reasons */
	int64_t dticks;
#else
	int32_t dticks;
#endif
};

typedef void (*k_thread_timeslice_fn_t)(pid_t thread, void *data);

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_KERNEL_INCLUDE_KERNEL_STRUCTS_H_ */
