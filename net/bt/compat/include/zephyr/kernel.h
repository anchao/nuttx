/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_KERNEL_H_
#define ZEPHYR_INCLUDE_KERNEL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/spinlock.h>
#include <nuttx/clock.h>
#include <pthread.h>

#include "sys_clock.h"
#include "sys/atomic.h"
#include "sys/slist.h"
#include "sys/sflist.h"
#include "sys/dlist.h"
#include "toolchain.h"
#include "arch/posix/arch.h"
#include "irq.h"
#include "init.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Semaphore ===== */
struct k_sem {
    sem_t sem;
    unsigned int limit;
    unsigned int count;
};

#define Z_SEM_INITIALIZER(obj, initial_count, count_limit) \
    { .sem = NXSEM_INITIALIZER(initial_count, count_limit), .limit = (count_limit), .count = (initial_count) }

int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit);
int k_sem_take(struct k_sem *sem, k_timeout_t timeout);
void k_sem_give(struct k_sem *sem);
unsigned int k_sem_count_get(struct k_sem *sem);
void k_sem_reset(struct k_sem *sem);

#define K_SEM_DEFINE(name, initial_count, count_limit) \
    struct k_sem name = Z_SEM_INITIALIZER(name, initial_count, count_limit)

/* ===== Mutex ===== */
struct k_mutex {
    pthread_mutex_t mutex;
    unsigned int lock_count;
};

#define Z_MUTEX_INITIALIZER(obj) { .mutex = PTHREAD_MUTEX_INITIALIZER, .lock_count = 0 }

int k_mutex_init(struct k_mutex *mutex);
int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout);
int k_mutex_unlock(struct k_mutex *mutex);

#define K_MUTEX_DEFINE(name) struct k_mutex name = Z_MUTEX_INITIALIZER(name)

/* ===== FIFO / Queue ===== */
struct k_fifo {
    sys_sflist_t data_q;
    sys_dlist_t wait_q;
    spinlock_t lock;  /* Use raw spinlock for ISR safety */
    struct k_spinlock _queue;  /* Internal queue wrapper for locking */
};

void k_fifo_init(struct k_fifo *fifo);
void k_fifo_cancel_wait(struct k_fifo *fifo);
void k_fifo_put(struct k_fifo *fifo, void *data);
void *k_fifo_get(struct k_fifo *fifo, k_timeout_t timeout);
bool k_fifo_is_empty(struct k_fifo *fifo);
void *k_fifo_peek_head(struct k_fifo *fifo);
void *k_fifo_peek_tail(struct k_fifo *fifo);
void z_k_queue_prepend_fifo(struct k_fifo *fifo, void *data);
void z_k_queue_prepend_spinlock(struct k_spinlock *queue, void *data);
bool z_k_queue_remove_fifo(struct k_fifo *fifo, void *data);
bool z_k_queue_remove_spinlock(struct k_spinlock *queue, void *data);

#define k_queue k_fifo
#define k_queue_init(queue) k_fifo_init(queue)
#define k_queue_append(queue, data) k_fifo_put(queue, data)
#define k_queue_get(queue, timeout) k_fifo_get(queue, timeout)
#define k_queue_peek_head(queue) k_fifo_peek_head(queue)
#define k_queue_prepend(queue, data) \
    _Generic((queue), \
             struct k_spinlock *: z_k_queue_prepend_spinlock, \
             default: z_k_queue_prepend_fifo)(queue, data)
#define k_queue_remove(queue, data) \
    _Generic((queue), \
             struct k_spinlock *: z_k_queue_remove_spinlock, \
             default: z_k_queue_remove_fifo)(queue, data)

#define K_FIFO_DEFINE(name) \
    struct k_fifo name = { \
        .data_q = SYS_SFLIST_STATIC_INIT(&name.data_q), \
        .wait_q = SYS_DLIST_STATIC_INIT(&name.wait_q), \
    }

/* ===== Thread ===== */
typedef struct k_thread *k_tid_t;

typedef void (*k_thread_entry_t)(void *p1, void *p2, void *p3);

struct k_thread {
    pthread_t pthread;
    k_thread_entry_t entry;
    void *p1;
    void *p2;
    void *p3;
    char *stack;
    size_t stack_size;
    int prio;
    const char *name;
};

#define K_THREAD_STACK_BUFFER(sym, size)
#define K_THREAD_STACK_MEMBER(sym, size) char sym[size]
#define K_THREAD_STACK_SIZEOF(sym) sizeof(sym)
#define K_THREAD_STACK_DEFINE(sym, size) K_THREAD_STACK_MEMBER(sym, size)

#define K_KERNEL_STACK_DEFINE(sym, size) K_THREAD_STACK_DEFINE(sym, size)
#define K_KERNEL_STACK_SIZEOF(sym) K_THREAD_STACK_SIZEOF(sym)
#define K_KERNEL_STACK_MEMBER(sym, size) K_THREAD_STACK_MEMBER(sym, size)

#define K_THREAD_DEFINE(name, stack_size, entry, p1, p2, p3, prio, options, delay) \
    K_THREAD_STACK_DEFINE(_k_thread_stack_##name, stack_size); \
    static struct k_thread _k_thread_obj_##name = { \
        .entry = entry, .p1 = p1, .p2 = p2, .p3 = p3, \
        .stack = _k_thread_stack_##name, .stack_size = stack_size, \
        .prio = prio, .name = #name, \
    }; \
    const k_tid_t name = (k_tid_t)&_k_thread_obj_##name

k_tid_t k_thread_create(struct k_thread *new_thread, char *stack,
                        size_t stack_size, k_thread_entry_t entry,
                        void *p1, void *p2, void *p3,
                        int prio, uint32_t options, k_timeout_t delay);
void k_thread_start(k_tid_t thread);
int k_thread_join(struct k_thread *thread, k_timeout_t timeout);
int k_thread_name_set(k_tid_t thread, const char *name);
struct k_thread *k_current_get(void);

/* ===== Work Queue ===== */
struct k_work {
    struct work_s work;
    void (*handler)(struct k_work *work);
};

struct k_work_delayable {
    struct k_work work;
    struct wdog_s wdog;
    struct k_work_q *queue;
};

struct k_work_q {
    struct work_s queue;
    k_tid_t thread_id;
};

struct k_work_queue_config {
    const char *name;
};

/* Work queue synchronization */
struct k_work_sync {
    struct k_sem sem;
};

typedef void (*k_work_handler_t)(struct k_work *work);

#define Z_WORK_INITIALIZER(work_handler) \
    { .handler = (k_work_handler_t)(work_handler) }

#define Z_WORK_DELAYABLE_INITIALIZER(work_handler) \
    { .work = Z_WORK_INITIALIZER(work_handler) }

#define K_WORK_DEFINE(work, work_handler) \
    struct k_work work = { .handler = (work_handler) }

#define K_WORK_DELAYABLE_DEFINE(dwork, work_handler) \
    struct k_work_delayable dwork = { .work = { .handler = (work_handler) } }

/* Work flags for k_work */
#define K_WORK_QUEUED    BIT(0)
#define K_WORK_DELAYED   BIT(1)
#define K_WORK_RUNNING   BIT(2)

void k_work_init(struct k_work *work, k_work_handler_t handler);
void k_work_init_delayable(struct k_work_delayable *dwork, k_work_handler_t handler);
int k_work_submit(struct k_work *work);
int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work);
int k_work_schedule(struct k_work_delayable *dwork, k_timeout_t delay);
int k_work_schedule_for_queue(struct k_work_q *queue,
                              struct k_work_delayable *dwork,
                              k_timeout_t delay);
int k_work_cancel_delayable(struct k_work_delayable *dwork);
int k_work_cancel_delayable_sync(struct k_work_delayable *dwork,
                                 struct k_work_sync *sync);
int k_work_reschedule(struct k_work_delayable *dwork, k_timeout_t delay);
int k_work_reschedule_for_queue(struct k_work_q *queue,
                                struct k_work_delayable *dwork,
                                k_timeout_t delay);
bool k_work_is_pending(const struct k_work *work);
int k_work_busy_get(const struct k_work *work);
static inline bool k_work_delayable_is_pending(const struct k_work_delayable *dwork)
{
    return k_work_is_pending(&dwork->work);
}
int k_work_flush(struct k_work *work, struct k_work_sync *sync);

int k_work_cancel(struct k_work *work);
int k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync);
struct k_work_delayable *k_work_delayable_from_work(struct k_work *work);
int k_work_delayable_busy_get(const struct k_work_delayable *dwork);
k_ticks_t k_work_delayable_remaining_get(const struct k_work_delayable *dwork);
struct k_thread *k_work_queue_thread_get(struct k_work_q *queue);
void k_work_queue_init(struct k_work_q *queue);
int k_work_queue_start(struct k_work_q *queue, char *stack,
                       size_t stack_size, int prio,
                       const struct k_work_queue_config *cfg);

#define K_PRIO_COOP(prio) (prio)

extern struct k_work_q k_sys_work_q;

/* ===== Time ===== */
typedef struct {
    int32_t ticks;
} k_timepoint_t;

static inline k_timepoint_t sys_timepoint_calc(k_timeout_t timeout)
{
    if (K_TIMEOUT_EQ(timeout, K_FOREVER))
        {
            return (k_timepoint_t){ .ticks = -1 };
        }

    return (k_timepoint_t){ .ticks = (int32_t)clock_systime_ticks() + timeout.ticks };
}

static inline bool sys_timepoint_expired(k_timepoint_t timepoint)
{
    if (timepoint.ticks < 0)
        {
            return false;
        }

    return ((int32_t)clock_systime_ticks() - timepoint.ticks) >= 0;
}

static inline k_timeout_t sys_timepoint_timeout(k_timepoint_t timepoint)
{
    int32_t ticks;

    if (timepoint.ticks < 0)
        {
            return K_FOREVER;
        }

    ticks = timepoint.ticks - (int32_t)clock_systime_ticks();
    return ticks > 0 ? (k_timeout_t){ .ticks = ticks } : K_NO_WAIT;
}

int32_t k_sleep(k_timeout_t timeout);
int32_t k_msleep(int32_t ms);
void k_busy_wait(int32_t usec_to_wait);
void k_yield(void);

/* ===== Memory Slab ===== */
struct k_mem_slab {
    char *buffer;
    char *free_list;
    size_t block_size;
    uint32_t num_blocks;
    uint32_t num_used;
    uint32_t max_used;
    spinlock_t lock;
    sys_dlist_t wait_q;
};

void k_mem_slab_init(struct k_mem_slab *slab, void *buffer, size_t block_size,
                     uint32_t num_blocks);
int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem, k_timeout_t timeout);
void k_mem_slab_free(struct k_mem_slab *slab, void **mem);
uint32_t k_mem_slab_num_free_get(struct k_mem_slab *slab);
uint32_t k_mem_slab_max_used_get(struct k_mem_slab *slab);

#define K_MEM_SLAB_DEFINE(name, slab_block_size, slab_num_blocks, slab_align) \
    static char __aligned(slab_align) _k_mem_slab_buf_##name[(slab_block_size) * (slab_num_blocks)]; \
    struct k_mem_slab name = { \
        .buffer = _k_mem_slab_buf_##name, \
        .block_size = (slab_block_size), \
        .num_blocks = (slab_num_blocks), \
    }

#define K_MEM_SLAB_DEFINE_TYPE(name, type, num_blocks, ...) \
    K_MEM_SLAB_DEFINE(name, sizeof(type), num_blocks, __alignof__(type))

#define K_MEM_SLAB_DEFINE_STATIC_TYPE(name, type, num_blocks, ...) \
    K_MEM_SLAB_DEFINE(name, sizeof(type), num_blocks, __alignof__(type))

#define K_MEM_SLAB_DEFINE_STATIC(name, slab_block_size, slab_num_blocks, slab_align) \
    static K_MEM_SLAB_DEFINE(name, slab_block_size, slab_num_blocks, slab_align)

/* ===== Poll ===== */
#define K_POLL_TYPE_SEM_AVAILABLE    0
#define K_POLL_TYPE_DATA_AVAILABLE   1
#define K_POLL_TYPE_SIGNAL           2
#define K_POLL_TYPE_MSGQ_DATA_AVAILABLE 3
#define K_POLL_TYPE_FIFO_DATA_AVAILABLE 4

#define K_POLL_STATE_SEM_AVAILABLE    0x001
#define K_POLL_STATE_DATA_AVAILABLE   0x002
#define K_POLL_STATE_FIFO_DATA_AVAILABLE 0x004
#define K_POLL_STATE_SIGNALED         0x008
#define K_POLL_STATE_MSGQ_DATA_AVAILABLE 0x010
#define K_POLL_STATE_CANCELLED        0x020
#define K_POLL_STATE_NOT_READY        0x080

#define K_POLL_MODE_NOTIFY_ONLY       0

struct k_poll_signal {
    sys_dlist_t poll_events;
    unsigned int signaled;
    int result;
};

#define K_POLL_SIGNAL_INITIALIZER(obj) \
    { .poll_events = SYS_DLIST_STATIC_INIT(&obj.poll_events), .signaled = 0, .result = 0 }

void k_poll_signal_init(struct k_poll_signal *signal);
void k_poll_signal_check(struct k_poll_signal *signal, unsigned int *signaled, int *result);
int k_poll_signal_raise(struct k_poll_signal *signal, int result);
void k_poll_signal_reset(struct k_poll_signal *signal);

struct k_poll_event {
    int type;
    uint32_t mode;
    uint32_t state;
    union {
        void *obj;
        struct k_poll_signal *signal;
        struct k_sem *sem;
        struct k_fifo *fifo;
    };
    int tag;
};

#define K_POLL_EVENT_STATIC_INITIALIZER(_type, _mode, _obj, _tag) \
    { .type = (_type), .mode = (_mode), .state = K_POLL_STATE_NOT_READY, \
      .obj = (void *)(_obj), .tag = (_tag) }

int k_poll(struct k_poll_event *events, int num_events, k_timeout_t timeout);

/* ===== net_buf fifo integration ===== */
struct net_buf;
void net_buf_put(struct k_fifo *fifo, struct net_buf *buf);
struct net_buf *net_buf_get(struct k_fifo *fifo, k_timeout_t timeout);

/* ===== Miscellaneous ===== */
struct k_condvar {
    pthread_cond_t cond;
    pthread_mutex_t *mutex;
};

#define Z_CONDVAR_INITIALIZER(obj) { .cond = PTHREAD_COND_INITIALIZER }
int k_condvar_init(struct k_condvar *condvar);
int k_condvar_wait(struct k_condvar *condvar, struct k_mutex *mutex, k_timeout_t timeout);
int k_condvar_broadcast(struct k_condvar *condvar);

/* k_panic / k_oops - stubs */
static inline void k_panic(void)
{
    PANIC();
}

static inline void k_oops(void)
{
    PANIC();
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_KERNEL_H_ */
