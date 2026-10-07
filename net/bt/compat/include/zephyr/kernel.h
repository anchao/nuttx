/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_KERNEL_H_
#define ZEPHYR_INCLUDE_KERNEL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sched.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <nuttx/wqueue.h>
#include <nuttx/spinlock.h>
#include <nuttx/clock.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <pthread.h>

#include "sys_clock.h"
#include <nuttx/atomic.h>
#include "toolchain.h"
#include "arch/posix/arch.h"
#include "irq.h"
#include "init.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== FIFO / Queue ===== */
struct k_fifo {
    struct list_node data_q;
    struct list_node wait_q;
    spinlock_t lock;  /* Use raw spinlock for ISR safety */
};

void k_fifo_init(struct k_fifo *fifo);
void k_fifo_cancel_wait(struct k_fifo *fifo);
void k_fifo_put(struct k_fifo *fifo, void *data);
void *k_fifo_get(struct k_fifo *fifo, clock_t timeout);
bool k_fifo_is_empty(struct k_fifo *fifo);
void *k_fifo_peek_head(struct k_fifo *fifo);
void *k_fifo_peek_tail(struct k_fifo *fifo);
void z_k_queue_prepend_fifo(struct k_fifo *fifo, void *data);
bool z_k_queue_remove_fifo(struct k_fifo *fifo, void *data);

#define k_queue k_fifo
#define k_queue_init(queue) k_fifo_init(queue)
#define k_queue_append(queue, data) k_fifo_put(queue, data)
#define k_queue_get(queue, timeout) k_fifo_get(queue, timeout)
#define k_queue_peek_head(queue) k_fifo_peek_head(queue)
#define k_queue_prepend(queue, data) z_k_queue_prepend_fifo(queue, data)
#define k_queue_remove(queue, data) z_k_queue_remove_fifo(queue, data)

#define K_FIFO_DEFINE(name) \
    struct k_fifo name = { \
        .data_q = LIST_INITIAL_VALUE(name.data_q), \
        .wait_q = LIST_INITIAL_VALUE(name.wait_q), \
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
                        int prio, uint32_t options, clock_t delay);
void k_thread_start(k_tid_t thread);
int k_thread_join(struct k_thread *thread, clock_t timeout);
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
    sem_t sem;
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
int k_work_schedule(struct k_work_delayable *dwork, clock_t delay);
int k_work_schedule_for_queue(struct k_work_q *queue,
                              struct k_work_delayable *dwork,
                              clock_t delay);
int k_work_cancel_delayable(struct k_work_delayable *dwork);
int k_work_cancel_delayable_sync(struct k_work_delayable *dwork,
                                 struct k_work_sync *sync);
int k_work_reschedule(struct k_work_delayable *dwork, clock_t delay);
int k_work_reschedule_for_queue(struct k_work_q *queue,
                                struct k_work_delayable *dwork,
                                clock_t delay);
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
clock_t k_work_delayable_remaining_get(const struct k_work_delayable *dwork);
struct k_thread *k_work_queue_thread_get(struct k_work_q *queue);
void k_work_queue_init(struct k_work_q *queue);
int k_work_queue_start(struct k_work_q *queue, char *stack,
                       size_t stack_size, int prio,
                       const struct k_work_queue_config *cfg);

#define K_PRIO_COOP(prio) (prio)

extern struct k_work_q k_sys_work_q;

/* ===== Memory Slab ===== */
struct k_mem_slab {
    char *buffer;
    char *free_list;
    size_t block_size;
    uint32_t num_blocks;
    uint32_t num_used;
    uint32_t max_used;
    spinlock_t lock;
    struct list_node wait_q;
};

void k_mem_slab_init(struct k_mem_slab *slab, void *buffer, size_t block_size,
                     uint32_t num_blocks);
int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem, clock_t timeout);
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
    struct list_node poll_events;
    unsigned int signaled;
    int result;
};

#define K_POLL_SIGNAL_INITIALIZER(obj) \
    { .poll_events = LIST_INITIAL_VALUE(obj.poll_events), .signaled = 0, .result = 0 }

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
        sem_t *sem;
        struct k_fifo *fifo;
    };
    int tag;
};

#define K_POLL_EVENT_STATIC_INITIALIZER(_type, _mode, _obj, _tag) \
    { .type = (_type), .mode = (_mode), .state = K_POLL_STATE_NOT_READY, \
      .obj = (void *)(_obj), .tag = (_tag) }

int k_poll(struct k_poll_event *events, int num_events, clock_t timeout);

/* ===== net_buf fifo integration ===== */
struct net_buf;
void net_buf_put(struct k_fifo *fifo, struct net_buf *buf);
struct net_buf *net_buf_get(struct k_fifo *fifo, clock_t timeout);

/* ===== Miscellaneous ===== */
struct k_condvar {
    pthread_cond_t cond;
};

#define Z_CONDVAR_INITIALIZER(obj) { .cond = PTHREAD_COND_INITIALIZER }
int k_condvar_init(struct k_condvar *condvar);
int k_condvar_wait(struct k_condvar *condvar, rmutex_t *mutex, clock_t timeout);
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
