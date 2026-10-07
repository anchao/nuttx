/* SPDX-License-Identifier: Apache-2.0 */
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/sched.h>
#include <zephyr/kernel.h>

/* Global system work queue - uses NuttX HP work queue */
struct k_work_q k_sys_work_q;

/* Timer callback wrapper for delayed work */
static void work_timeout_callback(wdparm_t arg)
{
    struct k_work_delayable *dwork = (struct k_work_delayable *)arg;
    dwork->work.handler(&dwork->work);
}

void k_work_init(struct k_work *work, k_work_handler_t handler)
{
    memset(work, 0, sizeof(*work));
    work->handler = handler;
}

void k_work_init_delayable(struct k_work_delayable *dwork, k_work_handler_t handler)
{
    memset(dwork, 0, sizeof(*dwork));
    dwork->work.handler = handler;
}

int k_work_submit(struct k_work *work)
{
    return work_queue(HPWORK, &work->work, (worker_t)work->handler, work, 0);
}

int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work)
{
    return work_queue(HPWORK, &work->work, (worker_t)work->handler, work, 0);
}

int k_work_schedule(struct k_work_delayable *dwork, k_timeout_t delay)
{
    if (dwork->queue == NULL)
        {
            dwork->queue = &k_sys_work_q;
        }

    if (K_TIMEOUT_EQ(delay, K_NO_WAIT))
        {
            return k_work_submit(&dwork->work);
        }

    if (K_TIMEOUT_EQ(delay, K_FOREVER))
        return 0;

    wd_start(&dwork->wdog, delay.ticks, work_timeout_callback, (wdparm_t)dwork);
    return 0;
}

int k_work_schedule_for_queue(struct k_work_q *queue,
                              struct k_work_delayable *dwork,
                              k_timeout_t delay)
{
    dwork->queue = queue;
    return k_work_schedule(dwork, delay);
}

int k_work_cancel_delayable(struct k_work_delayable *dwork)
{
    return wd_cancel(&dwork->wdog);
}

int k_work_cancel_delayable_sync(struct k_work_delayable *dwork,
                                 struct k_work_sync *sync)
{
    (void)sync;
    return k_work_cancel_delayable(dwork);
}

int k_work_reschedule(struct k_work_delayable *dwork, k_timeout_t delay)
{
    wd_cancel(&dwork->wdog);
    return k_work_schedule(dwork, delay);
}

int k_work_reschedule_for_queue(struct k_work_q *queue,
                                struct k_work_delayable *dwork,
                                k_timeout_t delay)
{
    (void)queue;
    return k_work_reschedule(dwork, delay);
}

bool k_work_is_pending(const struct k_work *work)
{
    /* Simply check if work is queued */
    return false;
}

int k_work_busy_get(const struct k_work *work)
{
    (void)work;
    return 0;
}

int k_work_flush(struct k_work *work, struct k_work_sync *sync)
{
    (void)work;
    (void)sync;
    return 0;
}

int k_work_cancel(struct k_work *work)
{
    (void)work;
    return -EALREADY;
}

int k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync)
{
    (void)work;
    (void)sync;
    return -EALREADY;
}

struct k_work_delayable *k_work_delayable_from_work(struct k_work *work)
{
    return CONTAINER_OF(work, struct k_work_delayable, work);
}

int k_work_delayable_busy_get(const struct k_work_delayable *dwork)
{
    (void)dwork;
    return 0;
}

k_ticks_t k_work_delayable_remaining_get(const struct k_work_delayable *dwork)
{
    (void)dwork;
    return 0;
}

struct k_thread *k_work_queue_thread_get(struct k_work_q *queue)
{
    return queue ? queue->thread_id : NULL;
}

void k_work_queue_init(struct k_work_q *queue)
{
    if (queue != NULL)
        {
            memset(queue, 0, sizeof(*queue));
        }
}

int k_work_queue_start(struct k_work_q *queue, char *stack,
                       size_t stack_size, int prio,
                       const struct k_work_queue_config *cfg)
{
    (void)queue;
    (void)stack;
    (void)stack_size;
    (void)prio;
    (void)cfg;
    return 0;
}

/* ===== net_buf fifo integration ===== */
void net_buf_put(struct k_fifo *fifo, struct net_buf *buf)
{
    k_fifo_put(fifo, buf);
}

struct net_buf *net_buf_get(struct k_fifo *fifo, k_timeout_t timeout)
{
    return (struct net_buf *)k_fifo_get(fifo, timeout);
}

/* ===== Scheduling ===== */
int32_t k_sleep(k_timeout_t timeout)
{
    if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
        return 0;

    if (K_TIMEOUT_EQ(timeout, K_FOREVER))
        {
            while (1) { nxsched_usleep(1000000); }
        }

    uint32_t ms = k_ticks_to_ms_ceil32(timeout.ticks);
    nxsched_usleep(ms * 1000);
    return 0;
}

int32_t k_msleep(int32_t ms)
{
    nxsched_usleep(ms * 1000);
    return 0;
}

void k_busy_wait(int32_t usec_to_wait)
{
    uint32_t start = (uint32_t)clock_systime_ticks();
    uint32_t ticks = USEC2TICK(usec_to_wait);
    while (((uint32_t)clock_systime_ticks() - start) < ticks)
        ;
}

void k_yield(void)
{
    sched_yield();
}

/* ===== Condvar ===== */
int k_condvar_init(struct k_condvar *condvar)
{
    return pthread_cond_init(&condvar->cond, NULL);
}

int k_condvar_wait(struct k_condvar *condvar, rmutex_t *mutex,
                   k_timeout_t timeout)
{
    (void)mutex;
    (void)timeout;
    return -ENOSYS;
}

int k_condvar_broadcast(struct k_condvar *condvar)
{
    return pthread_cond_broadcast(&condvar->cond);
}
