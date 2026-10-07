/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <errno.h>
#include <string.h>

void k_fifo_init(struct k_fifo *fifo)
{
    sys_sflist_init(&fifo->data_q);
    sys_dlist_init(&fifo->wait_q);
    fifo->lock = SP_UNLOCKED;
}

void k_fifo_cancel_wait(struct k_fifo *fifo)
{
    spin_lock_irqsave(&fifo->lock);
    /* Wake all waiters - simplified for single-core */
    spin_unlock_irqrestore(&fifo->lock, SP_UNLOCKED);
}

void k_fifo_put(struct k_fifo *fifo, void *data)
{
    irqstate_t flags = spin_lock_irqsave(&fifo->lock);
    sys_sflist_append(&fifo->data_q, (sys_sfnode_t *)data);
    spin_unlock_irqrestore(&fifo->lock, flags);
}

void *k_fifo_get(struct k_fifo *fifo, k_timeout_t timeout)
{
    irqstate_t flags;
    sys_sfnode_t *node;

    if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
        {
            flags = spin_lock_irqsave(&fifo->lock);
            node = sys_sflist_get(&fifo->data_q);
            spin_unlock_irqrestore(&fifo->lock, flags);
            return node;
        }

    /* Blocking wait - spin then poll with delay */
    if (K_TIMEOUT_EQ(timeout, K_FOREVER))
        {
            while (1)
                {
                    flags = spin_lock_irqsave(&fifo->lock);
                    node = sys_sflist_get(&fifo->data_q);
                    spin_unlock_irqrestore(&fifo->lock, flags);
                    if (node)
                        return node;
                    /* Yield to allow producer */
                    k_yield();
                }
        }

    /* Timed wait */
    {
        k_ticks_t deadline = k_uptime_ticks() + timeout.ticks;
        while (k_uptime_ticks() < deadline)
            {
                flags = spin_lock_irqsave(&fifo->lock);
                node = sys_sflist_get(&fifo->data_q);
                spin_unlock_irqrestore(&fifo->lock, flags);
                if (node)
                    return node;
                k_yield();
            }
    }

    return NULL;
}

bool k_fifo_is_empty(struct k_fifo *fifo)
{
    irqstate_t flags = spin_lock_irqsave(&fifo->lock);
    bool empty = sys_sflist_is_empty(&fifo->data_q);
    spin_unlock_irqrestore(&fifo->lock, flags);
    return empty;
}

void *k_fifo_peek_head(struct k_fifo *fifo)
{
    irqstate_t flags = spin_lock_irqsave(&fifo->lock);
    sys_sfnode_t *node = sys_sflist_peek_head(&fifo->data_q);
    spin_unlock_irqrestore(&fifo->lock, flags);
    return node;
}

void *k_fifo_peek_tail(struct k_fifo *fifo)
{
    irqstate_t flags = spin_lock_irqsave(&fifo->lock);
    sys_sfnode_t *node = sys_sflist_peek_tail(&fifo->data_q);
    spin_unlock_irqrestore(&fifo->lock, flags);
    return node;
}

void z_k_queue_prepend_fifo(struct k_fifo *fifo, void *data)
{
    irqstate_t flags = spin_lock_irqsave(&fifo->lock);
    sys_sflist_prepend(&fifo->data_q, (sys_sfnode_t *)data);
    spin_unlock_irqrestore(&fifo->lock, flags);
}

void z_k_queue_prepend_spinlock(struct k_spinlock *queue, void *data)
{
    struct k_fifo *fifo = container_of(queue, struct k_fifo, _queue);

    z_k_queue_prepend_fifo(fifo, data);
}

bool z_k_queue_remove_fifo(struct k_fifo *fifo, void *data)
{
    irqstate_t flags;
    bool removed;

    flags = spin_lock_irqsave(&fifo->lock);
    removed = sys_sflist_find_and_remove(&fifo->data_q,
                                         (sys_sfnode_t *)data);
    spin_unlock_irqrestore(&fifo->lock, flags);

    return removed;
}

bool z_k_queue_remove_spinlock(struct k_spinlock *queue, void *data)
{
    struct k_fifo *fifo = container_of(queue, struct k_fifo, _queue);

    return z_k_queue_remove_fifo(fifo, data);
}

int k_poll(struct k_poll_event *events, int num_events, k_timeout_t timeout)
{
    k_ticks_t deadline = 0;
    int i;

    if (!K_TIMEOUT_EQ(timeout, K_NO_WAIT) &&
        !K_TIMEOUT_EQ(timeout, K_FOREVER))
        {
            deadline = k_uptime_ticks() + timeout.ticks;
        }

    while (1)
        {
            for (i = 0; i < num_events; i++)
                {
                    events[i].state = K_POLL_STATE_NOT_READY;

                    switch (events[i].type)
                        {
                        case K_POLL_TYPE_FIFO_DATA_AVAILABLE:
                            if (events[i].fifo != NULL &&
                                k_fifo_peek_head(events[i].fifo) != NULL)
                                {
                                    events[i].state = K_POLL_STATE_FIFO_DATA_AVAILABLE;
                                    return 0;
                                }
                            break;

                        case K_POLL_TYPE_SEM_AVAILABLE:
                            if (events[i].sem != NULL &&
                                k_sem_count_get(events[i].sem) > 0)
                                {
                                    events[i].state = K_POLL_STATE_SEM_AVAILABLE;
                                    return 0;
                                }
                            break;

                        case K_POLL_TYPE_SIGNAL:
                            if (events[i].signal != NULL &&
                                events[i].signal->signaled != 0)
                                {
                                    events[i].state = K_POLL_STATE_SIGNALED;
                                    return 0;
                                }
                            break;

                        default:
                            break;
                        }
                }

            if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
                {
                    return -EAGAIN;
                }

            if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
                (k_ticks_t)k_uptime_ticks() >= deadline)
                {
                    return -EAGAIN;
                }

            k_yield();
        }
}
