/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <errno.h>
#include <sched.h>

void k_fifo_init(struct k_fifo *fifo)
{
    list_initialize(&fifo->data_q);
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
    list_add_tail(&fifo->data_q, (struct list_node *)data);
    spin_unlock_irqrestore(&fifo->lock, flags);
}

void *k_fifo_get(struct k_fifo *fifo, clock_t timeout)
{
    irqstate_t flags;
    struct list_node *node;

    if (((timeout) == (0)))
        {
            flags = spin_lock_irqsave(&fifo->lock);
            node = list_remove_head(&fifo->data_q);
            spin_unlock_irqrestore(&fifo->lock, flags);
            return node;
        }

    /* Blocking wait - spin then poll with delay */
    if (((timeout) == (((clock_t)-1))))
        {
            while (1)
                {
                    flags = spin_lock_irqsave(&fifo->lock);
                    node = list_remove_head(&fifo->data_q);
                    spin_unlock_irqrestore(&fifo->lock, flags);
                    if (node)
                        return node;
                    /* Yield to allow producer */
                    sched_yield();
                }
        }

    /* Timed wait */
    {
        clock_t deadline = clock_systime_ticks() + timeout;
        while (clock_systime_ticks() < deadline)
            {
                flags = spin_lock_irqsave(&fifo->lock);
                node = list_remove_head(&fifo->data_q);
                spin_unlock_irqrestore(&fifo->lock, flags);
                if (node)
                    return node;
                sched_yield();
            }
    }

    return NULL;
}

bool k_fifo_is_empty(struct k_fifo *fifo)
{
    irqstate_t flags = spin_lock_irqsave(&fifo->lock);
    bool empty = list_is_empty(&fifo->data_q);
    spin_unlock_irqrestore(&fifo->lock, flags);
    return empty;
}

void *k_fifo_peek_head(struct k_fifo *fifo)
{
    irqstate_t flags = spin_lock_irqsave(&fifo->lock);
    struct list_node *node = list_peek_head(&fifo->data_q);
    spin_unlock_irqrestore(&fifo->lock, flags);
    return node;
}

void z_k_queue_prepend_fifo(struct k_fifo *fifo, void *data)
{
    irqstate_t flags = spin_lock_irqsave(&fifo->lock);
    list_add_head(&fifo->data_q, (struct list_node *)data);
    spin_unlock_irqrestore(&fifo->lock, flags);
}

bool z_k_queue_remove_fifo(struct k_fifo *fifo, void *data)
{
    irqstate_t flags;
    bool removed;

    flags = spin_lock_irqsave(&fifo->lock);
    removed = list_find_and_remove(&fifo->data_q,
                                         (struct list_node *)data);
    spin_unlock_irqrestore(&fifo->lock, flags);

    return removed;
}

int k_poll(struct k_poll_event *events, int num_events, clock_t timeout)
{
    clock_t deadline = 0;
    int i;

    if (!((timeout) == (0)) &&
        !((timeout) == (((clock_t)-1))))
        {
            deadline = clock_systime_ticks() + timeout;
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

                        default:
                            break;
                        }
                }

            if (((timeout) == (0)))
                {
                    return -EAGAIN;
                }

            if (!((timeout) == (((clock_t)-1))) &&
                (clock_t)clock_systime_ticks() >= deadline)
                {
                    return -EAGAIN;
                }

            sched_yield();
        }
}
