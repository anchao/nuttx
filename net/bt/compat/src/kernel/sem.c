/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <nuttx/clock.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit)
{
    sem->limit = limit;
    sem->count = initial_count;
    return nxsem_init(&sem->sem, 0, initial_count);
}

int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
    int ret;

    if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
        {
            ret = nxsem_trywait(&sem->sem);
            if (ret < 0)
                return -EBUSY;
            sem->count++;
            return 0;
        }

    if (K_TIMEOUT_EQ(timeout, K_FOREVER))
        {
            ret = nxsem_wait_uninterruptible(&sem->sem);
            if (ret < 0)
                return -EINTR;
            sem->count++;
            return 0;
        }

    ret = nxsem_tickwait(&sem->sem, timeout.ticks);
    if (ret < 0)
        return -EAGAIN;
    sem->count++;
    return 0;
}

void k_sem_give(struct k_sem *sem)
{
    if (sem->count < sem->limit)
        {
            nxsem_post(&sem->sem);
            sem->count--;
        }
}

unsigned int k_sem_count_get(struct k_sem *sem)
{
    int count;
    if (nxsem_get_value(&sem->sem, &count) < 0)
        return 0;
    return (unsigned int)count;
}

void k_sem_reset(struct k_sem *sem)
{
    nxsem_reset(&sem->sem, 0);
    sem->count = 0;
}

int k_sem_init_with_struct(struct k_sem *sem, unsigned int count, unsigned int limit)
{
    return k_sem_init(sem, count, limit);
}