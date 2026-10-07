/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <time.h>
#include <zephyr/kernel.h>

int k_mutex_init(struct k_mutex *mutex)
{
    pthread_mutexattr_t attr;
    int ret;

    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    ret = pthread_mutex_init(&mutex->mutex, &attr);
    pthread_mutexattr_destroy(&attr);

    if (ret == 0)
        {
            mutex->lock_count = 0;
        }

    return ret == 0 ? 0 : -ret;
}

int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
    int ret;

    if (K_TIMEOUT_EQ(timeout, K_FOREVER))
        {
            ret = pthread_mutex_lock(&mutex->mutex);
            if (ret == 0)
                {
                    mutex->lock_count++;
                }
            return ret == 0 ? 0 : -ret;
        }

    if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
        {
            ret = pthread_mutex_trylock(&mutex->mutex);
            if (ret == 0)
                {
                    mutex->lock_count++;
                }
            return ret == 0 ? 0 : -EBUSY;
        }

    {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        uint32_t ms = k_ticks_to_ms_ceil32(timeout.ticks);
        ts.tv_sec += ms / 1000;
        ts.tv_nsec += (ms % 1000) * 1000000;
        if (ts.tv_nsec >= 1000000000)
            {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000;
            }
        ret = pthread_mutex_timedlock(&mutex->mutex, &ts);
        if (ret == 0)
            {
                mutex->lock_count++;
            }
        return ret == 0 ? 0 : -EAGAIN;
    }
}

int k_mutex_unlock(struct k_mutex *mutex)
{
    int ret = pthread_mutex_unlock(&mutex->mutex);
    if (ret == 0 && mutex->lock_count > 0)
        {
            mutex->lock_count--;
        }

    return ret == 0 ? 0 : -ret;
}
