/* SPDX-License-Identifier: Apache-2.0 */
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>
#include <zephyr/kernel.h>

static void *thread_wrapper(void *arg)
{
    struct k_thread *thread = (struct k_thread *)arg;

    /* Execute entry function */
    thread->entry(thread->p1, thread->p2, thread->p3);

    return NULL;
}

k_tid_t k_thread_create(struct k_thread *new_thread, char *stack,
                        size_t stack_size, k_thread_entry_t entry,
                        void *p1, void *p2, void *p3,
                        int prio, uint32_t options, clock_t delay)
{
    pthread_attr_t attr;
    int ret;

    (void)options;
    (void)delay;

    new_thread->entry = entry;
    new_thread->p1 = p1;
    new_thread->p2 = p2;
    new_thread->p3 = p3;
    new_thread->stack = stack;
    new_thread->stack_size = stack_size;
    new_thread->prio = prio;

    pthread_attr_init(&attr);
    pthread_attr_setstack(&attr, stack, stack_size);

    /* Convert Zephyr priority to NuttX priority.
     * Zephyr: lower number = higher priority (0 is highest)
     * NuttX: lower number = higher priority
     */
    struct sched_param sparam;
    sparam.sched_priority = prio;
    pthread_attr_setschedparam(&attr, &sparam);

    ret = pthread_create(&new_thread->pthread, &attr, thread_wrapper, new_thread);
    pthread_attr_destroy(&attr);

    if (ret != 0)
        return NULL;

    return new_thread;
}

void k_thread_start(k_tid_t thread)
{
    /* Thread created with pthread_create already starts running */
    (void)thread;
}

int k_thread_join(struct k_thread *thread, clock_t timeout)
{
    (void)timeout;
    return pthread_join(thread->pthread, NULL);
}

int k_thread_name_set(k_tid_t thread, const char *name)
{
    if (thread)
        {
            thread->name = name;
            pthread_setname_np(thread->pthread, name);
        }
    return 0;
}

struct k_thread *k_current_get(void)
{
    /* Return NULL stub - caller should handle this */
    return NULL;
}