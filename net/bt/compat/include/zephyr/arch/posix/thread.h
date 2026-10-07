/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_ARCH_POSIX_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_POSIX_THREAD_H_

#include <pthread.h>

typedef pthread_t k_thread_stack_t;
typedef void (*k_thread_entry_t)(void *, void *, void *);

#endif