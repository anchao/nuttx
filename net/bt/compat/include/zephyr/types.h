/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_TYPES_H_
#define ZEPHYR_INCLUDE_TYPES_H_

/* Zephyr types.h compatibility for NuttX.
 * NuttX already provides stdint.h, stddef.h, stdbool.h, etc.
 * This header re-exports types under Zephyr-compatible names.
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#include <errno.h>

/* Standard integer types */
typedef int8_t   s8_t;
typedef uint8_t  u8_t;
typedef int16_t  s16_t;
typedef uint16_t u16_t;
typedef int32_t  s32_t;
typedef uint32_t u32_t;
typedef int64_t  s64_t;
typedef uint64_t u64_t;

/* Time types */
typedef int64_t k_time_t;

#endif /* ZEPHYR_INCLUDE_TYPES_H_ */