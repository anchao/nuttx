/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_TIME_UNITS_H_
#define ZEPHYR_INCLUDE_SYS_TIME_UNITS_H_

#include <nuttx/clock.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SYS_FOREVER_MS (-1)
#define SYS_FOREVER_US (-1)

/* NuttX provides MSEC2TICK and USEC2TICK. Map the Zephyr conversions. */

/* ms -> ticks */
static inline uint32_t k_ms_to_ticks_ceil32(uint32_t ms)
{
    return (uint32_t)TICK2MSEC(MSEC2TICK(ms));  /* Round-trip via NuttX macros */
}

static inline uint64_t k_ms_to_ticks_ceil64(uint64_t ms)
{
    return k_ms_to_ticks_ceil32((uint32_t)ms);
}

/* ticks -> ms - if sys_clock.h hasn't already defined it */
#ifndef Z_HAVE_K_TICKS_TO_MS_CEIL32
static inline uint32_t k_ticks_to_ms_ceil32(uint32_t t)
{
    return (uint32_t)MSEC2TICK(TICK2MSEC(t));
}
#endif

/* us -> ticks */
static inline uint32_t k_us_to_ticks_ceil32(uint32_t us)
{
    return (uint32_t)USEC2TICK(us);
}

static inline uint64_t k_us_to_ticks_ceil64(uint64_t us)
{
    return k_us_to_ticks_ceil32((uint32_t)us);
}

/* ticks -> us */
static inline uint32_t k_ticks_to_us_ceil32(uint32_t t)
{
    return (uint32_t)TICK2USEC(t);
}

/* ns -> ticks */
static inline uint64_t k_ns_to_ticks_ceil64(uint64_t ns)
{
    return k_us_to_ticks_ceil64(ns / 1000);
}

static inline uint32_t k_ns_to_ticks_ceil32(uint32_t ns)
{
    return k_us_to_ticks_ceil32(ns / 1000);
}

/* sec -> ticks */
static inline uint32_t k_sec_to_ticks_ceil32(uint32_t s)
{
    return k_ms_to_ticks_ceil32(s * 1000);
}

/* cyc -> ticks */
static inline uint32_t k_cyc_to_ticks_ceil32(uint32_t cyc)
{
    return 0; /* No cycle counter in NuttX compat */
}

static inline uint64_t k_cyc_to_ticks_ceil64(uint64_t cyc)
{
    return 0;
}

static inline uint32_t k_cyc_to_ns_floor64(uint64_t cyc)
{
    return 0;
}

/* SYS_TIMEOUT_MS */
#define SYS_TIMEOUT_MS(ms) Z_TIMEOUT_TICKS((ms) == SYS_FOREVER_MS ? K_TICKS_FOREVER : (k_ticks_t)k_ms_to_ticks_ceil32(ms))

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_TIME_UNITS_H_ */