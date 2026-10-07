/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_CLOCK_H_
#define ZEPHYR_INCLUDE_SYS_CLOCK_H_

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/clock.h>

#define Z_HAVE_K_TICKS_TO_MS_CEIL32
#include "sys/time_units.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Tick type - use uint32_t for NuttX compatibility */
typedef uint32_t k_ticks_t;

#define K_TICKS_FOREVER ((k_ticks_t)-1)

/* Kernel timeout type */
typedef struct {
    k_ticks_t ticks;
} k_timeout_t;

#define K_TIMEOUT_EQ(a, b) ((a).ticks == (b).ticks)

/* Timeout constants */
#define K_NO_WAIT   Z_TIMEOUT_NO_WAIT
#define K_FOREVER   Z_FOREVER
#define K_SECONDS(s) Z_TIMEOUT_TICKS((k_ticks_t)k_sec_to_ticks_ceil32(s))
#define K_MSEC(ms)   Z_TIMEOUT_TICKS((k_ticks_t)k_ms_to_ticks_ceil32(ms))
#define K_USEC(us)   Z_TIMEOUT_TICKS((k_ticks_t)k_us_to_ticks_ceil32(us))
#define K_MINUTES(m) K_SECONDS((m) * 60)
#define K_HOURS(h)   K_MINUTES((h) * 60)

/* Internal */
#define Z_TIMEOUT_NO_WAIT ((k_timeout_t){0})
#define Z_TIMEOUT_TICKS(t) ((k_timeout_t){.ticks = (t)})
#define Z_FOREVER Z_TIMEOUT_TICKS(K_TICKS_FOREVER)

#define Z_TIMEOUT_MS(t) Z_TIMEOUT_TICKS((k_ticks_t)k_ms_to_ticks_ceil32((unsigned int)(t) > 0 ? (t) : 0))
#define Z_TIMEOUT_US(t) Z_TIMEOUT_TICKS((k_ticks_t)k_us_to_ticks_ceil32((unsigned int)(t) > 0 ? (t) : 0))

/* Sys clock */
#ifndef CONFIG_SYS_CLOCK_TICKS_PER_SEC
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC TICK_PER_SEC
#endif

static inline uint32_t sys_clock_tick_get_32(void)
{
    return (uint32_t)clock_systime_ticks();
}

static inline int64_t sys_clock_tick_get(void)
{
    return (int64_t)clock_systime_ticks();
}

#define Z_HAVE_K_TICKS_TO_MS_CEIL32
static inline uint32_t k_ticks_to_ms_ceil32(uint32_t ticks)
{
    return (uint32_t)TICK2MSEC(ticks);
}

static inline uint32_t k_ticks_to_ms_floor32(uint32_t ticks)
{
    return (uint32_t)TICK2MSEC(ticks);
}

static inline int64_t k_uptime_ticks(void)
{
    return (int64_t)clock_systime_ticks();
}

static inline int64_t k_uptime_get(void)
{
    return TICK2MSEC((uint32_t)clock_systime_ticks());
}

static inline uint32_t k_uptime_get_32(void)
{
    return (uint32_t)k_uptime_get();
}

static inline int64_t k_uptime_delta(int64_t *reftime)
{
    int64_t uptime = k_uptime_get();
    int64_t delta = uptime - *reftime;

    *reftime = uptime;
    return delta;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_CLOCK_H_ */
