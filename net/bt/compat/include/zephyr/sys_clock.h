/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_CLOCK_H_
#define ZEPHYR_INCLUDE_SYS_CLOCK_H_

#include <stdint.h>
#include <nuttx/clock.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_CLOCK_H_ */
