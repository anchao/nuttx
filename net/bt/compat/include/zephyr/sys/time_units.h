/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_TIME_UNITS_H_
#define ZEPHYR_INCLUDE_SYS_TIME_UNITS_H_

#include <nuttx/clock.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline uint32_t k_cyc_to_ns_floor64(uint64_t cyc)
{
    (void)cyc;
    return 0;
}

static inline uint32_t k_cyc_to_us_floor64(uint64_t cyc)
{
    (void)cyc;
    return 0;
}

static inline uint32_t k_cyc_to_us_floor32(uint32_t cyc)
{
    (void)cyc;
    return 0;
}

static inline uint64_t k_cycle_get_64(void)
{
    return (uint64_t)clock_systime_ticks();
}

static inline uint32_t k_cycle_get_32(void)
{
    return (uint32_t)clock_systime_ticks();
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_TIME_UNITS_H_ */
