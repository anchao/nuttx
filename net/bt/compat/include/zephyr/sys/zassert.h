/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_ZASSERT_H_
#define ZEPHYR_INCLUDE_SYS_ZASSERT_H_

#include <assert.h>
#include <zephyr/sys/util.h>

/* Minimal assertion compatibility for NuttX */

#define ZASSERT_LEVEL_OFF     0
#define ZASSERT_LEVEL_ERROR   1
#define ZASSERT_LEVEL_WARN    2
#define ZASSERT_LEVEL_INFO    3
#define ZASSERT_LEVEL_VERBOSE 4

#ifndef CONFIG_ASSERT_MODULE_DEFAULT_LEVEL
#define CONFIG_ASSERT_MODULE_DEFAULT_LEVEL ZASSERT_LEVEL_OFF
#endif

#define ZASSERT_IMPL_MODULE_LEVEL(m) CONFIG_ASSERT_MODULE_DEFAULT_LEVEL

/* Core assertion implementation - must be an expression for IF_ENABLED compat */
static inline int z_assert_panic(void) { PANIC(); return 0; }
#define ZASSERT_IMPL_LEVEL(level, test, ...) \
    ((void)((test) || (z_assert_panic(), 0)))

/* Public ZASSERT macro */
#define ZASSERT(test, ...) ZASSERT_IMPL_LEVEL(CONFIG_ASSERT_MODULE_DEFAULT_LEVEL, test, ##__VA_ARGS__)

/* zassert_post_action */
static inline void zassert_post_action(const char *file, unsigned int line)
{
    (void)file;
    (void)line;
    PANIC();
}

/* zassert_print */
static inline void zassert_print(const char *fmt, ...)
{
    (void)fmt;
}

#endif /* ZEPHYR_INCLUDE_SYS_ZASSERT_H_ */