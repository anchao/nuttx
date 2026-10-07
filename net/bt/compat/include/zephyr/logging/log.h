/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_LOGGING_LOG_H_
#define ZEPHYR_INCLUDE_LOGGING_LOG_H_

#include <syslog.h>

/* NuttX syslog.h defines LOG_ERR (3), LOG_WARNING (4), LOG_INFO (6),
 * LOG_DEBUG (7) as priority constants. These conflict with Zephyr's
 * function-like LOG_ERR macro. Undefine LOG_ERR so Zephyr code works.
 */
#undef LOG_ERR

/* Module registration stubs */
#define LOG_MODULE_REGISTER(name, ...)
#define LOG_MODULE_DECLARE(name, ...)

/* Log levels (map to NuttX syslog priorities) */
#define LOG_ERR(fmt, ...)    syslog(3,    "<err> %s: " fmt "\n", __func__, ##__VA_ARGS__)
#define LOG_WRN(fmt, ...)    syslog(4,    "<wrn> %s: " fmt "\n", __func__, ##__VA_ARGS__)
#define LOG_INF(fmt, ...)    syslog(6,    "<inf> %s: " fmt "\n", __func__, ##__VA_ARGS__)
#define LOG_DBG(fmt, ...)    syslog(7,    "<dbg> %s: " fmt "\n", __func__, ##__VA_ARGS__)

/* Hex dump */
#define LOG_HEXDUMP_DBG(_data, _length, _str)
#define LOG_HEXDUMP_INF(_data, _length, _str)
#define LOG_HEXDUMP_WRN(_data, _length, _str)
#define LOG_HEXDUMP_ERR(_data, _length, _str)

/* Printk fallback */
#define LOG_PRINTK(fmt, ...) syslog(LOG_INFO, fmt, ##__VA_ARGS__)

#endif /* ZEPHYR_INCLUDE_LOGGING_LOG_H_ */