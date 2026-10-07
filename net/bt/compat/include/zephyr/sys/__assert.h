/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS___ASSERT_H_
#define ZEPHYR_INCLUDE_SYS___ASSERT_H_

#include <assert.h>

#define __ASSERT(test, msg, ...) \
  ((void)((test) || (__assert(__ASSERT_FILE__, __ASSERT_LINE__, msg), 0)))
#define __ASSERT_NO_MSG(test) DEBUGASSERT(test)

#define __ASSERT_PRINT(fmt, ...) ((void)0)
#define __ASSERT_MSG_INFO(fmt, ...) ((void)0)
#define __ASSERT_LOC(test) ((void)0)
#define __ASSERT_POST_ACTION() ((void)0)
#define __ASSERT_EVAL(expr1, expr2, test, fmt, ...) \
  do \
    { \
      expr2; \
      __ASSERT(test, fmt, ##__VA_ARGS__); \
    } \
  while (false)

#endif /* ZEPHYR_INCLUDE_SYS___ASSERT_H_ */
