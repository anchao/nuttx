/* Copyright (c) 2022 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>

#define BT_ASSERT(cond) DEBUGASSERT(cond)
#define BT_ASSERT_MSG(cond, msg, ...) \
  ((void)((cond) || (__assert(__ASSERT_FILE__, __ASSERT_LINE__, msg), 0)))
