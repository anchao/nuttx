/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_TOOLCHAIN_H_
#define ZEPHYR_INCLUDE_TOOLCHAIN_H_

#define HAS_BUILTIN(x) HAS_BUILTIN_##x

#define TOOLCHAIN_WARNING_UNALIGNED_ACCESS
#define TOOLCHAIN_DISABLE_CLANG_WARNING(warning)
#define TOOLCHAIN_ENABLE_CLANG_WARNING(warning)

#ifndef __noinit
#define __noinit
#endif

#include "toolchain/gcc.h"

#endif /* ZEPHYR_INCLUDE_TOOLCHAIN_H_ */
