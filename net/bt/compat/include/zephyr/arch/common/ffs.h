/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_ARCH_COMMON_FFS_H_
#define ZEPHYR_INCLUDE_ARCH_COMMON_FFS_H_

#include <stdint.h>
#include <zephyr/toolchain.h>

static ALWAYS_INLINE unsigned int find_msb_set(uint32_t op)
{
	if (op == 0) {
		return 0;
	}

	return 32 - __builtin_clz(op);
}

static ALWAYS_INLINE unsigned int find_lsb_set(uint32_t op)
{
	return __builtin_ffs(op);
}

#endif /* ZEPHYR_INCLUDE_ARCH_COMMON_FFS_H_ */
