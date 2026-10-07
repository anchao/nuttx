/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_ARCH_CPU_H_
#define ZEPHYR_INCLUDE_ARCH_CPU_H_

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/compiler.h>

/* NuttX architectures provide their own CPU port layer.
 * For BT compat, we map Zephyr arch/cpu.h to the NuttX equivalents.
 */

#define NANO_ESF  void
#define ARCH_ISR_DIRECT_HEADER
#define ARCH_ISR_DIRECT_FOOTER

#endif /* ZEPHYR_INCLUDE_ARCH_CPU_H_ */