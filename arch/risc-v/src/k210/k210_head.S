/****************************************************************************
 * arch/risc-v/src/k210/k210_head.S
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

#include "chip.h"
#include "k210_memorymap.h"
#include "riscv_internal.h"
#include "riscv_macros.S"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start

__start:

  /* Load mhartid (cpuid) */

  csrr a0, CSR_MHARTID

  /* In case of single CPU config, stop here */

#if !defined(CONFIG_SMP) || (CONFIG_SMP_NCPUS == 1)
  beqz a0, 2f
  csrw CSR_MIE, zero
  wfi
#endif

2:
  /* Set stack pointer to the idle thread stack */
  riscv_set_inital_sp K210_IDLESTACK_BASE, SMP_STACK_SIZE, a0

  /* Disable all interrupts (i.e. timer, external) in mie */

  csrw CSR_MIE, zero

  /* Initialize the Machine Trap Vector */

  la   t0, __trap_vec
  csrw CSR_MTVEC, t0

  /* Jump to __k210_start with mhartid */

  j    __k210_start

  /* We shouldn't return from __k210_start */

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret
