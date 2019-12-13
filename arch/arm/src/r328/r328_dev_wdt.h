/****************************************************************************
 * arch/arm/src/r328/r328_dev_wdt.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_R328_DEV_WDT_H
#define __ARCH_ARM_SRC_R328_DEV_WDT_H

#include "sunxi_hal_wdt.h"

#define DEV_NAME    "r328-wdt"

#define R328_WDT_CTRL 0x10
#define R328_WDT_CFG  0x14
#define R328_WDT_MODE  0x18
#define R328_WDT_TIMEOUT_SHIFT  4
#define R328_WDT_RESET_MASK  0x03
#define R328_WDT_RESET_VAL  0x01

#define R328_WDT_BASE 0x030090a0

static struct sunxi_wdt_reg_t r328_wdt_reg =
{
	.wdt_ctrl = R328_WDT_CTRL,
	.wdt_cfg = R328_WDT_CFG,
	.wdt_mode = R328_WDT_MODE,
	.wdt_timeout_shift = R328_WDT_TIMEOUT_SHIFT,
	.wdt_reset_mask = R328_WDT_RESET_MASK,
	.wdt_reset_val = R328_WDT_RESET_VAL,
};

static struct sunxi_wdt_info_t r328_wdt_info = {
	.dev_name = DEV_NAME,
};

static struct sunxi_wdt_dev_t r328_wdt_dev = {
	.wdt_base = (void *)R328_WDT_BASE,
	.wdt_regs = &r328_wdt_reg,
	.timeout = 0,
	.wdt_info = &r328_wdt_info,
};

struct sunxi_wdt_dev_t* watchdog_get_dev(void)
{
	return &r328_wdt_dev;
};

#endif /* __ARCH_ARM_SRC_R328_HAL_WDT_H */
