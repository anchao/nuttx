/****************************************************************************
 * arch/arm/src/r328/sunxi_hal_wdt.h
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

#ifndef __ARCH_ARM_SRC_SUNXI_HAL_WDT_H
#define __ARCH_ARM_SRC_SUNXI_HAL_WDT_H

struct sunxi_wdt_reg_t {
	uint8_t wdt_ctrl;
	uint8_t wdt_cfg;
	uint8_t wdt_mode;
	uint8_t wdt_timeout_shift;
	uint8_t wdt_reset_mask;
	uint8_t wdt_reset_val;
};

struct sunxi_wdt_info_t {
	char dev_name[64];
	char drv_name[64];
	char drv_version[64];
};

struct sunxi_wdt_dev_t {
	void *wdt_base;
	const struct sunxi_wdt_reg_t *wdt_regs;
	unsigned int timeout;
	unsigned int max_timeout;
	unsigned int min_timeout;
	struct sunxi_wdt_info_t *wdt_info;
	struct sunxi_hal_driver_watchdog *wdt_ops;
};

struct sunxi_hal_driver_watchdog {
	int (*get_info)(struct sunxi_wdt_dev_t *sunxi_wdt_dev, struct sunxi_wdt_info_t **wdt_info);
	int (*start)(struct sunxi_wdt_dev_t *sunxi_wdt_dev);
	int (*stop)(struct sunxi_wdt_dev_t *sunxi_wdt_dev);
	int (*ping)(struct sunxi_wdt_dev_t *sunxi_wdt_dev);
	int (*set_timeout)(struct sunxi_wdt_dev_t *sunxi_wdt_dev, unsigned int timeout);
	int (*restart)(struct sunxi_wdt_dev_t *sunxi_wdt_dev, unsigned long action, void *data);
};

int sunxi_wdt_get_info(struct sunxi_wdt_dev_t *sunxi_wdt_dev, struct sunxi_wdt_info_t **wdt_info);
int sunxi_wdt_start(struct sunxi_wdt_dev_t *sunxi_wdt_dev);
int sunxi_wdt_stop(struct sunxi_wdt_dev_t *sunxi_wdt_dev);
int sunxi_wdt_ping(struct sunxi_wdt_dev_t *sunxi_wdt_dev);
int sunxi_wdt_set_timeout(struct sunxi_wdt_dev_t *sunxi_wdt_dev, unsigned int timeout);
int sunxi_wdt_restart(struct sunxi_wdt_dev_t *sunxi_wdt_dev, unsigned long action, void *data);
int sunxi_wdt_init(struct sunxi_wdt_dev_t **sunxi_wdt_dev, int init_mode);
int sunxi_wdt_exit(struct sunxi_wdt_dev_t *sunxi_wdt_dev);

#endif /* __ARCH_ARM_SRC_SUNXI_HAL_WDT_H */
