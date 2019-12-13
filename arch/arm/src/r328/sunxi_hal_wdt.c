/*
 *      sunxi Watchdog Driver
 *
 *      Copyright (c) 2013 Carlo Caione
 *                    2012 Henrik Nordstrom
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 *      Based on nrf52_wdt.c
 *      (c) Copyright 2010 Novell, Inc.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <debug.h>

#include "sunxi_hal_wdt.h"
#include "r328_dev_wdt.h"

#define DRV_NAME    "sunxi-drv-wdt"
#define DRV_VERSION "1.0"

#define WDT_MAX_TIMEOUT         16
#define WDT_MIN_TIMEOUT         1
#define WDT_TIMEOUT_MASK        0x0F

#define WDT_CTRL_RELOAD         ((1 << 0) | (0x0a57 << 1))

#define WDT_MODE_EN             (1 << 0)

#define readl(addr)         (*((volatile unsigned long  *)(addr)))
#define writel(v, addr)     (*((volatile unsigned long  *)(addr)) = (unsigned long)(v))

//print functions may be different on different operating systems
#define SYSTEM_NUTTX

//#define SUNXI_WDT_DEBUG

#ifdef SYSTEM_NUTTX

#ifdef SUNXI_WDT_DEBUG
#define WDT_DEBUG(format, ...)  wdinfo(format, ##__VA_ARGS__)
#else
#define WDT_DEBUG(format, ...)
#endif
#define WDT_INFO(format, ...) wdinfo(format, ##__VA_ARGS__)

#else

#ifdef SUNXI_WDT_DEBUG
#define WDT_DEBUG(format, ...)  printf("SUNXI WDT:"format, ##__VA_ARGS__)
#else
#define WDT_DEBUG(format, ...)
#endif
#define WDT_INFO(format, ...) printf("SUNXI WDT:"format, ##__VA_ARGS__)

#endif //SYSTEM_NUTTX

/*
 * wdt_timeout_map maps the watchdog timer interval value in seconds to
 * the value of the register WDT_MODE at bits .wdt_timeout_shift ~ +3
 *
 * [timeout seconds] = register value
 *
 */
static const int wdt_timeout_map[] = {
	[1] = 0x1,  /* 1s  */
	[2] = 0x2,  /* 2s  */
	[3] = 0x3,  /* 3s  */
	[4] = 0x4,  /* 4s  */
	[5] = 0x5,  /* 5s  */
	[6] = 0x6,  /* 6s  */
	[8] = 0x7,  /* 8s  */
	[10] = 0x8, /* 10s */
	[12] = 0x9, /* 12s */
	[14] = 0xA, /* 14s */
	[16] = 0xB, /* 16s */
};

int sunxi_wdt_get_info(struct sunxi_wdt_dev_t *sunxi_wdt_dev, struct sunxi_wdt_info_t **wdt_info)
{
	*wdt_info = sunxi_wdt_dev->wdt_info;
	return 0;
}

int sunxi_wdt_restart(struct sunxi_wdt_dev_t *sunxi_wdt_dev, unsigned long action, void *data)
{
	void *wdt_base = sunxi_wdt_dev->wdt_base;
	const struct sunxi_wdt_reg_t *regs = sunxi_wdt_dev->wdt_regs;
	uint32_t val;

	/* Set system reset function */
	val = readl(wdt_base + regs->wdt_cfg);
	val &= ~(regs->wdt_reset_mask);
	val |= regs->wdt_reset_val;
	writel(val, wdt_base + regs->wdt_cfg);

	/* Set lowest timeout and enable watchdog */
	val = readl(wdt_base + regs->wdt_mode);
	val &= ~(WDT_TIMEOUT_MASK << regs->wdt_timeout_shift);
	val |= WDT_MODE_EN;
	writel(val, wdt_base + regs->wdt_mode);

	/*
	 * Restart the watchdog. The default (and lowest) interval
	 * value for the watchdog is 0.5s.
	 */
	writel(WDT_CTRL_RELOAD, wdt_base + regs->wdt_ctrl);

	while (1) {
		//Need to delay some time
		//mdelay(5);
		val = readl(wdt_base + regs->wdt_mode);
		val |= WDT_MODE_EN;
		writel(val, wdt_base + regs->wdt_mode);
	}
	return 0;
}

int sunxi_wdt_ping(struct sunxi_wdt_dev_t *sunxi_wdt_dev)
{
	void *wdt_base = sunxi_wdt_dev->wdt_base;
	const struct sunxi_wdt_reg_t *regs = sunxi_wdt_dev->wdt_regs;

	writel(WDT_CTRL_RELOAD, wdt_base + regs->wdt_ctrl);

	return 0;
}

int sunxi_wdt_set_timeout(struct sunxi_wdt_dev_t *sunxi_wdt_dev, unsigned int timeout)
{
	void *wdt_base = sunxi_wdt_dev->wdt_base;
	const struct sunxi_wdt_reg_t *regs = sunxi_wdt_dev->wdt_regs;
	uint32_t reg;

	if (wdt_timeout_map[timeout] == 0)
		timeout++;

	sunxi_wdt_dev->timeout = timeout;

	reg = readl(wdt_base + regs->wdt_mode);
	reg &= ~(WDT_TIMEOUT_MASK << regs->wdt_timeout_shift);
	reg |= wdt_timeout_map[timeout] << regs->wdt_timeout_shift;
	writel(reg, wdt_base + regs->wdt_mode);

	sunxi_wdt_ping(sunxi_wdt_dev);

	return 0;
}

int sunxi_wdt_stop(struct sunxi_wdt_dev_t *sunxi_wdt_dev)
{
	void *wdt_base = sunxi_wdt_dev->wdt_base;
	const struct sunxi_wdt_reg_t *regs = sunxi_wdt_dev->wdt_regs;

	writel(0, wdt_base + regs->wdt_mode);

	return 0;
}

int sunxi_wdt_start(struct sunxi_wdt_dev_t *sunxi_wdt_dev)
{
	uint32_t reg;
	void *wdt_base = sunxi_wdt_dev->wdt_base;
	const struct sunxi_wdt_reg_t *regs = sunxi_wdt_dev->wdt_regs;
	int ret;

	ret = sunxi_wdt_set_timeout(sunxi_wdt_dev, sunxi_wdt_dev->timeout);
	if (ret < 0)
		return ret;

	/* Set system reset function */
	reg = readl(wdt_base + regs->wdt_cfg);
	reg &= ~(regs->wdt_reset_mask);
	reg |= regs->wdt_reset_val;
	writel(reg, wdt_base + regs->wdt_cfg);

	/* Enable watchdog */
	reg = readl(wdt_base + regs->wdt_mode);
	reg |= WDT_MODE_EN;
	writel(reg, wdt_base + regs->wdt_mode);

	return 0;
}

static struct sunxi_hal_driver_watchdog sunxi_hal_watchdog_ops =
{
	.get_info   = sunxi_wdt_get_info,
	.start		= sunxi_wdt_start,
	.stop		= sunxi_wdt_stop,
	.ping		= sunxi_wdt_ping,
	.set_timeout	= sunxi_wdt_set_timeout,
	.restart	= sunxi_wdt_restart,
};

//init_mode:
//       0 : Initialize watchdog and stop watchdog
//       1 : Initialize watchdog and start watchdog(timeout set to max timeout)
int sunxi_wdt_init(struct sunxi_wdt_dev_t **sunxi_wdt_dev, int init_mode)
{
	struct sunxi_wdt_dev_t *sunxi_wdt_dev_temp = NULL;

	sunxi_wdt_dev_temp = watchdog_get_dev();
	if (!sunxi_wdt_dev_temp) {
		WDT_INFO("sunxi Watchdog get dev fail\n");
		return -1;
	} else {
		*sunxi_wdt_dev = sunxi_wdt_dev_temp;
	}

	strcpy(sunxi_wdt_dev_temp->wdt_info->drv_name,DRV_NAME);
	strcpy(sunxi_wdt_dev_temp->wdt_info->drv_version,DRV_VERSION);
	sunxi_wdt_dev_temp->wdt_ops = &sunxi_hal_watchdog_ops;
	sunxi_wdt_dev_temp->timeout = WDT_MAX_TIMEOUT;
	sunxi_wdt_dev_temp->max_timeout = WDT_MAX_TIMEOUT;
	sunxi_wdt_dev_temp->min_timeout = WDT_MIN_TIMEOUT;

	struct sunxi_wdt_info_t *wdt_info = NULL;
	sunxi_wdt_get_info(sunxi_wdt_dev_temp, &wdt_info);
	WDT_DEBUG("dev name:%s\n", wdt_info->dev_name);
	WDT_DEBUG("drv name:%s\n", wdt_info->drv_name);
	WDT_DEBUG("drv version:%s\n", wdt_info->drv_version);

	//watchdog needs to be stop because it may have been start in phase bootloader
	sunxi_wdt_stop(sunxi_wdt_dev_temp);

	if (init_mode) {
		sunxi_wdt_start(sunxi_wdt_dev_temp);
		WDT_INFO("Watchdog enabled (timeout=%d sec)\n", sunxi_wdt_dev_temp->timeout);
	}

	return 0;
}

int sunxi_wdt_exit(struct sunxi_wdt_dev_t *sunxi_wdt_dev)
{
	sunxi_wdt_stop(sunxi_wdt_dev);
	return 0;
}
