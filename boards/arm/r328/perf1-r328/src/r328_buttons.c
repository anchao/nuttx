/****************************************************************************
 * boards/arm/r328/perf1-r328/src/r328_buttons.c
 *
 *   Copyright (C) 2013-2015, 2017 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <errno.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>
#include "perf1_r328.h"
#include "hal_gpio.h"
#include "r328_keyboard.h"
#include "r328_gpadc_key.h"
#include "gpio_key_polled.h"
#include "gpio_key_irq.h"

#ifdef CONFIG_ARCH_BUTTONS

#ifdef  KEYBOARD_DEBUG
#define BTN_INFO(fmt, arg...) printf("%s:%d "fmt, __func__,  __LINE__, ##arg)
#else
#define BTN_INFO(fmt, arg...)
#endif
#define BTN_ERR(fmt, arg...) printf("%s:%d "fmt, __func__, __LINE__, ##arg)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

void board_button_initialize(void)
{
#if defined(CONFIG_DRIVERS_LRADC) && defined(CONFIG_DRIVERS_R328_KEYBOARD)
	sunxi_keyboard_init();
	BTN_INFO("sunxi_keyboard_init is enable.\n");
#elif defined(CONFIG_DRIVERS_GPADC) && defined(CONFIG_DRIVERS_R328_GPADC_KEY)
	sunxi_gpadc_key_init();
	BTN_INFO("sunxi_gpadc_key_init is enable.\n");
#elif defined(CONFIG_DRIVERS_R328_GPIO_KEY)
	gpio_key_polled_init();
	BTN_INFO("gpio_key_polled_init is enable.\n");
#elif defined(CONFIG_DRIVERS_R328_GPIO_KEY_IRQ)
	gpio_key_irq_init();
	BTN_INFO("gpio_key_irq_init is enable.\n");
#else
	BTN_ERR("Please select the key driver.\n");
#endif
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.
 *   See the BUTTON* definitions above for the meaning of each bit in the
 *   returned value.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
#if defined(CONFIG_DRIVERS_LRADC) && defined(CONFIG_DRIVERS_R328_KEYBOARD)
	uint32_t irq_status;
	uint32_t reg_val;
	lradc_irq_status(&irq_status, &reg_val);
	return lradc_irq_callback(irq_status, reg_val);
#elif defined(CONFIG_DRIVERS_GPADC) && defined(CONFIG_DRIVERS_R328_GPADC_KEY)
	uint32_t irq_status;
	uint32_t reg_val;
	uint32_t channel;
	gpadc_irq_status(&channel, &irq_status, &reg_val);
	return gpadc_irq_callback(channel, irq_status, reg_val);
#elif defined(CONFIG_DRIVERS_R328_GPIO_KEY)
	uint32_t ret = gpio_key_polled_poll();
	return ret;
#elif defined(CONFIG_DRIVERS_R328_GPIO_KEY_IRQ)
	uint32_t ret = gpio_irq_status_func();
	return ret;
#endif
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BUTTON* definitions provided above.
 *
 * Configuration Notes:
 *   Configuration CONFIG_ARCH_IRQBUTTONS must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
	int ret = -EINVAL;
#if defined(CONFIG_DRIVERS_LRADC) && defined(CONFIG_DRIVERS_R328_KEYBOARD)
	if (id < NUM_BUTTONS) {
		ret = irq_attach(R328_IRQ_LRADC, irqhandler, arg);
		if (ret < 0)
			BTN_ERR("lradc irq attach err.\n");
		up_enable_irq(R328_IRQ_LRADC);
	}
#elif defined(CONFIG_DRIVERS_GPADC) && defined(CONFIG_DRIVERS_R328_GPADC_KEY)
	if (id < NUM_BUTTONS) {
		ret = irq_attach(R328_IRQ_GPADC, irqhandler, arg);
		if (ret < 0)
			BTN_ERR("gpadc irq attach err.\n");
		up_enable_irq(R328_IRQ_GPADC);
	}
#elif defined(CONFIG_DRIVERS_R328_GPIO_KEY_IRQ)
	ret = enable_gpio_key_irq(irqhandler, arg);
	if (ret < 0)
		BTN_ERR("enable_gpio_key_irq err.\n");
#elif defined(CONFIG_DRIVERS_R328_GPIO_KEY)
	return 0;
#else
	BTN_ERR("There are no key can enable irq.\n");:
#endif
	return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
