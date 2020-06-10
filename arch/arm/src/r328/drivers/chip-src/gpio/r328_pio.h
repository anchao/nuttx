/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the People's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY¡¯S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS¡¯SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY¡¯S TECHNOLOGY.
*
*
* THIS SOFTWARE IS PROVIDED BY ALLWINNER"AS IS" AND TO THE MAXIMUM EXTENT
* PERMITTED BY LAW, ALLWINNER EXPRESSLY DISCLAIMS ALL WARRANTIES OF ANY KIND,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING WITHOUT LIMITATION REGARDING
* THE TITLE, NON-INFRINGEMENT, ACCURACY, CONDITION, COMPLETENESS, PERFORMANCE
* OR MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* IN NO EVENT SHALL ALLWINNER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __GPIO_H__
#define __GPIO_H__

#include "hardware/r328_pio.h"
#include <stdint.h>
#include <stdlib.h>
#include <interrupt.h>
#include <nuttx/irq.h>

#ifdef __cplusplus
extern "C" {
#endif
#ifdef CONFIG_DRIVERS_GPIO_DEBUG
#define GPIO_INFO(fmt, arg...) printf("GPIO : %s()%d "fmt, __func__, __LINE__, ##arg)
#else
#define GPIO_INFO(fmt, arg...) do {}while(0)
#endif

#define GPIO_ERR(fmt, arg...) printf("GPIO : %s()%d "fmt, __func__, __LINE__, ##arg)



enum pin_config_param {
	GPIO_TYPE_FUNC,
	GPIO_TYPE_DAT,
	GPIO_TYPE_PUD,
	GPIO_TYPE_DRV,
	GPIO_CONFIG_END = 0x7F,
	GPIO_CONFIG_MAX = 0xFF,
};

enum {
	IRQ_TYPE_NONE		= 0x00000000,
	IRQ_TYPE_EDGE_RISING	= 0x00000001,
	IRQ_TYPE_EDGE_FALLING	= 0x00000002,
	IRQ_TYPE_EDGE_BOTH	= (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING),
	IRQ_TYPE_LEVEL_HIGH	= 0x00000004,
	IRQ_TYPE_LEVEL_LOW	= 0x00000008,
};

struct gpio_irq_desc {
	uint32_t virq;
	uint32_t pin;
	unsigned long flags;
	//interrupt_handler_t *handle_irq;
	xcpt_t handle_irq;
	void *data;
};

struct gpio_desc {
	const unsigned long membase;
	const uint32_t npins;
	const uint32_t irq_arry_size;
	const uint32_t *irq;
	const uint32_t banks;
	const uint32_t *bank_base;
	const uint32_t irq_banks;
	const uint32_t *irq_bank_base;
	uint32_t irq_desc_size;
	struct gpio_irq_desc *irq_desc;
};

#define BANK_MEM_SIZE		0x24
#define MUX_REGS_OFFSET		0x0
#define DATA_REGS_OFFSET	0x10
#define DLEVEL_REGS_OFFSET	0x14
#define PULL_REGS_OFFSET	0x1c

#define PINS_PER_BANK		32
#define MUX_PINS_PER_REG	8
#define MUX_PINS_BITS		4
#define MUX_PINS_MASK		0x0f
#define DATA_PINS_PER_REG	32
#define DATA_PINS_BITS		1
#define DATA_PINS_MASK		0x01
#define DLEVEL_PINS_PER_REG	16
#define DLEVEL_PINS_BITS	2
#define DLEVEL_PINS_MASK	0x03
#define PULL_PINS_PER_REG	16
#define PULL_PINS_BITS		2
#define PULL_PINS_MASK		0x03

#define IRQ_PER_BANK		32

#define IRQ_CFG_REG		0x200
#define IRQ_CFG_IRQ_PER_REG		8
#define IRQ_CFG_IRQ_BITS		4
#define IRQ_CFG_IRQ_MASK		((1 << IRQ_CFG_IRQ_BITS) - 1)
#define IRQ_CTRL_REG		0x210
#define IRQ_CTRL_IRQ_PER_REG		32
#define IRQ_CTRL_IRQ_BITS		1
#define IRQ_CTRL_IRQ_MASK		((1 << IRQ_CTRL_IRQ_BITS) - 1)
#define IRQ_STATUS_REG		0x214
#define IRQ_STATUS_IRQ_PER_REG		32
#define IRQ_STATUS_IRQ_BITS		1
#define IRQ_STATUS_IRQ_MASK		((1 << IRQ_STATUS_IRQ_BITS) - 1)
#define IRQ_DEBOUNCE_REG		0x218

#define IRQ_MEM_SIZE		0x20
#define GPIO_IRQ_START		(GIC_IRQ_NUM + 1)

#define IRQ_EDGE_RISING		0x00
#define IRQ_EDGE_FALLING	0x01
#define IRQ_LEVEL_HIGH		0x02
#define IRQ_LEVEL_LOW		0x03
#define IRQ_EDGE_BOTH		0x04


#define SUNXI_PIO_BANK_BASE(pin, irq_bank) \
	((pin-PA_BASE)/PINS_PER_BANK - irq_bank)

/*
 * This looks more complex than it should be. But we need to
 * get the type for the ~ right in round_down (it needs to be
 * as wide as the result!), and we want to evaluate the macro
 * arguments just once each.
 */
#define __round_mask(x, y) ((__typeof__(x))((y)-1))
#define round_up(x, y) ((((x)-1) | __round_mask(x, y))+1)
#define round_down(x, y) ((x) & ~__round_mask(x, y))

/*
 * gpio configuration (pull up/down and drive strength) type and its value are
 * packed together into a 32-bits. The lower 8-bits represent the configuration
 * type and the upper 24-bits hold the value of the configuration type.
 */
#define GPIO_CFG_PACK(type, value)	(((value) << 8) | ((unsigned long) type & 0xFFUL))
#define GPIO_CFG_UNPACK_TYPE(cfg)	((cfg) & 0xFFUL)
#define GPIO_CFG_UNPACK_VALUE(cfg)	(((cfg) & 0xFFFFFF00UL) >> 8)

bool check_gpio_valid(uint32_t pin);
int gpio_get_data(uint32_t pin, uint32_t *data);
int gpio_set_data(uint32_t pin, uint32_t data);
int gpio_set_direction(uint32_t pin, uint32_t direction);
int gpio_get_direction(uint32_t pin, uint32_t *direction);
int gpio_set_pull(uint32_t pin, uint32_t pull);
int gpio_get_pull(uint32_t pin, uint32_t *pull);
int gpio_set_driving_level(uint32_t pin, uint32_t level);
int gpio_get_driving_level(uint32_t pin, uint32_t *level);
int gpio_pinmux_set_function(uint32_t pin, uint32_t function_index);
int gpio_to_irq(uint32_t pin);
//int gpio_irq_request(uint32_t irq, interrupt_handler_t *hdle, unsigned long flags, void *data);
int gpio_irq_request(uint32_t irq, xcpt_t hdle, unsigned long flags, void *data);
int gpio_irq_free(uint32_t irq);
int gpio_irq_enable(uint32_t irq);
int gpio_irq_disable(uint32_t irq);
int gpio_init(struct gpio_desc *gpio_desc);

#ifdef __cplusplus
}
#endif
#endif
