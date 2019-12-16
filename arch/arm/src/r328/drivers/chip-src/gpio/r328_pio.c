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

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"


#include "r328_pio.h"
#include "hardware/r328_pio.h"
#include <stdio.h>
#include <string.h>
#include "gpio-sun8iw18p1.h"
#include "r328_lowputc.h"

/* how to fix more gpio device?*/
static struct gpio_desc * g_gpio_desc = NULL;

#define readl(addr)         (*((volatile unsigned long  *)(addr)))
#define writel(v, addr)     (*((volatile unsigned long  *)(addr)) = (unsigned long)(v))

typedef unsigned char       u8;
typedef unsigned int        u32;

/*
 * The following inlines stuffs a configuration parameter and data value
 * into and out of an unsigned long argument, as used by the generic pin config
 * system. We put the parameter in the lower 8 bits and the argument in the
 * upper 24 bits.
 */

static inline enum pin_config_param pinconf_to_config_param(unsigned long config)
{
	return (enum pin_config_param) (config & 0xffUL);
}

static inline uint32_t pinconf_to_config_argument(unsigned long config)
{
	return (uint32_t) ((config >> 8) & 0xffffffUL);
}

static inline uint64_t pinconf_to_config_packed(enum pin_config_param param,
						     unsigned long argument)
{
	return GPIO_CFG_PACK(param, argument);
}

/*
 * The sunXi PIO registers are organized as is:
 * 0x00 - 0x0c	Muxing values.
 *		8 pins per register, each pin having a 4bits value
 * 0x10		Pin values
 *		32 bits per register, each pin corresponding to one bit
 * 0x14 - 0x18	Drive level
 *		16 pins per register, each pin having a 2bits value
 * 0x1c - 0x20	Pull-Up values
 *		16 pins per register, each pin having a 2bits value
 *
 * This is for the first bank. Each bank will have the same layout,
 * with an offset being a multiple of 0x24.
 *
 * The following functions calculate from the pin number the register
 * and the bit offset that we should access.
 */
static inline uint32_t gpio_mux_reg(uint32_t pin)
{
	uint32_t bank = pin / PINS_PER_BANK;
	uint32_t offset = bank * BANK_MEM_SIZE;
	offset += MUX_REGS_OFFSET;
	offset += pin % PINS_PER_BANK / MUX_PINS_PER_REG * 0x04;
	return round_down(offset, 4);
}

static inline uint32_t gpio_mux_offset(uint32_t pin)
{
	uint32_t pin_num = pin % MUX_PINS_PER_REG;
	return pin_num * MUX_PINS_BITS;
}

static inline uint32_t gpio_data_reg(uint32_t pin)
{
	uint32_t bank = pin / PINS_PER_BANK;
	uint32_t offset = bank * BANK_MEM_SIZE;
	offset += DATA_REGS_OFFSET;
	offset += pin % PINS_PER_BANK / DATA_PINS_PER_REG * 0x04;
	return round_down(offset, 4);
}

static inline uint32_t gpio_data_offset(uint32_t pin)
{
	uint32_t pin_num = pin % DATA_PINS_PER_REG;
	return pin_num * DATA_PINS_BITS;
}

static inline uint32_t gpio_dlevel_reg(uint32_t pin)
{
	uint32_t bank = pin / PINS_PER_BANK;
	uint32_t offset = bank * BANK_MEM_SIZE;
	offset += DLEVEL_REGS_OFFSET;
	offset += pin % PINS_PER_BANK / DLEVEL_PINS_PER_REG * 0x04;
	return round_down(offset, 4);
}

static inline uint32_t gpio_dlevel_offset(uint32_t pin)
{
	uint32_t pin_num = pin % DLEVEL_PINS_PER_REG;
	return pin_num * DLEVEL_PINS_BITS;
}

static inline uint32_t gpio_pull_reg(uint32_t pin)
{
	uint32_t bank = pin / PINS_PER_BANK;
	uint32_t offset = bank * BANK_MEM_SIZE;
	offset += PULL_REGS_OFFSET;
	offset += pin % PINS_PER_BANK / PULL_PINS_PER_REG * 0x04;
	return round_down(offset, 4);
}

static inline uint32_t gpio_pull_offset(uint32_t pin)
{
	uint32_t pin_num = pin % PULL_PINS_PER_REG;
	return pin_num * PULL_PINS_BITS;
}

static inline uint32_t gpio_irq_ctrl_reg_from_bank(u8 bank, unsigned bank_base)
{
	return IRQ_CTRL_REG + (bank_base + bank) * IRQ_MEM_SIZE;
}

static inline uint32_t gpio_irq_ctrl_reg(uint32_t irq, unsigned bank_base)
{
	uint32_t bank = irq / IRQ_PER_BANK;

	return gpio_irq_ctrl_reg_from_bank(bank, bank_base);
}

static inline uint32_t gpio_irq_ctrl_offset(uint32_t irq)
{
	uint32_t offset = irq % IRQ_CTRL_IRQ_PER_REG;
	return offset * IRQ_CTRL_IRQ_BITS;
}

static inline uint32_t gpio_get_pin_base_from_bank(u8 bank, unsigned bank_base)
{
	return (bank_base + bank) * IRQ_MEM_SIZE;
}

static inline uint32_t gpio_irq_status_reg_from_bank(u8 bank, unsigned bank_base)
{
	return IRQ_STATUS_REG + (bank_base + bank) * IRQ_MEM_SIZE;
}

static inline uint32_t gpio_irq_status_reg(uint32_t irq, unsigned bank_base)
{
	uint32_t bank = irq / IRQ_PER_BANK;
	return gpio_irq_status_reg_from_bank(bank, bank_base);
}

static inline uint32_t gpio_irq_status_offset(uint32_t irq)
{
	uint32_t index = irq % IRQ_STATUS_IRQ_PER_REG;
	return index * IRQ_STATUS_IRQ_BITS;
}

static inline uint32_t gpio_irq_cfg_reg(uint32_t irq, unsigned bank_base)
{
	uint32_t bank = irq / IRQ_PER_BANK;
	uint32_t reg = (irq % IRQ_PER_BANK) / IRQ_CFG_IRQ_PER_REG * 0x04;

	return IRQ_CFG_REG + (bank_base + bank) * IRQ_MEM_SIZE + reg;
}

static inline uint32_t gpio_irq_cfg_offset(uint32_t irq)
{
	uint32_t index = irq % IRQ_CFG_IRQ_PER_REG;
	return index * IRQ_CFG_IRQ_BITS;
}

static int gpio_pconf_reg(uint32_t pin, enum pin_config_param param,
			   uint32_t *offset, uint32_t *shift, uint32_t *mask)
{
		switch(param) {
		case GPIO_TYPE_DRV:
			*offset = gpio_dlevel_reg(pin);
			*shift = gpio_dlevel_offset(pin);
			*mask = DLEVEL_PINS_MASK;
			break;

		case GPIO_TYPE_PUD:
			*offset = gpio_pull_reg(pin);
			*shift = gpio_pull_offset(pin);
			*mask = PULL_PINS_MASK;
			break;

		case GPIO_TYPE_DAT:
			*offset = gpio_data_reg(pin);
			*shift = gpio_data_offset(pin);
			*mask = DATA_PINS_MASK;
			break;

		case GPIO_TYPE_FUNC:
			*offset = gpio_mux_reg(pin);
			*shift = gpio_mux_offset(pin);
			*mask = MUX_PINS_MASK;
			break;

		default:
			GPIO_ERR("Invalid mux type\n");
			return -1;
		}
	return 0;
}


static uint32_t count_gpio_bank_mask(void)
{
	uint32_t max_bank = (uint32_t)GPIO_MAX_BANK;
	uint32_t mask = 0;
	do {
		mask |= 1 << (max_bank/PINS_PER_BANK);
		max_bank -= PINS_PER_BANK;
		if (max_bank == 0)
			mask |= 1;
	} while (max_bank);
	return mask;
}

static void gpio_irq_ack(struct gpio_irq_desc *dirq)
{
	struct gpio_desc *gpio_desc = g_gpio_desc;
	uint32_t hw_irq = dirq->virq - GPIO_IRQ_START;
	unsigned bank_base = gpio_desc->irq_bank_base[hw_irq / IRQ_PER_BANK];
	uint32_t reg = gpio_irq_status_reg(hw_irq, bank_base);
	uint32_t status_idx = gpio_irq_status_offset(hw_irq);

	/* clear the pending */
	writel(1<< status_idx, gpio_desc->membase + reg);
}

static int bad_gpio_irq_handle(int dummy, void *context, void *data)
{
	GPIO_INFO("No irq registered handler for this calling !!\n");
	return 0;
}

static void gpio_irq_set_type(struct gpio_irq_desc *dirq, unsigned long type)
{
	struct gpio_desc *gpio_desc = g_gpio_desc;
	uint32_t hw_irq = dirq->virq - GPIO_IRQ_START;
	unsigned bank_base = gpio_desc->irq_bank_base[hw_irq / IRQ_PER_BANK];
	uint32_t reg = gpio_irq_cfg_reg(hw_irq, bank_base);
	uint32_t index = gpio_irq_cfg_offset(hw_irq);
	uint32_t mode, regval;

	switch (type) {
		case IRQ_TYPE_EDGE_RISING:
			mode = IRQ_EDGE_RISING;
			break;
		case IRQ_TYPE_EDGE_FALLING:
			mode = IRQ_EDGE_FALLING;
			break;
		case IRQ_TYPE_EDGE_BOTH:
			mode = IRQ_EDGE_BOTH;
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			mode = IRQ_LEVEL_HIGH;
			break;
		case IRQ_TYPE_LEVEL_LOW:
			mode = IRQ_LEVEL_LOW;
			break;
		default:
			mode = IRQ_EDGE_RISING;
	}
	/*should use spin lock protect here*/
	regval = readl(gpio_desc->membase + reg);
	regval &= ~(IRQ_CFG_IRQ_MASK << index);
	writel(regval | (mode << index), gpio_desc->membase + reg);
}

static int gpio_irq_handle(int dummy, void *data, void *arg)
{
	uint32_t hwirq = *((uint32_t *)data);
	struct gpio_desc *gpio_desc = g_gpio_desc;
	uint32_t bank, reg, val, base_bank;

	for( bank=0; bank < gpio_desc->irq_banks; bank ++) {
		if (hwirq == gpio_desc->irq[bank])
			break;
	}

	if(bank == gpio_desc->irq_banks)
		return 0;

	base_bank = gpio_desc->irq_bank_base[bank];
	reg = gpio_irq_status_reg_from_bank(bank, base_bank);
	val = readl(gpio_desc->membase + reg);

	if (val) {
		uint32_t irqoffset;
		uint32_t irq_pin;
		int i;
		for (irqoffset = 0; irqoffset < IRQ_PER_BANK; irqoffset++) {
			if ( (1 << irqoffset) & val)
				break;
		}

		if (irqoffset >= IRQ_PER_BANK)
			return 0;

		irq_pin = ((base_bank + bank) * IRQ_PER_BANK) + irqoffset;

		for ( i = 0; i < gpio_desc->irq_desc_size; i++) {
			if (irq_pin == gpio_desc->irq_desc[i].pin)
				break;
		}

		if (i >= gpio_desc->irq_desc_size)
			return 0;
		gpio_desc->irq_desc[i].handle_irq(gpio_desc->irq_desc[i].virq, gpio_desc->irq_desc[i].data, NULL);
		gpio_irq_ack(&gpio_desc->irq_desc[i]);
	}
	return 0;
}

bool check_gpio_valid(uint32_t pin)
{
	uint32_t bank = pin/PINS_PER_BANK;
//	uint32_t shift = pin % PINS_PER_BANK;
	uint32_t mask = count_gpio_bank_mask();
	if  (!((1<<bank) & mask))
		return false;
	return true;
}

static int gpio_conf_set(uint32_t pin, unsigned long *gpio_config)
{
	struct gpio_desc *gpio_desc = g_gpio_desc;
	unsigned long config = (unsigned long)gpio_config;
	uint32_t offset, shift, mask, reg;
	uint32_t arg;
	//unsigned long gpio_reg_base = 0;
	enum pin_config_param param;
	int ret;

	param = pinconf_to_config_param(config);
	arg = pinconf_to_config_argument(config);

	ret = gpio_pconf_reg(pin, param, &offset, &shift, &mask);
	if (ret < 0) {
		GPIO_ERR("can't get reg for pin %u", pin);
		return -1;
	}
	/* fix me: shuold we keep spin_lock to protect here?*/
	reg = readl(gpio_desc->membase + offset);
	reg &= ~(mask << shift);
	writel(reg | arg << shift, gpio_desc->membase + offset);
	return 0;
}

static int gpio_conf_get(uint32_t pin, unsigned long *gpio_config)
{
	struct gpio_desc *gpio_desc = g_gpio_desc;
	uint32_t offset, shift, mask/*, reg*/;
	uint32_t arg, val;
//	unsigned long gpio_reg_base = 0;
	enum pin_config_param param = pinconf_to_config_param(*gpio_config);
	int ret = 0;

	ret = gpio_pconf_reg(pin, param, &offset, &shift, &mask);
	if (ret < 0) {
		GPIO_ERR("can't get reg for pin %u", pin);
		return -1;
	}
	val = (readl(gpio_desc->membase + offset)>> shift) & mask;
	switch (param) {
	case GPIO_TYPE_DRV:
	case GPIO_TYPE_DAT:
	case GPIO_TYPE_PUD:
	case GPIO_TYPE_FUNC:
		arg = val;
		break;
	default:
		ret = -1;
		GPIO_ERR("Invalid mux type\n");
		return -1;
	}
	if (!ret)
		*gpio_config = pinconf_to_config_packed(param, arg);
	return ret;
}

int gpio_get_data(uint32_t pin, uint32_t *data)
{
	unsigned long config;
	int ret = 0;

	if (NULL == data) {
		ret = -1;
		GPIO_ERR("Invalid parameter!\n");
		return ret;
	}

	config = GPIO_CFG_PACK(GPIO_TYPE_DAT,0xffffff);
	ret = gpio_conf_get(pin, &config);
	if (ret < 0) {
		GPIO_ERR("get conf error!\n");
		return ret;
	}

	*data = GPIO_CFG_UNPACK_VALUE(config);

	if (*data == 0xffffff) {
		ret = -1;
		GPIO_ERR("unpack error!\n");
		return ret;
	}

	return ret;
}

int gpio_set_data(uint32_t pin, uint32_t data)
{
	unsigned long config;
	int ret = 0;

	config = GPIO_CFG_PACK(GPIO_TYPE_DAT, data);
	ret = gpio_conf_set(pin, (unsigned long *)config);
	if (ret < 0) {
		GPIO_ERR("set conf error!\n");
		return ret;
	}
	return ret;
}

int gpio_set_direction(uint32_t pin, uint32_t direction)
{
	unsigned long config;
	int ret = 0;

	config = GPIO_CFG_PACK(GPIO_TYPE_FUNC, direction);
	ret = gpio_conf_set(pin, (unsigned long *)config);
	if (ret < 0) {
		GPIO_ERR("set conf error!\n");
		return ret;
	}
	return ret;
}

int gpio_get_direction(uint32_t pin, uint32_t *direction)
{
	unsigned long config;
	int ret = 0;

	if (NULL == direction) {
		ret = -1;
		GPIO_ERR("Invalid parameter!\n");
		return ret;
	}
	config = GPIO_CFG_PACK(GPIO_TYPE_FUNC, 0xffffff);
	ret = gpio_conf_get(pin, &config);
	if (ret < 0) {
		GPIO_ERR("get conf error!\n");
		return ret;
	}

	*direction = GPIO_CFG_UNPACK_VALUE(config);

	if (*direction == 0xffffff) {
		ret = -1;
		GPIO_ERR("unpack error!\n");
		return ret;
	}
	return ret;
}

int gpio_set_pull(uint32_t pin, uint32_t pull)
{
	unsigned long config;
	int ret = 0;

	config = GPIO_CFG_PACK(GPIO_TYPE_PUD, pull);
	ret = gpio_conf_set(pin, (unsigned long *)config);
	if (ret < 0) {
		GPIO_ERR("set conf error!\n");
		return ret;
	}
	return ret;
}

int gpio_get_pull(uint32_t pin, uint32_t *pull)
{
	unsigned long config;
	int ret = 0;

	if (NULL == pull) {
		ret = -1;
		GPIO_ERR("Invalid parameter!\n");
		return ret;
	}
	config = GPIO_CFG_PACK(GPIO_TYPE_PUD, 0xffffff);
	ret = gpio_conf_get(pin, &config);
	if (ret < 0) {
		GPIO_ERR("get conf error!\n");
		return ret;
	}

	*pull = GPIO_CFG_UNPACK_VALUE(config);

	if (*pull == 0xffffff) {
		ret = -1;
		GPIO_ERR("unpack error!\n");
		return ret;
	}
	return ret;

}

int gpio_set_driving_level(uint32_t pin, uint32_t level)
{
	unsigned long config;
	int ret = 0;

	config = GPIO_CFG_PACK(GPIO_TYPE_DRV, level);
	ret = gpio_conf_set(pin, (unsigned long *)config);
	if (ret < 0) {
		GPIO_ERR("set conf error!\n");
		return ret;
	}
	return ret;
}

int gpio_get_driving_level(uint32_t pin, uint32_t *level)
{
	unsigned long config;
	int ret = 0;

	if (NULL == level) {
		ret = -1;
		GPIO_ERR("Invalid parameter!\n");
		return ret;
	}
	config = GPIO_CFG_PACK(GPIO_TYPE_DRV, 0xffffff);
	ret = gpio_conf_get(pin, &config);
	if (ret < 0) {
		GPIO_ERR("get conf error!\n");
		return ret;
	}

	*level = GPIO_CFG_UNPACK_VALUE(config);

	if (*level == 0xffffff) {
		ret = -1;
		GPIO_ERR("unpack error!\n");
		return ret;
	}
	return ret;

}

int gpio_pinmux_set_function(uint32_t pin, uint32_t function_index)
{
	unsigned long config;
	int ret = 0;

	config = GPIO_CFG_PACK(GPIO_TYPE_FUNC, function_index);
	ret = gpio_conf_set(pin, (unsigned long *)config);
	if (ret < 0) {
		GPIO_ERR("set pin mux error!\n");
		return ret;
	}
	return ret;
}

int gpio_to_irq(uint32_t pin)
{
	int i = 0;
	struct gpio_desc *gpio_desc = g_gpio_desc;

	for(i = 0; i < gpio_desc->irq_banks * IRQ_PER_BANK; i++) {
		if ( pin != gpio_desc->irq_desc[i].pin) {
			continue;
		}
		GPIO_INFO("gpio %u to irq %u succeed!\n", pin, gpio_desc->irq_desc[i].virq);
		return gpio_desc->irq_desc[i].virq;
	}

	if ( i >= gpio_desc->irq_banks * IRQ_PER_BANK) {
		goto ERR;
	}
ERR:
	GPIO_ERR("gpio to irq error!\n");
	return -1;
}

//int gpio_irq_request(uint32_t irq, interrupt_handler_t *hdle, unsigned long flags, void *data)
int gpio_irq_request(uint32_t irq, xcpt_t hdle, unsigned long flags, void *data)
{
	struct gpio_desc *gpio_desc = g_gpio_desc;
	int irq_max_num = gpio_desc->irq_desc_size + GPIO_IRQ_START;
	int ret = 0;

	if ( irq >= GPIO_IRQ_START && irq < irq_max_num) {
		if (hdle && gpio_desc->irq_desc[irq-GPIO_IRQ_START].handle_irq == bad_gpio_irq_handle ) {
			gpio_desc->irq_desc[irq-GPIO_IRQ_START].handle_irq = hdle;
			gpio_desc->irq_desc[irq-GPIO_IRQ_START].flags = flags;
			gpio_desc->irq_desc[irq-GPIO_IRQ_START].data = data;
		}
		/*set irq tpye*/
		gpio_irq_set_type(&gpio_desc->irq_desc[irq-GPIO_IRQ_START], flags);

		/*set pin mux*/
		ret = gpio_pinmux_set_function(gpio_desc->irq_desc[irq-GPIO_IRQ_START].pin, 6);

		if (ret < 0) {
			GPIO_ERR("set pin mux error!\n");
			return -1;
		}
		GPIO_INFO("request irq %u succeed!\n", irq);
		return irq;
	}

	GPIO_ERR("Wrong irq NO.(%u) to request !!\n", irq);
	return -1;
}

int gpio_irq_free(uint32_t irq)
{
	struct gpio_desc *gpio_desc = g_gpio_desc;
	int irq_max_num = gpio_desc->irq_desc_size + GPIO_IRQ_START;
//	int ret = 0;

	if ( irq >= GPIO_IRQ_START && irq < irq_max_num) {
			gpio_desc->irq_desc[irq-GPIO_IRQ_START].handle_irq = bad_gpio_irq_handle;
			gpio_desc->irq_desc[irq-GPIO_IRQ_START].flags = 0;
			gpio_desc->irq_desc[irq-GPIO_IRQ_START].data = NULL;
			GPIO_INFO("free irq %u succeed!\n", irq);
			return irq;
	}

	GPIO_ERR("Wrong irq NO.(%u) to free !!\n", irq);
	return -1;
}

int gpio_irq_enable(uint32_t irq)
{
	struct gpio_desc *gpio_desc = g_gpio_desc;
	int irq_max_num = gpio_desc->irq_desc_size + GPIO_IRQ_START;
//	int ret = 0;
	uint32_t hw_irq = irq - GPIO_IRQ_START;
	unsigned bank_base = gpio_desc->irq_bank_base[hw_irq / IRQ_PER_BANK];
	uint32_t reg = gpio_irq_ctrl_reg(hw_irq, bank_base);
	uint32_t index = gpio_irq_ctrl_offset(hw_irq);
	uint32_t val = 0;

	if ( irq < GPIO_IRQ_START || irq >= irq_max_num) {
		GPIO_ERR("Wrong irq NO.(%u) to enable !!\n", irq);
		return -1;
	}

	/*clear pending*/
	gpio_irq_ack(&gpio_desc->irq_desc[hw_irq]);

	/*unmask the irq,should keep spin lock to protect*/
	val = readl(gpio_desc->membase + reg);
	writel(val | (1<<index), gpio_desc->membase + reg);
	return 0;
}

int gpio_irq_disable(uint32_t irq)
{
	struct gpio_desc *gpio_desc = g_gpio_desc;
	int irq_max_num = gpio_desc->irq_desc_size + GPIO_IRQ_START;
//	int ret = 0;
	uint32_t hw_irq = irq - GPIO_IRQ_START;
	unsigned bank_base = gpio_desc->irq_bank_base[hw_irq / IRQ_PER_BANK];
	uint32_t reg = gpio_irq_ctrl_reg(hw_irq, bank_base);
	uint32_t index = gpio_irq_ctrl_offset(hw_irq);
	uint32_t val = 0;

	if ( irq < GPIO_IRQ_START || irq >= irq_max_num) {
		GPIO_ERR("Wrong irq NO.(%u) to enable !!\n", irq);
		return -1;
	}

	/*mask the irq,should keep spin lock to protect*/
	val = readl(gpio_desc->membase + reg);
	writel(val &~ (1<<index), gpio_desc->membase + reg);
	return 0;
}

//#include "gpio-sun8iw18p1.c"
//extern struct gpio_irq_desc irq_desc[IRQ_DESC_ARRAY_SIZE];

int gpio_init(struct gpio_desc *gpio_desc)
{
	int i;
	struct gpio_irq_desc *irq_desc = NULL;
	int irq_desc_array_size = gpio_desc->irq_banks * IRQ_PER_BANK;

	gpio_desc->irq_desc_size = irq_desc_array_size;
	irq_desc = (struct gpio_irq_desc *)malloc(irq_desc_array_size * sizeof(struct gpio_irq_desc));
	if (irq_desc == NULL) {
		GPIO_ERR("alloc memory failed!\n");
		return -1;
	}
	memset(irq_desc, 0, irq_desc_array_size * sizeof(struct gpio_irq_desc));
	for (i = 0; i < irq_desc_array_size; i++) {
		unsigned int j = i / IRQ_PER_BANK;
		unsigned int k = i % IRQ_PER_BANK;
		unsigned bank_base = gpio_desc->irq_bank_base[j];
		irq_desc[i].pin = gpio_get_pin_base_from_bank(j, bank_base) + k;
		irq_desc[i].virq =  GPIO_IRQ_START + i;
		irq_desc[i].handle_irq = bad_gpio_irq_handle;
	}

	gpio_desc->irq_desc = irq_desc;

	for (i = 0; i < gpio_desc->irq_banks; i++) {
		/* mask all irq */
		unsigned bank_base = gpio_desc->irq_bank_base[i];
		writel(0, gpio_desc->membase +
				gpio_irq_ctrl_reg_from_bank(i, bank_base));
		/* clear pending flags */
		writel(0xffffffff, gpio_desc->membase +
				gpio_irq_status_reg_from_bank(i, bank_base));
	}

	/* request irq */
	for (i = 0; i < gpio_desc->irq_arry_size; i++)
		irq_attach(gpio_desc->irq[i], gpio_irq_handle, (void *)&gpio_desc->irq[i]);

	/* enable irq */
	for(i = 0; i < gpio_desc->irq_arry_size; i++)
		//irq_enable(gpio_desc->irq[i]);
		up_enable_irq(gpio_desc->irq[i]);


	g_gpio_desc = gpio_desc;
	return 0;
};

//void sunxi_gpio_init(void)
//{
//	sun8iw18p1_gpio_init();
//}
