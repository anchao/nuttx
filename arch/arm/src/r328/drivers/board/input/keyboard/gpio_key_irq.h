#ifndef __GPIO_KEY_IRQ_H
#define __GPIO_KEY_IRQ_H

struct gpio_key_irq_config
{
	int gpio;
	unsigned int code;
	int active_flag;
};

struct gpio_irq_button_data
{
	int gpio;
	unsigned int code;
	int active_low;
	char last_state;
};

struct gpio_key_irq_drvdata
{
	int nbuttons;
	struct sunxi_input_dev *input_dev;
	struct gpio_irq_button_data *bdata;
};


int enable_gpio_key_irq(xcpt_t irqhandler, FAR void *arg);
int gpio_irq_status_func(void);
int gpio_key_irq_init(void);
#endif
