#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <nuttx/irq.h>
#include "hal_gpio.h"
#include "sunxi_input.h"
#include "gpio_key_irq.h"
#include "input-event-codes.h"
#include "r328_pio.h"
#define IRQ_DEV_NAME	  "gpio-key-irq"
#define KEY_IRQ_NUM       3

#define ACTIVE_LOW        0
#define ACTIVE_HIGH       1

#define IRQ_INTERVAL      100
#define DEBOUNCE_INTERVAL 10

#ifdef  GPIO_KEY_IRQ_DEBUG
#define KEY_INFO(fmt, args...) printf(fmt, ##args)
#else
#define KEY_INFO(fmt, args...)
#endif

#define KEY_ERR(fmt, args...)  printf(":%d "fmt, __LINE__, ##args)

static struct gpio_key_irq_config key_irq_config[] = {
	{GPIOH(2), KEY_VOLUMEUP, ACTIVE_LOW},
	{GPIOH(3), KEY_VOLUMEDOWN, ACTIVE_LOW},
	{GPIOH(4), KEY_MUTE, ACTIVE_LOW},
};

struct gpio_key_irq_drvdata *key_irq_drvdata;

/********function**********/
static int gpio_key_irq_get_config(void)
{
	int i;
	int nbuttons;
	struct gpio_irq_button_data *bdata;

	nbuttons = sizeof(key_irq_config)/sizeof(key_irq_config[0]);
	bdata = (struct gpio_irq_button_data *)malloc\
		(sizeof(struct gpio_irq_button_data) * nbuttons);
	if (NULL == bdata) {
		KEY_ERR("bdata malloc err.\n");
		return -1;
	}

	for (i = 0; i < nbuttons; i++) {
		bdata[i].gpio = key_irq_config[i].gpio;
		bdata[i].code = key_irq_config[i].code;
		if (key_irq_config[i].active_flag == ACTIVE_LOW)
			bdata[i].active_low = 1;
		else
			bdata[i].active_low = 0;
	}
	key_irq_drvdata->nbuttons = nbuttons;
	key_irq_drvdata->bdata = bdata;

	return 0;
}

int enable_gpio_key_irq(xcpt_t irqhandler, FAR void *arg)
{
	int i;
	int ret = 0;
	uint32_t irqn;
	for (i = 0; i < key_irq_drvdata->nbuttons; i++) {
		ret = hal_gpio_to_irq(key_irq_config[i].gpio, &irqn);
		if (ret < 0) {
			KEY_ERR("can not get irq num.\n");
			return -1;
		}

		ret = hal_gpio_irq_request(irqn, irqhandler, IRQ_TYPE_EDGE_BOTH, arg);
		if (ret < 0) {
			KEY_ERR("request irq err.\n");
			return -1;
		}

		ret = hal_gpio_irq_enable(irqn);
		if (ret < 0) {
			KEY_ERR("enable irq err.\n");
			return -1;
		}
	}
	return 0;
}

int gpio_irq_status_func(void)
{
	int i;
	char state;
	uint32_t gpio_irq_status = 0;
	for (i = 0; i < key_irq_drvdata->nbuttons; i++) {
		hal_gpio_get_input((hal_gpio_pin_t)key_irq_drvdata->bdata[i].gpio,\
				(hal_gpio_data_t *)&state);
		if (state != key_irq_drvdata->bdata[i].last_state) {
			hal_gpio_get_input((hal_gpio_pin_t)key_irq_drvdata->bdata[i].gpio,\
					(hal_gpio_data_t *)&state);
			if (state != key_irq_drvdata->bdata[i].last_state) {
				input_report_key(key_irq_drvdata->input_dev,key_irq_drvdata->bdata[i].code,\
						!!(state ^ key_irq_drvdata->bdata[i].active_low));
				input_sync(key_irq_drvdata->input_dev);
				if (!!(state ^ key_irq_drvdata->bdata[i].active_low))
					gpio_irq_status |= (1 << i);
				else
					gpio_irq_status &= ~(1 << i);
				key_irq_drvdata->bdata[i].last_state = state;
			}
		}
		else {
			if (state ^ key_irq_drvdata->bdata[i].active_low) {
				hal_gpio_get_input((hal_gpio_pin_t)key_irq_drvdata->bdata[i].gpio,\
						(hal_gpio_data_t *)&state);
				if (state != key_irq_drvdata->bdata[i].last_state) {
					input_report_key(key_irq_drvdata->input_dev,key_irq_drvdata->bdata[i].code,\
							!!(state ^ key_irq_drvdata->bdata[i].active_low));
					input_sync(key_irq_drvdata->input_dev);
					if (!!(state ^ key_irq_drvdata->bdata[i].active_low))
						gpio_irq_status |= (1 << i);
					key_irq_drvdata->bdata[i].last_state = state;
				}
			}
		}

	}
	return gpio_irq_status;
}

int gpio_key_irq_init(void)
{
	int i;
	int ret;
	char state;
	struct sunxi_input_dev *gpio_irq_dev;
/****init strucr****/
	key_irq_drvdata = (struct gpio_key_irq_drvdata *)malloc \
			  (sizeof(struct gpio_key_irq_drvdata));
	if (NULL == key_irq_drvdata) {
		KEY_ERR("malloc data err.\n");
		return -1;
	} else {
		KEY_ERR("malloc data ok.\n");
	}
	ret = gpio_key_irq_get_config();
	if (ret < 0)
		goto err1;

/****init gpio****/
	for ( i = 0; i < key_irq_drvdata->nbuttons; i++) {
		hal_gpio_set_direction((hal_gpio_pin_t)key_irq_drvdata->bdata[i].gpio,\
				HAL_GPIO_DIRECTION_INPUT);
		hal_gpio_get_input((hal_gpio_pin_t)key_irq_drvdata->bdata[i].gpio,\
				(hal_gpio_data_t *)&state);
		key_irq_drvdata->bdata[i].last_state = state;
	}

/****init input****/
	gpio_irq_dev = sunxi_input_allocate_device();
	if (NULL == gpio_irq_dev) {
		KEY_ERR("allocate input err.\n");
		goto err2;
	}
	gpio_irq_dev->name = IRQ_DEV_NAME;
	for (i = 0; i < key_irq_drvdata->nbuttons; i++)
		input_set_capability(gpio_irq_dev, EV_KEY, key_irq_drvdata->bdata[i].code);
	key_irq_drvdata->input_dev = gpio_irq_dev;
	sunxi_input_register_device(key_irq_drvdata->input_dev);

	return 0;

err2:
	free(key_irq_drvdata->bdata);
err1:
	free(key_irq_drvdata);
	return ret;
}
