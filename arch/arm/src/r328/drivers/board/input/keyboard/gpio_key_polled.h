#ifndef __GPIO_KEY_POLLED_H
#define __GPIO_KEY_POLLED_H

#ifdef __cplusplus
extern "C" {
#endif

#define ACTIVE_LOW	0
#define ACTIVE_HIGH	1

//gpio key config.
struct gpio_key_config
{
	int gpio;
	unsigned int code;
	int active_flag;
};

//driver data
struct gpio_button_data
{
	int gpio;
	unsigned int code;
	int active_low;
	char last_state;
};

struct gpio_key_drvdata
{
	int nbuttons;
	struct sunxi_input_dev *input_dev;
	struct gpio_button_data *bdata;
};

int gpio_key_polled_init(void);


#ifdef __cplusplus
}
#endif

#endif
