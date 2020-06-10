#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hal_gpio.h"
#include "sunxi_input.h"
#include "gpio_key_polled.h"
#include "input-event-codes.h"

#define INPUT_DEV_NAME		"gpio-key-polled"
#define DEBOUNCE_INTERVAL	10
#define POLL_INTERVAL		100

#define GPIO_KEY_TASK_PRO	10
#define GPIO_KEY_STACK_SIZE	1024

#define REPEAT
#define TEST_TASK

static struct gpio_key_config key_config[] = {
	{GPIOH(2), KEY_VOLUMEUP, ACTIVE_LOW},
	{GPIOH(3), KEY_VOLUMEDOWN, ACTIVE_LOW},
	{GPIOH(4), KEY_MUTE, ACTIVE_LOW},
};

struct gpio_key_drvdata *key_drvdata;


/*-------------------------------define TEST TASK----------------------*/

#ifdef TEST_TASK

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
static int key_scan(int argc, char *argv[])
{
	int fd;
	char input_dev_name[256];
	struct sunxi_input_event event;

	sprintf(input_dev_name, "/dev/input/%s/", INPUT_DEV_NAME);
#ifdef POLL_DEBUG
	fd = open(input_dev_name, O_RDONLY);
	if (fd < 0) {
		printf("gpio key open err\n");
		return -1;
	}

	while(1) {
		memset(&event, 0, sizeof(struct sunxi_input_event));
		read(fd, &event, sizeof(struct sunxi_input_event));
		printf("event type code val : %d  %d %d\n", event.type, event.code, event.value);
	}
#endif
	return 0;

}
#endif


int gpio_key_polled_poll(void)
{
	int i;
	char state;
	uint32_t gpio_poll_status = 0;
	for (i = 0; i < key_drvdata->nbuttons; i++) {
		hal_gpio_get_input((hal_gpio_pin_t)key_drvdata->bdata[i].gpio, (hal_gpio_data_t *)&state);
		if (state != key_drvdata->bdata[i].last_state) {
			usleep(DEBOUNCE_INTERVAL * 1000);
			hal_gpio_get_input((hal_gpio_pin_t)key_drvdata->bdata[i].gpio, (hal_gpio_data_t *)&state);
			if (state != key_drvdata->bdata[i].last_state) {
				input_report_key(key_drvdata->input_dev,
						key_drvdata->bdata[i].code, !!(state ^ key_drvdata->bdata[i].active_low));
				input_sync(key_drvdata->input_dev);
#ifdef POLL_DEBUG
				printf("===press: %d===\n", key_drvdata->bdata[i].code);
#endif
				if (!!(state ^ key_drvdata->bdata[i].active_low))
					gpio_poll_status |= (1 << i);
				else
					gpio_poll_status &= ~(1 << i);
				key_drvdata->bdata[i].last_state = state;
			}
		}
#ifdef REPEAT
		else {
			if (state ^ key_drvdata->bdata[i].active_low) {
				usleep(DEBOUNCE_INTERVAL * 1000);
				hal_gpio_get_input((hal_gpio_pin_t)key_drvdata->bdata[i].gpio, (hal_gpio_data_t *)&state);
				if (state == key_drvdata->bdata[i].last_state) {
					input_report_key(key_drvdata->input_dev,
							key_drvdata->bdata[i].code, !!(state ^ key_drvdata->bdata[i].active_low));
					input_sync(key_drvdata->input_dev);
					if (!!(state ^ key_drvdata->bdata[i].active_low))
						gpio_poll_status |= (1 << i);
					key_drvdata->bdata[i].last_state = state;

				}
			}
		}
#endif
	}
	usleep((POLL_INTERVAL-DEBOUNCE_INTERVAL) * 1000);
	return gpio_poll_status;
}


static int gpio_key_polled_get_config(void)
{
	int i;
	int nbuttons;
	struct gpio_button_data *bdata;

	nbuttons = sizeof(key_config)/sizeof(key_config[0]);
	bdata = (struct gpio_button_data *)malloc(sizeof(struct gpio_button_data) * nbuttons);
	if (NULL == bdata) {
		printf("button data malloc err\n");
		return -1;
	}

	for (i = 0; i < nbuttons; i++) {
		bdata[i].gpio = key_config[i].gpio;
		bdata[i].code = key_config[i].code;
		if (key_config[i].active_flag == ACTIVE_LOW)
			bdata[i].active_low = 1;
		else
			bdata[i].active_low = 0;
	}

	key_drvdata->nbuttons = nbuttons;
	key_drvdata->bdata = bdata;

	return 0;
}

int gpio_key_polled_init(void)
{
	int i;
	int ret;
	char state;
	struct sunxi_input_dev *input_dev;

	key_drvdata = (struct gpio_key_drvdata *)malloc(sizeof(struct gpio_key_drvdata));
	if (NULL == key_drvdata) {
		printf("button data malloc err\n");
		return -1;
	}

	ret = gpio_key_polled_get_config();
	if (ret < 0)
		goto err1;

	for (i = 0; i < key_drvdata->nbuttons; i++) {
#if 1
		hal_gpio_set_direction((hal_gpio_pin_t)key_drvdata->bdata[i].gpio, HAL_GPIO_DIRECTION_INPUT);
		hal_gpio_get_input((hal_gpio_pin_t)key_drvdata->bdata[i].gpio, (hal_gpio_data_t *)&state);
		key_drvdata->bdata[i].last_state = state;
#endif
	}

	//input dev init
	input_dev = sunxi_input_allocate_device();
	if (NULL == input_dev) {
		printf("allocate input_dev err\n");
		goto err2;
	}
	input_dev->name = INPUT_DEV_NAME;
	for (i = 0; i < key_drvdata->nbuttons; i++)
		input_set_capability(input_dev, EV_KEY, key_drvdata->bdata[i].code);

	key_drvdata->input_dev = input_dev;
	sunxi_input_register_device(key_drvdata->input_dev);

	task_create("gpio_key_poll", GPIO_KEY_TASK_PRO, GPIO_KEY_STACK_SIZE, gpio_key_polled_poll, NULL);

#ifdef TEST_TASK
	task_create("gpio_key_app_getevent", GPIO_KEY_TASK_PRO, GPIO_KEY_STACK_SIZE, key_scan, NULL);
#endif

	return 0;

err2:
	free(key_drvdata->bdata);

err1:
	free(key_drvdata);
	return ret;
}
