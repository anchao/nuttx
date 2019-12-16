#include <nuttx/config.h>
#include <nuttx/kmalloc.h>

#include <queue.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>

#include "../../../chip-src/gpadc/r328_gpadc.h"
#include <hal_gpadc.h>
#include "r328_gpadc_key.h"
//#include "../../../../include/sunxi-input.h"

#ifdef	SUNXIKBD_DEBUG
#define sunxikbd_info(fmt, args...)  printf("%s()%d - "fmt, __func__, __LINE__, ##args)
#else
#define sunxikbd_info(fmt, args...)
#endif

#define sunxikbd_err(fmt, args...)  printf("%s()%d - "fmt, __func__, __LINE__, ##args)

#define ADC_RESOL  64
#define KEY_MAX_CNT 13
#define INITIAL_VALUE 0xff

#define MAXIMUM_INPUT_VOLTAGE           1800
#define MAXIMUM_SCALE                   128
#define SCALE_UNIT                      (MAXIMUM_INPUT_VOLTAGE/MAXIMUM_SCALE)
#define VOL_RANGE			(1800000UL)

//key config
struct sunxikbd_config{
	char *name;
	unsigned int key_num;
	unsigned int scankeycodes[KEY_MAX_CNT];
	unsigned int key_vol[KEY_MAX_CNT];
};
struct sunxikbd_config key_config = {
	.name = "sunxi-keyboard",
	.key_num = 5,
	.key_vol = {164,415,646,900,1157},
	.scankeycodes = {115,114,139,28,102}
};


static unsigned char keypad_mapindex[128] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0,              /* key 1, 0-8 */
	1, 1, 1, 1, 1,                          /* key 2, 9-13 */
	2, 2, 2, 2, 2, 2,                       /* key 3, 14-19 */
	3, 3, 3, 3, 3, 3,                       /* key 4, 20-25 */
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,        /* key 5, 26-36 */
	5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,        /* key 6, 37-39 */
	6, 6, 6, 6, 6, 6, 6, 6, 6,              /* key 7, 40-49 */
	7, 7, 7, 7, 7, 7, 7                     /* key 8, 50-63 */
};


struct sunxi_gpadc_key{
	struct sunxi_input_dev *input_dev;
	unsigned int scankeycodes[KEY_MAX_CNT];
	unsigned char compare_later;
	unsigned char compare_before;
	unsigned char key_code;
	unsigned char last_key_code;
	unsigned char key_cnt;
};
struct sunxi_gpadc_key *key_data  = NULL;

static void sunxi_report_key_down_event(struct sunxi_gpadc_key *key_data)
{
	if (key_data->last_key_code == INITIAL_VALUE) {
		/* first time report key down event */
		/*
		input_report_key(key_data->input_dev,
				key_data->scankeycodes[key_data->key_code], 1);
		input_sync(key_data->input_dev);*/
		key_data->last_key_code = key_data->key_code;
		return;
	}
	if (key_data->scankeycodes[key_data->key_code] ==
			key_data->scankeycodes[key_data->last_key_code]) {
#ifdef REPORT_REPEAT_KEY_BY_INPUT_CORE
		/* report repeat key down event */
		/*
		input_report_key(key_data->input_dev,
				key_data->scankeycodes[key_data->key_code], 1);
		input_sync(key_data->input_dev);*/
#endif
	} else {
		/* report previous key up event
		 ** and report current key down event
		 **/
		/*
		input_report_key(key_data->input_dev,
				key_data->scankeycodes[key_data->last_key_code], 0);
		input_sync(key_data->input_dev);
		input_report_key(key_data->input_dev,
				key_data->scankeycodes[key_data->key_code], 1);
		input_sync(key_data->input_dev);*/
		key_data->last_key_code = key_data->key_code;
	}
}

void gpadc_irq_callback(unsigned int channel, int irq_status, unsigned int key_vol)
{
#if 1
	sunxikbd_info("Key Interrupt\n");

	//printf("=======key_vol %d=========\n", key_vol);
	if (irq_status == 0) {
		sinfo("key data down\n");
		key_data->key_cnt++;
		key_data->compare_before = (key_vol/SCALE_UNIT/1000)&0xff;
		//printf("=======compare before  %d=========\n", key_data->compare_before);
		if (key_data->compare_before == key_data->compare_later) {
			key_data->compare_later = key_data->compare_before;
			key_data->key_code = keypad_mapindex[key_data->compare_before];
			sunxi_report_key_down_event(key_data);
			key_data->key_cnt = 0;
		}
		if (key_data->key_cnt == 5) {
			key_data->compare_later = key_data->compare_before;
			key_data->key_cnt = 0;
		}
	}

	if (irq_status == 1) {
/*		input_report_key(key_data->input_dev,
				key_data->scankeycodes[key_data->key_code], 0);
		input_sync(key_data->input_dev);*/
		sinfo("key up\n");
		key_data->compare_later = 0;
		key_data->key_cnt = 0;
		key_data->last_key_code = INITIAL_VALUE;
	}
#endif
}

static int sunxikbd_data_init(struct sunxi_gpadc_key *key_data, struct sunxikbd_config *sunxikbd_config)
{
	int i, j = 0;
	int key_num = 0;
	unsigned int resol;
	unsigned int key_vol[KEY_MAX_CNT];

	key_num = sunxikbd_config->key_num;
	if (key_num < 1 || key_num > KEY_MAX_CNT)
		return -1;

	for (i = 0; i < key_num; i++)
	{
		key_data->scankeycodes[i] = sunxikbd_config->scankeycodes[i];
	}

	for (i = 0; i < key_num; i++)
	{
		key_vol[i] = sunxikbd_config->key_vol[i];
		//printf("key_vol[%d] : %d\n", i, key_vol[i]);
	}
	key_vol[key_num] = MAXIMUM_INPUT_VOLTAGE;

	for (i = 0; i < key_num; i++)
	{
		key_vol[i] += (key_vol[i+1] - key_vol[i])/2;
	}

	for (i = 0; i < 128; i++) {
		if (i * SCALE_UNIT > key_vol[j])
			j++;
		keypad_mapindex[i] = j;
		//printf("keypad_mapindex[%d] : %d\n", i, keypad_mapindex[i]);
	}

	key_data->last_key_code = INITIAL_VALUE;

	return 0;
}

static struct sunxikbd_config *get_keyboard_data(void)
{
	return &key_config;
}

int sunxi_gpadc_key_init(void)
{
	int i, ret = -1;
	struct sunxikbd_config *sunxikbd_config = NULL;
	struct sunxi_input_dev *sunxikbd_dev = NULL;

/*	key_data = pvPortMalloc(sizeof(struct sunxi_gpadc_key));
	if (NULL == key_data) {
		sunxikbd_err("key data malloc err\n");
		return -1;
	}*/
	key_data = (struct sunxikbd_drv_data *)malloc(sizeof(struct sunxi_gpadc_key));
	if (NULL == key_data) {
//		sinfo("gpadc malloc failed\n");
		return -1;
	}
	memset(key_data, 0, sizeof(struct sunxi_gpadc_key));

	//get keyboard info
	sunxikbd_config = get_keyboard_data();

	//init key data
	ret = sunxikbd_data_init(key_data, sunxikbd_config);
	if(ret < 0) {
		return -1;
	}

	//input dev init
	/*
	sunxikbd_dev = sunxi_input_allocate_device();
	if (NULL == sunxikbd_dev) {
		sunxikbd_err("allocate sunxikbd_dev err\n");
		return -1;
	}
	sunxikbd_dev->name = sunxikbd_config->name;
	for (i = 0; i < sunxikbd_config->key_num; i++)
		input_set_capability(sunxikbd_dev, EV_KEY, key_data->scankeycodes[i]);
	key_data->input_dev = sunxikbd_dev;
	sunxi_input_register_device(key_data->input_dev);
	*/
	//init lradc
	hal_gpadc_init();
	hal_gpadc_register_callback(gpadc_irq_callback);

	sunxikbd_info("sunxi-keyboard init success===========\n");

	return 0;
}
