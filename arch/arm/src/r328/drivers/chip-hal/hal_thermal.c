/*
 * Copyright 2019 AllWinnertech  Co., Ltd
 * frank@allwinnertech.com
 */

#include "./r328/drivers/chip-src/thermal/thermal.h"
#include <hal_thermal.h>

hal_thermal_status_t hal_thermal_init(void)
{
	char buf[8];
	int ret;

	//TODO: clk_init();
	ret = hal_efuse_read("thermal_sensor", buf, 64);
	if (ret <= 0)
		return HAL_THERMAL_STATUS_ERROR_CALIBRATE;

	ths_calibrate((short int *)buf, 4);
	ths_init();

	return HAL_THERMAL_STATUS_OK;
}

hal_thermal_status_t hal_thermal_get_temp(unsigned int num, int *temp)
{
	int ret;

	ret = ths_get_temp(num, temp);
	if (ret)
		return HAL_THERMAL_STATUS_ERROR_BUSY;

	return HAL_THERMAL_STATUS_OK;
}
