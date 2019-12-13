/*
 * Copyright 2019 AllWinnertech  Co., Ltd
 * frank@allwinnertech.com
 */

#ifndef __HAL_THERMAL_H__
#define __HAL_THERMAL_H__

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Enums
 *****************************************************************************/
typedef enum{
	HAL_THERMAL_STATUS_ERROR_BUSY = -3,
	HAL_THERMAL_STATUS_ERROR_CALIBRATE = -2,
	HAL_THERMAL_STATUS_ERROR = -1,
	HAL_THERMAL_STATUS_OK = 0
}hal_thermal_status_t;

/*****************************************************************************
 * Functions
 *****************************************************************************/

hal_thermal_status_t hal_thermal_init(void);
hal_thermal_status_t hal_thermal_get_temp(unsigned int num, int *temp);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_THERMAL_H__ */
