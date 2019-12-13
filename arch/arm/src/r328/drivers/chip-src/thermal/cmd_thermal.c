/*#####################################################################
# File Describe:cmd_thermal.c
# Author: flyranchaoflyranchao
# Created Time:flyranchao@allwinnertech.com
# Created Time:2019年09月11日 星期三 13时42分59秒
#====================================================================*/
#include <stdio.h>
#include <stdint.h>
#include <console.h>
#include <hal_thermal.h>
static int init;
int cmd_ths_gt(int argc, char ** argv)
{
	int ret,temp;

	if(!init){
		ret = hal_thermal_init();
		if(ret){
			return -1;
		}

		init = 1;
		vTaskDelay(pdMS_TO_TICKS(2000));
	}

	hal_thermal_get_temp(0, &temp);
	printf("\ntemp:%d\n", temp);

	return 0;
}

FINSH_FUNCTION_EXPORT_CMD(cmd_ths_gt, ths_gt, thermal get temp)
