#include <stdio.h>
#include <stdint.h>

/* FreeRTOS includes. */
//#include <FreeRTOS.h>
//#include <task.h>
#include <queue.h>
//#include <semphr.h>

#include <drivers/hal_udc.h>

#include "usb_msg.h"

static int vUsbGadgetDemoTask(int argc, char **argv)
{
#if 1
	usb_msg_desc_init();
	hal_udc_init();
	hal_udc_register_callback(usb_msg_callback);

	printf("init success=======\n");
//	while (1) {
//		//vTaskDelay(configTICK_RATE_HZ);
//		sleep(1);
//	}

#endif
	//vTaskDelete(NULL);
	return 0;
}

void vUsbGadgetDemoStart(void)
{
	printf("Creating usb gadget demo task...\r\n");
	vUsbGadgetDemoTask(0, NULL);
#if 0
	ret = task_create("usb-gadget", 2, 4096, vUsbGadgetDemoTask, NULL);
	printf("ret = %d\n", ret);
	if (ret < 0) {
		printf("Error creating usb gadget demo task, status was %d\r\n", ret);
		return;
	}
#endif
}
