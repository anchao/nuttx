#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "sunxi_input.h"

struct sunxi_input_dev *dev;

#define TEST_INPUT_NAME	"test"
#define TEST_KEY_CODE	114

int sunxi_input_test(void)
{
	int fd = -1;
	char input_dev_name[256];
	struct sunxi_input_event event;

	//register input dev.
	dev = sunxi_input_allocate_device();
	dev->name = TEST_INPUT_NAME;
	input_set_capability(dev, EV_KEY, TEST_KEY_CODE);
	sunxi_input_register_device(dev);


	//open input dev.
	sprintf(input_dev_name, "/dev/input/%s", TEST_INPUT_NAME);
	fd = open(input_dev_name, O_RDONLY);
	if (fd < 0) {
		printf("open err : %s", input_dev_name);
		return -1;
	}

	//test.
	while(1) {
		//driver report input event.
		input_report_key(dev, TEST_KEY_CODE, 1);
		input_sync(dev);
		printf("driver report key: %d", TEST_KEY_CODE);

		//app read input event.
		read(fd, &event, sizeof(struct sunxi_input_event));
		if (event.type != EV_KEY)
			continue;
		printf("app read : type: %d, code:%d, val: %d\n", event.type, event.code, event.value);

		sleep(1);
	}

	return 0;

}
