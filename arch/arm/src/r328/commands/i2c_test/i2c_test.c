#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdint.h>
#include "hal_i2c.h"
#include "hal_gpio.h"

#define DEBUG_ERR
#ifdef DEBUG
#define I2C_INFO(fmt, arg...) printf("%s()%d "fmt, __func__, __LINE__, ##arg)
#else
#define I2C_INFO(fmt, arg...)
#endif

#ifdef DEBUG_ERR
#define I2C_ERR(fmt, arg...) printf("%s()%d "fmt, __func__, __LINE__, ##arg)
#else
#define I2C_ERR(fmt, arg...) sinfo(":%d "fmt, __LINE__, ##arg)
#endif

//void i2c_test(void *unused)
void i2c_test(int argc, char **argv)
{
	//i2c init.
	int ret = 0;
	//hal_gpio_init();
	hal_i2c_config_t config = {
		.port = HAL_I2C_MASTER_1,
		.freq = HAL_I2C_FREQUENCY_200K,
		//trans mode : default twi engine.
		//.trans_mode = HAL_TWI_DRV_XFER,
		.clk  = {GPIOH(2), 2, 7},
		.sda  = {GPIOH(3), 2, 7},
	};

	ret = hal_i2c_master_init(&config);
	if (ret < 0) {
		printf("init i2c err\n");
		//vTaskDelete(NULL);
	}

	//read slave device(0x34), reg 0x10~0x19.
	printf("\n\n=========i2c read ===========\n\n");

	int i;
	unsigned char command[] = {0x10};
	unsigned char data[10] = {0};
	int flag = 0;	//I2C_M_TEN : 10bit address

	hal_i2c_master_receive(1, 0x34, &command, sizeof(command), data, sizeof(data), flag);
	for (i = 0; i < sizeof(data); i++) {
		if (i == 10)
			printf("\n");
		printf("data: %0x  ", data[i]);
	}
	//write slave device(0x34), reg 0x10~0x19.
	printf("\n\n=========i2c write ===========\n\n");

	unsigned char send[10] = {0};
	send[0] = 0x10;
	for (i = 1; i < sizeof(send); i++) {
		send[i] = i;
	}
	hal_i2c_master_send(1, 0x34, send, sizeof(send) ,flag);

	printf("\n\n=========check write ===========\n\n");
	//check write if success.
	hal_i2c_master_receive(1, 0x34, &command, sizeof(command), data, sizeof(data), flag);
	for (i = 0; i < sizeof(data); i++) {
		if (i == 10)
			printf("\n");
		printf("data: %0x  ", data[i]);
	}

	//vTaskDelete(NULL);

}
void i2c_ioctl_test(int fd, FAR struct i2c_msg_s *msgv, int msgc)
{
	int ret = -1;
	struct i2c_transfer_s xfer;
	xfer.msgv = msgv;
	xfer.msgc = msgc;
	ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&xfer);
	//ret = ioctl(fd, I2CIOC_TRANSFER, (FAR struct i2c_transfer_s *)((uintptr_t)&xfer));
	if (ret != -1)
		I2C_INFO("ioctl success ======\n");
	else
		I2C_ERR("ioctl failed ======\n");
}

void i2c_ioctl_read(int fd)
{
	unsigned char command[] = {0x10};
	unsigned char data[10] = {0};
	int i = 0, flag = 0;

	struct i2c_msg_s msg[2];

	msg[0].frequency = HAL_I2C_FREQUENCY_200K;
	msg[0].addr = 0x34;
	//msg[0].flags = flag;
	msg[0].flags = flag & ~I2C_M_RD;
	//msg[0].flags = I2C_M_NOSTOP;
	msg[0].length = sizeof(command);
	msg[0].buffer = command;

	msg[1].frequency = HAL_I2C_FREQUENCY_200K;
	msg[1].addr = 0x34;
	msg[1].flags = I2C_M_RD;
	msg[1].length = sizeof(data);
	msg[1].buffer = data;

	printf("\n\n=========`%s`:i2c read ===========\n\n",__func__);
	i2c_ioctl_test(fd ,msg, 2);

	printf("command: %0x  \n",command[0]);
	for (i = 0; i < 10; i++) {
		printf("data: %0x  ", data[i]);
		if (i == 9)
			printf("\n\n");
	}
}
void i2c_ioctl_write(int fd)
{
	int i = 0, flag = 0;
	unsigned char send[10] = {0};
	send[0] = 0x10;

	printf("\n\n=========`%s`:i2c write ===========\n\n",__func__);
	for (i = 1; i < sizeof(send); i++) {
		if (i == 9)
			send[i] = (i + 1);
		else
			send[i] = i;
	}
	for (i = 0; i < 10; i++) {
		printf("send: %0x  ", send[i]);
		if (i == 9)
			printf("\n");

	}

	struct i2c_msg_s msg = {
		.frequency = HAL_I2C_FREQUENCY_200K,
		.addr = 0x34,
		.flags = flag & ~I2C_M_RD,
		.length = sizeof(send),
		.buffer = send,
	};
	i2c_ioctl_test(fd ,&msg, 1);
}
#ifdef CONFIG_I2C_RESET
void i2c_ioctl_reset(int fd)
{
	int ret = ioctl(fd, I2CIOC_RESET, 0);
	if (ret < 0 )
		I2C_ERR("reset failed\n");
	else
		I2C_INFO("reset success\n");
}
#endif

int main(void)
{
	i2c_test(0, NULL);
	printf("\n\n");
	int fd = open("/dev/i2c1", O_RDWR);
	if (fd < 0)
		I2C_ERR("open i2c1 failed\n");
	else
		I2C_INFO("open i2c1 success\n");

	printf("\n\n");
	printf("!!!!!!!!!!!!!!!!!ioctl test!!!!!!!!!!!!!!!!!!\n");
	i2c_ioctl_read(fd);
	i2c_ioctl_write(fd);
	i2c_ioctl_read(fd);

#ifdef CONFIG_I2C_RESET
	i2c_ioctl_reset(fd);
#endif
	return 0;
}
