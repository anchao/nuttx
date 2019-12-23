/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the people's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY’S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS’SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY’S TECHNOLOGY.
*
*
* THIS SOFTWARE IS PROVIDED BY ALLWINNER"AS IS" AND TO THE MAXIMUM EXTENT
* PERMITTED BY LAW, ALLWINNER EXPRESSLY DISCLAIMS ALL WARRANTIES OF ANY KIND,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING WITHOUT LIMITATION REGARDING
* THE TITLE, NON-INFRINGEMENT, ACCURACY, CONDITION, COMPLETENESS, PERFORMANCE
* OR MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* IN NO EVENT SHALL ALLWINNER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "codec_common.h"

int codec_i2c_init(codec_i2c_config_t *config)
{
	int ret;

	ret = hal_i2c_master_init(config);
	if (ret == HAL_I2C_STATUS_ERROR_BUSY) {
		snd_info("i2c[%d] already init?\n", config->port);
		return 0;
	}
	if (ret != HAL_I2C_STATUS_OK)
		snd_err("codec_i2c_init error, return %d\n", ret);
	return ret;
}

int codec_i2c_deinit(unsigned int bus)
{
	int ret;

	ret = hal_i2c_master_deinit(bus);
	if (ret != HAL_I2C_STATUS_OK)
		snd_err("codec_i2c_deinit error, return %d\n", ret);
	return ret;
}

int codec_i2c_read(struct codec_i2c_device *twi_dev,
	unsigned char reg, unsigned char *rt_value)
{
	int ret;
	unsigned char read_cmd[1] = {0};

	read_cmd[0] = reg;
	ret = hal_i2c_master_receive(twi_dev->bus,
				twi_dev->addr,
				read_cmd, sizeof(read_cmd), rt_value, 1, 0);
	if (ret != HAL_I2C_STATUS_OK) {
		snd_err("codec_i2c_read error, [REG-0x%02x] ret = %d.\n", reg, ret);
		return ret;
	}

	return 0;
}

int codec_i2c_write(struct codec_i2c_device *twi_dev,
	unsigned char reg, unsigned char value)
{
	int ret = 0;
	unsigned char write_cmd[2] = {0};

	write_cmd[0] = reg;
	write_cmd[1] = value;

	ret = hal_i2c_master_send(twi_dev->bus,
				twi_dev->addr, write_cmd,
				sizeof(write_cmd), 0);
	if (ret != HAL_I2C_STATUS_OK) {
		snd_err("codec_i2c_write error [REG-0x%02x,val-0x%02x] ret = %d.\n",
				reg, value, ret);
		return ret;
	}

	return 0;
}

int codec_i2c_update_bits(struct codec_i2c_device *twi_dev,
	unsigned char reg, unsigned char mask, unsigned char value)
{
	unsigned char val_old = 0;
	unsigned char val_new = 0;

	codec_i2c_read(twi_dev, reg, &val_old);
	val_new = (val_old & ~mask) | (value & mask);
	if (val_new != val_old)
		codec_i2c_write(twi_dev, reg, val_new);

	return 0;
}

