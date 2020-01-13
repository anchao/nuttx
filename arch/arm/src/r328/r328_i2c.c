/****************************************************************************
 * arch/arm/src/r328/r328_i2c.c
 *
 *   Copyright (C) 2013-2014, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <semaphore.h>
//#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
//#include <nuttx/arch.h>
//#include <nuttx/fs/ioctl.h>
#include <nuttx/nuttx.h>
#include <nuttx/i2c/i2c_master.h>

#include "./drivers/chip-src/i2c/i2c.h"
#include <hal_clk.h>
#include <hal_i2c.h>
#include <hal_gpio.h>

#ifdef DEBUG
#define I2C_INFO(fmt, arg...) printf("%s:%d "fmt,__func__,  __LINE__, ##arg)
#else
#define I2C_INFO(fmt, arg...)
#endif

#define I2C_ERR(fmt, arg...) printf("%s:%d "fmt, __func__, __LINE__, ##arg)

#define R328_I2C_OK      0
#define R328_I2C_FAIL   -1
#define R328_I2C_RETRY  -2
#define R328_I2C_SFAIL  -3  /* start fail */
#define R328_I2C_TFAIL  -4  /* stop  fail */

/************************************************************************************
 * Structure
 ************************************************************************************/
struct r328_i2c_priv_s {
	const struct i2c_ops_s *ops;
	hal_i2c_config_t *config;
	//int port;
};


/************************************************************************************
 * Function declaration
 ************************************************************************************/
static int sunxi_i2c_transfer(FAR struct i2c_master_s *dev,
		FAR struct i2c_msg_s *msg_s, int count);
static int sunxi_i2c_reset(FAR struct i2c_master_s *dev);
FAR struct i2c_master_s *r328_i2cbus_initialize(int port);
int r328_i2cbus_uninitialize(FAR struct i2c_master_s *dev);
void r328_i2c_register(int bus);
void r328_i2c_choose(void);

/************************************************************************************
 * structure member definition
 ************************************************************************************/
struct i2c_ops_s sunxi_i2c_ops = {
	.transfer = sunxi_i2c_transfer,
#ifdef CONFIG_I2C_RESET
	.reset = sunxi_i2c_reset,
#endif
};
/*
struct i2c_master_s sunxi_i2c_master = {
	.ops = &sunxi_i2c_ops,
};
*/
hal_i2c_config_t config_s0 = {
        .port = HAL_I2C_MASTER_0,
        .freq = HAL_I2C_FREQUENCY_400K,
        .clk = {GPIOH(0), 2, 7},
	.sda = {GPIOH(1), 2, 7},
};
struct r328_i2c_priv_s priv_s0 = {
	.ops = &sunxi_i2c_ops,
	.config = &config_s0,
};

hal_i2c_config_t config_s1 = {
        .port = HAL_I2C_MASTER_1,
        .freq = HAL_I2C_FREQUENCY_200K,
	//.trans_mode = HAL_TWI_DRV_XFER,
        .clk = {GPIOH(2), 2, 7},
        .sda = {GPIOH(3), 2, 7},
};
struct r328_i2c_priv_s priv_s1 = {
	.ops = &sunxi_i2c_ops,
	.config = &config_s1,
};

/************************************************************************************
 * Function
 ************************************************************************************/
static int sunxi_i2c_transfer(FAR struct i2c_master_s *dev,
		FAR struct i2c_msg_s *msg_s, int count)
{
	int ret;
	i2c_msg_t msg_t[2];
	struct r328_i2c_priv_s *priv = (struct r328_i2c_priv_s *)dev;

	if (count <= 0 || count > 2) {
		I2C_ERR("[i2c%d] invalid argument\n", priv->config->port);
		return R328_I2C_FAIL;
	} else {
		msg_t[0].addr  = msg_s[0].addr;
		msg_t[0].flags = msg_s[0].flags;
		msg_t[0].buf   = msg_s[0].buffer;
		msg_t[0].len   = msg_s[0].length;
		if (count == 2) {
			msg_t[1].addr  = msg_s[1].addr;
			msg_t[1].flags = msg_s[1].flags;
			msg_t[1].buf   = msg_s[1].buffer;
			msg_t[1].len   = msg_s[1].length;
		}
	}

	if (msg_t[0].flags == I2C_M_RD || msg_t[1].flags == I2C_M_RD) {
		I2C_ERR("choose i2c%d to transfer read.\n", priv->config->port);
		ret = hal_i2c_msg_receive(priv->config->port, msg_t, count);
		if (ret < 0) {
			I2C_ERR("ERROR:i2c msg receive failed %d.\n",count);
			return R328_I2C_FAIL;
		}
	} else {
		I2C_ERR("choose i2c%d to transfer write.\n", priv->config->port);
		ret = hal_i2c_msg_send(priv->config->port, msg_t, count);
		if (ret != 0) {
			I2C_ERR("ERROR:i2c msg send failed %d.\n",count);
			return R328_I2C_FAIL;
		}
	}
	return R328_I2C_OK;
}

#ifdef CONFIG_I2C_RESET
static int sunxi_i2c_reset(FAR struct i2c_master_s *dev)
{
	struct r328_i2c_priv_s *priv = (struct r328_i2c_priv_s *)dev;
	const uint32_t base_addr = SUNXI_TWI0_PBASE + \
			(priv->config->port * SUNXI_I2C_REG_SIZE);
	uint32_t reg_val = readl(base_addr + TWI_SRST_REG);
	reg_val |= TWI_SRST_SRST;
	writel(reg_val, base_addr + TWI_SRST_REG);
	return R328_I2C_OK;
}
#endif


FAR struct i2c_master_s *r328_i2cbus_initialize(int port)
{
	int ret = -1;
	//hal_i2c_config_t *config_t = NULL;
	struct r328_i2c_priv_s *priv = NULL;
	switch (port)
	{
		case 0:
			//config_t = (hal_i2c_config_t *)&priv_s0;
			priv = &priv_s0;
			break;
		case 1:
			//config_t = (hal_i2c_config_t *)&priv_s1;
			priv = &priv_s1;
			break;
		default:
			return NULL;
	}
	I2C_INFO("choose the i2c%d channel.\n",port);
	ret = hal_i2c_master_init(priv->config);
	if(ret == 0)
		I2C_INFO("i2c init success.\n");
	else {
		I2C_ERR("ERROR:i2c init failed.\n");
		return NULL;
	}
	//return (struct i2c_master_s *)&sunxi_i2c_master;
	return (struct i2c_master_s *)priv;
}

int r328_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
	int ret = -1;
	struct r328_i2c_priv_s *priv = (struct r328_i2c_priv_s *)dev;
	I2C_INFO("deinit the i2c%d bus.\n", priv->config->port);
	ret = hal_i2c_master_deinit(priv->config->port);
	if (ret < 0)
		return R328_I2C_FAIL;
	else
		return R328_I2C_OK;
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:r328_i2c_register
 *
 * Description:
 *	register a i2c drivers node.
 *

 ****************************************************************************/
#if defined(CONFIG_I2C) && defined(CONFIG_DRIVERS_I2C)
void r328_i2c_register(int bus)
{
	FAR struct i2c_master_s *i2c;
	int ret;

	i2c = r328_i2cbus_initialize(bus);
	if (i2c == NULL) {
		I2C_ERR("ERROR: Failed to get I2C%d interface\n", bus);
	}
	else {
		I2C_INFO("initialize success.\n");
		ret = i2c_register(i2c, bus);
		if (ret < 0) {
			I2C_ERR("ERROR: Failed to register I2C%d driver: %d\n",bus, ret);
			r328_i2cbus_uninitialize(i2c);
		}
	}
}
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_DRIVERS_I2C)
void r328_i2c_choose(void)
{
/*i2c0 and uart0 are connected to the same pin in r328s1-perf1,
 * so i2c0 should not initialized here.*/
#if 0
	r328_i2c_register(0);
#endif
	r328_i2c_register(1);
}
#endif

