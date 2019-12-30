/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the People's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY¡¯S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS¡¯SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY¡¯S TECHNOLOGY.
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

//#include <awlog.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <hal_i2c.h>
#include "../chip-src/i2c/i2c.h"

#ifdef DEBUG
#define HAL_I2C_ERR(fmt, arg...) printf("%s()%d "fmt, __func__, __LINE__, ##arg)
#else
#define HAL_I2C_ERR(fmt, arg...)
#endif

volatile static uint8_t s_i2c_master_status[HAL_I2C_MASTER_MAX] = {0};

static inline bool hal_i2c_is_freq_valid(hal_i2c_frequency_t freq)
{
    return freq < HAL_I2C_FREQUENCY_MAX;
}

static inline bool hal_i2c_is_config_valid(const hal_i2c_config_t *config)
{
    if (NULL == config) {
	HAL_I2C_ERR("invalid parameter\n");
        return false;
    }
    if (!hal_i2c_is_freq_valid(config->freq)) {
	HAL_I2C_ERR("frequency invalid\n");
        return false;
    }
    return true;
}

static inline bool hal_i2c_is_port_valid(hal_i2c_port_t port)
{
    return port < HAL_I2C_MASTER_MAX;
}

hal_i2c_status_t hal_i2c_master_init(hal_i2c_config_t *config)
{
    hal_i2c_status_t busy_status;

    if (NULL == config)
	    return HAL_I2C_STATUS_INVALID_PARAMETER;
    if (!hal_i2c_is_port_valid(config->port)) {
        HAL_I2C_ERR("Wrong I2C port: %d", config->port);
        return HAL_I2C_STATUS_INVALID_PORT_NUMBER;
    }
    if (!hal_i2c_is_config_valid(config)) {
        HAL_I2C_ERR("Config is invalid");
        return HAL_I2C_STATUS_INVALID_PARAMETER;
    }

    i2c_check_and_set_busy(config->port, busy_status);
    if (HAL_I2C_STATUS_ERROR_BUSY == busy_status) {
	HAL_I2C_ERR("i2c port is busy\n");
        return HAL_I2C_STATUS_ERROR_BUSY;
    }

    sunxi_i2c_init(config);

    return HAL_I2C_STATUS_OK;
}

hal_i2c_status_t hal_i2c_master_deinit(hal_i2c_port_t port)
{
    if (!hal_i2c_is_port_valid(port)) {
        HAL_I2C_ERR("Wrong I2C port: %d", port);
        return HAL_I2C_STATUS_INVALID_PORT_NUMBER;
    }

    sunxi_i2c_deinit(port);

    /* unlock i2c */
    i2c_set_idle(port);
    return HAL_I2C_STATUS_OK;
}

hal_i2c_status_t hal_i2c_master_send(hal_i2c_port_t port, uint8_t slave_address,
		uint8_t *data, uint32_t size, uint16_t flag)
{
	 i2c_msg_t msg;
	 uint8_t num = 1;

	 msg.addr =  slave_address;
	 msg.flags = flag & ~I2C_M_RD;
	 msg.len = size;
	 msg.buf = data;

	return sunxi_i2c_xfer(port, &msg, num);

}

hal_i2c_status_t hal_i2c_master_receive(hal_i2c_port_t port, uint8_t slave_address,
		uint8_t *command, uint8_t command_len, uint8_t *buffer,
		uint32_t size, uint16_t flag)
{
	 i2c_msg_t msg[2];
	 uint8_t num = 2;

	 msg[0].addr =  slave_address;
	 msg[0].flags = flag & ~I2C_M_RD;
	 msg[0].len = command_len;
	 msg[0].buf = command;

	 msg[1].addr =  slave_address;
	 msg[1].flags = I2C_M_RD;
	 msg[1].len = size;
	 msg[1].buf = buffer;
	return sunxi_i2c_xfer(port, msg, num);
}

/* Function: hal_i2c_msg_receive/send
 *
 * These two functions are only used for i2ctool(file:r328_i2c.c).
 * If you want to use them for drivers,
 * please use the above functions: "hal_i2c_master_receive/send".
 */
hal_i2c_status_t hal_i2c_msg_receive(hal_i2c_port_t port, i2c_msg_t *msg, uint8_t num)
{
	return sunxi_i2c_xfer(port, msg, num);
}
hal_i2c_status_t hal_i2c_msg_send(hal_i2c_port_t port, i2c_msg_t *msg, uint8_t num)
{
	if (num == 2) {
		memcpy(msg[0].buf + msg[0].len, msg[1].buf, msg[1].len);
		msg[0].len = msg[0].len + msg[1].len;
		return sunxi_i2c_xfer(port, &msg[0], num);
	}
	return sunxi_i2c_xfer(port, msg, num);
}
