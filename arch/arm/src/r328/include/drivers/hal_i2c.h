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

#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_BUSY  2
#define I2C_IDLE  0
#define i2c_check_and_set_busy(i2c_port,busy_status)  \
do{ \
/*    uint32_t saved_mask; */\
/*   saved_mask = save_and_set_interrupt_mask();*/ \
    if(s_i2c_master_status[i2c_port] == I2C_BUSY){ \
        busy_status = HAL_I2C_STATUS_ERROR_BUSY; \
    } else { \
        s_i2c_master_status[i2c_port] = I2C_BUSY;  \
        busy_status = HAL_I2C_STATUS_OK; \
    } \
/*	   restore_interrupt_mask(saved_mask);*/ \
}while(0)

#define i2c_set_idle(i2c_port)   \
do{  \
       s_i2c_master_status[i2c_port] = I2C_IDLE;  \
}while(0)

typedef enum {
    HAL_I2C_MASTER_0 = 0,           /**< i2c master 0. */
    HAL_I2C_MASTER_1 = 1,           /**< i2c master 1. */
    HAL_I2C_MASTER_MAX              /**< max i2c master number, \<invalid\> */
} hal_i2c_port_t;

/** @brief This enum defines the HAL interface return value. */
typedef enum {
    HAL_I2C_STATUS_ERROR = -4,                        /**<  An error occurred and the transaction has failed. */
    //HAL_I2C_STATUS_ERROR_TIMEOUT = -4,                /**<  The I2C bus xfer timeout, an error occurred. */
    HAL_I2C_STATUS_ERROR_BUSY = -3,                   /**<  The I2C bus is busy, an error occurred. */
    HAL_I2C_STATUS_INVALID_PORT_NUMBER = -2,          /**<  A wrong port number is given. */
    HAL_I2C_STATUS_INVALID_PARAMETER = -1,            /**<  A wrong parameter is given. */
    HAL_I2C_STATUS_OK = 0                             /**<  No error occurred during the function call. */
} hal_i2c_status_t;

/** @brief This enum defines the I2C bus status. */
typedef enum {
    HAL_I2C_STATUS_IDLE = 0,                         /**<  The I2C bus is idle. */
    HAL_I2C_STATUS_BUS_BUSY = 1                    /**<  The I2C bus is busy. */
} hal_i2c_running_type_t;

/** @brief This enum defines the I2C transaction speed.  */
typedef enum {
    HAL_I2C_FREQUENCY_100K = 0,          /**<  100kbps. */
    HAL_I2C_FREQUENCY_200K = 1,          /**<  200kbps. */
    HAL_I2C_FREQUENCY_400K = 2,          /**<  400kbps. */
    HAL_I2C_FREQUENCY_MAX                /**<  The total number of supported I2C frequencies (invalid I2C frequency).*/
} hal_i2c_frequency_t;

/** @brief This enum defines the I2C transaction speed.  */
typedef enum {
	HAL_ENGINE_XFER = 0,
	HAL_TWI_DRV_XFER = 1,
}hal_i2c_mode_t;

/** @brief This enum defines the I2C transaction speed.  */
typedef struct {
	uint32_t	gpio;
	uint8_t		enable_mux;
	uint8_t		disable_mux;
} hal_i2c_gpio_t;

/** @brief This structure defines the configuration settings to initialize the I2C master.  */
typedef struct {
	hal_i2c_port_t		port;
	hal_i2c_frequency_t	freq;             /**<  The transfer speed. Please refer to #hal_i2c_frequency_t for speed definition. */
	hal_i2c_mode_t		trans_mode;
	hal_i2c_gpio_t		clk;
	hal_i2c_gpio_t		sda;
} hal_i2c_config_t;


/** @brief This structure defines the I2C bus status. */
typedef struct {
    hal_i2c_running_type_t running_status;   /**<  The running status is defined in #hal_i2c_running_type_t. */
} hal_i2c_running_status_t;

typedef struct i2c_msg {
	uint16_t addr;			/* slave address */
	uint16_t flags;
#define I2C_M_RD		0x0001	/* read data, from slave to master
					 * I2C_M_RD is guaranteed to be 0x0001!
					 * */
#define I2C_M_TEN		0x0002 /* Ten bit address */
#define I2C_M_NOSTOP		0x0040 /* Message should not end with a STOP */
#define I2C_M_NOSTART		0x0080 /* Message should not begin with a START */
//#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
	uint16_t len;			/* msg length */
	uint8_t *buf;		/* pointer to msg data */
} i2c_msg_t;





hal_i2c_status_t hal_i2c_master_init(hal_i2c_config_t *i2c_config);

hal_i2c_status_t hal_i2c_master_deinit(hal_i2c_port_t i2c_port);

hal_i2c_status_t hal_i2c_master_send(hal_i2c_port_t i2c_port, uint8_t slave_address,
		uint8_t *data, uint32_t size, uint16_t flag);

hal_i2c_status_t hal_i2c_master_receive(hal_i2c_port_t i2c_port, uint8_t slave_address,
		uint8_t *command, uint8_t command_len, uint8_t *buffer,
		uint32_t size, uint16_t flag);

hal_i2c_status_t hal_i2c_msg_receive(hal_i2c_port_t port, i2c_msg_t *msg, uint8_t num);

hal_i2c_status_t hal_i2c_msg_send(hal_i2c_port_t port, i2c_msg_t *msg, uint8_t num);
#ifdef __cplusplus
}
#endif

#endif

