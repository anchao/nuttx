/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the people's Republic of China and other countries.
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

#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__

#include "../../drivers/chip-src/gpio/hardware/r328_pio.h"
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** This enum defines the GPIO direction. */
typedef enum {
    HAL_GPIO_DIRECTION_INPUT  = 0,              /**<  GPIO input direction. */
    HAL_GPIO_DIRECTION_OUTPUT = 1               /**<  GPIO output direction. */
} hal_gpio_direction_t;

/** This enum defines the data type of GPIO. */
typedef enum {
    HAL_GPIO_DATA_LOW  = 0,                     /**<  GPIO data low. */
    HAL_GPIO_DATA_HIGH = 1                      /**<  GPIO data high. */
} hal_gpio_data_t;

/** This enum defines the return type of GPIO API. */
typedef enum {
    HAL_GPIO_STATUS_ERROR             = -3,     /**< The GPIO function failed to execute.*/
    HAL_GPIO_STATUS_ERROR_PIN         = -2,     /**< Invalid input pin number. */
    HAL_GPIO_STATUS_INVALID_PARAMETER = -1,     /**< Invalid input parameter. */
    HAL_GPIO_STATUS_OK                = 0       /**< The GPIO function executed successfully. */
} hal_gpio_status_t;

/** This enum defines the return type of pinmux API. */
typedef enum {
    HAL_PINMUX_STATUS_ERROR_PIN   = -4,
    HAL_PINMUX_STATUS_ERROR             = -3,   /**< The pinmux function failed to execute.*/
    HAL_PINMUX_STATUS_ERROR_PORT        = -2,   /**< Invalid input pin port. */
    HAL_PINMUX_STATUS_INVALID_FUNCTION  = -1,   /**< Invalid input function. */
    HAL_PINMUX_STATUS_OK                = 0     /**< The pinmux function executed successfully. */
} hal_pinmux_status_t;

typedef enum {
    HAL_GPIO_DRIVING_LEVEL0    = 0,        /**< Defines GPIO driving current as level0.  */
    HAL_GPIO_DRIVING_LEVEL1    = 1,        /**< Defines GPIO driving current as level1.  */
    HAL_GPIO_DRIVING_LEVEL2    = 2,        /**< Defines GPIO driving current as level2. */
    HAL_GPIO_DRIVING_LEVEL3    = 3         /**< Defines GPIO driving current as level3. */
} hal_gpio_driving_level_t;

typedef enum {
    HAL_GPIO_PULL_DOWN_DISABLE    = 0,        /**< Defines GPIO pull up and pull down disable.  */
    HAL_GPIO_PULL_UP		  = 1,        /**< Defines GPIO is pull up state.  */
    HAL_GPIO_PULL_DOWN            = 2,        /**< Defines GPIO is pull down state. */
} hal_gpio_pull_down_t;

/**
 * @hua       This function configures the pinmux of target GPIO.
 *            Pin Multiplexer (pinmux) connects the pin and the onboard peripherals,
 *            so the pin will operate in a specific mode once the pin is programmed to a peripheral's function.
 *            The alternate functions of every pin are provided in hal_pinmux_define.h.
 * @param[in] gpio_pin specifies the pin number to configure.
 * @param[in] function_index specifies the function for the pin.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_PINMUX_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_PINMUX_STATUS_INVALID_FUNCTION, a wrong alternate function is given, the parameter must be verified.
 *            If the return value is #HAL_PINMUX_STATUS_ERROR_PORT, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_PINMUX_STATUS_ERROR, the operation failed.
 * @notes
 * @warning
 */
hal_pinmux_status_t hal_gpio_pinmux_set_function(hal_gpio_pin_t gpio_pin, uint32_t function_index);

/**
 * @hua       This function gets the input data of target GPIO when the direction of the GPIO is input.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[out] gpio_data is the input data received from the target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_input(hal_gpio_pin_t gpio_pin, hal_gpio_data_t *gpio_data);

/**
 * @hua       This function sets the output data of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[in] gpio_data is the output data of the target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_output(hal_gpio_pin_t gpio_pin, hal_gpio_data_t gpio_data);


/**
 * @hua       This function gets the output data of the target GPIO when the direction of the GPIO is output.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[in] gpio_data is output data of the target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_output(hal_gpio_pin_t gpio_pin, hal_gpio_data_t *gpio_data);

/**
 * @hua       This function sets the direction of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to set.
 * @param[in] gpio_direction is the direction of the target GPIO, the direction can be input or output.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_direction(hal_gpio_pin_t gpio_pin, hal_gpio_direction_t gpio_direction);


/**
 * @hua       This function gets the direction of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[output] gpio_direction is the direction of target GPIO, the direction can be input or output.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_direction(hal_gpio_pin_t gpio_pin, hal_gpio_direction_t *gpio_direction);

/**
 * @hua       This function sets the pull or down state of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[in] gpio_pull_state is the pull state of target GPIO, the state can be disable(float),pull up,pull down.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_pull_state(hal_gpio_pin_t gpio_pin, hal_gpio_pull_down_t gpio_pull_state);

/**
 * @hua       This function gets the pull or down state of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[output] gpio_pull_state is the pull state of target GPIO, the state can be disable(float),pull up,pull down.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_pull_state(hal_gpio_pin_t gpio_pin, hal_gpio_pull_down_t *gpio_pull_state);

/**
 * @hua       This function sets the driving current of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to configure.
 * @param[in] level specifies the driving current to set to target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_driving_level(hal_gpio_pin_t gpio_pin, hal_gpio_driving_level_t gpio_driving_level);

/**
 * @hua       This function gets the driving current of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to configure.
 * @param[out] level specifies the driving current to be set to target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_driving_level(hal_gpio_pin_t gpio_pin, hal_gpio_driving_level_t *gpio_driving_level);

/**
 * @hua       This function sets the debounce of the bank which has the target GPIO.
 * @param[in] gpio_pin specifies the pin number to find the bank to configure.
 * @param[out] freq specifies the freq to be set to target bank of the gpio,max freq is 24M.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_debounce(hal_gpio_pin_t gpio_pin,uint32_t freq);

/**
 * @hua       This function gets the debounce of the bank which has the target GPIO.
 * @param[in] gpio_pin specifies the pin number to find the bank to get.
 * @param[out] freq specifies the freq to be get to target bank of the gpio,max freq is 24M.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
  *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_get_debounce(hal_gpio_pin_t gpio_pin,uint32_t *freq);

/**
 * @hua       This function gets the irqnum of the gpio.
 * @param[in] gpio_pin specifies the pin number to find the irq num to get.
 * @param[out] freq specifies the irq to be set to target irq num of the gpio.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_to_irq(hal_gpio_pin_t gpio_pin, uint32_t *irq);

/**
 * @hua       This function request an irq of gpio.
 * @param[in] irq specifies the irq to request.
 * @param[in] hdle specifies the irq handler
 * @param[in] flags specifiles the irq detach type
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
//hal_gpio_status_t hal_gpio_irq_request(uint32_t irq, interrupt_handler_t *hdle, unsigned long flags, void *data);
hal_gpio_status_t hal_gpio_irq_request(uint32_t irq, xcpt_t hdle, unsigned long flags, void *data);

/**
 * @hua       This function free an irq of gpio.
 * @param[in] irq specifies the irq to free.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_irq_free(uint32_t irq);

/**
 * @hua       This function enable an irq of the gpio.
 * @param[in] irq specifies the irq to enable.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_irq_enable(uint32_t irq);

/**
 * @hua       This function disable an irq of the gpio.
 * @param[in] irq specifies the irq to disable.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_irq_disable(uint32_t irq);
hal_gpio_status_t hal_gpio_init();

#ifdef __cplusplus
}
#endif

#endif

