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

#include "hal_gpio.h"
#include "interrupt.h"
#include "../chip-src/gpio/r328_pio.h"
#include "../chip-src/gpio/gpio-sun8iw18p1.h"
#include "r328_lowputc.h"
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
hal_pinmux_status_t hal_gpio_pinmux_set_function(hal_gpio_pin_t gpio_pin, uint32_t function_index)
{
	/*fix me: how to check the pin and the function compatible?*/
	int ret = 0;

	if (function_index > 7) {
		return HAL_PINMUX_STATUS_INVALID_FUNCTION;
	}

	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_PINMUX_STATUS_ERROR_PIN;
	}

	ret = gpio_pinmux_set_function((uint32_t)gpio_pin, function_index);

	if (ret) {
		return HAL_PINMUX_STATUS_ERROR;
	}

	return HAL_PINMUX_STATUS_OK;
}

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
hal_gpio_status_t hal_gpio_get_input(hal_gpio_pin_t gpio_pin, hal_gpio_data_t *gpio_data)
{
	int ret = 0;
	uint32_t data;
	if (NULL == gpio_data) {
		return HAL_GPIO_STATUS_INVALID_PARAMETER;
	}
	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_get_data((uint32_t)gpio_pin, &data);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}
	*gpio_data = (hal_gpio_data_t)data;

	return HAL_GPIO_STATUS_OK;
}

/**
 * @hua       This function sets the output data of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to operate.
 * @param[in] gpio_data is the output data of the target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_INVALID_PARAMETER, a wrong parameter (except for pin number) is given, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, invalid input pin number, the parameter must be verified.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_output(hal_gpio_pin_t gpio_pin, hal_gpio_data_t gpio_data)
{
	int ret = 0;

	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_set_data((uint32_t)gpio_pin,(uint32_t)gpio_data);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}
	return HAL_GPIO_STATUS_OK;
}


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
 /* should we keep this function? merge with function hal_gpio_get_input()*/
hal_gpio_status_t hal_gpio_get_output(hal_gpio_pin_t gpio_pin, hal_gpio_data_t *gpio_data)
{
	int ret = 0;
	uint32_t data;
	if (NULL == gpio_data) {
		return HAL_GPIO_STATUS_INVALID_PARAMETER;
	}
	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_get_data((uint32_t)gpio_pin, &data);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}
	*gpio_data = (hal_gpio_data_t)data;

	return HAL_GPIO_STATUS_OK;

}

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
hal_gpio_status_t hal_gpio_set_direction(hal_gpio_pin_t gpio_pin, hal_gpio_direction_t gpio_direction)
{
	int ret = 0;

	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_set_direction((uint32_t)gpio_pin, (uint32_t)gpio_direction);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}

	return HAL_GPIO_STATUS_OK;
}


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
hal_gpio_status_t hal_gpio_get_direction(hal_gpio_pin_t gpio_pin, hal_gpio_direction_t *gpio_direction)
{
	int ret = 0;
	uint32_t direction;
	if (NULL == gpio_direction) {
		return HAL_GPIO_STATUS_INVALID_PARAMETER;
	}
	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_get_direction((uint32_t)gpio_pin, &direction);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}
	*gpio_direction = (hal_gpio_data_t)direction;

	return HAL_GPIO_STATUS_OK;
}

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
hal_gpio_status_t hal_gpio_set_pull_state(hal_gpio_pin_t gpio_pin, hal_gpio_pull_down_t gpio_pull_state)
{
	int ret = 0;

	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_set_pull((uint32_t)gpio_pin, (uint32_t)gpio_pull_state);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}

	return HAL_GPIO_STATUS_OK;
}

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
hal_gpio_status_t hal_gpio_get_pull_state(hal_gpio_pin_t gpio_pin, hal_gpio_pull_down_t *gpio_pull_state)
{
	int ret = 0;
	uint32_t pull;

	if (NULL == gpio_pull_state) {
		return HAL_GPIO_STATUS_INVALID_PARAMETER;
	}

	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_get_pull((uint32_t)gpio_pin, &pull);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}

	*gpio_pull_state = (hal_gpio_pull_down_t)pull;

	return HAL_GPIO_STATUS_OK;
}

/**
 * @hua       This function sets the driving current of the target GPIO.
 * @param[in] gpio_pin specifies the pin number to configure.
 * @param[in] gpio_driving_level specifies the driving current to set to target GPIO.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR_PIN, the operation failed.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_set_driving_level(hal_gpio_pin_t gpio_pin, hal_gpio_driving_level_t gpio_driving_level)
{
	int ret = 0;

	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_set_driving_level((uint32_t)gpio_pin, (uint32_t)gpio_driving_level);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}

	return HAL_GPIO_STATUS_OK;

}

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
hal_gpio_status_t hal_gpio_get_driving_level(hal_gpio_pin_t gpio_pin, hal_gpio_driving_level_t *gpio_driving_level)
{
	int ret = 0;
	uint32_t level;

	if (NULL == gpio_driving_level) {
		return HAL_GPIO_STATUS_INVALID_PARAMETER;
	}

	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	ret = gpio_get_driving_level((uint32_t)gpio_pin, &level);
	if (ret) {
		return HAL_GPIO_STATUS_ERROR;
	}

	*gpio_driving_level = (hal_gpio_pull_down_t)level;

	return HAL_GPIO_STATUS_OK;

}

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
hal_gpio_status_t hal_gpio_set_debounce(hal_gpio_pin_t gpio_pin,uint32_t freq)
{
	return HAL_GPIO_STATUS_OK;
}

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
hal_gpio_status_t hal_gpio_get_debounce(hal_gpio_pin_t gpio_pin,uint32_t *freq)
{
	return HAL_GPIO_STATUS_OK;
}

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
hal_gpio_status_t hal_gpio_to_irq(hal_gpio_pin_t gpio_pin, uint32_t *irq)
{
//	int ret = 0;

	if (NULL == irq) {
		return HAL_GPIO_STATUS_INVALID_PARAMETER;
	}
	if (!check_gpio_valid((uint32_t)gpio_pin)) {
		return HAL_GPIO_STATUS_ERROR_PIN;
	}

	*irq = gpio_to_irq((uint32_t)gpio_pin);
	if (*irq < 0)
		return HAL_GPIO_STATUS_ERROR;

	return HAL_GPIO_STATUS_OK;
}

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
//hal_gpio_status_t hal_gpio_irq_request(uint32_t irq, interrupt_handler_t *hdle, unsigned long flags, void *data)
hal_gpio_status_t hal_gpio_irq_request(uint32_t irq, xcpt_t hdle, unsigned long flags, void *data)
{
	int ret = 0;

	ret = gpio_irq_request(irq, hdle, flags, data);
	if (ret < 0)
		return HAL_GPIO_STATUS_ERROR;

	return HAL_GPIO_STATUS_OK;
}

/**
 * @hua       This function free an irq of gpio.
 * @param[in] irq specifies the irq to free.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_irq_free(uint32_t irq)
{
	int ret = 0;

	ret = gpio_irq_free(irq);
	if (ret < 0)
		return HAL_GPIO_STATUS_ERROR;

	return HAL_GPIO_STATUS_OK;
}

/**
 * @hua       This function enable an irq of the gpio.
 * @param[in] irq specifies the irq to enable.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_irq_enable(uint32_t irq)
{
	int ret = 0;

	ret = gpio_irq_enable(irq);
	if (ret < 0)
		return HAL_GPIO_STATUS_ERROR;

	return HAL_GPIO_STATUS_OK;
}

/**
 * @hua       This function disable an irq of the gpio.
 * @param[in] irq specifies the irq to disable.
 * @return    To indicate whether this function call is successful or not.
 *            If the return value is #HAL_GPIO_STATUS_OK, the operation completed successfully.
 *            If the return value is #HAL_GPIO_STATUS_ERROR, the operation failed.
 * @note
 * @warning
 */
hal_gpio_status_t hal_gpio_irq_disable(uint32_t irq)
{
	int ret = 0;

	ret = gpio_irq_disable(irq);
	if (ret < 0)
		return HAL_GPIO_STATUS_ERROR;

	return HAL_GPIO_STATUS_OK;
}

hal_gpio_status_t hal_gpio_init(void)
{
//	sunxi_gpio_init();
	sun8iw18p1_gpio_init();
	return HAL_GPIO_STATUS_OK;
}
