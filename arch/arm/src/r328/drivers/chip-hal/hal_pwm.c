/* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.

 * Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
 * the the People's Republic of China and other countries.
 * All Allwinner Technology Co.,Ltd. trademarks are used with permission.

 * DISCLAIMER
 * THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
 * IF YOU NEED TO INTEGRATE THIRD PARTY¡¯S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
 * IN ALLWINNERS¡¯SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
 * ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
 * ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
 * COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
 * YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY¡¯S TECHNOLOGY.


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

#include <stdint.h>

#include "hal_gpio.h"
#include "hal_clk.h"
//#include "../../../include/drivers/hal_pwm.h"
#include "pwm.h"

hal_pwm_status_t hal_pwm_init(void)
{
	//clk_init();
	if(hal_clock_enable(HAL_CLK_PERIPH_PWM))
		return HAL_PWM_STATUS_ERROR;

	return HAL_PWM_STATUS_OK;
}

hal_pwm_status_t hal_pwm_config(uint32_t channel, uint32_t duty_ns, uint32_t period_ns)
{
	unsigned long long c = 0;
	unsigned long entire_cycles = 0, active_cycles = 0;
	uint32_t pre_scal_id = 0, div_m = 0, prescale = 0;
	uint32_t pre_scal[][2] = {
	/*reg_val   clk_pre_div*/
		{0, 1},
		{1, 2},
		{2, 4},
		{3, 8},
		{4, 16},
		{5, 32},
		{6, 64},
		{7, 128},
		{8, 256},
	};

	if (period_ns <= 10) {
		return HAL_PWM_STATUS_ERROR_PARAMETER;
	} else if (period_ns > 10 && period_ns <= 334) {
		/* if freq between 3M~100M, then select 100M as clock */
		c = 100000000;
		/*set src apb1*/
		pwm_clk_src_set(channel, 1);
	} else {
		/* if freq < 3M, then select 24M clock */
		c = 24000000;
		/*set src osc24*/
		pwm_clk_src_set(channel, 0);
	}

	c = c * period_ns / 1000000000;
	entire_cycles = (unsigned long)c;

	/*check the div_m and prescale*/
	for (pre_scal_id = 0; pre_scal_id < 9; pre_scal_id++) {
		if (entire_cycles <= 65536)
			break;
		for (prescale = 0; prescale < 256; prescale++) {
			entire_cycles = (entire_cycles/pre_scal[pre_scal_id][1])/(prescale + 1);
			if (entire_cycles <= 65536) {
				div_m = pre_scal[pre_scal_id][0];
				break;
			}
		}
	}

	c = (unsigned long long)entire_cycles * duty_ns;
	active_cycles = c/period_ns;

	if (entire_cycles == 0)
		entire_cycles++;

	/*config div_m*/
	pwm_clk_div_m(channel, div_m);

	/*config prescale*/
	pwm_prescal_set(channel, prescale);

	/*config active_cycles*/
	pwm_set_active_cycles(channel, active_cycles);

	/*config entire_cycles*/
	pwm_set_period_cycles(channel, entire_cycles);

	return HAL_PWM_STATUS_OK;
}

hal_pwm_status_t hal_pwm_enable(uint32_t channel)
{
	hal_gpio_pinmux_set_function(GPIOB(channel), 3);
	/*enable clk gating*/
	pwm_enable_clk_gating(channel);
	/*enable pwm controller*/
	pwm_enable_controller(channel);

	return HAL_PWM_STATUS_OK;
}

hal_pwm_status_t hal_pwm_disable(uint32_t channel)
{
	/*disable single channel*/
	pwm_disable_controller(channel);

	return HAL_PWM_STATUS_OK;
}

hal_pwm_status_t hal_pwm_set_polarity(uint32_t channel, hal_pwm_polarity polarity)
{
	pwm_porality(channel, polarity);

	return HAL_PWM_STATUS_OK;
}
