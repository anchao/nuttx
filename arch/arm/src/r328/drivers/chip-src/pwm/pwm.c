/* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.

 * Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
 * the the People's Republic of China and other countries.
 * All Allwinner Technology Co.,Ltd. trademarks are used with permission.

 * DISCLAIMER
 * THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
 * IF YOU NEED TO INTEGRATE THIRD PARTY’S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
 * IN ALLWINNERS’SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
 * ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
 * ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
 * COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
 * YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY’S TECHNOLOGY.


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
#include <stdint.h>

#include "pwm.h"

#define SET_REG_VAL(reg_val, shift, width, set_val)     ((reg_val & ~((-1UL) >> (32 - width) << shift)) | (set_val << shift))

/************ config ***************/

void pwm_clk_src_set(uint32_t channel_in, hal_pwm_clk_src clk_src)  //20-7
{
	uint32_t reg_val;
	uint32_t channel = channel_in / 2;
	/*set clock source OSC24M or apb1*/
	reg_val = *(uint32_t *)((char *)&(PWM->PWM_PCCR01) + 4 * channel);
	reg_val = SET_REG_VAL(reg_val, PWM_CLK_SRC_SHIFT, PWM_CLK_SRC_WIDTH, clk_src);
	*(uint32_t *)((char *)&(PWM->PWM_PCCR01) + 4 * channel) = reg_val;
}

void pwm_clk_div_m(uint32_t channel_in, uint32_t div_m)   //20-0
{
	uint32_t reg_val;
	uint32_t channel = channel_in / 2;
	/*set clock div_m*/
	reg_val = *(uint32_t *)((char *)&(PWM->PWM_PCCR01) + 4 * channel);
	reg_val = SET_REG_VAL(reg_val, PWM_DIV_M_SHIFT, PWM_DIV_M_WIDTH, div_m);
	*(uint32_t *)((char *)&(PWM->PWM_PCCR01) + 4 * channel) = reg_val;


}

void pwm_prescal_set(uint32_t channel_in, uint32_t prescal)  //60
{
	uint32_t reg_val;
	uint32_t channel = channel_in;
	/*set prescal*/
	reg_val = *(uint32_t *)((char *)&(PWM->PWM_PCR) + 0x20 * channel);
	reg_val = SET_REG_VAL(reg_val, PWM_PRESCAL_SHIFT, PWM_PRESCAL_WIDTH, prescal);
	*(uint32_t *)((char *)&(PWM->PWM_PCR) + 0x20 * channel) = reg_val;
}

void pwm_set_active_cycles(uint32_t channel_in, uint32_t active_cycles)  //64
{
	uint32_t reg_val;
	uint32_t channel = channel_in;
	/*set active*/
	reg_val = *(uint32_t *)((char *)&(PWM->PWM_PPR) + 0x20 * channel);
	reg_val = SET_REG_VAL(reg_val, PWM_ACT_CYCLES_SHIFT, PWM_ACT_CYCLES_WIDTH, active_cycles);
	*(uint32_t *)((char *)&(PWM->PWM_PPR) + 0x20 * channel) = reg_val;
}

void pwm_set_period_cycles(uint32_t channel_in, uint32_t period_cycles)
{
	uint32_t reg_val;
	uint32_t channel = channel_in;
	/*set period*/

	reg_val = *(uint32_t *)((char *)&(PWM->PWM_PPR) + 0x20 * channel);
	reg_val = SET_REG_VAL(reg_val, PWM_PERIOD_CYCLES_SHIFT, PWM_PERIOD_CYCLES_WIDTH, period_cycles);
	*(uint32_t *)((char *)&(PWM->PWM_PPR) + 0x20 * channel) = reg_val;

}

/************   enable  **************/

void pwm_enable_clk_gating(uint32_t channel_in)
{
	uint32_t reg_val;
	uint32_t channel = channel_in / 2;
	/*enable clk_gating*/
	reg_val = *(uint32_t *)((char *)&(PWM->PWM_PCCR01) + 4 * channel);
	reg_val = SET_REG_VAL(reg_val, PWM_CLK_GATING_SHIFT, PWM_CLK_GATING_WIDTH, 1);
	*(uint32_t *)((char *)&(PWM->PWM_PCCR01) + 4 * channel) = reg_val;
}

void pwm_enable_controller(uint32_t channel)
{
	uint32_t reg_val;
	/*enable pwm*/
	reg_val = *(uint32_t *)(&(PWM->PWM_PER));
	reg_val = SET_REG_VAL(reg_val, channel, 1, 1);
	*(uint32_t *)((char *)&(PWM->PWM_PER)) = reg_val;
}

/************   disable  **************/

void pwm_disable_controller(uint32_t channel)
{
	uint32_t reg_val;
	/*enable pwm*/
	reg_val = *(uint32_t *)(&(PWM->PWM_PER));
	reg_val = SET_REG_VAL(reg_val, channel, 1, 0);
	*(uint32_t *)((char *)&(PWM->PWM_PER)) = reg_val;
}

//void pwm_disable_clk_gating(uint32_t channel)
//{
//	uint32_t reg_val;
//	uint32_t channel = channel / 2;
//	/*enable clk_gating*/
//	reg_val = *(uint32_t *)(&(PWM->PWM_PCCR01) + 4 * channel);
//	SET_REG_VAL(reg_val, PWM_CLK_GATING_SHIFT, PWM_CLK_GATING_WIDTH, 0);
//	*(uint32_t *)(&(PWM->PWM_PCCR01) + 4 * channel) = reg_val;
//}

/*************** polarity *****************/

void pwm_porality(uint32_t channel, hal_pwm_polarity polarity)
{
	uint32_t reg_val;
	/*set polarity*/
	reg_val = *(uint32_t *)((char *)&(PWM->PWM_PCR) + 0x20 * channel);
	reg_val = SET_REG_VAL(reg_val, PWM_ACT_STA_SHIFT, PWM_ACT_STA_WIDTH, polarity);
	*(uint32_t *)((char *)&(PWM->PWM_PCR) + 0x20 * channel) = reg_val;
}
