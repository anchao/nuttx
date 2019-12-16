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
#include <stdio.h>

#include <hal_clk.h>
#include <hal_gpadc.h>

#include "../chip-src/gpadc/r328_gpadc.h"

//static uint32_t channel_select = SELSECT_CHANNEL0|SELSECT_CHANNEL1|SELSECT_CHANNEL2|SELSECT_CHANNEL3;
static uint32_t channel_select = SELSECT_CHANNEL0;

hal_gpadc_status_t hal_gpadc_init(void)
{
	int i;

	//clock_init();
	hal_clock_enable(HAL_CLK_PERIPH_GPADC);

	gpadc_sample_rate_set(GPADC_SAMPLE_RATE);
	gpadc_ch_select(channel_select);
	gpadc_cmp_select(channel_select);

	for (i = 0; i < MAX_CHANNEL; i++) {
		if (channel_select & (1 << i)) {
			gpadc_enable_lowirq_ch(i);
			gpadc_ch_cmp_low(i, COMPARE_LOWDATA);
			//gpadc_enable_highirq_ch(i);
			gpadc_ch_cmp_high(i, COMPARE_HIGDATA);
		}
	}

	gpadc_calibration_enabled();
	gpadc_mode_select();
	gpadc_enable();
	if (gpadc_init_irq() < 0)
		return HAL_GPADC_STATUS_ERROR;
	return HAL_GPADC_STATUS_OK;
}

hal_gpadc_status_t hal_gpadc_deinit(void)
{

	gpadc_disable();

	return HAL_GPADC_STATUS_OK;
}

hal_gpadc_status_t hal_gpadc_register_callback(gpadc_callback_t callback)
{
	gpadc_register_callback(callback);

	return HAL_GPADC_STATUS_OK;
}

