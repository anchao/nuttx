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

#include <stdint.h>
#include <stdio.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <debug.h>
#include <interrupt.h>
#include <r328_irq.h>

#include "../../../include/drivers/hal_gpadc.h"
#include "./r328_gpadc.h"

#ifdef  SUNXIKBD_DEBUG
#define sunxikbd_info(fmt, args...) sinfo(fmt, ##args)
#else
#define sunxikbd_info(fmt, args...)
#endif
#define sunxikbd_err(fmt, args...) sinfo(":%d "fmt, __LINE__, ##args)

gpadc_func_data gpadc_priv;

void gpadc_sample_rate_set(uint32_t sample_rate)
{
//	uint32_t div, reg_val;
	uint32_t div;

	div = OSC_24MHZ / sample_rate - 1;
	GPADC->GP_SR_CON &= ~GP_SR_CON_MASK;
	GPADC->GP_SR_CON |= div << 16;
}

void gpadc_calibration_enabled(void)
{
	GPADC->GP_CTRL |= GP_CALIBRATION_EN;
}

void gpadc_mode_select(void)
{
	GPADC->GP_CTRL &= ~(3 << 18);
	GPADC->GP_CTRL |= GP_CONTINOUS_MODE << 18;
}

void gpadc_enable(void)
{
	GPADC->GP_CTRL |= GP_ADC_EN;
}

void gpadc_disable(void)
{
	GPADC->GP_CTRL &= ~GP_ADC_EN;
}

void gpadc_ch_select(uint32_t channel_sel)
{
	GPADC->GP_CS_EN |= channel_sel;
}

void gpadc_cmp_select(uint32_t channel_sel)
{
	GPADC->GP_CS_EN |= channel_sel << 16;
}

void gpadc_enable_lowirq_ch(uint32_t channel)
{
	if (channel < MAX_CHANNEL) {
		GPADC->GP_DATAL_INTC |= 1 << channel;
	}
}

void gpadc_disable_lowirq_ch(uint32_t channel)
{
	if (channel < MAX_CHANNEL) {
		GPADC->GP_DATAL_INTC &= ~(1 << channel);
	}
}

void gpadc_enable_highirq_ch(uint32_t channel)
{
	if (channel < MAX_CHANNEL) {
		GPADC->GP_DATAH_INTC |= 1 << channel;
	}
}

void gpadc_disable_highirq_ch(uint32_t channel)
{
	if (channel < MAX_CHANNEL) {
		GPADC->GP_DATAH_INTC &= ~(1 << channel);
	}
}

void gpadc_ch_cmp_low(uint32_t channel, uint32_t low_data)
{
	uint32_t per_val, low_val, reg_val;

	per_val = VCC / 4096; /*12bits sample rate*/
	low_val = low_data / per_val;

	if (low_val > 0xfff) {
		low_val = 0xfff;
	}
	if (channel < MAX_CHANNEL) {
		reg_val = *(uint32_t *)((char *)&(GPADC->GP_CH0_CMP_DATA) + 4 * channel);
		reg_val &= ~0xfff;
		reg_val |= low_val;
		*(uint32_t *)((char *)&(GPADC->GP_CH0_CMP_DATA) + 4 * channel) = reg_val;
	}
}

void gpadc_ch_cmp_high(uint32_t channel, uint32_t high_data)
{
	uint32_t per_val, high_val, reg_val;

	per_val = VCC / 4096; /*12bits sample rate*/
	high_val = high_data / per_val;

	if (high_val > 0xfff) {
		high_val = 0xfff;
	}
	if (channel < MAX_CHANNEL) {
		reg_val = *(uint32_t *)((char *)&(GPADC->GP_CH0_CMP_DATA) + 4 * channel);
		//printf("address : %0x\n", &(GPADC->GP_CH0_CMP_DATA));
		//printf("hign_val : %d\n", high_val);
		reg_val &= ~(0xfff << 16);
		reg_val |= (high_val << 16);
		*(uint32_t *)((char *)&(GPADC->GP_CH0_CMP_DATA) + 4 * channel) = reg_val;
		//printf("hign_val : %0x\n", *(uint32_t *)((char *)&(GPADC->GP_CH0_CMP_DATA) + 4 * channel));
	}
}

uint32_t gpadc_ch_data_get(uint32_t channel)
{
	return *(uint32_t *)(&(GPADC->GP_CH0_DATA) + 4 * channel);
}

uint32_t gpadc_irq_low_status(void)
{
	return GPADC->GP_DATAL_INTS;
}

uint32_t gpadc_irq_high_status(void)
{
	return GPADC->GP_DATAH_INTS;
}

uint32_t gpadc_irq_data_status(void)
{
	return GPADC->GP_DATA_INTS;
}

void gpadc_register_callback(gpadc_callback_t user_callback)
{
	gpadc_priv.func = user_callback;
}

static int gpadc_irq_handler(int dummy, void *context, void *priv_data)
{
	sunxikbd_info("gpadc_irq_handler gpadc_irq_handler\n");
	gpadc_func_data *gpadc_priv = priv_data;
	gpadc_callback_t callback = gpadc_priv->func;

	uint32_t reg_irq_low, reg_irq_high;
	uint32_t reg_enable_high;
	uint32_t reg_val;
	uint32_t data, i;

	sunxikbd_info("========high intreg : %0x=========\n", GPADC->GP_DATAH_INTS);
	reg_irq_low = gpadc_irq_low_status();
	reg_irq_high = gpadc_irq_high_status();
	GPADC->GP_DATAH_INTS = reg_irq_high;
	GPADC->GP_DATAL_INTS = reg_irq_low;
	reg_enable_high = GPADC->GP_DATAH_INTC;
	for (i = 0; i < 4; i++) {
		if (reg_irq_low & (1 << i)) {
			GPADC->GP_DATAL_INTS = reg_irq_low;
			reg_val = gpadc_ch_data_get(i);
			sunxikbd_info("========low  : reg_val = %0x=========\n", reg_val);
			sunxikbd_info("========reg_irq_high : %0x=========\n", reg_irq_high);
			sunxikbd_info("========high intreg : %0x=========\n", GPADC->GP_DATAH_INTS);
			data = (VCC/ 4096)*reg_val;
			sunxikbd_info("%d	%d	%d===\n",i, GPADC_IRQ_LOW, data);
			callback(i, GPADC_IRQ_LOW, data);
			GPADC->GP_DATAH_INTS = reg_irq_high;
			gpadc_enable_highirq_ch(i);
			reg_irq_low = 0;
		}

		if (reg_irq_high & (1 << i) & reg_enable_high) {
			GPADC->GP_DATAH_INTS = reg_irq_high;
			gpadc_disable_highirq_ch(i);
			sunxikbd_info("========high int=========\n");
			sunxikbd_info("========high intreg : %0x=========\n", &GPADC->GP_DATAH_INTS);
			sunxikbd_info("========reg_irq_high : %0x=========\n", reg_irq_high);

			reg_val = gpadc_ch_data_get(i);
			sunxikbd_info("========reg_val = %0x=========\n", reg_val);
			data = (VCC/ 4096)*reg_val;
			callback(i, GPADC_IRQ_HIGH, data);
		}
	}

	return 0;
}
void gpadc_irq_status(unsigned int *channel, uint32_t *irq_status, uint32_t *reg_val)
{
	int i = 0;
	uint32_t data;
	uint32_t reg_enable_high;
	uint32_t reg_irq_low, reg_irq_high;
	reg_irq_low = gpadc_irq_low_status();
	reg_irq_high = gpadc_irq_high_status();
	reg_enable_high = GPADC->GP_DATAH_INTC;

	for (i = 0; i < 4; i++) {
		if (reg_irq_low & (1 << i)) {
			GPADC->GP_DATAL_INTS = reg_irq_low;
			data = gpadc_ch_data_get(i);
			*channel = i;
			*irq_status = GPADC_IRQ_LOW;
			*reg_val = (VCC / 4096)*data;
			GPADC->GP_DATAH_INTS = reg_irq_high;
			gpadc_enable_highirq_ch(i);
			reg_irq_low = 0;
		}
		if (reg_irq_high & (1 << i) & reg_enable_high) {
			GPADC->GP_DATAH_INTS = reg_irq_high;
			gpadc_disable_highirq_ch(i);
			data = gpadc_ch_data_get(i);
			*channel = i;
			*irq_status = GPADC_IRQ_HIGH;
			*reg_val = (VCC / 4096)*data;
		}
	}
}

int32_t gpadc_init_irq(void)
{
	uint32_t irqn = R328_IRQ_GPADC;

	int ret = irq_attach(irqn, gpadc_irq_handler, &gpadc_priv);
	if (ret == OK) {
		sunxikbd_info("gpadc irq_attach ok!=====\n");
		up_enable_irq(irqn);
		sunxikbd_info("gpadc irq enable ok!=====\n");
	}
	else {
		sunxikbd_err("gpadc irq_attach failed=====\n");
		return -1;
	}

	return 0;
}

