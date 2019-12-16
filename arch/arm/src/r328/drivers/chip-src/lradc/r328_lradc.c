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
#include "./r328_lradc.h"

lradc_func_data lradc_priv;

static uint32_t ctrl_para = FIRST_CONCERT_DLY | ADC_CHAN_SELECT | KEY_MODE_SELECT
				| LRADC_HOLD_EN & LEVELB_VOL & LRADC_SAMPLE_2KHZ | LRADC_EN;
static uint32_t irq_para = LRADC_ADC0_UP_EN | LRADC_ADC0_DOWN_EN | LRADC_ADC0_DATA_EN;

void lradc_ctrl_set(void)
{
	uint32_t reg_val = 0;

	reg_val = LRADC->LRADC_CTRL;
	reg_val |= ctrl_para;
	LRADC->LRADC_CTRL = reg_val;
}

void lradc_ctrl_reset(void)
{
	uint32_t reg_val = 0;

	reg_val = LRADC->LRADC_CTRL;
	reg_val &= ~ctrl_para;
	LRADC->LRADC_CTRL = reg_val;
}

void lradc_irq_set(void)
{
	uint32_t reg_val = 0;

	reg_val = LRADC->LRADC_INTC;
	reg_val |= irq_para;
	LRADC->LRADC_INTC = reg_val;
}

void lradc_irq_reset(void)
{
	uint32_t reg_val = 0;

	reg_val = LRADC->LRADC_INTC;
	reg_val &= ~irq_para;
	LRADC->LRADC_INTC = reg_val;
}

void lradc_register_callback(lradc_callback_t user_callback)
{
	sinfo("irq callback ==============\n");
	lradc_priv.func = user_callback;
}

static int lradc_irq_handler(int dummy, void *context, void *priv_data)
{
	lradc_func_data *lradcpriv = priv_data;
	lradc_callback_t callback = lradcpriv->func;
	void *arg = lradcpriv->arg;

	uint32_t irq_status = LRADC->LRADC_INTS;
	uint32_t reg_val = LRADC->LRADC_DATA0;
	sinfo("irqhander========================\n");
	if (NULL != callback)
		callback(irq_status, reg_val);

	LRADC->LRADC_INTS = irq_status;
	return 0;
}

int32_t lradc_init_irq(void)
{
	uint32_t irqn = R328_IRQ_LRADC;

/*	if (irq_attach(irqn, lradc_irq_handler, &lradc_priv) < 0) {
		return -1;
	}


	if (up_enable_irq(irqn) < 0) {
		return -1;
	}*/
	sinfo(" irqtest1======================\n");
	int ret = irq_attach(irqn, lradc_irq_handler, &lradc_priv);
	sinfo(" irqtest1======================\n");
	if (ret == OK)
	{
		sinfo("lradc attach irq.\n");
		up_enable_irq(irqn);
		sinfo("lradc enable irq.\n");
	}
	else
	{
		sinfo("lradc attach  failed.\n");
		return -1;
	}
	return 0;
}

