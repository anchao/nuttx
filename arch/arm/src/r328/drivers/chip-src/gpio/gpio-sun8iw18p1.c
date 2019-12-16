/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the People's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY’S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS’SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY’S TECHNOLOGY.
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
#include <stdio.h>
#include "r328_pio.h"

static const unsigned int sun8iw18p1_irq_bank_base[] = {
	SUNXI_PIO_BANK_BASE(PB_BASE, 0),		/* 1  + 0 = 0*/
	SUNXI_PIO_BANK_BASE(PE_BASE, 1),		/* 3  + 1 = 4*/
	SUNXI_PIO_BANK_BASE(PG_BASE, 2),		/* 4  + 2 = 6*/
	SUNXI_PIO_BANK_BASE(PH_BASE, 3),		/* 4  + 3 = 7*/
};

static const unsigned int sun8iw18p1_bank_base[] = {
	SUNXI_PIO_BANK_BASE(PB_BASE, 0),		/* 1 + 0 = 1*/
	SUNXI_PIO_BANK_BASE(PC_BASE, 1),		/* 1 + 1 = 2*/
	SUNXI_PIO_BANK_BASE(PE_BASE, 2),		/* 2 + 2 = 4*/
	SUNXI_PIO_BANK_BASE(PG_BASE, 3),		/* 3 + 3 = 6*/
	SUNXI_PIO_BANK_BASE(PH_BASE, 4),		/* 3 + 4 = 7*/
};

static const int sun8iw18p1_bank_irq_num[] = {
	SUNXI_IRQ_GPIOB,
	SUNXI_IRQ_GPIOE,
	SUNXI_IRQ_GPIOG,
	SUNXI_IRQ_GPIOH,
};

static struct gpio_desc sun8iw18p1_gpio_desc = {
	.membase = SUNXI_GPIO_PBASE,
	.npins = 106,							/* form spec*/
	.irq_arry_size = ARRAY_SIZE(sun8iw18p1_bank_irq_num),
	.irq = sun8iw18p1_bank_irq_num,
	.banks = ARRAY_SIZE(sun8iw18p1_bank_base),
	.bank_base = sun8iw18p1_bank_base,
	.irq_banks = ARRAY_SIZE(sun8iw18p1_irq_bank_base),
	.irq_bank_base = sun8iw18p1_irq_bank_base,
};

//#define IRQ_DESC_ARRAY_SIZE (ARRAY_SIZE(sun8iw18p1_irq_bank_base) * IRQ_PER_BANK)
//struct gpio_irq_desc irq_desc[IRQ_DESC_ARRAY_SIZE];

void sun8iw18p1_gpio_init(void)
{
	int ret = 0;
	ret = gpio_init(&sun8iw18p1_gpio_desc);

	if (ret) {
		up_lowputc('F');
		//GPIO_ERR("gpio init failed!\n");
		return;
	}
	up_lowputc('S');
	//GPIO_INFO("gpio init succeed!\n");
	return;
}
