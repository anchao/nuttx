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

#ifndef __PWM_H__
#define __PWM_H__

#include <hal_pwm.h>

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************  
 * register
 *****************************************************************************/
typedef struct {
	uint32_t PWM_PIER;        /*IRQ enable register 0x00*/
	uint32_t PWM_PISR;        /*IRQ status register*/
	uint32_t reserve0[2];
	uint32_t PWM_CIER;       /*capture IRQ enable register 0x10*/
	uint32_t PWM_CISR;       /*capture IRQ status register*/
	uint32_t reserve1[2];
	uint32_t PWM_PCCR01;     /*PWM01 clock configuration register 0x20*/
	uint32_t PWM_PCCR23;     /*PWM23 clock configuretion register*/
	uint32_t PWM_PCCR45;     /*PWM45 clock configuretion register*/
	uint32_t PWM_PCCR67;     /*PWM67 clock configuretion register*/
	uint32_t reserve4[4];
	uint32_t PWM_PER;        /*enable register 0x40*/
	uint32_t PWM_CER;        /*capture enable register*/
	uint32_t reserve2[2];
	uint32_t PWM_VER;        /*version register 0x50*/
	uint32_t reserve3[3];
	uint32_t PWM_PCR;        /*control register(0~8) 0x60*/
	uint32_t PWM_PPR;        /*period register(0~8)*/
	uint32_t PWM_PCNTR;      /*count register(0~8)*/
	uint32_t PWM_CCR;        /*capture control register(0~8)*/
	uint32_t PWM_CRLR;       /*capture rise lock register(0~8)*/
	uint32_t PWM_CFLR;       /*capture fall lock register(0~8)*/
} PWM_REGISTER_T;

#define  PWM_PRE      0x20
#define  PWM_BASE     0x0300a000
#define  PWM          ((PWM_REGISTER_T *)PWM_BASE)


//20
#define PWM_CLK_SRC_SHIFT   0x7
#define PWM_CLK_SRC_WIDTH   0x2
#define PWM_CLK_GATING_SHIFT    0x4
#define PWM_CLK_GATING_WIDTH    0x1
#define PWM_DIV_M_SHIFT     0x0
#define PWM_DIV_M_WIDTH     0x4
//60
#define PWM_PRESCAL_SHIFT   0x0
#define PWM_PRESCAL_WIDTH   0x8
#define PWM_ACT_STA_SHIFT   0x8
#define PWM_ACT_STA_WIDTH   0x1
//64
#define PWM_ACT_CYCLES_SHIFT    0x0
#define PWM_ACT_CYCLES_WIDTH    0x10
#define PWM_PERIOD_CYCLES_SHIFT 0x10
#define PWM_PERIOD_CYCLES_WIDTH 0x10

#define PWM_DZ_EN_SHIFT     0x0
#define PWM_DZ_EN_WIDTH     0x1
#define PWM_PDZINTV_SHIFT   0x8
#define PWM_PDZINTV_WIDTH   0x8

#define PWM_BYPASS_WIDTH    0x1


/****************************************************************************
 * Functions
 *****************************************************************************/
void pwm_clk_src_set(uint32_t channel, hal_pwm_clk_src clk_src);
void pwm_clk_div_m(uint32_t channel, uint32_t div_m);
void pwm_prescal_set(uint32_t channel, uint32_t prescal);
void pwm_set_active_cycles(uint32_t channel, uint32_t active_cycles);
void pwm_set_period_cycles(uint32_t channel, uint32_t period_cycles);
void pwm_enable_clk_gating(uint32_t channel);
void pwm_enable_controller(uint32_t channel);
void pwm_disable_controller(uint32_t channel);
void pwm_porality(uint32_t channel, hal_pwm_polarity polarity);

#ifdef __cplusplus
}
#endif

#endif /* __PWM_H__ */
