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

#ifndef __R328_GPADC_H__
#define __R328_GPADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Structures
 *****************************************************************************/
typedef struct{
	uint32_t GP_SR_CON;                    /*sample rate config register*/
	uint32_t GP_CTRL;                      /*control register*/
	uint32_t GP_CS_EN;                     /*compare and select enable register*/
	uint32_t GP_FIFO_INTC;                 /*FIFO interrupt control register*/
	uint32_t GP_FIFO_INTS;                 /*FIFO interrupt status register*/
	uint32_t GP_FIFO_DATA;
	uint32_t GP_CDATA;

	uint32_t reserve0;                      /**/
	uint32_t GP_DATAL_INTC;
	uint32_t GP_DATAH_INTC;
	uint32_t GP_DATA_INTC;
	uint32_t reserve1;                      /**/
	uint32_t GP_DATAL_INTS;                /*data low interrupt status register*/
	uint32_t GP_DATAH_INTS;                /*data high interrupt status register*/
	uint32_t GP_DATA_INTS;                 /*data interrupt status register*/
	uint32_t reserve2;                      /**/
	uint32_t GP_CH0_CMP_DATA;              /*CH0 compare data register*/
	uint32_t GP_CH1_CMP_DATA;              /*CH1 compare data register*/
	uint32_t GP_CH2_CMP_DATA;              /*CH2 compare data register*/
	uint32_t GP_CH3_CMP_DATA;              /*CH3 compare data register*/
	uint32_t reserve3[12];
	/*0x80*/
	uint32_t GP_CH0_DATA;                  /*CH0 data register*/
	uint32_t GP_CH1_DATA;                  /*CH1 data register*/
	uint32_t GP_CH2_DATA;                  /*CH2 data register*/
	uint32_t GP_CH3_DATA;                  /*CH3 data register*/
	uint32_t reserve4[4];                      /**/
	uint32_t GP_VER;                       /*Version register*/
}GPADC_REGISTER_T;

/*****************************************************************************
 * Enums
 *****************************************************************************/
typedef enum{
	GPADC_IRQ_LOW,
	GPADC_IRQ_HIGH,
	GPADC_IRQ_DATA,
}GPADC_IRQ_STATUS_T;

#define GPADC_BASE    0x05070000

#define GPADC        ((GPADC_REGISTER_T *)GPADC_BASE)

#define OSC_24MHZ     24000000UL
#define VCC           1800000UL
#define MAX_CHANNEL   4

#define GPADC_SAMPLE_RATE 1000UL
#define COMPARE_LOWDATA   1700000UL
#define COMPARE_HIGDATA   1200000UL

#define SELSECT_CHANNEL0   0x01
#define SELSECT_CHANNEL1   0x02
#define SELSECT_CHANNEL2   0x04
#define SELSECT_CHANNEL3   0x08

/*sample_rate_register*/
#define GP_SR_CON_MASK         (0xffff << 16)

/*control_register*/
#define GP_CALIBRATION_EN       (0x01 << 17)
#define GP_ADC_EN               (1 << 16)
#define GP_CONTINOUS_MODE       2

typedef void (*gpadc_callback_t)(uint32_t channel, int irq_status, uint32_t vol_mV);

typedef struct {
	gpadc_callback_t func;
	void *arg;
}gpadc_func_data;

/*****************************************************************************
 * function
 *****************************************************************************/
void gpadc_sample_rate_set(uint32_t sample_rate);
void gpadc_calibration_enabled(void);
void gpadc_mode_select(void);
void gpadc_enable(void);
void gpadc_disable(void);
void gpadc_ch_select(uint32_t channel_sel);
void gpadc_cmp_select(uint32_t channel_sel);
void gpadc_enable_lowirq_ch(uint32_t channel);
void gpadc_disable_lowirq_ch(uint32_t channel);
void gpadc_enable_highirq_ch(uint32_t channel);
void gpadc_disable_highirq_ch(uint32_t channel);
void gpadc_ch_cmp_low(uint32_t channel, uint32_t low_data);
void gpadc_ch_cmp_high(uint32_t channel, uint32_t high_data);
uint32_t gpadc_ch_data_get(uint32_t channel);
uint32_t gpadc_irq_low_status(void);
uint32_t gpadc_irq_high_status(void);
uint32_t gpadc_irq_data_status(void);
void gpadc_register_callback(gpadc_callback_t user_callback);
int32_t gpadc_init_irq(void);
#ifdef __cplusplus
}
#endif

#endif /* __PWM__ */
