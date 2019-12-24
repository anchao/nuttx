/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the people's Republic of China and other countries.
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
#include <stdlib.h>
#include <string.h>
#include <snd_core.h>
#include <snd_pcm.h>

#include "codec_common.h"
#include "ac107.h"

#define AC107_DEBUG_EN 0
#if AC107_DEBUG_EN
#define AC107_DEBUG(...)	printf(__VA_ARGS__)
#else
#define AC107_DEBUG(...)
#endif

#define AC107_ADC_PATTERN_SEL   ADC_PTN_NORMAL  /* 0:ADC normal,  1:0x5A5A5A,  2:0x123456,  3:0x000000,  4~7:I2S_RX_DATA,  other:reserved */

/* AC107 config */
#define AC107_CHIP_NUMS			1	/* range[1, 8] */
#define AC107_CHIP_NUMS_MAX		8	/* range[1, 8] */
#define AC107_SLOT_WIDTH		32	/* 8/12/16/20/24/28/32bit Slot Width */
#define AC107_ENCODING_EN		0	/* TX Encoding mode enable */
#define AC107_ENCODING_CH_NUMS	2	/* TX Encoding channel numbers, must be dual, range[1, 16] */
#define AC107_ENCODING_FMT		0	/* TX Encoding format:	0:first channel number 0,  other:first channel number 1 */
/*range[1, 1024], default PCM mode, I2S/LJ/RJ mode shall divide by 2 */
//#define AC107_LRCK_PERIOD		(AC107_SLOT_WIDTH*(AC107_ENCODING_EN ? 2 : AC107_CHIP_NUMS*2))
#define AC107_LRCK_PERIOD		(AC107_SLOT_WIDTH*(AC107_ENCODING_EN ? 2 : AC107_CHIP_NUMS))
#define AC107_MATCH_DTS_EN		1	/* AC107 match method select: 0: i2c_detect, 1:devices tree */

#define AC107_KCONTROL_EN		1
#define AC107_DAPM_EN			0
#define AC107_CODEC_RW_USER_EN	1
#define AC107_PGA_GAIN			ADC_PGA_GAIN_28dB	//-6dB and 0dB, 3~30dB, 1dB step
#define AC107_DMIC_EN			0	//0:ADC  1:DMIC
#define AC107_PDM_EN			0	//0:I2S  1:PDM

#define AC107_DVCC_NAME			"ac107_dvcc_1v8"
#define AC107_AVCC_VCCIO_NAME	"ac107_avcc_vccio_3v3"
#define AC107_RATES			(SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT)
#define AC107_FORMATS			(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

#define AC107_CHIP_CFG \
{ \
	[0] = { \
		.bus = HAL_I2C_MASTER_1, .addr = 0x36, \
	}, \
	[1] = { \
		.bus = HAL_I2C_MASTER_1, .addr = 0x37, \
	}, \
	[2] = { \
		.bus = HAL_I2C_MASTER_1, .addr = 0x38, \
	}, \
	[3] = { \
		.bus = HAL_I2C_MASTER_1, .addr = 0x39, \
	}, \
}

static codec_i2c_config_t twi_config = {
	.port = HAL_I2C_MASTER_1,
	.freq = HAL_I2C_FREQUENCY_200K,
	.clk = {GPIOH(2), 2, 7},
	.sda = {GPIOH(3), 2, 7},
};

struct ac107_param {
	unsigned int chip_num;
	struct codec_i2c_device twi_dev[AC107_CHIP_NUMS_MAX];
	unsigned char pga_gain[AC107_CHIP_NUMS_MAX];
};

struct ac107_priv {
	struct snd_codec *codec;
	struct ac107_param param;
	struct codec_i2c_device *i2c;
};

struct real_val_to_reg_val {
	unsigned int real_val;
	unsigned int reg_val;
};

struct pll_div {
	u32 freq_in;
	u32 freq_out;
	u32 m1;
	u32 m2;
	u32 n;
	u32 k1;
	u32 k2;
};

static const struct real_val_to_reg_val ac107_sample_rate[] = {
	{8000, 0},
	{11025, 1},
	{12000, 2},
	{16000, 3},
	{22050, 4},
	{24000, 5},
	{32000, 6},
	{44100, 7},
	{48000, 8},
};

static const struct real_val_to_reg_val ac107_bclk_div[] = {
	{0, 0},
	{1, 1},
	{2, 2},
	{4, 3},
	{6, 4},
	{8, 5},
	{12, 6},
	{16, 7},
	{24, 8},
	{32, 9},
	{48, 10},
	{64, 11},
	{96, 12},
	{128, 13},
	{176, 14},
	{192, 15},
};

//FOUT =(FIN * N) / [(M1+1) * (M2+1)*(K1+1)*(K2+1)] ;	M1[0,31],  M2[0,1],  N[0,1023],  K1[0,31],  K2[0,1]
static const struct pll_div ac107_pll_div[] = {
	{400000, 12288000, 0, 0, 983, 15, 1},	//<out: 12.2875M>
	{512000, 12288000, 0, 0, 960, 19, 1},	//24576000/48
	{768000, 12288000, 0, 0, 640, 19, 1},	//24576000/32
	{800000, 12288000, 0, 0, 768, 24, 1},
	{1024000, 12288000, 0, 0, 480, 19, 1},	//24576000/24
	{1600000, 12288000, 0, 0, 384, 24, 1},
	{2048000, 12288000, 0, 0, 240, 19, 1},	//24576000/12
	{3072000, 12288000, 0, 0, 160, 19, 1},	//24576000/8
	{4096000, 12288000, 0, 0, 120, 19, 1},	//24576000/6
	{6000000, 12288000, 4, 0, 512, 24, 1},
	{6144000, 12288000, 1, 0, 160, 19, 1},	//24576000/4
	{12000000, 12288000, 9, 0, 512, 24, 1},
	{13000000, 12288000, 12, 0, 639, 25, 1},	//<out: 12.2885M>
	{15360000, 12288000, 9, 0, 320, 19, 1},
	{16000000, 12288000, 9, 0, 384, 24, 1},
	{19200000, 12288000, 11, 0, 384, 24, 1},
	{19680000, 12288000, 15, 1, 999, 24, 1},	//<out: 12.2877M>
	{24000000, 12288000, 9, 0, 256, 24, 1},

	{400000, 11289600, 0, 0, 1016, 17, 1},	//<out: 11.2889M>
	{512000, 11289600, 0, 0, 882, 19, 1},
	{768000, 11289600, 0, 0, 588, 19, 1},
	{800000, 11289600, 0, 0, 508, 17, 1},	//<out: 11.2889M>
	{1024000, 11289600, 0, 0, 441, 19, 1},
	{1600000, 11289600, 0, 0, 254, 17, 1},	//<out: 11.2889M>
	{2048000, 11289600, 1, 0, 441, 19, 1},
	{3072000, 11289600, 0, 0, 147, 19, 1},
	{4096000, 11289600, 3, 0, 441, 19, 1},
	{6000000, 11289600, 1, 0, 143, 18, 1},	//<out: 11.2895M>
	{6144000, 11289600, 1, 0, 147, 19, 1},
	{12000000, 11289600, 3, 0, 143, 18, 1},	//<out: 11.2895M>
	{13000000, 11289600, 12, 0, 429, 18, 1},	//<out: 11.2895M>
	{15360000, 11289600, 14, 0, 441, 19, 1},
	{16000000, 11289600, 24, 0, 882, 24, 1},
	{19200000, 11289600, 4, 0, 147, 24, 1},
	{19680000, 11289600, 13, 1, 771, 23, 1},	//<out: 11.28964M>
	{24000000, 11289600, 24, 0, 588, 24, 1},

	{12288000, 12288000, 9, 0, 400, 19, 1},	//24576000/2
	{11289600, 11289600, 9, 0, 400, 19, 1},	//22579200/2

	{24576000 / 1, 12288000, 9, 0, 200, 19, 1},	//24576000
	{24576000 / 16, 12288000, 0, 0, 320, 19, 1},	//1536000
	{24576000 / 64, 12288000, 0, 0, 640, 9, 1},	//384000
	{24576000 / 96, 12288000, 0, 0, 960, 9, 1},	//256000
	{24576000 / 128, 12288000, 0, 0, 512, 3, 1},	//192000
	{24576000 / 176, 12288000, 0, 0, 880, 4, 1},	//140000
	{24576000 / 192, 12288000, 0, 0, 960, 4, 1},	//128000

	{22579200 / 1, 11289600, 9, 0, 200, 19, 1},	//22579200
	{22579200 / 4, 11289600, 4, 0, 400, 19, 1},	//5644800
	{22579200 / 16, 11289600, 0, 0, 320, 19, 1},	//1411200
	{22579200 / 64, 11289600, 0, 0, 640, 9, 1},	//352800
	{22579200 / 96, 11289600, 0, 0, 960, 9, 1},	//235200
	{22579200 / 128, 11289600, 0, 0, 512, 3, 1},	//176400
	{22579200 / 176, 11289600, 0, 0, 880, 4, 1},	//128290
	{22579200 / 192, 11289600, 0, 0, 960, 4, 1},	//117600

	{22579200 / 6, 11289600, 2, 0, 360, 19, 1},	//3763200
	{22579200 / 8, 11289600, 0, 0, 160, 19, 1},	//2822400
	{22579200 / 12, 11289600, 0, 0, 240, 19, 1},	//1881600
	{22579200 / 24, 11289600, 0, 0, 480, 19, 1},	//940800
	{22579200 / 32, 11289600, 0, 0, 640, 19, 1},	//705600
	{22579200 / 48, 11289600, 0, 0, 960, 19, 1},	//470400
};

#define ac107_read(reg, rt_value, i2c_dev) \
	codec_i2c_read(i2c_dev, reg, rt_value)
#define ac107_write(reg, value, i2c_dev) \
	codec_i2c_write(i2c_dev, reg, value)
#define ac107_update_bits(reg, mask, value, i2c_dev) \
	codec_i2c_update_bits(i2c_dev, reg, mask, value)

static int ac107_multi_chips_write(struct ac107_priv *ac107, u8 reg, unsigned char value)
{
	u8 i;

	for (i = 0; i < AC107_CHIP_NUMS; i++) {
		ac107_write(reg, value, &(ac107->param.twi_dev[i]));
	}

	return 0;
}

static int ac107_multi_chips_update_bits(struct ac107_priv *ac107, u8 reg, u8 mask, u8 value)
{
	u8 i;

	for (i = 0; i < AC107_CHIP_NUMS; i++) {
		ac107_update_bits(reg, mask, value, &(ac107->param.twi_dev[i]));
	}

	return 0;
}

static void ac107_hw_init(struct codec_i2c_device *i2c)
{
	u8 reg_val;

	/*** Analog voltage enable ***/
	ac107_write(PWR_CTRL1, 0x80, i2c);	/*0x01=0x80: VREF Enable */
	ac107_write(PWR_CTRL2, 0x55, i2c);	/*0x02=0x55: MICBIAS1&2 Enable */

	/*** SYSCLK Config ***/
	ac107_update_bits(SYSCLK_CTRL, 0x1 << SYSCLK_EN, 0x1 << SYSCLK_EN, i2c);	/*SYSCLK Enable */
	ac107_write(MOD_CLK_EN, 0x07, i2c);	/*0x21=0x07: Module clock enable<I2S, ADC digital,  ADC analog> */
	ac107_write(MOD_RST_CTRL, 0x03, i2c);	/*0x22=0x03: Module reset de-asserted<I2S, ADC digital> */

	/*** I2S Common Config ***/
	ac107_update_bits(I2S_CTRL, 0x1 << SDO_EN, 0x1 << SDO_EN, i2c);	/*SDO enable */
	ac107_update_bits(I2S_BCLK_CTRL, 0x1 << EDGE_TRANSFER, 0x0 << EDGE_TRANSFER, i2c);	/*SDO drive data and SDI sample data at the different BCLK edge */
	ac107_update_bits(I2S_LRCK_CTRL1, 0x3 << LRCK_PERIODH,
			  ((AC107_LRCK_PERIOD - 1) >> 8) << LRCK_PERIODH, i2c);
	ac107_write(I2S_LRCK_CTRL2, (u8) (AC107_LRCK_PERIOD - 1), i2c);	/*config LRCK period */
	/*Encoding mode format select 0~N-1, Encoding mode enable, Turn to hi-z state (TDM) when not transferring slot */
	ac107_update_bits(I2S_FMT_CTRL1,
			0x1 << ENCD_FMT | 0x1 << ENCD_SEL | 0x1 << TX_SLOT_HIZ
			| 0x1 << TX_STATE,
			!!AC107_ENCODING_FMT << ENCD_FMT |
			!!AC107_ENCODING_EN << ENCD_SEL | 0x0 << TX_SLOT_HIZ
			| 0x1 << TX_STATE, i2c);
	ac107_update_bits(I2S_FMT_CTRL2, 0x7 << SLOT_WIDTH_SEL, (AC107_SLOT_WIDTH / 4 - 1) << SLOT_WIDTH_SEL, i2c);	/*8/12/16/20/24/28/32bit Slot Width */
	/*0x36=0x60: TX MSB first, SDOUT normal, PCM frame type, Linear PCM Data Mode */
	ac107_update_bits(I2S_FMT_CTRL3,
			0x1 << TX_MLS | 0x1 << SDOUT_MUTE | 0x1 << LRCK_WIDTH
			| 0x3 << TX_PDM,
			0x0 << TX_MLS | 0x0 << SDOUT_MUTE | 0x0 << LRCK_WIDTH
			| 0x0 << TX_PDM, i2c);

	ac107_update_bits(I2S_TX_CHMP_CTRL1, !AC107_DAPM_EN * 0xff, 0xaa, i2c);	/*0x3c=0xaa: TX CH1/3/5/7 map to adc1, TX CH2/4/6/8 map to adc2 */
	ac107_update_bits(I2S_TX_CHMP_CTRL2, !AC107_DAPM_EN * 0xff, 0xaa, i2c);	/*0x3d=0xaa: TX CH9/11/13/15 map to adc1, TX CH10/12/14/16 map to adc2 */

	/*PDM Interface Latch ADC1 data on rising clock edge. Latch ADC2 data on falling clock edge, PDM Enable */
	ac107_update_bits(PDM_CTRL, 0x1 << PDM_TIMING | 0x1 << PDM_EN,
			  0x0 << PDM_TIMING | !!AC107_PDM_EN << PDM_EN, i2c);

	/*** ADC DIG part Config***/
	ac107_update_bits(ADC_DIG_EN, !AC107_DAPM_EN * 0x7, 0x7, i2c);	/*0x61=0x07: Digital part globe enable, ADCs digital part enable */
	ac107_update_bits(DMIC_EN, !AC107_DAPM_EN * 0x1, !!AC107_DMIC_EN, i2c);	/*DMIC Enable */

#if AC107_KCONTROL_EN
	ac107_read(ADC_DIG_DEBUG, &reg_val, i2c);
	ac107_write(HPF_EN, !(reg_val & 0x7) * 0x03, i2c);
#else
	/* ADC pattern select */
	ac107_write(HPF_EN, !AC107_ADC_PATTERN_SEL * 0x03, i2c);
	ac107_update_bits(ADC_DIG_DEBUG, 0x7 << ADC_PTN_SEL,
			  (AC107_ADC_PATTERN_SEL & 0x7) << ADC_PTN_SEL, i2c);
#endif

	//ADC Digital Volume Config
	ac107_update_bits(ADC1_DVOL_CTRL, !AC107_KCONTROL_EN * 0xff, 0xA0, i2c);
	ac107_update_bits(ADC2_DVOL_CTRL, !AC107_KCONTROL_EN * 0xff, 0xA0, i2c);

	/*** ADCs analog PGA gain Config***/
	ac107_update_bits(ANA_ADC1_CTRL3,
			  !AC107_KCONTROL_EN * 0x1f << RX1_PGA_GAIN_CTRL,
			  AC107_PGA_GAIN << RX1_PGA_GAIN_CTRL, i2c);
	ac107_update_bits(ANA_ADC2_CTRL3,
			  !AC107_KCONTROL_EN * 0x1f << RX2_PGA_GAIN_CTRL,
			  AC107_PGA_GAIN << RX2_PGA_GAIN_CTRL, i2c);

	/*** ADCs analog global Enable***/
	ac107_update_bits(ANA_ADC1_CTRL5, !AC107_DAPM_EN * 0x1 << RX1_GLOBAL_EN,
			  0x1 << RX1_GLOBAL_EN, i2c);
	ac107_update_bits(ANA_ADC2_CTRL5, !AC107_DAPM_EN * 0x1 << RX2_GLOBAL_EN,
			  0x1 << RX2_GLOBAL_EN, i2c);

	//VREF Fast Start-up Disable
	ac107_update_bits(PWR_CTRL1, 0x1 << VREF_FSU_DISABLE,
			  0x1 << VREF_FSU_DISABLE, i2c);
}

static int ac107_set_sysclk(struct snd_dai *dai, int clk_id,
			    unsigned int freq, int dir)
{
	struct snd_codec *codec = dai->component;
	struct ac107_priv *ac107 = codec->private_data;
	AC107_DEBUG("\n--->%s\n", __FUNCTION__);
	switch (clk_id) {
	case SYSCLK_SRC_MCLK:
		AC107_DEBUG("AC107 SYSCLK source select MCLK\n\n");
		ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL, 0x3 << SYSCLK_SRC, SYSCLK_SRC_MCLK << SYSCLK_SRC);	//System Clock Source Select MCLK
		break;
	case SYSCLK_SRC_BCLK:
		AC107_DEBUG("AC107 SYSCLK source select BCLK\n\n");
		ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL, 0x3 << SYSCLK_SRC, SYSCLK_SRC_BCLK << SYSCLK_SRC);	//System Clock Source Select BCLK
		break;
	case SYSCLK_SRC_PLL:
		AC107_DEBUG("AC107 SYSCLK source select PLL\n\n");
		ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL, 0x3 << SYSCLK_SRC, SYSCLK_SRC_PLL << SYSCLK_SRC);	//System Clock Source Select PLL
		break;
	default:
		snd_err("AC107 SYSCLK source config error:%d\n\n", clk_id);
		return -EINVAL;
	}

	//SYSCLK Enable
	ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL, 0x1 << SYSCLK_EN,
				      0x1 << SYSCLK_EN);
	return 0;
}

static int ac107_set_pll(struct snd_dai *dai, int pll_id, int source,
			 unsigned int freq_in, unsigned int freq_out)
{
	struct snd_codec *codec = dai->component;
	struct ac107_priv *ac107 = codec->private_data;
	u32 i, m1, m2, n, k1, k2;
	AC107_DEBUG("\n--->%s\n", __FUNCTION__);

	freq_in = freq_in / 2;
	if (!freq_out)
		return 0;

	if (freq_in < 128000 || freq_in > 24576000) {
		snd_err
		    ("AC107 PLLCLK source input freq only support [128K,24M],while now %u\n\n",
		     freq_in);
		return -EINVAL;
	} else if ((freq_in == 12288000 || freq_in == 11289600)
		   && (pll_id == PLLCLK_SRC_MCLK || pll_id == PLLCLK_SRC_BCLK)) {
		//System Clock Source Select MCLK/BCLK, SYSCLK Enable
		AC107_DEBUG
		    ("AC107 don't need to use PLL, SYSCLK source select %s\n\n",
		     pll_id ? "BCLK" : "MCLK");
		ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL,
					      0x3 << SYSCLK_SRC | 0x1 <<
					      SYSCLK_EN,
					      pll_id << SYSCLK_SRC | 0x1 <<
					      SYSCLK_EN);
		return 0;	//Don't need to use PLL
	}
	//PLL Clock Source Select
	switch (pll_id) {
	case PLLCLK_SRC_MCLK:
		AC107_DEBUG("AC107 PLLCLK input source select MCLK\n");
		ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL, 0x3 << PLLCLK_SRC,
					      PLLCLK_SRC_MCLK << PLLCLK_SRC);
		break;
	case PLLCLK_SRC_BCLK:
		AC107_DEBUG("AC107 PLLCLK input source select BCLK\n");
		ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL, 0x3 << PLLCLK_SRC,
					      PLLCLK_SRC_BCLK << PLLCLK_SRC);
		break;
	case PLLCLK_SRC_PDMCLK:
		AC107_DEBUG("AC107 PLLCLK input source select PDMCLK\n");
		ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL, 0x3 << PLLCLK_SRC,
					      PLLCLK_SRC_PDMCLK << PLLCLK_SRC);
		break;
	default:
		snd_err("AC107 PLLCLK source config error:%d\n\n", pll_id);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(ac107_pll_div); i++) {
		if (ac107_pll_div[i].freq_in == freq_in
		    && ac107_pll_div[i].freq_out == freq_out) {
			m1 = ac107_pll_div[i].m1;
			m2 = ac107_pll_div[i].m2;
			n = ac107_pll_div[i].n;
			k1 = ac107_pll_div[i].k1;
			k2 = ac107_pll_div[i].k2;
			AC107_DEBUG
			    ("AC107 PLL freq_in match:%u, freq_out:%u\n\n",
			     freq_in, freq_out);
			break;
		}
	}

	if (i == ARRAY_SIZE(ac107_pll_div)) {
		snd_err
		    ("AC107 don't match PLLCLK freq_in and freq_out table\n\n");
		return -EINVAL;
	}
	//Config PLL DIV param M1/M2/N/K1/K2
	ac107_multi_chips_update_bits(ac107, PLL_CTRL2,
				      0x1f << PLL_PREDIV1 | 0x1 << PLL_PREDIV2,
				      m1 << PLL_PREDIV1 | m2 << PLL_PREDIV2);
	ac107_multi_chips_update_bits(ac107, PLL_CTRL3, 0x3 << PLL_LOOPDIV_MSB,
				      (n >> 8) << PLL_LOOPDIV_MSB);
	ac107_multi_chips_update_bits(ac107, PLL_CTRL4, 0xff << PLL_LOOPDIV_LSB,
				      (u8) n << PLL_LOOPDIV_LSB);
	ac107_multi_chips_update_bits(ac107, PLL_CTRL5,
				      0x1f << PLL_POSTDIV1 | 0x1 <<
				      PLL_POSTDIV2,
				      k1 << PLL_POSTDIV1 | k2 << PLL_POSTDIV2);

	//Config PLL module current
	//ac107_multi_chips_update_bits(PLL_CTRL1, 0x7<<PLL_IBIAS, 0x4<<PLL_IBIAS);
	//ac107_multi_chips_update_bits(PLL_CTRL6, 0x1f<<PLL_CP, 0xf<<PLL_CP);

	//PLL module enable
	ac107_multi_chips_update_bits(ac107, PLL_LOCK_CTRL, 0x1 << PLL_LOCK_EN, 0x1 << PLL_LOCK_EN);	//PLL CLK lock enable
	//ac107_multi_chips_update_bits(PLL_CTRL1, 0x1<<PLL_EN | 0x1<<PLL_COM_EN, 0x1<<PLL_EN | 0x1<<PLL_COM_EN);	//PLL Common voltage Enable, PLL Enable

	//PLLCLK Enable, SYSCLK Enable
	ac107_multi_chips_update_bits(ac107, SYSCLK_CTRL,
				      0x1 << PLLCLK_EN | 0x1 << SYSCLK_EN,
				      0x1 << PLLCLK_EN | 0x1 << SYSCLK_EN);

	return 0;
}

static int ac107_set_clkdiv(struct snd_dai *dai, int div_id, int div)
{
	struct snd_codec *codec = dai->component;
	struct ac107_priv *ac107 = codec->private_data;
	u32 i, bclk_div, bclk_div_reg_val;
	AC107_DEBUG("\n--->%s\n", __FUNCTION__);

	if (!div_id) {		//use div_id to judge Master/Slave mode,  0: Slave mode, 1: Master mode
		AC107_DEBUG
		    ("AC107 work as Slave mode, don't need to config BCLK_DIV\n\n");
		return 0;
	}

	//bclk_div = div / (AC107_LRCK_PERIOD);	//default PCM mode
	bclk_div = div/(2*AC107_LRCK_PERIOD); //I2S/LJ/RJ mode

	for (i = 0; i < ARRAY_SIZE(ac107_bclk_div); i++) {
		if (ac107_bclk_div[i].real_val == bclk_div) {
			bclk_div_reg_val = ac107_bclk_div[i].reg_val;
			AC107_DEBUG("AC107 set BCLK_DIV_[%u]\n\n", bclk_div);
			break;
		}
	}

	if (i == ARRAY_SIZE(ac107_bclk_div)) {
		snd_err("AC107 don't support BCLK_DIV_[%u]\n\n", bclk_div);
		return -EINVAL;
	}
	//AC107 set BCLK DIV
	ac107_multi_chips_update_bits(ac107, I2S_BCLK_CTRL, 0xf << BCLKDIV,
				      bclk_div_reg_val << BCLKDIV);
	return 0;
}

static int ac107_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_dai *dai)
{
	struct snd_codec *codec = dai->component;
	struct ac107_priv *ac107 = codec->private_data;
	struct ac107_param *param = &ac107->param;
	u16 i, channels, channels_en, sample_resolution;
	AC107_DEBUG("\n--->%s\n", __FUNCTION__);

	//AC107 hw init
	for (i = 0; i < AC107_CHIP_NUMS; i++) {
		ac107_hw_init(&(ac107->param.twi_dev[i]));
	}

	//AC107 set sample rate
	for (i = 0; i < ARRAY_SIZE(ac107_sample_rate); i++) {
		if (ac107_sample_rate[i].real_val ==
		    params_rate(params) /
		    (AC107_ENCODING_EN ? AC107_ENCODING_CH_NUMS / 2 : 1)) {
			ac107_multi_chips_update_bits(ac107, ADC_SPRC,
						      0xf << ADC_FS_I2S,
						      ac107_sample_rate
						      [i].reg_val <<
						      ADC_FS_I2S);
			break;
		}
	}

	//AC107 set channels
	channels =
	    params_channels(params) *
	    (AC107_ENCODING_EN ? AC107_ENCODING_CH_NUMS / 2 : 1);
	for (i = 0; i < (channels + 1) / 2; i++) {
		channels_en =
		    (channels >=
		     2 * (i + 1)) ? 0x0003 << (2 * i) : ((1 << (channels % 2)) -
							 1) << (2 * i);
		ac107_write(I2S_TX_CTRL1, channels - 1, &param->twi_dev[i]);
		ac107_write(I2S_TX_CTRL2, (u8) channels_en, &param->twi_dev[i]);
		ac107_write(I2S_TX_CTRL3, channels_en >> 8, &param->twi_dev[i]);
	}
	for (; i < AC107_CHIP_NUMS; i++) {
		ac107_write(I2S_TX_CTRL1, 0, &param->twi_dev[i]);
		ac107_write(I2S_TX_CTRL2, 0, &param->twi_dev[i]);
		ac107_write(I2S_TX_CTRL3, 0, &param->twi_dev[i]);
	}

	//AC107 set sample resorution
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		sample_resolution = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		sample_resolution = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		sample_resolution = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sample_resolution = 32;
		break;
	default:
		snd_err("AC107 don't supported the sample resolution: %u\n",
			params_format(params));
		return -EINVAL;
	}
	ac107_multi_chips_update_bits(ac107, I2S_FMT_CTRL2, 0x7 << SAMPLE_RESOLUTION,
				      (sample_resolution / 4 -
				       1) << SAMPLE_RESOLUTION);

	//AC107 TX enable, Globle enable
	ac107_multi_chips_update_bits(ac107, I2S_CTRL, 0x1 << TXEN | 0x1 << GEN,
				      0x1 << TXEN | 0x1 << GEN);

	//AC107 PLL Enable and through MCLK Pin output Enable
	ac107_read(SYSCLK_CTRL, (u8 *)&i, ac107->i2c);
	if (i & 0x80) {		//PLLCLK Enable
		if (!(i & 0x0c)) {	//SYSCLK select MCLK
			//MCLK output Clock 24MHz from DPLL
			ac107_update_bits(I2S_CTRL, 0x1 << MCLK_IOEN,
					  0x1 << MCLK_IOEN, ac107->i2c);
			ac107_update_bits(I2S_PADDRV_CTRL, 0x03 << MCLK_DRV,
					  0x03 << MCLK_DRV, ac107->i2c);
			for (i = 0; i < AC107_CHIP_NUMS; i++) {	//multi_chips: only one chip MCLK output PLL_test, and the others MCLK config as input
				if (&param->twi_dev[i] == ac107->i2c)
					continue;
				ac107_update_bits(I2S_CTRL, 0x1 << MCLK_IOEN,
						  0x0 << MCLK_IOEN,
						  &param->twi_dev[i]);
			}
			//the chip which MCLK config as output, should select PLL as its SYCCLK source
			ac107_update_bits(SYSCLK_CTRL, 0x3 << SYSCLK_SRC,
					  SYSCLK_SRC_PLL << SYSCLK_SRC,
					  ac107->i2c);
			//the chip which MCLK config as output, PLL Common voltage Enable, PLL Enable
			ac107_update_bits(PLL_CTRL1,
					  0x1 << PLL_EN | 0x1 << PLL_COM_EN,
					  0x1 << PLL_EN | 0x1 << PLL_COM_EN,
					  ac107->i2c);
		} else if ((i & 0x0c) >> 2 == 0x2) {	//SYSCLK select PLL
			ac107_multi_chips_update_bits(ac107, PLL_LOCK_CTRL,
						      0x7 << SYSCLK_HOLD_TIME,
						      0x3 << SYSCLK_HOLD_TIME);
			//All chips PLL Common voltage Enable, PLL Enable
			ac107_multi_chips_update_bits(ac107, PLL_CTRL1,
						      0x1 << PLL_EN | 0x1 <<
						      PLL_COM_EN,
						      0x1 << PLL_EN | 0x1 <<
						      PLL_COM_EN);
		}
	}

	return 0;
}

static int ac107_hw_free(struct snd_pcm_substream *substream,
			 struct snd_dai *dai)
{
	struct snd_codec *codec = dai->component;
	struct ac107_priv *ac107 = codec->private_data;
	AC107_DEBUG("\n--->%s\n", __FUNCTION__);

	//AC107 I2S Globle disable
	ac107_multi_chips_update_bits(ac107, I2S_CTRL, 0x1 << GEN, 0x0 << GEN);

	AC107_DEBUG("AC107 reset all register to their default value\n\n");
	ac107_multi_chips_write(ac107, CHIP_AUDIO_RST, 0x12);

	return 0;
}

static int ac107_set_fmt(struct snd_dai *dai, unsigned int fmt)
{
	struct snd_codec *codec = dai->component;
	struct ac107_priv *ac107 = codec->private_data;
	struct ac107_param *param = &ac107->param;
	u8 i, tx_offset, i2s_mode, sign_ext, lrck_polarity, brck_polarity;
	AC107_DEBUG("\n--->%s\n", __FUNCTION__);

	//AC107 config Master/Slave mode
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:	//AC107 Master
		AC107_DEBUG("AC107 set to work as Master\n");
		ac107_update_bits(I2S_CTRL, 0x3 << LRCK_IOEN, 0x3 << LRCK_IOEN, ac107->i2c);	//BCLK & LRCK output
		break;
	case SND_SOC_DAIFMT_CBS_CFS:	//AC107 Slave
		AC107_DEBUG("AC107 set to work as Slave\n");
		ac107_update_bits(I2S_CTRL, 0x3 << LRCK_IOEN, 0x0 << LRCK_IOEN, ac107->i2c);	//BCLK & LRCK input
		break;
	default:
		snd_err("AC107 Master/Slave mode config error:%u\n\n",
		       (fmt & SND_SOC_DAIFMT_MASTER_MASK) >> 12);
		return -EINVAL;
	}
	for (i = 0; i < AC107_CHIP_NUMS; i++) {	//multi_chips: only one chip set as Master, and the others also need to set as Slave
		if (&param->twi_dev[i] == ac107->i2c)
			continue;
		ac107_update_bits(I2S_CTRL, 0x3 << LRCK_IOEN, 0x0 << LRCK_IOEN,
				  &param->twi_dev[i]);
	}

	//AC107 config I2S/LJ/RJ/PCM format
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		AC107_DEBUG("AC107 config I2S format\n");
		i2s_mode = LEFT_JUSTIFIED_FORMAT;
		tx_offset = 1;
		sign_ext = TRANSFER_ZERO_AFTER;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		AC107_DEBUG("AC107 config RIGHT-JUSTIFIED format\n");
		i2s_mode = RIGHT_JUSTIFIED_FORMAT;
		tx_offset = 0;
		sign_ext = SIGN_EXTENSION_MSB;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		AC107_DEBUG("AC107 config LEFT-JUSTIFIED format\n");
		i2s_mode = LEFT_JUSTIFIED_FORMAT;
		tx_offset = 0;
		sign_ext = TRANSFER_ZERO_AFTER;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		AC107_DEBUG("AC107 config PCM-A format\n");
		i2s_mode = PCM_FORMAT;
		tx_offset = 1;
		sign_ext = TRANSFER_ZERO_AFTER;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		AC107_DEBUG("AC107 config PCM-B format\n");
		i2s_mode = PCM_FORMAT;
		tx_offset = 0;
		sign_ext = TRANSFER_ZERO_AFTER;
		break;
	default:
		snd_err("AC107 I2S format config error:%u\n\n",
		       fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}
	ac107_multi_chips_update_bits(ac107, I2S_FMT_CTRL1,
				      0x3 << MODE_SEL | 0x1 << TX_OFFSET,
				      i2s_mode << MODE_SEL | tx_offset <<
				      TX_OFFSET);
	ac107_multi_chips_update_bits(ac107, I2S_FMT_CTRL3, 0x3 << SEXT,
				      sign_ext << SEXT);

	//AC107 config BCLK&LRCK polarity
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		AC107_DEBUG
		    ("AC107 config BCLK&LRCK polarity: BCLK_normal,LRCK_normal\n");
		brck_polarity = BCLK_NORMAL_DRIVE_N_SAMPLE_P;
		lrck_polarity = LRCK_LEFT_LOW_RIGHT_HIGH;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		AC107_DEBUG
		    ("AC107 config BCLK&LRCK polarity: BCLK_normal,LRCK_invert\n");
		brck_polarity = BCLK_NORMAL_DRIVE_N_SAMPLE_P;
		lrck_polarity = LRCK_LEFT_HIGH_RIGHT_LOW;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		AC107_DEBUG
		    ("AC107 config BCLK&LRCK polarity: BCLK_invert,LRCK_normal\n");
		brck_polarity = BCLK_INVERT_DRIVE_P_SAMPLE_N;
		lrck_polarity = LRCK_LEFT_LOW_RIGHT_HIGH;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		AC107_DEBUG
		    ("AC107 config BCLK&LRCK polarity: BCLK_invert,LRCK_invert\n");
		brck_polarity = BCLK_INVERT_DRIVE_P_SAMPLE_N;
		lrck_polarity = LRCK_LEFT_HIGH_RIGHT_LOW;
		break;
	default:
		snd_err("AC107 config BCLK/LRCLK polarity error:%u\n\n",
		       (fmt & SND_SOC_DAIFMT_INV_MASK) >> 8);
		return -EINVAL;
	}
	ac107_multi_chips_update_bits(ac107, I2S_BCLK_CTRL, 0x1 << BCLK_POLARITY,
				      brck_polarity << BCLK_POLARITY);
	ac107_multi_chips_update_bits(ac107, I2S_LRCK_CTRL1, 0x1 << LRCK_POLARITY,
				      lrck_polarity << LRCK_POLARITY);

	return 0;
}

static int ac107_codec_probe(struct snd_codec *codec)
{
	struct ac107_priv *ac107 = NULL;
	struct ac107_param default_param = {
		.chip_num = AC107_CHIP_NUMS,
		.twi_dev = AC107_CHIP_CFG,
		.pga_gain = {
			ADC_PGA_GAIN_0dB,
			AC107_PGA_GAIN,
			},
	};

	if (!codec->codec_dai) {
		snd_err("codec->codec_dai is null.\n");
		return -EFAULT;
	}

	ac107 = snd_malloc(sizeof(struct ac107_priv));
	if (!ac107) {
		snd_err("no memory\n");
		return -ENOMEM;
	}

	codec->private_data = (void *)ac107;
	ac107->param = default_param;
	ac107->codec = codec;
	ac107->i2c = &ac107->param.twi_dev[0];
	codec->codec_dai->component = codec;

	if (codec_i2c_init(&twi_config) < 0) {
		snd_err("ac107 i2c init failed\n");
		snd_free(ac107);
		return -1;
	}

#if AC107_KCONTROL_EN
	ac107_multi_chips_update_bits(ac107, ANA_ADC1_CTRL3, 0x1f << RX1_PGA_GAIN_CTRL, ac107->param.pga_gain[0] << RX1_PGA_GAIN_CTRL);
	ac107_multi_chips_update_bits(ac107, ANA_ADC2_CTRL3, 0x1f << RX2_PGA_GAIN_CTRL, ac107->param.pga_gain[1] << RX2_PGA_GAIN_CTRL);
#endif

	snd_info("ac107 codec register finished.\n");

	return 0;
}

static int ac107_codec_remove(struct snd_codec *codec)
{
	struct ac107_priv *ac107 = codec->private_data;

	snd_free(ac107);
	codec->private_data = NULL;
	return 0;
}

static int ac107_codec0_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_info *info)
{
	struct snd_codec *codec = kcontrol->private_data;
	struct ac107_priv *ac107 = (struct ac107_priv *)codec->private_data;
	u8 reg_val;
	unsigned int reg = kcontrol->reg;
	unsigned int shift = kcontrol->shift;
	unsigned int max = kcontrol->max;
	unsigned int mask = (1 << __fls(max)) - 1;

	if (ac107_read(reg, &reg_val, &ac107->param.twi_dev[0]) < 0)
		return -1;

	info->id = kcontrol->id;
	info->name = kcontrol->name;
	info->min = kcontrol->min;
	info->max = kcontrol->max;
	info->value = (reg_val >> shift) & mask;

	return 0;
}

static int ac107_codec0_put(struct snd_kcontrol *kcontrol, unsigned long val)
{
	struct snd_codec *codec = kcontrol->private_data;
	struct ac107_priv *ac107 = (struct ac107_priv *)codec->private_data;
	u8 reg_val;
	unsigned int reg = kcontrol->reg;
	unsigned int shift = kcontrol->shift;
	unsigned int max = kcontrol->max;
	unsigned int mask = (1 << __fls(max)) - 1;

	reg_val = val & mask;
	if (ac107_update_bits(reg, mask<<shift, reg_val<<shift,
				&ac107->param.twi_dev[0]) < 0)
		return -1;

	return 0;
}

static struct snd_kcontrol ac107_controls[] = {
	SND_CTL_KCONTROL_EXT_REG("Channel 1 PGA Gain", ANA_ADC1_CTRL3,
				RX1_PGA_GAIN_CTRL, 0x1f,
				ac107_codec0_get, ac107_codec0_put),
	SND_CTL_KCONTROL_EXT_REG("Channel 2 PGA Gain", ANA_ADC2_CTRL3,
				RX2_PGA_GAIN_CTRL, 0x1f,
				ac107_codec0_get, ac107_codec0_put),
	SND_CTL_KCONTROL_EXT_REG("Channel 1 Digital Volume", ADC1_DVOL_CTRL,
				0, 0xff, ac107_codec0_get, ac107_codec0_put),
	SND_CTL_KCONTROL_EXT_REG("Channel 2 Digital Volume", ADC2_DVOL_CTRL,
				0, 0xff, ac107_codec0_get, ac107_codec0_put),
};

static struct snd_dai_ops ac107_codec_dai_ops = {
	.set_sysclk	= ac107_set_sysclk,
	.set_pll	= ac107_set_pll,
	.set_clkdiv	= ac107_set_clkdiv,
	.hw_params	= ac107_hw_params,
	.hw_free	= ac107_hw_free,
	.set_fmt	= ac107_set_fmt,
	.shutdown	= NULL,
	.startup	= NULL,
	.trigger	= NULL,
	.prepare	= NULL,
};

static struct snd_dai ac107_codec_dai[] = {
	{
		.name		= "ac107-codecdai",
		.capture	= {
			.stream_name	= "Capture",
			.channels_min	= 1,
			.channels_max	= AC107_CHIP_NUMS * 2,
			.rates		= AC107_RATES,
			.formats	= AC107_FORMATS,
			.rate_min       = 8000,
			.rate_max       = 48000,
		},
		.ops		= &ac107_codec_dai_ops,
	},
};

struct snd_codec ac107_codec = {
	.name		= "ac107-codec",
	.codec_dai	= ac107_codec_dai,
	.codec_dai_num  = ARRAY_SIZE(ac107_codec_dai),
	.private_data	= NULL,
	.probe          = ac107_codec_probe,
	.remove         = ac107_codec_remove,
	.controls       = ac107_controls,
	.num_controls   = ARRAY_SIZE(ac107_controls),
};
