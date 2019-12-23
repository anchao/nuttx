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
#include <snd_core.h>
#include <snd_pcm.h>
#include <snd_io.h>
#include "sunxi-pcm.h"
#include "sunxi-daudio.h"

#ifdef CONFIG_SND_MULTI_SOUNDCARD
#include <sun8iw18-codec.h>
extern struct snd_codec sun8iw18_codec;
#endif

#define	SUNXI_DAUDIO_RATES	(SNDRV_PCM_RATE_8000_192000 \
				| SNDRV_PCM_RATE_KNOT)

#define SUNXI_DAUDIO_DRQDST(sunxi_daudio, x)			\
	((sunxi_daudio)->playback_dma_param.dma_drq_type_num =	\
				DRQDST_DAUDIO_##x##_TX)
#define SUNXI_DAUDIO_DRQSRC(sunxi_daudio, x)			\
	((sunxi_daudio)->capture_dma_param.dma_drq_type_num =	\
				DRQSRC_DAUDIO_##x##_RX)
#if 0
typedef struct {
	hal_gpio_pin_t gpio;
	unsigned int mux;
} i2s_gpio_t;
typedef struct {
	i2s_gpio_t lrclk;
	i2s_gpio_t bclk;
	i2s_gpio_t dout;
	i2s_gpio_t din;
	i2s_gpio_t mclk;
} daudio_gpio_t;

daudio_gpio_t g_daudio_gpio[DAUDIO_NUM_MAX] = {
	[0] = {
		.lrclk		= {GPIOB(2), 5},
		.bclk		= {GPIOB(3), 5},
		.dout		= {GPIOB(4), 4},
		.din		= {GPIOB(5), 5},
		.mclk		= {GPIOB(13), 5},
	},
	[1] = {
		.lrclk		= {GPIOB(8), 2},
		.bclk		= {GPIOB(9), 2},
		.dout		= {GPIOB(10), 2},
		.din		= {GPIOB(11), 4},
		.mclk		= {GPIOB(12), 2},
	},
	[2] = {
		.lrclk		= {GPIOG(11), 3},
		.bclk		= {GPIOG(12), 3},
		.dout		= {GPIOG(13), 3},
		.din		= {GPIOG(14), 4},
		.mclk		= {GPIOG(10), 3},
	},
};
#endif

static const struct sunxi_daudio_param g_default_param[DAUDIO_NUM_MAX] = {
	[0] = {
		.tdm_num		= 0,
		.daudio_master		= 4,
		.audio_format		= 1,
		.signal_inversion	= 1,
		.pcm_lrck_period	= 128,
		.slot_width_select	= 32,
		.msb_lsb_first		= 0,
		.frametype		= 0,
		.tx_data_mode		= 0,
		.rx_data_mode		= 0,
		.tdm_config		= 1,
		.mclk_div		= 1,
	},
	[1] = {
		.tdm_num		= 1,
		.daudio_master		= 4,
		.audio_format		= 1,
		.signal_inversion	= 1,
		.pcm_lrck_period	= 128,
		.slot_width_select	= 32,
		.msb_lsb_first		= 0,
		.frametype		= 0,
		.tx_data_mode		= 0,
		.rx_data_mode		= 0,
		.tdm_config		= 1,
		.mclk_div		= 2,
	},
	[2] = {
		.tdm_num		= 2,
		.daudio_master		= 4,
		.audio_format		= 1,
		.signal_inversion	= 1,
		.pcm_lrck_period	= 128,
		.slot_width_select	= 32,
		.msb_lsb_first		= 0,
		.frametype		= 0,
		.tx_data_mode		= 0,
		.rx_data_mode		= 0,
		.tdm_config		= 1,
		.mclk_div		= 1,
	},
};

static int sunxi_daudio_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_dai *dai)
{
	struct snd_platform *platform = dai->component;
	struct sunxi_daudio_info *sunxi_daudio = platform->private_data;
	struct sunxi_daudio_param *param = &sunxi_daudio->param;
	struct snd_dai *codec_dai = snd_soc_get_codec_dai(substream);
	int ret = 0;

	/* set codec dai fmt */
	ret = snd_soc_dai_set_fmt(codec_dai, param->audio_format
			| (param->signal_inversion << SND_SOC_DAIFMT_SIG_SHIFT)
			| (param->daudio_master << SND_SOC_DAIFMT_MASTER_SHIFT));
	if (ret < 0) {
		snd_err("codec dai set fmt failed.\n");
		return -EINVAL;
	}
	/* set cpu dai fmt */
	ret = snd_soc_dai_set_fmt(dai, param->audio_format
			| (param->signal_inversion << SND_SOC_DAIFMT_SIG_SHIFT)
			| (param->daudio_master << SND_SOC_DAIFMT_MASTER_SHIFT));
	if (ret < 0) {
		snd_err("cpu dai set fmt failed.\n");
		return -EINVAL;
	}

	switch (params_format(params)) {
	case	SNDRV_PCM_FORMAT_S16_LE:

		snd_platform_update_bits(platform,
			SUNXI_DAUDIO_FMT0,
			(SUNXI_DAUDIO_SR_MASK<<DAUDIO_SAMPLE_RESOLUTION),
			(SUNXI_DAUDIO_SR_16BIT<<DAUDIO_SAMPLE_RESOLUTION));
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			snd_platform_update_bits(platform,
				SUNXI_DAUDIO_FIFOCTL,
				(SUNXI_DAUDIO_TXIM_MASK<<TXIM),
				(SUNXI_DAUDIO_TXIM_VALID_LSB<<TXIM));
		else
			snd_platform_update_bits(platform,
				SUNXI_DAUDIO_FIFOCTL,
				(SUNXI_DAUDIO_RXOM_MASK<<RXOM),
				(SUNXI_DAUDIO_RXOM_EXPH<<RXOM));
		break;
	case	SNDRV_PCM_FORMAT_S24_LE:
		snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT0,
				(SUNXI_DAUDIO_SR_MASK<<DAUDIO_SAMPLE_RESOLUTION),
				(SUNXI_DAUDIO_SR_24BIT<<DAUDIO_SAMPLE_RESOLUTION));
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			snd_platform_update_bits(platform,
					SUNXI_DAUDIO_FIFOCTL,
					(SUNXI_DAUDIO_TXIM_MASK<<TXIM),
					(SUNXI_DAUDIO_TXIM_VALID_LSB<<TXIM));
		else
			snd_platform_update_bits(platform,
					SUNXI_DAUDIO_FIFOCTL,
					(SUNXI_DAUDIO_RXOM_MASK<<RXOM),
					(SUNXI_DAUDIO_RXOM_EXP0<<RXOM));
		break;
	case	SNDRV_PCM_FORMAT_S32_LE:
		snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT0,
				(SUNXI_DAUDIO_SR_MASK<<DAUDIO_SAMPLE_RESOLUTION),
				(SUNXI_DAUDIO_SR_32BIT<<DAUDIO_SAMPLE_RESOLUTION));
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			snd_platform_update_bits(platform,
					SUNXI_DAUDIO_FIFOCTL,
					(SUNXI_DAUDIO_TXIM_MASK<<TXIM),
					(SUNXI_DAUDIO_TXIM_VALID_LSB<<TXIM));
		else
			snd_platform_update_bits(platform,
					SUNXI_DAUDIO_FIFOCTL,
					(SUNXI_DAUDIO_RXOM_MASK<<RXOM),
					(SUNXI_DAUDIO_RXOM_EXPH<<RXOM));
		break;
	default:
		snd_err("unrecognized format\n");
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_platform_update_bits(platform, SUNXI_DAUDIO_CHCFG,
				(SUNXI_DAUDIO_TX_SLOT_MASK<<TX_SLOT_NUM),
				((params_channels(params)-1)<<TX_SLOT_NUM));
		snd_platform_write(platform,
			SUNXI_DAUDIO_TX0CHMAP0, SUNXI_DEFAULT_CHMAP0);
		snd_platform_write(platform,
			SUNXI_DAUDIO_TX0CHMAP1, SUNXI_DEFAULT_CHMAP1);
		snd_platform_update_bits(platform,
			SUNXI_DAUDIO_TX0CHSEL,
			(SUNXI_DAUDIO_TX_CHSEL_MASK<<TX_CHSEL),
			((params_channels(params)-1)<<TX_CHSEL));
		snd_platform_update_bits(platform,
			SUNXI_DAUDIO_TX0CHSEL,
			(SUNXI_DAUDIO_TX_CHEN_MASK<<TX_CHEN),
			((1<<params_channels(params))-1)<<TX_CHEN);
	} else {
		unsigned int SUNXI_DAUDIO_RXCHMAPX = 0;
		int index = 0;

		for (index = 0; index < 16; index++) {
			if (index >= 12)
				SUNXI_DAUDIO_RXCHMAPX = SUNXI_DAUDIO_RXCHMAP0;
			else if (index >= 8)
				SUNXI_DAUDIO_RXCHMAPX = SUNXI_DAUDIO_RXCHMAP1;
			else if (index >= 4)
				SUNXI_DAUDIO_RXCHMAPX = SUNXI_DAUDIO_RXCHMAP2;
			else
				SUNXI_DAUDIO_RXCHMAPX = SUNXI_DAUDIO_RXCHMAP3;
			snd_platform_update_bits(platform,
				SUNXI_DAUDIO_RXCHMAPX,
				DAUDIO_RXCHMAP(index),
				DAUDIO_RXCH_DEF_MAP(index));
		}
		snd_platform_update_bits(platform, SUNXI_DAUDIO_CHCFG,
				(SUNXI_DAUDIO_RX_SLOT_MASK<<RX_SLOT_NUM),
				((params_channels(params)-1)<<RX_SLOT_NUM));
		snd_platform_update_bits(platform, SUNXI_DAUDIO_RXCHSEL,
				(SUNXI_DAUDIO_RX_CHSEL_MASK<<RX_CHSEL),
				((params_channels(params)-1)<<RX_CHSEL));
	}

	return 0;
}

static int sunxi_daudio_set_sysclk(struct snd_dai *dai,
				int clk_id, unsigned int freq, int dir)
{
#if 0
	struct snd_platform *platform = dai->component;
	struct sunxi_daudio_info *sunxi_daudio = platform->private_data;

	snd_print("\n");
	if (hal_clk_set_rate(sunxi_daudio->pllclk, freq)) {
		snd_err("set pllclk rate %u failed\n", freq);
		return -EINVAL;
	}
#endif
	return 0;
}

static int sunxi_daudio_set_clkdiv(struct snd_dai *dai,
				int clk_id, int clk_div)
{
	struct snd_platform *platform = dai->component;
	struct sunxi_daudio_info *sunxi_daudio = platform->private_data;
	struct sunxi_daudio_param *param = &sunxi_daudio->param;
	unsigned int bclk_div, div_ratio;

	if (param->tdm_config)
		/* I2S/TDM two channel mode */
		div_ratio = clk_div/(2 * param->pcm_lrck_period);
	else
		/* PCM mode */
		div_ratio = clk_div / param->pcm_lrck_period;

	switch (div_ratio) {
	case	1:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_1;
		break;
	case	2:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_2;
		break;
	case	4:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_3;
		break;
	case	6:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_4;
		break;
	case	8:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_5;
		break;
	case	12:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_6;
		break;
	case	16:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_7;
		break;
	case	24:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_8;
		break;
	case	32:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_9;
		break;
	case	48:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_10;
		break;
	case	64:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_11;
		break;
	case	96:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_12;
		break;
	case	128:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_13;
		break;
	case	176:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_14;
		break;
	case	192:
		bclk_div = SUNXI_DAUDIO_BCLK_DIV_15;
		break;
	default:
		snd_err("unsupport clk_div\n");
		return -EINVAL;
	}
	/* setting bclk to driver external codec bit clk */
	snd_platform_update_bits(platform, SUNXI_DAUDIO_CLKDIV,
			(SUNXI_DAUDIO_BCLK_DIV_MASK<<BCLK_DIV),
			(bclk_div<<BCLK_DIV));

	return 0;
}

static int sunxi_daudio_init_fmt(struct snd_platform *platform, unsigned int fmt)
{
	unsigned int offset, mode;
	unsigned int lrck_polarity, brck_polarity;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case	SND_SOC_DAIFMT_CBM_CFM:
		snd_platform_update_bits(platform, SUNXI_DAUDIO_CTL,
				(SUNXI_DAUDIO_LRCK_OUT_MASK<<LRCK_OUT),
				(SUNXI_DAUDIO_LRCK_OUT_DISABLE<<LRCK_OUT));
		break;
	case	SND_SOC_DAIFMT_CBS_CFS:
		snd_platform_update_bits(platform, SUNXI_DAUDIO_CTL,
				(SUNXI_DAUDIO_LRCK_OUT_MASK<<LRCK_OUT),
				(SUNXI_DAUDIO_LRCK_OUT_ENABLE<<LRCK_OUT));
		break;
	default:
		snd_err("unknown maser/slave format\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case	SND_SOC_DAIFMT_I2S:
		offset = SUNXI_DAUDIO_TX_OFFSET_1;
		mode = SUNXI_DAUDIO_MODE_CTL_I2S;
		break;
	case	SND_SOC_DAIFMT_RIGHT_J:
		offset = SUNXI_DAUDIO_TX_OFFSET_0;
		mode = SUNXI_DAUDIO_MODE_CTL_RIGHT;
		break;
	case	SND_SOC_DAIFMT_LEFT_J:
		offset = SUNXI_DAUDIO_TX_OFFSET_0;
		mode = SUNXI_DAUDIO_MODE_CTL_LEFT;
		break;
	case	SND_SOC_DAIFMT_DSP_A:
		offset = SUNXI_DAUDIO_TX_OFFSET_1;
		mode = SUNXI_DAUDIO_MODE_CTL_PCM;
		break;
	case	SND_SOC_DAIFMT_DSP_B:
		offset = SUNXI_DAUDIO_TX_OFFSET_0;
		mode = SUNXI_DAUDIO_MODE_CTL_PCM;
		break;
	default:
		snd_err("format setting failed\n");
		return -EINVAL;
	}

	snd_platform_update_bits(platform, SUNXI_DAUDIO_CTL,
			(SUNXI_DAUDIO_MODE_CTL_MASK<<MODE_SEL),
			(mode<<MODE_SEL));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_TX0CHSEL,
			(SUNXI_DAUDIO_TX_OFFSET_MASK<<TX_OFFSET),
			(offset<<TX_OFFSET));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_RXCHSEL,
			(SUNXI_DAUDIO_RX_OFFSET_MASK<<RX_OFFSET),
			(offset<<RX_OFFSET));

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case	SND_SOC_DAIFMT_NB_NF:
		lrck_polarity = SUNXI_DAUDIO_LRCK_POLARITY_NOR;
		brck_polarity = SUNXI_DAUDIO_BCLK_POLARITY_NOR;
		break;
	case	SND_SOC_DAIFMT_NB_IF:
		lrck_polarity = SUNXI_DAUDIO_LRCK_POLARITY_INV;
		brck_polarity = SUNXI_DAUDIO_BCLK_POLARITY_NOR;
		break;
	case	SND_SOC_DAIFMT_IB_NF:
		lrck_polarity = SUNXI_DAUDIO_LRCK_POLARITY_NOR;
		brck_polarity = SUNXI_DAUDIO_BCLK_POLARITY_INV;
		break;
	case	SND_SOC_DAIFMT_IB_IF:
		lrck_polarity = SUNXI_DAUDIO_LRCK_POLARITY_INV;
		brck_polarity = SUNXI_DAUDIO_BCLK_POLARITY_INV;
		break;
	default:
		snd_err("invert clk setting failed\n");
		return -EINVAL;
	}
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT0,
			(1<<LRCK_POLARITY), (lrck_polarity<<LRCK_POLARITY));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT0,
			(1<<BRCK_POLARITY), (brck_polarity<<BRCK_POLARITY));

	return 0;
}

static int sunxi_daudio_set_fmt(struct snd_dai *dai, unsigned int fmt)
{
	struct snd_platform *platform = dai->component;

	sunxi_daudio_init_fmt(platform, fmt);
	return 0;
}

static int sunxi_daudio_startup(struct snd_pcm_substream *substream,
				struct snd_dai *dai)
{
	struct snd_platform *platform = dai->component;
	struct sunxi_daudio_info *sunxi_daudio = platform->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dai->playback_dma_data = &sunxi_daudio->playback_dma_param;
	else
		dai->capture_dma_data = &sunxi_daudio->capture_dma_param;

	return 0;
}

static void sunxi_daudio_txctrl_enable(struct snd_platform *platform, bool enable)
{
	if (enable) {
		snd_platform_update_bits(platform,
				SUNXI_DAUDIO_CTL,
				(1 << SDO0_EN), (1 << SDO0_EN));
		snd_platform_update_bits(platform,
				SUNXI_DAUDIO_CTL,
				(1 << CTL_TXEN), (1 << CTL_TXEN));
		snd_platform_update_bits(platform, SUNXI_DAUDIO_INTCTL,
					(1 << TXDRQEN), (1 << TXDRQEN));
	} else {
		snd_platform_update_bits(platform, SUNXI_DAUDIO_INTCTL,
					(1 << TXDRQEN), (0 << TXDRQEN));
		snd_platform_update_bits(platform,
				SUNXI_DAUDIO_CTL,
				(1 << CTL_TXEN), (0 << CTL_TXEN));
		snd_platform_update_bits(platform,
				SUNXI_DAUDIO_CTL,
				(1 << SDO0_EN), (0 << SDO0_EN));
	}
}

static void sunxi_daudio_rxctrl_enable(struct snd_platform *platform, bool enable)
{
	if (enable) {
		snd_platform_update_bits(platform,
				SUNXI_DAUDIO_CTL,
				(1<<CTL_RXEN), (1<<CTL_RXEN));
		snd_platform_update_bits(platform,
				SUNXI_DAUDIO_INTCTL,
				(1<<RXDRQEN), (1<<RXDRQEN));
	} else {
		snd_platform_update_bits(platform,
				SUNXI_DAUDIO_INTCTL,
				(1<<RXDRQEN), (0<<RXDRQEN));
		snd_platform_update_bits(platform,
				SUNXI_DAUDIO_CTL,
				(1<<CTL_RXEN), (0<<CTL_RXEN));
	}
}

static int sunxi_daudio_global_enable(struct snd_platform *platform, bool enable)
{
	struct sunxi_daudio_info *sunxi_daudio = platform->private_data;

	if (enable) {
#ifdef CONFIG_SND_MULTI_SOUNDCARD
		struct snd_codec *codec = &sun8iw18_codec;
		snd_codec_update_bits(codec, SUNXI_ADC_FIFO_CTL,
			(0x1 << EN_AD), (0x01 << EN_AD));
		snd_codec_update_bits(codec, SUNXI_ADC_FIFO_CTL,
			(0x1 << ADC_DRQ_EN), (0x01 << ADC_DRQ_EN));
#endif
		if (sunxi_daudio->global_enable++ == 0)
			snd_platform_update_bits(platform,
				SUNXI_DAUDIO_CTL,
				(1<<GLOBAL_EN), (1<<GLOBAL_EN));
	} else {
		if (--sunxi_daudio->global_enable == 0)
			snd_platform_update_bits(platform,
				SUNXI_DAUDIO_CTL,
				(1<<GLOBAL_EN), (0<<GLOBAL_EN));
#ifdef CONFIG_SND_MULTI_SOUNDCARD
		struct snd_codec *codec = &sun8iw18_codec;
		snd_codec_update_bits(codec, SUNXI_ADC_FIFO_CTL,
			(0x1 << EN_AD), (0x00 << EN_AD));
		snd_codec_update_bits(codec, SUNXI_ADC_FIFO_CTL,
			(0x1 << ADC_DRQ_EN), (0x00 << ADC_DRQ_EN));
#endif
	}

	return 0;
}

static int sunxi_daudio_trigger(struct snd_pcm_substream *substream,
				int cmd, struct snd_dai *dai)
{
	struct snd_platform *platform = dai->component;

	switch (cmd) {
	case	SNDRV_PCM_TRIGGER_START:
	case	SNDRV_PCM_TRIGGER_RESUME:
	case	SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sunxi_daudio_txctrl_enable(platform, true);
			/* Global enable I2S/TDM module */
			sunxi_daudio_global_enable(platform, true);
		} else {
			sunxi_daudio_rxctrl_enable(platform, true);
			sunxi_daudio_global_enable(platform, true);
		}
		break;
	case	SNDRV_PCM_TRIGGER_STOP:
	case	SNDRV_PCM_TRIGGER_SUSPEND:
	case	SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			sunxi_daudio_txctrl_enable(platform, false);
			/* Global enable I2S/TDM module */
			sunxi_daudio_global_enable(platform, false);
		} else {
			sunxi_daudio_rxctrl_enable(platform, false);
			sunxi_daudio_global_enable(platform, false);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sunxi_daudio_prepare(struct snd_pcm_substream *substream,
				struct snd_dai *dai)
{
	struct snd_platform *platform = dai->component;
	unsigned int i;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		for (i = 0 ; i < SUNXI_DAUDIO_FTX_TIMES ; i++) {
			snd_platform_update_bits(platform,
				SUNXI_DAUDIO_FIFOCTL,
				(1<<FIFO_CTL_FTX), (1<<FIFO_CTL_FTX));
			usleep(1000);
		}
		snd_platform_write(platform, SUNXI_DAUDIO_TXCNT, 0);
	} else {

		snd_platform_update_bits(platform, SUNXI_DAUDIO_FIFOCTL,
				(1<<FIFO_CTL_FRX), (1<<FIFO_CTL_FRX));
		snd_platform_write(platform, SUNXI_DAUDIO_RXCNT, 0);
	}

	return 0;
}

static void sunxi_daudio_shutdown(struct snd_pcm_substream *substream,
				struct snd_dai *dai)
{
	snd_print("\n");
}

static int sunxi_daudio_mclk_setting(struct snd_platform *platform)
{
	struct sunxi_daudio_info *sunxi_daudio = platform->private_data;
	struct sunxi_daudio_param *param = &sunxi_daudio->param;
	uint16_t mclk_div;

	if (!param->mclk_div) {
		snd_platform_update_bits(platform, SUNXI_DAUDIO_CLKDIV,
					(1<<MCLKOUT_EN), (0<<MCLKOUT_EN));
		return 0;
	}

	switch (param->mclk_div) {
	case 1:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_1;
		break;
	case 2:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_2;
		break;
	case 4:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_3;
		break;
	case 6:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_4;
		break;
	case 8:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_5;
		break;
	case 12:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_6;
		break;
	case 16:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_7;
		break;
	case 24:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_8;
		break;
	case 32:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_9;
		break;
	case	48:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_10;
		break;
	case	64:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_11;
		break;
	case	96:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_12;
		break;
	case	128:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_13;
		break;
	case	176:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_14;
		break;
	case	192:
		mclk_div = SUNXI_DAUDIO_MCLK_DIV_15;
		break;
	default:
		snd_err("unsupport  mclk_div\n");
		return -EINVAL;
	}
	snd_platform_update_bits(platform, SUNXI_DAUDIO_CLKDIV,
			(SUNXI_DAUDIO_MCLK_DIV_MASK<<MCLK_DIV),
			(mclk_div<<MCLK_DIV));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_CLKDIV,
				(1<<MCLKOUT_EN), (1<<MCLKOUT_EN));

	return 0;
}

static int sunxi_daudio_init(struct snd_platform *platform)
{
	struct sunxi_daudio_info *sunxi_daudio = platform->private_data;
	struct sunxi_daudio_param *param = &sunxi_daudio->param;

	snd_print("\n");
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT0,
				(1 << LRCK_WIDTH),
				(param->frametype << LRCK_WIDTH));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT0,
				(SUNXI_DAUDIO_LRCK_PERIOD_MASK) << LRCK_PERIOD,
				((param->pcm_lrck_period - 1) << LRCK_PERIOD));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT0,
				(SUNXI_DAUDIO_SLOT_WIDTH_MASK) << SLOT_WIDTH,
				(((param->slot_width_select >> 2) - 1) << SLOT_WIDTH));

	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT1,
				(0x1 << TX_MLS),
				(param->msb_lsb_first << TX_MLS));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT1,
				(0x1 << RX_MLS),
				(param->msb_lsb_first << RX_MLS));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT1,
				(0x3 << SEXT),
				(param->sign_extend << SEXT));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT1,
				(0x3 << TX_PDM),
				(param->tx_data_mode << TX_PDM));
	snd_platform_update_bits(platform, SUNXI_DAUDIO_FMT1,
				(0x3 << RX_PDM),
				(param->rx_data_mode << RX_PDM));
#if 0
	snd_platform_update_bits(platform, SUNXI_DAUDIO_CTL,
				(0x1 << LOOP_EN),
				(0x1 << LOOP_EN));
#endif

	return sunxi_daudio_mclk_setting(platform);
}

static int sunxi_daudio_dai_probe(struct snd_dai *dai)
{
	struct snd_platform *platform = dai->component;

	snd_print("\n");
	dai->playback_dma_data = 0;
	dai->capture_dma_data = 0;
	sunxi_daudio_init(platform);
	return 0;
}

static void sunxi_daudio_gpio_init(unsigned int daudio_num)
{
	if (daudio_num >= DAUDIO_NUM_MAX) {
		snd_err("invalid daudio num %d\n", daudio_num);
		return;
	}
#if 0
	/*LRCLK*/
	hal_gpio_pinmux_set_function(g_daudio_gpio[daudio_num].lrclk.gpio,
				g_daudio_gpio[daudio_num].lrclk.mux);
	/*BCLK*/
	hal_gpio_pinmux_set_function(g_daudio_gpio[daudio_num].bclk.gpio,
				g_daudio_gpio[daudio_num].bclk.mux);
	/*DOUT0*/
	hal_gpio_pinmux_set_function(g_daudio_gpio[daudio_num].dout.gpio,
				g_daudio_gpio[daudio_num].dout.mux);
	/*DIN0*/
	hal_gpio_pinmux_set_function(g_daudio_gpio[daudio_num].din.gpio,
				g_daudio_gpio[daudio_num].din.mux);
	/*MCLK*/
	hal_gpio_pinmux_set_function(g_daudio_gpio[daudio_num].mclk.gpio,
				g_daudio_gpio[daudio_num].mclk.mux);
#endif
}

static struct snd_dai_ops sunxi_daudio_dai_ops = {
	.hw_params = sunxi_daudio_hw_params,
	.set_sysclk = sunxi_daudio_set_sysclk,
	.set_clkdiv = sunxi_daudio_set_clkdiv,
	.set_fmt = sunxi_daudio_set_fmt,
	.startup = sunxi_daudio_startup,
	.trigger = sunxi_daudio_trigger,
	.prepare = sunxi_daudio_prepare,
	.shutdown = sunxi_daudio_shutdown,
};

static struct snd_dai sunxi_daudio_dai = {
	.id		= 1,
	.name		= "sunxi-daudio-cpudai",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 8,
		.rates		= SUNXI_DAUDIO_RATES,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE,
		.rate_min       = 8000,
		.rate_max       = 192000,
	},
	.capture	= {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 8,
		.rates		= SUNXI_DAUDIO_RATES,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE
				| SNDRV_PCM_FMTBIT_S24_LE
				| SNDRV_PCM_FMTBIT_S32_LE,
		.rate_min       = 8000,
		.rate_max       = 48000,
	},
	.probe		= sunxi_daudio_dai_probe,
	.ops		= &sunxi_daudio_dai_ops,
};

static int sunxi_daudio_platform_probe(struct snd_platform *platform)
{
	int ret = 0;
	struct sunxi_daudio_info *sunxi_daudio;

	sunxi_daudio = snd_malloc(sizeof(struct sunxi_daudio_info));
	if (!sunxi_daudio) {
		snd_err("no memory\n");
		return -ENOMEM;
	}
	platform->private_data = (void *)sunxi_daudio;
	platform->cpu_dai->component = platform;

	/* parser para */
	sunxi_daudio->param = g_default_param[platform->type - SND_PLATFORM_TYPE_DAUDIO0];
#if 0
	/* clk */
	sunxi_daudio->pllclk = HAL_CLK_PLL_AUDIO;
	sunxi_daudio->pllclkx4 = HAL_CLK_PLL_AUDIOX4;
	sunxi_daudio->moduleclk = HAL_CLK_PERIPH_I2S0 + sunxi_daudio->param.tdm_num;

	hal_clk_set_parent(sunxi_daudio->moduleclk, sunxi_daudio->pllclkx4);
	hal_clock_enable(sunxi_daudio->pllclk);
	hal_clock_enable(sunxi_daudio->pllclkx4);
	hal_clock_enable(sunxi_daudio->moduleclk);
#endif
	/* mem base */
	platform->mem_base = (void *)SUNXI_DAUDIO_MEMBASE + (0x1000 * sunxi_daudio->param.tdm_num);
	/* pinctrl */
	sunxi_daudio_gpio_init(sunxi_daudio->param.tdm_num);

	/* dma config */
	sunxi_daudio->playback_dma_param.src_maxburst = 4;
	sunxi_daudio->playback_dma_param.dst_maxburst = 4;
	sunxi_daudio->playback_dma_param.dma_addr =
			(dma_addr_t)platform->mem_base + SUNXI_DAUDIO_TXFIFO;
	sunxi_daudio->capture_dma_param.src_maxburst = 4;
	sunxi_daudio->capture_dma_param.dst_maxburst = 4;
	sunxi_daudio->capture_dma_param.dma_addr =
			(dma_addr_t)platform->mem_base + SUNXI_DAUDIO_RXFIFO;
	switch (sunxi_daudio->param.tdm_num) {
	case 0:
		SUNXI_DAUDIO_DRQDST(sunxi_daudio, 0);
		SUNXI_DAUDIO_DRQSRC(sunxi_daudio, 0);
		break;
	case 1:
		SUNXI_DAUDIO_DRQDST(sunxi_daudio, 1);
		SUNXI_DAUDIO_DRQSRC(sunxi_daudio, 1);
		break;
	case 2:
		SUNXI_DAUDIO_DRQDST(sunxi_daudio, 2);
		SUNXI_DAUDIO_DRQSRC(sunxi_daudio, 2);
		break;
	default:
		snd_err("tdm_num:%u overflow\n", sunxi_daudio->param.tdm_num);
		ret = -EFAULT;
		goto err1;
	}
err1:

	return ret;
}

static int sunxi_daudio_platform_remove(struct snd_platform *platform)
{
	struct sunxi_daudio_info *info;

	snd_print("\n");
	info = platform->private_data;
	if (!info)
		return 0;
	snd_free(info);
	platform->private_data = NULL;
	return 0;
}


#if 0
struct snd_platform gDaudio0Platform = {
	.name		= "daudio0-cpudai",
	.ops		= &sunxi_pcm_ops,
	.pcm_new	= sunxi_pcm_new,
	.pcm_free	= sunxi_pcm_free,
	.cpu_dai	= &sunxi_daudio_dai,
	.private_data	= NULL,
};
#endif

#define DAUDIO_NAME_LEN		(16)
int snd_platform_daudio_register(struct snd_platform *platform,
					int num)
{
	if (num >= DAUDIO_NUM_MAX) {
		snd_err("num=%d, but the number of daudio is %d\n",
			num, DAUDIO_NUM_MAX);
		return -1;
	}
	platform->name = snd_malloc(DAUDIO_NAME_LEN);
	if (!platform->name) {
		snd_err("no memory\n");
		return -ENOMEM;
	}
	snprintf(platform->name, DAUDIO_NAME_LEN, "daudio%d-cpudai", num);
	platform->ops = &sunxi_pcm_ops;
	platform->pcm_new = sunxi_pcm_new;
	platform->pcm_free = sunxi_pcm_free;
	platform->cpu_dai = &sunxi_daudio_dai;
	platform->probe = sunxi_daudio_platform_probe;
	platform->remove = sunxi_daudio_platform_remove;

	return 0;
}

/*#define SUNXI_DAUDIO_DEBUG_REG*/

#ifdef SUNXI_DAUDIO_DEBUG_REG

#define REG_LABEL(constant)		{#constant, constant}
#define REG_LABEL_END			{NULL, 0}
static struct daudio_label {
	const char *name;
	const unsigned int address;
	/*int value;*/
} reg_labels[] = {
	REG_LABEL(SUNXI_DAUDIO_CTL),
	REG_LABEL(SUNXI_DAUDIO_FMT0),
	REG_LABEL(SUNXI_DAUDIO_FMT1),
	REG_LABEL(SUNXI_DAUDIO_INTSTA),
	REG_LABEL(SUNXI_DAUDIO_FIFOCTL),
	REG_LABEL(SUNXI_DAUDIO_FIFOSTA),
	REG_LABEL(SUNXI_DAUDIO_INTCTL),
	REG_LABEL(SUNXI_DAUDIO_CLKDIV),
	REG_LABEL(SUNXI_DAUDIO_TXCNT),
	REG_LABEL(SUNXI_DAUDIO_RXCNT),
	REG_LABEL(SUNXI_DAUDIO_CHCFG),
	REG_LABEL(SUNXI_DAUDIO_TX0CHSEL),
	REG_LABEL(SUNXI_DAUDIO_TX0CHMAP0),
	REG_LABEL(SUNXI_DAUDIO_TX0CHMAP1),
	REG_LABEL(SUNXI_DAUDIO_TX1CHMAP0),
	REG_LABEL(SUNXI_DAUDIO_TX1CHMAP1),
	REG_LABEL(SUNXI_DAUDIO_TX2CHMAP0),
	REG_LABEL(SUNXI_DAUDIO_TX2CHMAP1),
	REG_LABEL(SUNXI_DAUDIO_TX3CHMAP0),
	REG_LABEL(SUNXI_DAUDIO_TX3CHMAP1),
	REG_LABEL(SUNXI_DAUDIO_RXCHSEL),
	REG_LABEL(SUNXI_DAUDIO_RXCHMAP0),
	REG_LABEL(SUNXI_DAUDIO_RXCHMAP1),
	REG_LABEL(SUNXI_DAUDIO_RXCHMAP2),
	REG_LABEL(SUNXI_DAUDIO_RXCHMAP3),
	REG_LABEL(SUNXI_DAUDIO_DEBUG),
	REG_LABEL_END,
};

/* for debug */
#include <console.h>
int cmd_daudio_dump(int argc, char *argv[])
{
	int daudio_num = 0;
	void *membase;
	int i = 0;

	if (argc == 2) {
		daudio_num = atoi(argv[1]);
	}
	membase = (void *)SUNXI_DAUDIO_MEMBASE + (0x1000 * daudio_num);

	while (reg_labels[i].name != NULL) {
		printf("%-20s[0x%03x]: 0x%-10x\n",
			reg_labels[i].name,
			reg_labels[i].address,
			snd_readl(membase + reg_labels[i].address));
		i++;
	}
}
FINSH_FUNCTION_EXPORT_CMD(cmd_daudio_dump, daudio, daudio dump reg);
#endif /* SUNXI_DAUDIO_DEBUG_REG */
