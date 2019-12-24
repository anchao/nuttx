#include <snd_core.h>

#ifdef CONFIG_SND_CODEC_SUN8IW18_AUDIOCODEC
extern struct snd_codec sun8iw18_codec;
#endif
#ifdef CONFIG_SND_CODEC_DUMMY
extern struct snd_codec dummy_codec;
#endif
#ifdef CONFIG_SND_CODEC_AC108
extern struct snd_codec ac108_codec;
#endif
#ifdef CONFIG_SND_CODEC_AC107
extern struct snd_codec ac107_codec;
#endif
#ifdef CONFIG_SND_CODEC_TAS5805
extern struct snd_codec tas5805_codec;
#endif

__attribute__((weak)) void snd_pcm_version(void)
{

}

int sunxi_soundcard_init(void)
{
	int ret = 0;

	snd_core_version();
	snd_pcm_version();
#ifdef CONFIG_SND_CODEC_SUN8IW18_AUDIOCODEC
	ret = snd_card_register("audiocodec", &sun8iw18_codec, SND_PLATFORM_TYPE_CPUDAI);
	if (ret != 0)
		return ret;
#endif

#if defined(CONFIG_SND_PLATFORM_SUNXI_DAUDIO) && defined(CONFIG_SND_CODEC_DUMMY)
	ret = snd_card_register("snddaudio0", &dummy_codec, SND_PLATFORM_TYPE_DAUDIO0);
	if (ret != 0)
		return ret;
#endif

#if defined(CONFIG_SND_PLATFORM_SUNXI_DAUDIO) && defined(CONFIG_SND_CODEC_AC107)
	ret = snd_card_register("ac107", &ac107_codec, SND_PLATFORM_TYPE_DAUDIO1);
	if (ret != 0)
		return ret;
#endif

#if defined(CONFIG_SND_PLATFORM_SUNXI_DAUDIO) && defined(CONFIG_SND_CODEC_TAS5805)
	ret = snd_card_register("tas5805", &tas5805_codec, SND_PLATFORM_TYPE_DAUDIO1);
	if (ret != 0)
		return ret;
#endif
	return ret;
}

