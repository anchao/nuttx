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
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <aw-alsa-lib/pcm.h>
#include "common.h"
#include "wav_parser.h"
#include "audio_wav/audio_wav.h"

static unsigned int g_playback_loop_enable = 0;
static char *g_pcm_name;
extern unsigned int g_verbose;

#if 0
static int fs_read(int fd, void *buf, size_t count)
{
	ssize_t result = 0, res;

	while (count > 0) {
		if ((res = read(fd, buf, count)) == 0)
			break;
		if (res < 0)
			return result > 0 ? result : res;
		count -= res;
		result += res;
		buf = (char *)buf + res;
	}
	return result;
}
#endif

static int pcm_write(snd_pcm_t *handle, char *data, snd_pcm_uframes_t frames_total, unsigned int frame_bytes)
{
	snd_pcm_sframes_t size;
	snd_pcm_uframes_t frames_loop = 400;
	snd_pcm_uframes_t frames_count = 0;
	snd_pcm_uframes_t frames = 0;

	while (1) {
		if ((frames_total - frames_count) < frames_loop)
			frames = frames_total - frames_count;
		if (frames == 0)
			frames = frames_loop;
		size = snd_pcm_writei(handle, data, frames);
		if (size != frames) {
			printf("snd_pcm_writei return %ld\n", size);
		}
		if (size == -EAGAIN) {
			usleep(10000);
			continue;
		} else if (size == -EPIPE) {
			xrun(handle);
			continue;
		} else if (size == -ESTRPIPE) {

			continue;
		} else if (size < 0) {
			printf("-----snd_pcm_writei failed!!, return %ld\n", size);
			return size;
		}
		data += (size * frame_bytes);
		frames_count += size;
		frames -= size;
		if (frames_total == frames_count)
			break;
		/*printf("frames_count = %ld, frames_total = %ld\n", frames_count, frames_total);*/
	}

	return frames_count;
}



/*
 * arg0: aplay
 * arg1: card
 * arg2: format
 * arg3: rate
 * arg4: channels
 * arg5: data
 * arg6: len
 */
int aplay(const char *card_name, snd_pcm_format_t format, unsigned int rate,
			unsigned int channels, const char *data, unsigned int datalen)
{
	int ret = 0;
	snd_pcm_t *handle;
	int mode = 0;
	snd_pcm_uframes_t period_frames = 1024, buffer_frames = 4096;

	printf("dump args:\n");
	printf("card:        %s\n", card_name);
	printf("format:      %u\n", format);
	printf("rate:        %u\n", rate);
	printf("channels:    %u\n", channels);
	printf("data:        %p\n", data);
	printf("datalen:     %u\n", datalen);
	printf("period_size: %u\n", period_frames);
	printf("buffer_size: %u\n", buffer_frames);

	/* open card */
	ret = snd_pcm_open(&handle, card_name, SND_PCM_STREAM_PLAYBACK, mode);
	if (ret < 0) {
		printf("audio open error:%d\n", ret);
		return -1;
	}

	ret = set_param(handle, format, rate, channels, period_frames, buffer_frames);
	if (ret < 0)
		goto err1;

	ret = pcm_write(handle, (char *)data,
			snd_pcm_bytes_to_frames(handle, datalen),
			snd_pcm_frames_to_bytes(handle, 1));
	if (ret < 0) {
		printf("pcm_write error:%d\n", ret);
		goto err1;
	}

	ret = snd_pcm_drain(handle);
	/*ret = snd_pcm_drop(handle);*/
	if (ret < 0)
		printf("stop failed!, return %d\n", ret);
err1:
	/* close card */
	ret = snd_pcm_close(handle);
	if (ret < 0) {
		printf("audio close error:%d\n", ret);
		return ret;
	}
	return ret;
}

static int play_builtin_music(audio_mgr_t *mgr, const char *name)
{
	int ret = 0;
	wav_hw_params_t hwparams = {16000, SND_PCM_FORMAT_UNKNOWN, 2};
	wav_file_t *wav_file = NULL;
	wav_header_t *wav_header = NULL;

	if (name)
		wav_file = find_builtin_wav_file(name);
	else
		wav_file = find_builtin_wav_file(NULL);
	if (!wav_file) {
		printf("can't find wav file\n");
		return -1;
	}
	wav_header = (wav_header_t *)wav_file->start;
	if (check_wav_header(wav_header, &hwparams) != 0) {
		printf("check wav header failed\n");
		return -1;
	}
	mgr->format = hwparams.format;
	mgr->rate = hwparams.rate;
	mgr->channels = hwparams.channels;

	/* open card */
	ret = snd_pcm_open(&mgr->handle, g_pcm_name, SND_PCM_STREAM_PLAYBACK, 0);
	if (ret < 0) {
		printf("audio open error:%d\n", ret);
		return -1;
	}

	ret = set_param(mgr->handle, mgr->format, mgr->rate, mgr->channels,
			mgr->period_size, mgr->buffer_size);
	if (ret < 0)
		goto err1;

	do {
		printf("pcm_write start...\n");
		ret = pcm_write(mgr->handle, (char *)wav_file->start + sizeof(wav_header_t),
				snd_pcm_bytes_to_frames(mgr->handle, wav_header->dataSize),
				snd_pcm_frames_to_bytes(mgr->handle, 1));
		if (ret < 0) {
			printf("pcm_write error:%d\n", ret);
			goto err1;
		}
	} while (g_playback_loop_enable);

	ret = snd_pcm_drain(mgr->handle);
	if (ret < 0)
		printf("stop failed!, return %d\n", ret);
err1:
	/* close card */
	ret = snd_pcm_close(mgr->handle);
	if (ret < 0) {
		printf("audio close error:%d\n", ret);
		return ret;
	}
	return ret;
}

static int play_fs_music(audio_mgr_t *mgr, const char *path)
{
	int ret = 0, fd = 0;
	wav_header_t wav_header;
	wav_hw_params_t wav_hwparams = {16000, SND_PCM_FORMAT_UNKNOWN, 2};
	unsigned int c, written = 0, count;
	unsigned int chunk_bytes, frame_bytes = 0;
	ssize_t r = 0;
	char *audiobuf = NULL;

	fd = open(path, O_RDONLY);
	if (fd < 0) {
		printf("no such wav file\n");
		return -1;
	}
	r = read(fd, &wav_header, sizeof(wav_header_t));
	if (r != sizeof(wav_header_t)) {
		printf("read wav file header failed, return %d\n", r);
		goto err;
	}

	if (check_wav_header(&wav_header, &wav_hwparams) != 0) {
		printf("check wav header failed\n");
		goto err;
	}

	/* open card */
	ret = snd_pcm_open(&mgr->handle, g_pcm_name, SND_PCM_STREAM_PLAYBACK, 0);
	if (ret < 0) {
		printf("audio open error:%d\n", ret);
		goto err;
	}
	mgr->format = wav_hwparams.format;
	mgr->rate = wav_hwparams.rate;
	mgr->channels = wav_hwparams.channels;

	ret = set_param(mgr->handle, mgr->format, mgr->rate, mgr->channels,
			mgr->period_size, mgr->buffer_size);
	if (ret < 0)
		goto err;

	count = wav_header.dataSize;
	frame_bytes = snd_pcm_frames_to_bytes(mgr->handle, 1);
	chunk_bytes = snd_pcm_frames_to_bytes(mgr->handle, mgr->period_size);

	audiobuf = malloc(chunk_bytes);
	if (!audiobuf) {
		printf("no memory...\n");
		goto err;
	}
	while (written < count) {
		c = count - written;
		if (c > chunk_bytes)
			c = chunk_bytes;
		r = read(fd, audiobuf, c);
		if (r < 0 || r != c) {
			printf("read file error, r=%d,c=%u\n", r, c);
			break;
		}
		r = pcm_write(mgr->handle, audiobuf, r/frame_bytes, frame_bytes);
		if (r != c/frame_bytes)
			break;
		written += c;
	}
	snd_pcm_drain(mgr->handle);

	free(audiobuf);
err:
	/* close card */
	if (mgr->handle > 0)
		snd_pcm_close(mgr->handle);
	close(fd);
	return ret;
}

static void usage(void)
{
	printf("Usage: aplay [option] wav_file\n");
	printf("-D,          pcm device name\n");
	printf("-p,          period size\n");
	printf("-b,          buffer size\n");
	printf("-l,          loop playback test\n");
	printf("-s,          stop loop playback test\n");
	printf("-v,          show pcm setup\n");
	printf("\n");
	printf("builtin wav file:\n");
	wav_file_list();
}

int cmd_aplay(int argc, char ** argv)
{
	int c;
	audio_mgr_t *audio_mgr = NULL;
	g_pcm_name = "default";
	g_verbose = 0;

	audio_mgr = audio_mgr_create();
	if (!audio_mgr)
		return -1;

	optind = 0;
	while ((c = getopt(argc, argv, "D:p:b:hslv")) != -1) {
		switch (c) {
		case 'D':
			g_pcm_name = optarg;
			break;
		case 'p':
			audio_mgr->period_size = atoi(optarg);
			break;
		case 'b':
			audio_mgr->buffer_size = atoi(optarg);
			break;
		case 's':
			g_playback_loop_enable = 0;
			goto err;
		case 'l':
			g_playback_loop_enable = 1;
			break;
		case 'v':
			g_verbose = 1;
			break;
		default:
			usage();
			goto err;
		}
	}

	if (optind < argc) {
		if (play_fs_music(audio_mgr, argv[optind]) < 0) {
			play_builtin_music(audio_mgr, argv[optind]);
		}
	} else {
		play_builtin_music(audio_mgr, NULL);
	}

err:
	audio_mgr_release(audio_mgr);
	return 0;
}
//FINSH_FUNCTION_EXPORT_CMD(cmd_aplay, aplay, Play music);
