/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <aw-alsa-lib/pcm.h>
#include <aw-alsa-lib/control.h>
#include <sound/card.h>

#include "sys_ctrl/sys_ctrl.h"
#include "kernel/os/os.h"

//#include "audio/pcm/audio_pcm.h"
//#include "driver/ac100/drv_ac100.h"
#include "bt_app_audio.h"
#include "sys_ctrl/config.h"


#define BT_APP_AUDIO_DEBUG(fmt, arg...)		printf("[BT_APP_AUDIO debug] <%s : %d> " fmt "\n", __func__, __LINE__, ##arg)
#define BT_APP_AUDIO_ALERT(fmt, arg...)		printf("[BT_APP_AUDIO alert] <%s : %d> " fmt "\n", __func__, __LINE__, ##arg)
#define BT_APP_AUDIO_ERROR(fmt, arg...)		printf("[BT_APP_AUDIO error] <%s : %d> " fmt "\n", __func__, __LINE__, ##arg)

typedef enum {
	BT_AUDIO_STATES_XR871_AC100_DISABLE	= 0x00 << 0,
	BT_AUDIO_STATES_XR871_AC100_ENABLE	= 0x01 << 0,
	BT_AUDIO_STATES_XR829_AC100_DISABLE = 0x00 << 1,
	BT_AUDIO_STATES_XR829_AC100_ENABLE	= 0x01 << 1,
} bt_audio_states;

#define IS_AUDIO_XR871_AC100_ENABLE(x) (x & BT_AUDIO_STATES_XR871_AC100_ENABLE)
#define IS_AUDIO_XR871_AC100_DISABLE(x) (!(x & BT_AUDIO_STATES_XR871_AC100_ENABLE))
#define IS_AUDIO_XR829_AC100_ENABLE(x) (x & BT_AUDIO_STATES_XR829_AC100_ENABLE)
#define IS_AUDIO_XR829_AC100_DISABLE(x) (!(x & BT_AUDIO_STATES_XR829_AC100_ENABLE))


static bt_audio_states audio_state = BT_AUDIO_STATES_XR871_AC100_DISABLE
									 | BT_AUDIO_STATES_XR829_AC100_DISABLE;
static OS_Mutex_t audio_state_lock;
/*
static struct pcm_config xr871_ac100_pcm_config = {
        .channels = 2,
        .rate = 44100,
        .period_size = 1024,
        .period_count = 2,
        .format = PCM_FORMAT_S16_LE,
};

static struct pcm_config xr829_ac100_pcm_in_config = {
	.channels = 1,
	.rate = 8000,
	.format = PCM_FORMAT_S16_LE,
	.period_count = 2,
	.period_size = 512,
};

static struct pcm_config xr829_ac100_pcm_out_config = {
	.channels = 2,
	.rate = 8000,
	.format = PCM_FORMAT_S16_LE,
	.period_count = 2,
	.period_size = 512,
};

static struct pcm_config r328_pcm_config = {
	.device = NULL,
	.channels = 2,
	.rate = 44100,
	.period_size = 1024,
	.period_count = 2,
	.format = PCM_FORMAT_S16_LE,
};
*/
typedef struct {
	int stream;
	snd_pcm_format_t format;
	unsigned int channels, rate;
	snd_pcm_uframes_t period_frames, buffer_frames;
	unsigned int periods, period_time;
	snd_pcm_uframes_t type;
} hw_params_t;

static hw_params_t g_hw_params = {
	.stream 	= SND_PCM_STREAM_PLAYBACK,
	.format 	= SND_PCM_FORMAT_S16_LE,
	.channels	= 2,
	.rate		= 44100,
	.period_time	= 100000,
	.period_frames	= 512,
	.buffer_frames	= 4096,
	.periods	= 4,
};

snd_pcm_t *handle = NULL;

int bt_app_audio_init(void)
{
	audio_state = BT_AUDIO_STATES_XR871_AC100_DISABLE
				  | BT_AUDIO_STATES_XR829_AC100_DISABLE;
	//snd_pcm_init();
	
	return OS_MutexCreate(&audio_state_lock);
}

int bt_app_audio_deinit(void)
{
	return OS_MutexDelete(&audio_state_lock);
}

void bt_app_audio_config(uint32_t samplerate, uint32_t channels)
{
//	r328_pcm_config.rate = samplerate;
//	r328_pcm_config.channels = channels;
}


void bt_app_audio_ctrl(event_msg *msg)
{
	int ret = -1;
	//if (OS_MutexLock(&audio_state_lock, -1) != OS_OK) {
	if (OS_MutexLock(&audio_state_lock, 0) != OS_OK) {
		BT_APP_AUDIO_DEBUG("bt_app_audio_ctrl lock fail\n");
		BT_APP_AUDIO_ERROR("lock failed");
	}


	BT_APP_AUDIO_DEBUG("audio event: %d", msg->data);
#if 1
	switch (msg->data)
	{
		case BT_APP_AUDIO_EVENTS_A2DP_START:
			/*if (snd_pcm_open(&r328_pcm_config) != 0) {
		        BT_APP_AUDIO_ERROR("sound card open err");
		        break;
		    }*/
			BT_APP_AUDIO_DEBUG("bt_app_audio_ctrl BT_APP_AUDIO_EVENTS_A2DP_START\n");
			ret = snd_pcm_open(&handle, "default", g_hw_params.stream, 0);
			if (ret < 0) {
				BT_APP_AUDIO_ERROR("audio open error:%d\n", ret);
			}
	
			//ret = set_param(handle, format, rate, channels, period_frames, buffer_frames);
			ret = set_param(handle, SND_PCM_FORMAT_S16_LE, 44100, 1, 1024, 4096);
			if (ret < 0) {
				ret = snd_pcm_close(handle);
				if (ret < 0) {
					printf("audio close error:%d\n", ret);
				}
			}
			break;
		case BT_APP_AUDIO_EVENTS_A2DP_STOP:
			/*if (snd_pcm_close(&r328_pcm_config) != 0){
				BT_APP_AUDIO_ERROR("sound card close err");
				break;
			}*/
			ret = snd_pcm_close(handle);
			if (ret < 0) {
				BT_APP_AUDIO_ERROR("audio close error:%d\n", ret);
			}

			break;
		default:
			BT_APP_AUDIO_ERROR("bt audio request err");
			break;
	}
#endif
	OS_MutexUnlock(&audio_state_lock);
}

extern void xrun(snd_pcm_t *handle);
static int pcm_write(snd_pcm_t *_handle, char *data, snd_pcm_uframes_t frames_total, unsigned int frame_bytes)
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
		size = snd_pcm_writei(_handle, data, frames);
		if (size != frames) {
			BT_APP_AUDIO_ERROR("snd_pcm_writei return %ld\n", size);
		}
		if (size == -EAGAIN) {
			usleep(10000);
			continue;
		} else if (size == -EPIPE) {
			xrun(_handle);
			continue;
		} else if (size == -ESTRPIPE) {

			continue;
		} else if (size < 0) {
			BT_APP_AUDIO_ERROR("snd_pcm_writei failed!!, return %ld\n", size);
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

int bt_app_audio_write(const uint8_t *data, uint32_t len)
{
	int ret = 0;

	if (OS_MutexLock(&audio_state_lock, 1000) != OS_OK)
		return -1;
	//ret = snd_pcm_write(&r328_pcm_config,(uint8_t *)data, len);
	if (handle == NULL) {
		BT_APP_AUDIO_ERROR("bt_app_audio_write handle is null\n");
		 return -1;
	}
	//printf("bt_app_audio_write pcm_write\n");
	ret = pcm_write(handle, (char *)data,
			snd_pcm_bytes_to_frames(handle, len),
			snd_pcm_frames_to_bytes(handle, 1));
	if (ret < 0) {
		BT_APP_AUDIO_ERROR("pcm_write error:%d\n", ret);
	}

	//ret = snd_pcm_drain(handle);
	/*ret = snd_pcm_drop(handle);*/
	//if (ret < 0)
	//	printf("stop failed!, return %d\n", ret);

	OS_MutexUnlock(&audio_state_lock);
	return ret;
}



