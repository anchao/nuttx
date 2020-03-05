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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "kernel/os/os_mutex.h"

#include "bt_app_audio.h"
#include "bt_app_core.h"
#include "aw_bt_gap_api.h"
#include "aw_bt_a2dp_api.h"
#include "aw_bt_avrcp_app.h"
#include "sys_ctrl/config.h"
#define loge(format, arg...) printf("Err : <%s : %d> " format, __func__, __LINE__, ##arg)
#define logw(format, arg...) printf("Warn: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logi(format, arg...) printf("Info: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logd(format, arg...) printf("Dbg : <%s : %d> " format, __func__, __LINE__, ##arg)

static t_bt_a2dp_audio_state s_audio_state = XR_A2DP_AUDIO_STATE_STOPPED;
static const char *s_a2d_conn_state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
static const char *s_a2d_audio_state_str[] = {"Suspended", "Stopped", "Started"};

static void pcm_config(uint8_t * codec_info_element)
{
    int sample_rate = 16000;
    int channels = 2;
    char oct0 = codec_info_element[0];
    if (oct0 & (0x01 << 6)) {
        sample_rate = 32000;
    } else if (oct0 & (0x01 << 5)) {
        sample_rate = 44100;
    } else if (oct0 & (0x01 << 4)) {
        sample_rate = 48000;
    }

    if ((oct0 & 0x0F) == 0x08) {
        channels = 1;
    }

	bt_app_audio_config(sample_rate, channels);

    logi("pcm_config: channels:%d, sampling:%d",channels, sample_rate);
}

static void bt_a2dp_handle_event(uint16_t event, void *p_param)
{
    logd("%s evt %d\n", __func__, event);
    t_bt_a2dp_cb_param *a2d = NULL;
    switch (event) {
    case XR_A2DP_CONNECTION_STATE_EVT: {
        a2d = (t_bt_a2dp_cb_param *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        logi("A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]\n",
             s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (a2d->conn_stat.state == XR_A2DP_CONNECTION_STATE_DISCONNECTED) {
            bt_gap_set_scan_mode(XR_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        } else if (a2d->conn_stat.state == XR_A2DP_CONNECTION_STATE_CONNECTED){
            bt_gap_set_scan_mode(XR_BT_SCAN_MODE_NONE);
        }
        break;
    }
    case XR_A2DP_AUDIO_STATE_EVT: {
        a2d = (t_bt_a2dp_cb_param *)(p_param);
        logi("****A2DP audio state: %s state %d\n", s_a2d_audio_state_str[a2d->audio_stat.state], a2d->audio_stat.state);
        s_audio_state = a2d->audio_stat.state;
        if (XR_A2DP_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
			printf("==33===gptp bt-app_audio_ctrl\n");
           sys_handler_send(bt_app_audio_ctrl, BT_APP_AUDIO_EVENTS_A2DP_START, 5000);
        } else {
           sys_handler_send(bt_app_audio_ctrl, BT_APP_AUDIO_EVENTS_A2DP_STOP, 5000);
        }
        break;
    }
    case XR_A2DP_AUDIO_CFG_EVT: {
        a2d = (t_bt_a2dp_cb_param *)(p_param);
        logi("A2DP audio stream configuration, codec type %d\n", a2d->audio_cfg.mcc.type);
        if (a2d->audio_cfg.mcc.type == XR_A2DP_MCT_SBC) {
            pcm_config(a2d->audio_cfg.mcc.cie.sbc);
        }
        break;
    }
    default:
        loge("%s unhandled evt %d", __func__, event);
        break;
    }
}

/* callback for A2DP sink */
static void bt_app_a2dp_cb_handle(t_bt_a2dp_cb_event event, t_bt_a2dp_cb_param *param)
{
	bt_app_work_dispatch(bt_a2dp_handle_event, event, param, sizeof(t_bt_a2dp_cb_param), NULL);
}

static void bt_app_a2d_data_cb_handle(const uint8_t *data, uint32_t len)
{
    bt_app_audio_write((uint8_t *)data, len);
}

void bt_app_a2dp_sink_demo()
{
	/* initialize A2DP sink */
	bt_a2dp_reg_cb(&bt_app_a2dp_cb_handle);
	bt_a2dp_sink_reg_data_cb(bt_app_a2d_data_cb_handle);
	bt_a2dp_sink_init();
}