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

#include "bt_app_audio.h"

#include "kernel/os/os_mutex.h"

#include "bt_app_core.h"
#include "aw_bt_a2dp_api.h"
#include "aw_bt_avrcp_api.h"
#include "aw_bt_avrcp_app.h"

#include "aw_bt_avrcp_api.h"
#include "sys_ctrl/config.h"
#define loge(format, arg...) printf("Err : <%s : %d> " format, __func__, __LINE__, ##arg)
#define logw(format, arg...) printf("Warn: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logi(format, arg...) printf("Info: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logd(format, arg...) printf("Dbg : <%s : %d> " format, __func__, __LINE__, ##arg)

static void bt_app_alloc_meta_buffer(t_bt_avrcp_cb_param *param)
{
    t_bt_avrcp_cb_param *rc = (t_bt_avrcp_cb_param *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);
    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;

    rc->meta_rsp.attr_text = attr_text;
}

static void bt_av_new_track()
{
    //Register notifications and request metadata
    bt_avrcp_send_metadata_cmd(0, XR_AVRC_MD_ATTR_TITLE | XR_AVRC_MD_ATTR_ARTIST | XR_AVRC_MD_ATTR_ALBUM | XR_AVRC_MD_ATTR_GENRE);
    bt_avrcp_send_register_notification_cmd(1, XR_AVRC_RN_TRACK_CHANGE, 0);
}

static void bt_av_notify_evt_handler(uint8_t event_id, uint32_t event_parameter)
{
    switch (event_id) {
    case XR_AVRC_RN_TRACK_CHANGE: {
        bt_av_new_track();
        break;
    }

    case XR_AVRC_RN_VOLUME_CHANGE: {
        logi("AVRC_RN_VOLUME_CHANGE volume[%d]\n", event_parameter);
        t_bt_avrcp_rn_rsp rsp_param;
        rsp_param.volume = bt_avrcp_handle_abs_vol(BT_AVRCP_GET_ABS_VOL, 0);
        bt_avrcp_send_register_notification_rsp(event_id, AVRCP_NOTIFICATION_TYPE_INTERIM, rsp_param);
        break;
    }

    default:
        loge("%s unhandled notify evt %d\n", __func__, event_id);
    }
}

static void bt_avrcp_handle_event(uint16_t event, void *p_param)
{
    logd("%s evt %d\n", __func__, event);
    t_bt_avrcp_cb_param *rc = (t_bt_avrcp_cb_param *)(p_param);
    switch (event) {	
    case XR_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        logi("AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]\n",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (rc->conn_stat.connected) {
            bt_av_new_track();
        }
        break;
    }
    case XR_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        logi("AVRC passthrough rsp: key_code 0x%x, key_state %d\n", rc->psth_rsp.key_code, rc->psth_rsp.key_state);
        break;
    }
    case XR_AVRC_CT_METADATA_RSP_EVT: {
		bt_app_alloc_meta_buffer(rc);
        logi("AVRC metadata rsp: attribute id 0x%x\n", rc->meta_rsp.attr_id);
        free(rc->meta_rsp.attr_text);
        break;
    }
    case XR_AVRC_CT_CHANGE_NOTIFY_EVT: {
        logi("AVRC event notification: %d, param: %d\n", rc->change_ntf.event_id, rc->change_ntf.event_parameter);
        bt_av_notify_evt_handler(rc->change_ntf.event_id, rc->change_ntf.event_parameter);
        break;
    }
    case XR_AVRC_CT_REMOTE_FEATURES_EVT: {
        logi("AVRC remote features %x\n", rc->rmt_feats.feat_mask);
        break;
    }
    case XR_AVRC_CT_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        logi("AVRC remote ctrl tl[%d] volume[%x]\n", rc->abs_volume.tl, rc->abs_volume.volume);
        rc->abs_volume.volume = bt_avrcp_handle_abs_vol(BT_AVRCP_SET_ABS_VOL, rc->abs_volume.volume);
        bt_avrcp_send_abs_vol_rsp(rc->abs_volume.tl, XR_AVRC_RSP_ACCEPT, rc->abs_volume.volume);
        break;
    }
    default:
        loge("%s unhandled evt %d\n", __func__, event);
        break;
    }
}

static void bt_app_avcp_cb_handle(t_bt_avrcp_cb_event event, t_bt_avrcp_cb_param *param)
{
	bt_app_work_dispatch(bt_avrcp_handle_event, event, param, sizeof(t_bt_avrcp_cb_param), NULL);
}

void bt_app_avrcp_demo()
{
	//initialize AVRCP controller
	bt_avrcp_init();
	bt_avrcp_reg_cb(bt_app_avcp_cb_handle);
}