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

#ifndef __AW_BT_AVRCP_API_H__
#define __AW_BT_AVRCP_API_H__

#include <stdint.h>
#include <stdbool.h>

#include "aw_bt_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XR_AVRC_MAX_VOLUME 0x7F
// AVRC feature bit mask
typedef enum {
    XR_AVRC_FEAT_RCTG = 0x0001,                 /*!< remote control target */
    XR_AVRC_FEAT_RCCT = 0x0002,                 /*!< remote control controller */
    XR_AVRC_FEAT_VENDOR = 0x0008,               /*!< remote control vendor dependent commands */
    XR_AVRC_FEAT_BROWSE = 0x0010,               /*!< use browsing channel */
    XR_AVRC_FEAT_META_DATA = 0x0040,            /*!< remote control metadata transfer command/response */
    XR_AVRC_FEAT_ADV_CTRL = 0x0200,             /*!< remote control advanced control commmand/response */
} t_bt_avrcp_features;

// AVRC passthrough command code
typedef enum {
    XR_AVRC_PT_CMD_VOL_UP   =  0x41,            /*!< volume up */
    XR_AVRC_PT_CMD_VOL_DOWN =  0x42,            /*!< volume down */
    XR_AVRC_PT_CMD_MUTE     =  0x43,            /*!< mute */

    XR_AVRC_PT_CMD_PLAY = 0x44,                 /*!< play */
    XR_AVRC_PT_CMD_STOP = 0x45,                 /*!< stop */
    XR_AVRC_PT_CMD_PAUSE = 0x46,                /*!< pause */
    XR_AVRC_PT_CMD_FORWARD = 0x4B,              /*!< forward */
    XR_AVRC_PT_CMD_BACKWARD = 0x4C,             /*!< backward */
    XR_AVRC_PT_CMD_REWIND = 0x48,               /*!< rewind */
    XR_AVRC_PT_CMD_FAST_FORWARD = 0x49          /*!< fast forward */
} t_bt_avrcp_cmd;

// AVRC passthrough command state
typedef enum {
    XR_AVRC_PT_CMD_STATE_PRESSED = 0,           /*!< key pressed */
    XR_AVRC_PT_CMD_STATE_RELEASED = 1           /*!< key released */
} t_bt_avrcp_cmd_state;

// AVRC Controller callback events
typedef enum {
    XR_AVRC_CT_CONNECTION_STATE_EVT = 0,        /*!< connection state changed event */
    XR_AVRC_CT_PASSTHROUGH_RSP_EVT = 1,         /*!< passthrough response event */
    XR_AVRC_CT_METADATA_RSP_EVT = 2,            /*!< metadata response event */
    XR_AVRC_CT_PLAY_STATUS_RSP_EVT = 3,         /*!< play status response event */
    XR_AVRC_CT_CHANGE_NOTIFY_EVT = 4,           /*!< notification event */
    XR_AVRC_CT_REMOTE_FEATURES_EVT = 5,         /*!< feature of remote device indication event */

    XR_AVRC_CT_PASSTHROUGH_CMD_EVT,             /*!< passthrough cmd event */
    XR_AVRC_CT_SET_ABSOLUTE_VOLUME_CMD_EVT,     /*!< set absolute volume cmd event */
    XR_AVRC_CT_MAX_CMD_EVT
} t_bt_avrcp_cb_event;

// AVRC metadata attribute mask
typedef enum {
    XR_AVRC_MD_ATTR_TITLE = 0x1,                 /*!< title of the playing track */
    XR_AVRC_MD_ATTR_ARTIST = 0x2,                /*!< track artist */
    XR_AVRC_MD_ATTR_ALBUM = 0x4,                 /*!< album name */
    XR_AVRC_MD_ATTR_TRACK_NUM = 0x8,             /*!< track position on the album */
    XR_AVRC_MD_ATTR_NUM_TRACKS = 0x10,           /*!< number of tracks on the album */
    XR_AVRC_MD_ATTR_GENRE = 0x20,                /*!< track genre */
    XR_AVRC_MD_ATTR_PLAYING_TIME = 0x40          /*!< total album playing time in miliseconds */
} t_bt_avrcp_md_attr_mask;

// AVRC event notification ids
typedef enum {
    XR_AVRC_RN_PLAY_STATUS_CHANGE = 0x01,        /*!< track status change, eg. from playing to paused */
    XR_AVRC_RN_TRACK_CHANGE = 0x02,              /*!< new track is loaded */
    XR_AVRC_RN_TRACK_REACHED_END = 0x03,         /*!< current track reached end */
    XR_AVRC_RN_TRACK_REACHED_START = 0x04,       /*!< current track reached start position */
    XR_AVRC_RN_PLAY_POS_CHANGED = 0x05,          /*!< track playing position changed */
    XR_AVRC_RN_BATTERY_STATUS_CHANGE = 0x06,     /*!< battery status changed */
    XR_AVRC_RN_SYSTEM_STATUS_CHANGE = 0x07,      /*!< system status changed */
    XR_AVRC_RN_APP_SETTING_CHANGE = 0x08,        /*!< application settings changed */

    /* added in AVRCP 1.4 */
    XR_AVRC_RN_NOW_PLAYING_CHANGE = 0x09,        /*!< now playing changed */
    XR_AVRC_RN_AVAL_PLAYERS_CHANGE = 0x0a,       /*!< aval players changed */
    XR_AVRC_RN_ADDR_PLAYER_CHANGE = 0x0b,        /*!< addr player changed */
    XR_AVRC_RN_UIDS_CHANGE = 0x0c,               /*!< uids changed */
    XR_AVRC_RN_VOLUME_CHANGE = 0x0d,             /*!< volume changed */
    XR_AVRC_RN_MAX_EVT
} t_bt_avrcp_rn_event_ids;

typedef enum {
    AVRCP_NOTIFICATION_TYPE_INTERIM = 0,
    AVRCP_NOTIFICATION_TYPE_CHANGED = 1,
} t_bt_avrcp_notification_type;

typedef enum {
    XR_AVRC_RSP_NOT_IMPL = 8,
    XR_AVRC_RSP_ACCEPT = 9,
    XR_AVRC_RSP_REJ = 10,
} t_bt_avrcp_rsp_code;

typedef enum {
    BT_AVRCP_SET_ABS_VOL = 0,
    BT_AVRCP_GET_ABS_VOL,
    BT_AVRCP_UP_ABS_VOL,
    BT_AVRCP_DOWN_ABS_VOL,
} t_bt_avrcp_abs_vol_act;

typedef union {
    uint8_t volume;
    uint32_t param;
} t_bt_avrcp_rn_rsp;

// AVRC player setting ids
typedef enum {
    XR_AVRC_PS_EQUALIZER = 0x01,                 /*!< equalizer, on or off */
    XR_AVRC_PS_REPEAT_MODE = 0x02,               /*!< repeat mode */
    XR_AVRC_PS_SHUFFLE_MODE = 0x03,              /*!< shuffle mode */
    XR_AVRC_PS_SCAN_MODE = 0x04,                 /*!< scan mode on or off */
    XR_AVRC_PS_MAX_ATTR
} t_bt_avrcp_ps_attr_ids;

// AVRC equalizer modes
typedef enum {
    XR_AVRC_PS_EQUALIZER_OFF = 0x1,              /*!< equalizer OFF */
    XR_AVRC_PS_EQUALIZER_ON = 0x2                /*!< equalizer ON */
} t_bt_avrcp_ps_eq_value_ids;

typedef enum {
    XR_AVRC_PS_REPEAT_OFF = 0x1,                /*!< repeat mode off */
    XR_AVRC_PS_REPEAT_SINGLE = 0x2,            	/*!< single track repeat */
    XR_AVRC_PS_REPEAT_GROUP = 0x3              	/*!< group repeat */
} t_bt_avrcp_ps_rpt_value_ids;				/// AVRC repeat modes


// AVRC shuffle modes
typedef enum {
    XR_AVRC_PS_SHUFFLE_OFF = 0x1,                /*<! shuffle off */
    XR_AVRC_PS_SHUFFLE_ALL = 0x2,                /*<! shuffle all tracks */
    XR_AVRC_PS_SHUFFLE_GROUP = 0x3               /*<! group shuffle */
} t_bt_avrcp_ps_shf_value_ids;

// AVRC scan modes
typedef enum {
    XR_AVRC_PS_SCAN_OFF = 0x1,                   /*!< scan off */
    XR_AVRC_PS_SCAN_ALL = 0x2,                   /*!< all tracks scan */
    XR_AVRC_PS_SCAN_GROUP = 0x3                  /*!< group scan */
} t_bt_avrcp_ps_scn_value_ids;

// AVRC controller callback parameters
typedef union {
    struct avrcp_ct_conn_stat_param {			//XR_AVRC_CT_CONNECTION_STATE_EVT
        bool connected;                         /*!< whether AVRC connection is set up */
        t_xr_bd_addr remote_bda;               /*!< remote bluetooth device address */
    } conn_stat;                                /*!< AVRC connection status */

    struct avrcp_ct_psth_rsp_param {			//XR_AVRC_CT_PASSTHROUGH_RSP_EVT
        uint8_t tl;                              /*!< transaction label, 0 to 15 */
        uint8_t key_code;                        /*!< passthrough command code */
        uint8_t key_state;                       /*!< 0 for PRESSED, 1 for RELEASED */
    } psth_rsp;                                  /*!< passthrough command response */

    struct avrcp_ct_meta_rsp_param {			//XR_AVRC_CT_METADATA_RSP_EVT
        uint8_t attr_id;                         /*!< id of metadata attribute */
        uint8_t *attr_text;                      /*!< attribute itself */
        int attr_length;                         /*!< attribute character length */
    } meta_rsp;                                  /*!< metadata attributes response */

    struct avrcp_ct_change_notify_param {		//XR_AVRC_CT_CHANGE_NOTIFY_EVT
        uint8_t event_id;                        /*!< id of AVRC event notification */
        uint32_t event_parameter;                /*!< event notification parameter */
    } change_ntf;                                /*!< notifications */

    struct avrcp_ct_rmt_feats_param {			//XR_AVRC_CT_REMOTE_FEATURES_EVT
        uint32_t feat_mask;                      /*!< AVRC feature mask of remote device */
        t_xr_bd_addr remote_bda;                /*!< remote bluetooth device address */
    } rmt_feats;                                 /*!< AVRC features discovered from remote SDP server */

    struct avrcp_ct_set_abs_volume_param {	//XR_AVRC_CT_SET_ABSOLUTE_VOLUME_CMD_EVT
        uint8_t tl;                              /*!< transaction label, 0 to 15 */
        uint8_t volume;                          /*!< volume parameter */
    } abs_volume;                                /*!< AVRC absolute volume */
} t_bt_avrcp_cb_param;

typedef void (* t_bt_avrcp_callback)(t_bt_avrcp_cb_event event, t_bt_avrcp_cb_param *param);

int32_t bt_avrcp_init(void);

int32_t bt_avrcp_deinit(void);

int32_t bt_avrcp_reg_cb(t_bt_avrcp_callback callback);

int32_t bt_avrcp_send_register_notification_cmd(uint8_t tl, uint8_t event_id, uint32_t event_parameter);

//Send metadata command to AVRCP target, called this function after CONNECTION_STATE_EVT
int32_t bt_avrcp_send_metadata_cmd(uint8_t tl, uint8_t attr_mask);

int32_t bt_avrcp_send_register_notification_rsp(t_bt_avrcp_rn_event_ids event_id,
                            t_bt_avrcp_notification_type type, t_bt_avrcp_rn_rsp p_param);

int32_t bt_avrcp_send_abs_vol_rsp(uint8_t tl, t_bt_avrcp_rsp_code rsp_code, uint8_t abs_vol);

uint8_t bt_avrcp_handle_abs_vol(uint8_t cmd_id, uint8_t vol);


#ifdef __cplusplus
}
#endif

#endif /* __XR_AVRC_API_H__ */
