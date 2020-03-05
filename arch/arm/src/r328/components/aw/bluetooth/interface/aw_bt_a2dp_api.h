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

#ifndef __AW_BT_A2DP_API_H__
#define __AW_BT_A2DP_API_H__

#include "aw_bt_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Media codec types supported by A2DP
#define XR_A2DP_MCT_SBC         (0)             /*!< SBC */
#define XR_A2DP_MCT_M12         (0x01)          /*!< MPEG-1, 2 Audio */
#define XR_A2DP_MCT_M24         (0x02)          /*!< MPEG-2, 4 AAC */
#define XR_A2DP_MCT_ATRAC       (0x04)          /*!< ATRAC family */
#define XR_A2DP_MCT_NON_A2DP    (0xff)

typedef uint8_t t_bt_a2dp_mct;

/// A2DP media codec capabilities union
typedef struct {
    t_bt_a2dp_mct type;                        /*!< A2DP media codec type */
#define XR_A2DP_CIE_LEN_SBC          (4)
#define XR_A2DP_CIE_LEN_M12          (4)
#define XR_A2DP_CIE_LEN_M24          (6)
#define XR_A2DP_CIE_LEN_ATRAC        (7)
    union {
        uint8_t sbc[XR_A2DP_CIE_LEN_SBC];
        uint8_t m12[XR_A2DP_CIE_LEN_M12];
        uint8_t m24[XR_A2DP_CIE_LEN_M24];
        uint8_t atrac[XR_A2DP_CIE_LEN_ATRAC];
    } cie;                                     /*!< A2DP codec information element */
} __attribute__((packed)) t_bt_a2dp_mcc;

/// Bluetooth A2DP connection states
typedef enum {
    XR_A2DP_CONNECTION_STATE_DISCONNECTED = 0, /*!< connection released  */
    XR_A2DP_CONNECTION_STATE_CONNECTING,       /*!< connecting remote device */
    XR_A2DP_CONNECTION_STATE_CONNECTED,        /*!< connection established */
    XR_A2DP_CONNECTION_STATE_DISCONNECTING     /*!< disconnecting remote device */
} t_bt_a2dp_connection_state;

/// Bluetooth A2DP disconnection reason
typedef enum {
    XR_A2DP_DISC_RSN_NORMAL = 0,               /*!< Finished disconnection that is initiated by local or remote device */
    XR_A2DP_DISC_RSN_ABNORMAL                  /*!< Abnormal disconnection caused by signal loss */
} t_bt_a2dp_disc_rsn;

/// Bluetooth A2DP datapath states
typedef enum {
    XR_A2DP_AUDIO_STATE_REMOTE_SUSPEND = 0,    /*!< audio stream datapath suspended by remote device */
    XR_A2DP_AUDIO_STATE_STOPPED,               /*!< audio stream datapath stopped */
    XR_A2DP_AUDIO_STATE_STARTED,               /*!< audio stream datapath started */
} t_bt_a2dp_audio_state;

/// A2DP media control command acknowledgement code
typedef enum {
    XR_A2DP_MEDIA_CTRL_ACK_SUCCESS = 0,        /*!< media control command is acknowledged with success */
    XR_A2DP_MEDIA_CTRL_ACK_FAILURE,            /*!< media control command is acknowledged with failure */
    XR_A2DP_MEDIA_CTRL_ACK_BUSY,               /*!< media control command is rejected, as previous command is not yet acknowledged */
} t_bt_a2dp_media_ctrl_ack;

/// A2DP media control commands
typedef enum {
    XR_A2DP_MEDIA_CTRL_NONE = 0,               /*!< dummy command */
    XR_A2DP_MEDIA_CTRL_CHECK_SRC_RDY,          /*!< check whether AVDTP is connected, only used in A2DP source */
    XR_A2DP_MEDIA_CTRL_START,                  /*!< command to set up media transmission channel */
    XR_A2DP_MEDIA_CTRL_STOP,                   /*!< command to stop media transmission */
    XR_A2DP_MEDIA_CTRL_SUSPEND,                /*!< command to suspend media transmission  */
} t_bt_a2dp_media_ctrl;

/// A2DP callback events
typedef enum {
    XR_A2DP_CONNECTION_STATE_EVT = 0,          /*!< connection state changed event */
    XR_A2DP_AUDIO_STATE_EVT,                   /*!< audio stream transmission state changed event */
    XR_A2DP_AUDIO_CFG_EVT,                     /*!< audio codec is configured, only used for A2DP SINK */
    XR_A2DP_MEDIA_CTRL_ACK_EVT,                /*!< acknowledge event in response to media control commands */
} t_bt_a2dp_cb_event;

/// A2DP state callback parameters
typedef union {
    struct bt_a2dp_conn_stat_param {			//XR_A2DP_CONNECTION_STATE_EVT
        t_bt_a2dp_connection_state state;      /*!< one of values from t_bt_a2dp_connection_state */
        t_xr_bd_addr remote_bda;              /*!< remote bluetooth device address */
        t_bt_a2dp_disc_rsn disc_rsn;           /*!< reason of disconnection for "DISCONNECTED" */
    } conn_stat;                               /*!< A2DP connection status */

    struct bt_a2dp_audio_stat_param {			//XR_A2DP_AUDIO_STATE_EVT
        t_bt_a2dp_audio_state state;           /*!< one of the values from t_bt_a2dp_connection_state */
        t_xr_bd_addr remote_bda;              /*!< remote bluetooth device address */
    } audio_stat;                              /*!< audio stream playing state */
	
    struct bt_a2dp_audio_cfg_param {				//XR_A2DP_AUDIO_CFG_EVT
        t_xr_bd_addr remote_bda;              /*!< remote bluetooth device address */
        t_bt_a2dp_mcc mcc;                     /*!< A2DP media codec capability information */
    } audio_cfg;                               /*!< media codec configuration information */

    struct bt_media_ctrl_stat_param {			//XR_A2DP_MEDIA_CTRL_ACK_EVT
        t_bt_a2dp_media_ctrl cmd;              /*!< media control commands to acknowledge */
        t_bt_a2dp_media_ctrl_ack status;       /*!< acknowledgement to media control commands */
    } media_ctrl_stat;                         /*!< status in acknowledgement to media control commands */
} t_bt_a2dp_cb_param;

typedef void (* t_bt_a2dp_cb)(t_bt_a2dp_cb_event event, t_bt_a2dp_cb_param *param);

typedef void (* t_bt_a2dp_sink_data_cb)(const uint8_t *buf, uint32_t len);

int32_t bt_a2dp_reg_cb(t_bt_a2dp_cb callback);

int32_t bt_a2dp_sink_reg_data_cb(t_bt_a2dp_sink_data_cb callback);

int32_t bt_a2dp_sink_init();

int32_t bt_a2dp_sink_connect(t_xr_bd_addr remote_bda);

int32_t bt_a2dp_sink_disconnect(t_xr_bd_addr remote_bda);

#ifdef __cplusplus
}
#endif


#endif /* __XR_A2DP_API_H__ */
