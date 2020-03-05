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

#ifndef __XR_GAP_BT_API_H__
#define __XR_GAP_BT_API_H__

#include <stdint.h>
#include "aw_bt_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/// RSSI threshold
#define XR_BT_GAP_RSSI_HIGH_THRLD  -20             /*!< High RSSI threshold */
#define XR_BT_GAP_RSSI_LOW_THRLD   -45             /*!< Low RSSI threshold */

/// Class of device
typedef struct {
    uint32_t      reserved_2: 2;                    /*!< undefined */
    uint32_t      minor: 6;                         /*!< minor class */
    uint32_t      major: 5;                         /*!< major class */
    uint32_t      service: 11;                      /*!< service class */
    uint32_t      reserved_8: 8;                    /*!< undefined */
} t_xr_bt_cod;

/// class of device settings
typedef enum {
    XR_BT_SET_COD_MAJOR_MINOR     = 0x01,          /*!< overwrite major, minor class */
    XR_BT_SET_COD_SERVICE_CLASS   = 0x02,          /*!< set the bits in the input, the current bit will remain */
    XR_BT_CLR_COD_SERVICE_CLASS   = 0x04,          /*!< clear the bits in the input, others will remain */
    XR_BT_SET_COD_ALL             = 0x08,          /*!< overwrite major, minor, set the bits in service class */
    XR_BT_INIT_COD                = 0x0a,          /*!< overwrite major, minor, and service class */
} t_xr_bt_cod_mode;

/// Discoverability and Connectability mode
typedef enum {
    XR_BT_SCAN_MODE_NONE = 0,                      /*!< Neither discoverable nor connectable */
    XR_BT_SCAN_MODE_CONNECTABLE,                   /*!< Connectable but not discoverable */
    XR_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE       /*!< both discoverable and connectable */
} t_xr_bt_scan_mode;

/// Bluetooth Device Property type
typedef enum {
    XR_BT_GAP_DEV_PROP_BDNAME = 1,                 /*!< Bluetooth device name, value type is int8 [] */
    XR_BT_GAP_DEV_PROP_COD,                        /*!< Class of Device, value type is uint32_t */
    XR_BT_GAP_DEV_PROP_RSSI,                       /*!< Received Signal strength Indication, value type is int8, ranging from -128 to 127 */
    XR_BT_GAP_DEV_PROP_EIR,                        /*!< Extended Inquiry Rt_xronse, value type is uint8 [] */
} t_xr_bt_gap_dev_propype;

/// Maximum bytes of Bluetooth device name
#define XR_BT_GAP_MAX_BDNAME_LEN             (248)

/// Maximum size of EIR Significant part
#define XR_BT_GAP_EIR_DATA_LEN               (240)

/// Bluetooth Device Property Descriptor
typedef struct {
    t_xr_bt_gap_dev_propype type;                /*!< device property type */
    int len;                                        /*!< device property value length */
    void *val;                                      /*!< device property value */
} t_xr_bt_gap_dev_prop;

/// Extended Inquiry Rt_xronse data type
typedef enum {
    XR_BT_EIR_TYPE_FLAGS                    = 0x01,     /*!< Flag with information such as BR/EDR and LE support */
    XR_BT_EIR_TYPE_INCMPL_16BITS_UUID       = 0x02,     /*!< Incomplete list of 16-bit service UUIDs */
    XR_BT_EIR_TYPE_CMPL_16BITS_UUID         = 0x03,     /*!< Complete list of 16-bit service UUIDs */
    XR_BT_EIR_TYPE_INCMPL_32BITS_UUID       = 0x04,     /*!< Incomplete list of 32-bit service UUIDs */
    XR_BT_EIR_TYPE_CMPL_32BITS_UUID         = 0x05,     /*!< Complete list of 32-bit service UUIDs */
    XR_BT_EIR_TYPE_INCMPL_128BITS_UUID      = 0x06,     /*!< Incomplete list of 128-bit service UUIDs */
    XR_BT_EIR_TYPE_CMPL_128BITS_UUID        = 0x07,     /*!< Complete list of 128-bit service UUIDs */
    XR_BT_EIR_TYPE_SHORT_LOCAL_NAME         = 0x08,     /*!< Shortened Local Name */
    XR_BT_EIR_TYPE_CMPL_LOCAL_NAME          = 0x09,     /*!< Complete Local Name */
    XR_BT_EIR_TYPE_TX_POWER_LEVEL           = 0x0a,     /*!< Tx power level, value is 1 octet ranging from  -127 to 127, unit is dBm*/
    XR_BT_EIR_TYPE_MANU_SPECIFIC            = 0xff,     /*!< Manufacturer specific data */
} t_xr_bt_eirype;

/// Major service class field of Class of Device, mutiple bits can be set
typedef enum {
    XR_BT_COD_SRVC_NONE                     =     0,    /*!< None indicates an invalid value */
    XR_BT_COD_SRVC_LMTD_DISCOVER            =   0x1,    /*!< Limited Discoverable Mode */
    XR_BT_COD_SRVC_POSITIONING              =   0x8,    /*!< Positioning (Location identification) */
    XR_BT_COD_SRVC_NETWORKING               =  0x10,    /*!< Networking, e.g. LAN, Ad hoc */
    XR_BT_COD_SRVC_RENDERING                =  0x20,    /*!< Rendering, e.g. Printing, Speakers */
    XR_BT_COD_SRVC_CAPTURING                =  0x40,    /*!< Capturing, e.g. Scanner, Microphone */
    XR_BT_COD_SRVC_OBJ_TRANSFER             =  0x80,    /*!< Object Transfer, e.g. v-Inbox, v-Folder */
    XR_BT_COD_SRVC_AUDIO                    = 0x100,    /*!< Audio, e.g. Speaker, Microphone, Headset service */
    XR_BT_COD_SRVC_TELEPHONY                = 0x200,    /*!< Telephony, e.g. Cordless telephony, Modem, Headset service */
    XR_BT_COD_SRVC_INFORMATION              = 0x400,    /*!< Information, e.g., WEB-server, WAP-server */
} t_xr_bt_cod_srvc;


typedef enum {
    XR_BT_SP_IOCAP_MODE = 0,                            /*!< Set IO mode */
    //XR_BT_SP_OOB_DATA, //TODO                         /*!< Set OOB data */
} t_xr_bt_sp_param;

/* relate to BTM_IO_CAP_xxx in stack/btm_api.h */
#define XR_BT_IO_CAP_OUT                      0        /*!< DisplayOnly */         /* relate to BTM_IO_CAP_OUT in stack/btm_api.h */
#define XR_BT_IO_CAP_IO                       1        /*!< DisplayYesNo */        /* relate to BTM_IO_CAP_IO in stack/btm_api.h */
#define XR_BT_IO_CAP_IN                       2        /*!< KeyboardOnly */        /* relate to BTM_IO_CAP_IN in stack/btm_api.h */
#define XR_BT_IO_CAP_NONE                     3        /*!< NoInputNoOutput */     /* relate to BTM_IO_CAP_NONE in stack/btm_api.h */
typedef uint8_t t_xr_bt_io_cap;                        /*!< combination of the io capability */

/// Bits of major service class field
#define XR_BT_COD_SRVC_BIT_MASK              (0xffe000) /*!< Major service bit mask */
#define XR_BT_COD_SRVC_BIT_OFFSET            (13)       /*!< Major service bit offset */

/// Major device class field of Class of Device
typedef enum {
    XR_BT_COD_MAJOR_DEV_MISC                = 0,    /*!< Miscellaneous */
    XR_BT_COD_MAJOR_DEV_COMPUTER            = 1,    /*!< Computer */
    XR_BT_COD_MAJOR_DEV_PHONE               = 2,    /*!< Phone(cellular, cordless, pay phone, modem */
    XR_BT_COD_MAJOR_DEV_LAN_NAP             = 3,    /*!< LAN, Network Access Point */
    XR_BT_COD_MAJOR_DEV_AV                  = 4,    /*!< Audio/Video(headset, speaker, stereo, video display, VCR */
    XR_BT_COD_MAJOR_DEV_PERIPHERAL          = 5,    /*!< Peripheral(mouse, joystick, keyboard) */
    XR_BT_COD_MAJOR_DEV_IMAGING             = 6,    /*!< Imaging(printer, scanner, camera, display */
    XR_BT_COD_MAJOR_DEV_WEARABLE            = 7,    /*!< Wearable */
    XR_BT_COD_MAJOR_DEV_TOY                 = 8,    /*!< Toy */
    XR_BT_COD_MAJOR_DEV_HEALTH              = 9,    /*!< Health */
    XR_BT_COD_MAJOR_DEV_UNCATEGORIZED       = 31,   /*!< Uncategorized: device not specified */
} t_xr_bt_cod_major_dev;

/// Bits of major device class field
#define XR_BT_COD_MAJOR_DEV_BIT_MASK         (0x1f00) /*!< Major device bit mask */
#define XR_BT_COD_MAJOR_DEV_BIT_OFFSET       (8)      /*!< Major device bit offset */

/// Bits of minor device class field
#define XR_BT_COD_MINOR_DEV_BIT_MASK         (0xfc)   /*!< Minor device bit mask */
#define XR_BT_COD_MINOR_DEV_BIT_OFFSET       (2)      /*!< Minor device bit offset */

/// Bits of format type
#define XR_BT_COD_FORMAT_TYPE_BIT_MASK       (0x03)   /*!< Format type bit mask */
#define XR_BT_COD_FORMAT_TYPE_BIT_OFFSET     (0)      /*!< Format type bit offset */

/// Class of device format type 1
#define XR_BT_COD_FORMAT_TYPE_1              (0x00)

/** Bluetooth Device Discovery state */
typedef enum {
    XR_BT_GAP_DISCOVERY_STOPPED,                   /*!< device discovery stopped */
    XR_BT_GAP_DISCOVERY_STARTED,                   /*!< device discovery started */
} t_xr_bt_gap_discovery_state;

/// BT GAP callback events
typedef enum {
    XR_BT_GAP_DISC_RES_EVT = 0,                    /*!< device discovery result event */
    XR_BT_GAP_DISC_STATE_CHANGED_EVT,              /*!< discovery state changed event */
    XR_BT_GAP_INQ_RES_EVT,                         /*!< device inquiry result event */
    XR_BT_GAP_RMT_SRVCS_EVT,                       /*!< get remote services event */
    XR_BT_GAP_RMT_SRVC_REC_EVT,                    /*!< get remote service record event */
    XR_BT_GAP_AUTH_CMPL_EVT,                       /*!< AUTH complete event */
    XR_BT_GAP_CFM_REQ_EVT,                         /*!< Simple Pairing User Confirmation request. */
    XR_BT_GAP_KEY_NOTIF_EVT,                       /*!< Simple Pairing Passkey Notification */
    XR_BT_GAP_KEY_REQ_EVT,                         /*!< Simple Pairing Passkey request */
    XR_BT_GAP_READ_RSSI_DELTA_EVT,                 /*!< read rssi event */
    XR_BT_GAP_EVT_MAX,
} t_xr_bt_gap_cb_event;

/** Inquiry Mode */
typedef enum {
    XR_BT_INQ_MODE_GENERAL_INQUIRY,                /*!< General inquiry mode */
    XR_BT_INQ_MODE_LIMITED_INQUIRY,                /*!< Limited inquiry mode */
} t_xr_bt_inq_mode;

/** Minimum and Maximum inquiry length*/
#define XR_BT_GAP_MIN_INQ_LEN                (0x01)  /*!< Minimum inquiry duration, unit is 1.28s */
#define XR_BT_GAP_MAX_INQ_LEN                (0x30)  /*!< Maximum inquiry duration, unit is 1.28s */

/// A2DP state callback parameters
typedef union {
    struct bt_disc_res_param {					//XR_BT_GAP_DISC_RES_EVT
        t_xr_bd_addr bda;                     /*!< remote bluetooth device address*/
        int num_prop;                          /*!< number of properties got */
        t_xr_bt_gap_dev_prop *prop;           /*!< properties discovered from the new device */
    } disc_res;                                /*!< discovery result parameter struct */

    struct bt_disc_state_changed_param {			//XR_BT_GAP_DISC_STATE_CHANGED_EVT
        t_xr_bt_gap_discovery_state state;    /*!< discovery state */
    } disc_st_chg;                             /*!< discovery state changed parameter struct */

    struct bt_rmt_srvcs_param {				//XR_BT_GAP_RMT_SRVCS_EVT
        t_xr_bd_addr bda;                     /*!< remote bluetooth device address*/
        t_bt_status stat;                  /*!< service search status */
        int num_uuids;                         /*!< number of UUID in uuid_list */
        t_bt_uuid *uuid_list;              /*!< list of service UUIDs of remote device */
    } rmt_srvcs;                               /*!< services of remote device parameter struct */

    struct bt_rmt_srvc_rec_param {				//XR_BT_GAP_RMT_SRVC_REC_EVT
        t_xr_bd_addr bda;                     /*!< remote bluetooth device address*/
        t_bt_status stat;                  /*!< service search status */
    } rmt_srvc_rec;                            /*!< specific service record from remote device parameter struct */

    struct bt_read_rssi_delta_param {			//XR_BT_GAP_READ_RSSI_DELTA_EVT
        t_xr_bd_addr bda;                     /*!< remote bluetooth device address*/
        t_bt_status stat;                  /*!< read rssi status */
        int8_t rssi_delta;                     /*!< rssi delta value range -128 ~127, The value zero indicates that the RSSI is inside the Golden Receive Power Range, the Golden Receive Power Range is from XR_BT_GAP_RSSI_LOW_THRLD to XR_BT_GAP_RSSI_HIGH_THRLD */
    } read_rssi_delta;                         /*!< read rssi parameter struct */

    struct bt_auth_cmpl_param {					//XR_BT_GAP_AUTH_CMPL_EVT
        t_xr_bd_addr bda;                     /*!< remote bluetooth device address*/
        t_bt_status stat;                  /*!< authentication complete status */
        uint8_t device_name[XR_BT_GAP_MAX_BDNAME_LEN + 1]; /*!< device name */
    } auth_cmpl;                               /*!< authentication complete parameter struct */

    struct bt_cfm_req_param {						//XR_BT_GAP_CFM_REQ_EVT
        t_xr_bd_addr bda;                     /*!< remote bluetooth device address*/
        uint32_t num_val;                      /*!< the numeric value for comparison. */
    } cfm_req;                                 /*!< confirm request parameter struct */

    struct bt_key_notif_param {					//XR_BT_GAP_KEY_NOTIF_EVT
        t_xr_bd_addr bda;                     /*!< remote bluetooth device address*/
        uint32_t passkey;                      /*!< the numeric value for passkey entry. */
    } key_notif;                               /*!< passkey notif parameter struct */
	
    struct bt_key_req_param {						//XR_BT_GAP_KEY_REQ_EVT
        t_xr_bd_addr bda;                     /*!< remote bluetooth device address*/
    } key_req;                                 /*!< passkey request parameter struct */
} t_xr_bt_gap_cb_param;

typedef void (* t_bt_gap_cb)(t_xr_bt_gap_cb_event event, t_xr_bt_gap_cb_param *param);

int32_t bt_gap_reg_cb(t_bt_gap_cb callback);

//Set a GAP security parameter value. Overrides the default value.
int32_t bt_gap_set_security_param(t_xr_bt_sp_param param_type,
                                        void *value, uint8_t len);

//
int32_t bt_gap_set_scan_mode(t_xr_bt_scan_mode mode);

#if CONFIG_BT_SSP_ENABLE
//
int32_t bt_gap_ssp_confirm_reply(t_xr_bd_addr bd_addr, bool accept);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __XR_GAP_BT_API_H__ */
