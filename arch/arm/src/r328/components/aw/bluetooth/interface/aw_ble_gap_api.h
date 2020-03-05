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


#ifndef __AW_BLE_GAP_API_H__
#define __AW_BLE_GAP_API_H__

#include <stdint.h>
#include <stdbool.h>

#include "aw_bt_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

//BLE_ADV_DATA_FLAG data flag bit definition used for advertising data flag
#define XR_BLE_ADV_FLAG_LIMIT_DISC         (0x01 << 0)
#define XR_BLE_ADV_FLAG_GEN_DISC           (0x01 << 1)
#define XR_BLE_ADV_FLAG_BREDR_NOT_SPT      (0x01 << 2)
#define XR_BLE_ADV_FLAG_DMT_CONTROLLER_SPT (0x01 << 3)
#define XR_BLE_ADV_FLAG_DMT_HOST_SPT       (0x01 << 4)
#define XR_BLE_ADV_FLAG_NON_LIMIT_DISC     (0x00 )

/* relate to BTM_LE_KEY_xxx in stack/btm_api.h */
#define XR_LE_KEY_NONE                    0                                                                         /* relate to BTM_LE_KEY_NONE in stack/btm_api.h */
#define XR_LE_KEY_PENC                    (1 << 0)   /*!< encryption key, encryption information of peer device */  /* relate to BTM_LE_KEY_PENC in stack/btm_api.h */
#define XR_LE_KEY_PID                     (1 << 1)   /*!< identity key of the peer device */                        /* relate to BTM_LE_KEY_PID in stack/btm_api.h */
#define XR_LE_KEY_PCSRK                   (1 << 2)   /*!< peer SRK */                                               /* relate to BTM_LE_KEY_PCSRK in stack/btm_api.h */
#define XR_LE_KEY_PLK                     (1 << 3)   /*!< Link key*/                                                /* relate to BTM_LE_KEY_PLK in stack/btm_api.h */
#define XR_LE_KEY_LLK                     (XR_LE_KEY_PLK << 4)                                                     /* relate to BTM_LE_KEY_LLK in stack/btm_api.h */
#define XR_LE_KEY_LENC                    (XR_LE_KEY_PENC << 4)   /*!< master role security information:div */     /* relate to BTM_LE_KEY_LENC in stack/btm_api.h */
#define XR_LE_KEY_LID                     (XR_LE_KEY_PID << 4)    /*!< master device ID key */                     /* relate to BTM_LE_KEY_LID in stack/btm_api.h */
#define XR_LE_KEY_LCSRK                   (XR_LE_KEY_PCSRK << 4)  /*!< local CSRK has been deliver to peer */      /* relate to BTM_LE_KEY_LCSRK in stack/btm_api.h */
typedef uint8_t t_xr_ble_key_type;

/* relate to BTM_LE_AUTH_xxx in stack/btm_api.h */
#define XR_LE_AUTH_NO_BOND                 0x00                                     /*!< 0*/                     /* relate to BTM_LE_AUTH_NO_BOND in stack/btm_api.h */
#define XR_LE_AUTH_BOND                    0x01                                     /*!< 1 << 0 */               /* relate to BTM_LE_AUTH_BOND in stack/btm_api.h */
#define XR_LE_AUTH_REQ_MITM                (1 << 2)                                 /*!< 1 << 2 */               /* relate to BTM_LE_AUTH_REQ_MITM in stack/btm_api.h */
#define XR_LE_AUTH_REQ_SC_ONLY             (1 << 3)                                 /*!< 1 << 3 */               /* relate to BTM_LE_AUTH_REQ_SC_ONLY in stack/btm_api.h */
#define XR_LE_AUTH_REQ_SC_BOND             (XR_LE_AUTH_BOND | XR_LE_AUTH_REQ_SC_ONLY)            /*!< 1001 */  /* relate to BTM_LE_AUTH_REQ_SC_BOND in stack/btm_api.h */
#define XR_LE_AUTH_REQ_SC_MITM             (XR_LE_AUTH_REQ_MITM | XR_LE_AUTH_REQ_SC_ONLY)        /*!< 1100 */  /* relate to BTM_LE_AUTH_REQ_SC_MITM in stack/btm_api.h */
#define XR_LE_AUTH_REQ_SC_MITM_BOND        (XR_LE_AUTH_REQ_MITM | XR_LE_AUTH_REQ_SC_ONLY | XR_LE_AUTH_BOND)   /*!< 1101 */  /* relate to BTM_LE_AUTH_REQ_SC_MITM_BOND in stack/btm_api.h */
typedef uint8_t   t_xr_ble_auth_req;         /*!< combination of the above bit pattern */

/* relate to BTM_IO_CAP_xxx in stack/btm_api.h */
#define XR_IO_CAP_OUT                      0   /*!< DisplayOnly */         /* relate to BTM_IO_CAP_OUT in stack/btm_api.h */
#define XR_IO_CAP_IO                       1   /*!< DisplayYesNo */        /* relate to BTM_IO_CAP_IO in stack/btm_api.h */
#define XR_IO_CAP_IN                       2   /*!< KeyboardOnly */        /* relate to BTM_IO_CAP_IN in stack/btm_api.h */
#define XR_IO_CAP_NONE                     3   /*!< NoInputNoOutput */     /* relate to BTM_IO_CAP_NONE in stack/btm_api.h */
#define XR_IO_CAP_KBDISP                   4   /*!< Keyboard display */    /* relate to BTM_IO_CAP_KBDISP in stack/btm_api.h */

#define XR_BLE_APPEARANCE_UNKNOWN                 0x0000 /* relate to BTM_BLE_APPEARANCE_UNKNOWN in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_PHONE           0x0040 /* relate to BTM_BLE_APPEARANCE_GENERIC_PHONE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_COMPUTER        0x0080 /* relate to BTM_BLE_APPEARANCE_GENERIC_COMPUTER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_WATCH           0x00C0 /* relate to BTM_BLE_APPEARANCE_GENERIC_WATCH in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_SPORTS_WATCH            0x00C1 /* relate to BTM_BLE_APPEARANCE_SPORTS_WATCH in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_CLOCK           0x0100 /* relate to BTM_BLE_APPEARANCE_GENERIC_CLOCK in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_DISPLAY         0x0140 /* relate to BTM_BLE_APPEARANCE_GENERIC_DISPLAY in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_REMOTE          0x0180 /* relate to BTM_BLE_APPEARANCE_GENERIC_REMOTE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_EYEGLASSES      0x01C0 /* relate to BTM_BLE_APPEARANCE_GENERIC_EYEGLASSES in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_TAG             0x0200 /* relate to BTM_BLE_APPEARANCE_GENERIC_TAG in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_KEYRING         0x0240 /* relate to BTM_BLE_APPEARANCE_GENERIC_KEYRING in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_MEDIA_PLAYER    0x0280 /* relate to BTM_BLE_APPEARANCE_GENERIC_MEDIA_PLAYER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_BARCODE_SCANNER 0x02C0 /* relate to BTM_BLE_APPEARANCE_GENERIC_BARCODE_SCANNER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_THERMOMETER     0x0300 /* relate to BTM_BLE_APPEARANCE_GENERIC_THERMOMETER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_THERMOMETER_EAR         0x0301 /* relate to BTM_BLE_APPEARANCE_THERMOMETER_EAR in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_HEART_RATE      0x0340 /* relate to BTM_BLE_APPEARANCE_GENERIC_HEART_RATE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HEART_RATE_BELT         0x0341 /* relate to BTM_BLE_APPEARANCE_HEART_RATE_BELT in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE  0x0380 /* relate to BTM_BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_BLOOD_PRESSURE_ARM      0x0381 /* relate to BTM_BLE_APPEARANCE_BLOOD_PRESSURE_ARM in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_BLOOD_PRESSURE_WRIST    0x0382 /* relate to BTM_BLE_APPEARANCE_BLOOD_PRESSURE_WRIST in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_HID             0x03C0 /* relate to BTM_BLE_APPEARANCE_GENERIC_HID in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HID_KEYBOARD            0x03C1 /* relate to BTM_BLE_APPEARANCE_HID_KEYBOARD in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HID_MOUSE               0x03C2 /* relate to BTM_BLE_APPEARANCE_HID_MOUSE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HID_JOYSTICK            0x03C3 /* relate to BTM_BLE_APPEARANCE_HID_JOYSTICK in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HID_GAMEPAD             0x03C4 /* relate to BTM_BLE_APPEARANCE_HID_GAMEPAD in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HID_DIGITIZER_TABLET    0x03C5 /* relate to BTM_BLE_APPEARANCE_HID_DIGITIZER_TABLET in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HID_CARD_READER         0x03C6 /* relate to BTM_BLE_APPEARANCE_HID_CARD_READER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HID_DIGITAL_PEN         0x03C7 /* relate to BTM_BLE_APPEARANCE_HID_DIGITAL_PEN in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_HID_BARCODE_SCANNER     0x03C8 /* relate to BTM_BLE_APPEARANCE_HID_BARCODE_SCANNER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_GLUCOSE         0x0400 /* relate to BTM_BLE_APPEARANCE_GENERIC_GLUCOSE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_WALKING         0x0440 /* relate to BTM_BLE_APPEARANCE_GENERIC_WALKING in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_WALKING_IN_SHOE         0x0441 /* relate to BTM_BLE_APPEARANCE_WALKING_IN_SHOE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_WALKING_ON_SHOE         0x0442 /* relate to BTM_BLE_APPEARANCE_WALKING_ON_SHOE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_WALKING_ON_HIP          0x0443 /* relate to BTM_BLE_APPEARANCE_WALKING_ON_HIP in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_CYCLING         0x0480 /* relate to BTM_BLE_APPEARANCE_GENERIC_CYCLING in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_CYCLING_COMPUTER        0x0481 /* relate to BTM_BLE_APPEARANCE_CYCLING_COMPUTER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_CYCLING_SPEED           0x0482 /* relate to BTM_BLE_APPEARANCE_CYCLING_SPEED in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_CYCLING_CADENCE         0x0483 /* relate to BTM_BLE_APPEARANCE_CYCLING_CADENCE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_CYCLING_POWER           0x0484 /* relate to BTM_BLE_APPEARANCE_CYCLING_POWER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_CYCLING_SPEED_CADENCE   0x0485 /* relate to BTM_BLE_APPEARANCE_CYCLING_SPEED_CADENCE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_PULSE_OXIMETER  0x0C40 /* relate to BTM_BLE_APPEARANCE_GENERIC_PULSE_OXIMETER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_PULSE_OXIMETER_FINGERTIP 0x0C41 /* relate to BTM_BLE_APPEARANCE_PULSE_OXIMETER_FINGERTIP in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_PULSE_OXIMETER_WRIST    0x0C42 /* relate to BTM_BLE_APPEARANCE_PULSE_OXIMETER_WRIST in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_WEIGHT          0x0C80 /* relate to BTM_BLE_APPEARANCE_GENERIC_WEIGHT in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_PERSONAL_MOBILITY_DEVICE    0x0CC0 /* relate to BTM_BLE_APPEARANCE_GENERIC_PERSONAL_MOBILITY_DEVICE in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_POWERED_WHEELCHAIR                  0x0CC1 /* relate to BTM_BLE_APPEARANCE_POWERED_WHEELCHAIR in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_MOBILITY_SCOOTER                    0x0CC2 /* relate to BTM_BLE_APPEARANCE_MOBILITY_SCOOTER in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_CONTINUOUS_GLUCOSE_MONITOR  0x0D00 /* relate to BTM_BLE_APPEARANCE_GENERIC_CONTINUOUS_GLUCOSE_MONITOR in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_INSULIN_PUMP                0x0D40 /* relate to BTM_BLE_APPEARANCE_GENERIC_INSULIN_PUMP in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_INSULIN_PUMP_DURABLE_PUMP           0x0D41 /* relate to BTM_BLE_APPEARANCE_INSULIN_PUMP_DURABLE_PUMP in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_INSULIN_PUMP_PATCH_PUMP             0x0D44 /* relate to BTM_BLE_APPEARANCE_INSULIN_PUMP_PATCH_PUMP in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_INSULIN_PEN                         0x0D48 /* relate to BTM_BLE_APPEARANCE_INSULIN_PEN in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_MEDICATION_DELIVERY         0x0D80 /* relate to BTM_BLE_APPEARANCE_GENERIC_MEDICATION_DELIVERY in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_GENERIC_OUTDOOR_SPORTS  0x1440             /* relate to BTM_BLE_APPEARANCE_GENERIC_OUTDOOR_SPORTS in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_OUTDOOR_SPORTS_LOCATION 0x1441             /* relate to BTM_BLE_APPEARANCE_OUTDOOR_SPORTS_LOCATION in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_OUTDOOR_SPORTS_LOCATION_AND_NAV     0x1442 /* relate to BTM_BLE_APPEARANCE_OUTDOOR_SPORTS_LOCATION_AND_NAV in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_OUTDOOR_SPORTS_LOCATION_POD         0x1443 /* relate to BTM_BLE_APPEARANCE_OUTDOOR_SPORTS_LOCATION_POD in stack/btm_ble_api.h */
#define XR_BLE_APPEARANCE_OUTDOOR_SPORTS_LOCATION_POD_AND_NAV 0x1444 /* relate to BTM_BLE_APPEARANCE_OUTDOOR_SPORTS_LOCATION_POD_AND_NAV in stack/btm_ble_api.h */

typedef uint8_t t_xr_ble_io_cap;               /*!< combination of the io capability */

/// GAP BLE callback event type
typedef enum {
    XR_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT        = 0,       /*!< When advertising data set complete, the event comes */
    XR_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT,             /*!< When scan rt_xronse data set complete, the event comes */
    XR_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT,                /*!< When scan parameters set complete, the event comes */
    XR_GAP_BLE_SCAN_RESULT_EVT,                            /*!< When one scan result ready, the event comes each time */
    XR_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT,              /*!< When raw advertising data set complete, the event comes */
    XR_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT,         /*!< When raw advertising data set complete, the event comes */
    XR_GAP_BLE_ADV_START_COMPLETE_EVT,                     /*!< When start advertising complete, the event comes */
    XR_GAP_BLE_SCAN_START_COMPLETE_EVT,                    /*!< When start scan complete, the event comes */
    XR_GAP_BLE_AUTH_CMPL_EVT,                              /* Authentication complete indication. */
    XR_GAP_BLE_KEY_EVT,                                    /* BLE  key event for peer device keys */
    XR_GAP_BLE_SEC_REQ_EVT,                                /* BLE  security request */
    XR_GAP_BLE_PASSKEY_NOTIF_EVT,                          /* passkey notification event */
    XR_GAP_BLE_PASSKEY_REQ_EVT,                            /* passkey request event */
    XR_GAP_BLE_OOB_REQ_EVT,                                /* OOB request event */
    XR_GAP_BLE_LOCAL_IR_EVT,                               /* BLE local IR event */
    XR_GAP_BLE_LOCAL_ER_EVT,                               /* BLE local ER event */
    XR_GAP_BLE_NC_REQ_EVT,                                 /* Numeric Comparison request event */
    XR_GAP_BLE_ADV_STOP_COMPLETE_EVT,                      /*!< When stop adv complete, the event comes */
    XR_GAP_BLE_SCAN_STOP_COMPLETE_EVT,                     /*!< When stop scan complete, the event comes */
    XR_GAP_BLE_SET_STATIC_RAND_ADDR_EVT,                   /*!< When set the static rand address complete, the event comes */
    XR_GAP_BLE_UPDATE_CONN_PARAMS_EVT,                     /*!< When update connection parameters complete, the event comes */
    XR_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT,                /*!< When set pkt length complete, the event comes */
    XR_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT,             /*!< When  Enable/disable privacy on the local device complete, the event comes */
    XR_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT,               /*!< When remove the bond device complete, the event comes */
    XR_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT,                /*!< When clear the bond device clear complete, the event comes */
    XR_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT,                  /*!< When get the bond device list complete, the event comes */
    XR_GAP_BLE_READ_RSSI_COMPLETE_EVT,                     /*!< When read the rssi complete, the event comes */
    XR_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT,              /*!< When add or remove whitelist complete, the event comes */
    XR_GAP_BLE_EVT_MAX,
} t_xr_gap_ble_cb_event;
/// This is the old name, just for backwards compatibility
#define XR_GAP_BLE_ADD_WHITELIST_COMPLETE_EVT XR_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT

/// Advertising data maximum length
#define XR_BLE_ADV_DATA_LEN_MAX               31
/// Scan rt_xronse data maximum length
#define XR_BLE_SCAN_RSP_DATA_LEN_MAX          31

/* relate to BTM_BLE_AD_TYPE_xxx in stack/btm_ble_api.h */
/// The type of advertising data(not adv_type)
typedef enum {
    XR_BLE_AD_TYPE_FLAG                     = 0x01,    /* relate to BTM_BLE_AD_TYPE_FLAG in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_16SRV_PART               = 0x02,    /* relate to BTM_BLE_AD_TYPE_16SRV_PART in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_16SRV_CMPL               = 0x03,    /* relate to BTM_BLE_AD_TYPE_16SRV_CMPL in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_32SRV_PART               = 0x04,    /* relate to BTM_BLE_AD_TYPE_32SRV_PART in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_32SRV_CMPL               = 0x05,    /* relate to BTM_BLE_AD_TYPE_32SRV_CMPL in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_128SRV_PART              = 0x06,    /* relate to BTM_BLE_AD_TYPE_128SRV_PART in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_128SRV_CMPL              = 0x07,    /* relate to BTM_BLE_AD_TYPE_128SRV_CMPL in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_NAME_SHORT               = 0x08,    /* relate to BTM_BLE_AD_TYPE_NAME_SHORT in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_NAME_CMPL                = 0x09,    /* relate to BTM_BLE_AD_TYPE_NAME_CMPL in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_TX_PWR                   = 0x0A,    /* relate to BTM_BLE_AD_TYPE_TX_PWR in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_DEV_CLASS                = 0x0D,    /* relate to BTM_BLE_AD_TYPE_DEV_CLASS in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_SM_TK                    = 0x10,    /* relate to BTM_BLE_AD_TYPE_SM_TK in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_SM_OOB_FLAG              = 0x11,    /* relate to BTM_BLE_AD_TYPE_SM_OOB_FLAG in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_INT_RANGE                = 0x12,    /* relate to BTM_BLE_AD_TYPE_INT_RANGE in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_SOL_SRV_UUID             = 0x14,    /* relate to BTM_BLE_AD_TYPE_SOL_SRV_UUID in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_128SOL_SRV_UUID          = 0x15,    /* relate to BTM_BLE_AD_TYPE_128SOL_SRV_UUID in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_SERVICE_DATA             = 0x16,    /* relate to BTM_BLE_AD_TYPE_SERVICE_DATA in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_PUBLIC_TARGET            = 0x17,    /* relate to BTM_BLE_AD_TYPE_PUBLIC_TARGET in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_RANDOM_TARGET            = 0x18,    /* relate to BTM_BLE_AD_TYPE_RANDOM_TARGET in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_APPEARANCE               = 0x19,    /* relate to BTM_BLE_AD_TYPE_APPEARANCE in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_ADV_INT                  = 0x1A,    /* relate to BTM_BLE_AD_TYPE_ADV_INT in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_LE_DEV_ADDR              = 0x1b,    /* relate to BTM_BLE_AD_TYPE_LE_DEV_ADDR in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_LE_ROLE                  = 0x1c,    /* relate to BTM_BLE_AD_TYPE_LE_ROLE in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_SPAIR_C256               = 0x1d,    /* relate to BTM_BLE_AD_TYPE_SPAIR_C256 in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_SPAIR_R256               = 0x1e,    /* relate to BTM_BLE_AD_TYPE_SPAIR_R256 in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_32SOL_SRV_UUID           = 0x1f,    /* relate to BTM_BLE_AD_TYPE_32SOL_SRV_UUID in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_32SERVICE_DATA           = 0x20,    /* relate to BTM_BLE_AD_TYPE_32SERVICE_DATA in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_128SERVICE_DATA          = 0x21,    /* relate to BTM_BLE_AD_TYPE_128SERVICE_DATA in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_LE_SECURE_CONFIRM        = 0x22,    /* relate to BTM_BLE_AD_TYPE_LE_SECURE_CONFIRM in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_LE_SECURE_RANDOM         = 0x23,    /* relate to BTM_BLE_AD_TYPE_LE_SECURE_RANDOM in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_URI                      = 0x24,    /* relate to BTM_BLE_AD_TYPE_URI in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_INDOOR_POSITION          = 0x25,    /* relate to BTM_BLE_AD_TYPE_INDOOR_POSITION in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_TRANS_DISC_DATA          = 0x26,    /* relate to BTM_BLE_AD_TYPE_TRANS_DISC_DATA in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_LE_SUPPORT_FEATURE       = 0x27,    /* relate to BTM_BLE_AD_TYPE_LE_SUPPORT_FEATURE in stack/btm_ble_api.h */
    XR_BLE_AD_TYPE_CHAN_MAP_UPDATE          = 0x28,    /* relate to BTM_BLE_AD_TYPE_CHAN_MAP_UPDATE in stack/btm_ble_api.h */
    XR_BLE_AD_MANUFACTURER_SPECIFIC_TYPE    = 0xFF,    /* relate to BTM_BLE_AD_MANUFACTURER_SPECIFIC_TYPE in stack/btm_ble_api.h */
} t_xr_ble_adv_data_type;

/// Advertising mode
typedef enum {
    BLE_ADV_TYPE_IND                = 0x00,
    BLE_ADV_TYPE_DIRECT_IND_HIGH    = 0x01,
    BLE_ADV_TYPE_SCAN_IND           = 0x02,
    BLE_ADV_TYPE_NONCONN_IND        = 0x03,
    BLE_ADV_TYPE_DIRECT_IND_LOW     = 0x04,
} t_xr_ble_adv_type;

/// Advertising channel mask
typedef enum {
    BLE_ADV_CHNL_37     = 0x01,
    BLE_ADV_CHNL_38     = 0x02,
    BLE_ADV_CHNL_39     = 0x04,
    BLE_ADV_CHNL_ALL    = 0x07,
} t_xr_ble_adv_channel;

typedef enum {
    ///Allow both scan and connection requests from anyone
    BLE_ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY  = 0x00,
    ///Allow both scan req from White List devices only and connection req from anyone
    BLE_ADV_FILTER_ALLOW_SCAN_WLST_CON_ANY,
    ///Allow both scan req from anyone and connection req from White List devices only
    BLE_ADV_FILTER_ALLOW_SCAN_ANY_CON_WLST,
    ///Allow scan and connection requests from White List devices only
    BLE_ADV_FILTER_ALLOW_SCAN_WLST_CON_WLST,
    ///Enumeration end value for advertising filter policy value check
} t_xr_ble_adv_filter;


/* relate to BTA_DM_BLE_SEC_xxx in bta/bta_api.h */
typedef enum {
    XR_BLE_SEC_ENCRYPT = 1,            /* relate to BTA_DM_BLE_SEC_ENCRYPT in bta/bta_api.h. If the device has already
                                           bonded, the stack will used LTK to encrypt with the remote device directly.
                                           Else if the device hasn't bonded, the stack will used the default authentication request
                                           used the t_xr_ble_gap_set_security_param function set by the user. */
    XR_BLE_SEC_ENCRYPT_NO_MITM,        /* relate to BTA_DM_BLE_SEC_ENCRYPT_NO_MITM in bta/bta_api.h. If the device has already
                                           bonded, the stack will check the LTK Whether the authentication request has been met, if met, used the LTK
                                           to encrypt with the remote device directly, else Re-pair with the remote device.
                                           Else if the device hasn't bonded, the stack will used NO MITM authentication request in the current link instead of
                                           used the authreq in the t_xr_ble_gap_set_security_param function set by the user. */
    XR_BLE_SEC_ENCRYPT_MITM,           /* relate to BTA_DM_BLE_SEC_ENCRYPT_MITM in bta/bta_api.h. If the device has already
                                           bonded, the stack will check the LTK Whether the authentication request has been met, if met, used the LTK
                                           to encrypt with the remote device directly, else Re-pair with the remote device.
                                           Else if the device hasn't bonded, the stack will used MITM authentication request in the current link instead of
                                           used the authreq in the t_xr_ble_gap_set_security_param function set by the user. */
}t_xr_ble_sec_act;

typedef enum {
    XR_BLE_SM_PASSKEY = 0,
    XR_BLE_SM_AUTHEN_REQ_MODE,
    XR_BLE_SM_IOCAP_MODE,
    XR_BLE_SM_SET_INIT_KEY,
    XR_BLE_SM_SET_RSP_KEY,
    XR_BLE_SM_MAX_KEY_SIZE,
} t_xr_ble_sm_param;

/// Advertising parameters
typedef struct {
    uint16_t                adv_int_min;        /*!< Minimum advertising interval for
                                                  undirected and low duty cycle directed advertising.
                                                  Range: 0x0020 to 0x4000 Default: N = 0x0800 (1.28 second)
                                                  Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec */
    uint16_t                adv_int_max;        /*!< Maximum advertising interval for
                                                  undirected and low duty cycle directed advertising.
                                                  Range: 0x0020 to 0x4000 Default: N = 0x0800 (1.28 second)
                                                  Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec Advertising max interval */
    t_xr_ble_adv_type      adv_type;           /*!< Advertising type */
    t_ble_addr_type     own_addr_type;      /*!< Owner bluetooth device address type */
    t_xr_bd_addr           peer_addr;          /*!< Peer device bluetooth device address */
    t_ble_addr_type     peer_addr_type;     /*!< Peer device bluetooth device address type */
    t_xr_ble_adv_channel   channel_map;        /*!< Advertising channel map */
    t_xr_ble_adv_filter    adv_filter_policy;  /*!< Advertising filter policy */
} t_xr_ble_adv_params;

/// Advertising data content, according to "Supplement to the Bluetooth Core Specification"
typedef struct {
    bool                    set_scan_rsp;           /*!< Set this advertising data as scan rt_xronse or not*/
    bool                    include_name;           /*!< Advertising data include device name or not */
    bool                    include_txpower;        /*!< Advertising data include TX power */
    int                     min_interval;           /*!< Advertising data show advertising min interval */
    int                     max_interval;           /*!< Advertising data show advertising max interval */
    int                     appearance;             /*!< External appearance of device */
    uint16_t                manufacturer_len;       /*!< Manufacturer data length */
    uint8_t                 *p_manufacturer_data;   /*!< Manufacturer data point */
    uint16_t                service_data_len;       /*!< Service data length */
    uint8_t                 *p_service_data;        /*!< Service data point */
    uint16_t                service_uuid_len;       /*!< Service uuid length */
    uint8_t                 *p_service_uuid;        /*!< Service uuid array point */
    uint8_t                 flag;                   /*!< Advertising flag of discovery mode, see BLE_ADV_DATA_FLAG detail */
} t_xr_ble_adv_data;

/// Ble scan type
typedef enum {
    XR_BLE_SCAN_TYPE_PASSIVE   =   0x0,            /*!< Passive scan */
    XR_BLE_SCAN_TYPE_ACTIVE    =   0x1,            /*!< Active scan */
} t_xr_ble_scan_type;

/// Ble scan filter type
typedef enum {
    XR_BLE_SCAN_FILTER_ALLOW_ALL           = 0x0,  /*!< Accept all :
                                                  1. advertisement packets except directed advertising packets not addressed to this device (default). */
    XR_BLE_SCAN_FILTER_ALLOW_ONLY_WLST     = 0x1,  /*!< Accept only :
                                                  1. advertisement packets from devices where the advertiser’s address is in the White list.
                                                  2. Directed advertising packets which are not addressed for this device shall be ignored. */
    XR_BLE_SCAN_FILTER_ALLOW_UND_RPA_DIR   = 0x2,  /*!< Accept all :
                                                  1. undirected advertisement packets, and
                                                  2. directed advertising packets where the initiator address is a resolvable private address, and
                                                  3. directed advertising packets addressed to this device. */
    XR_BLE_SCAN_FILTER_ALLOW_WLIST_PRA_DIR = 0x3,  /*!< Accept all :
                                                  1. advertisement packets from devices where the advertiser’s address is in the White list, and
                                                  2. directed advertising packets where the initiator address is a resolvable private address, and
                                                  3. directed advertising packets addressed to this device.*/
} t_xr_ble_scan_filter;

/// Ble scan duplicate type
typedef enum {
    XR_BLE_SCAN_DUPLICATE_DISABLE           = 0x0,  /*!< the Link Layer should generate advertising reports to the host for each packet received */
    XR_BLE_SCAN_DUPLICATE_ENABLE            = 0x1,  /*!< the Link Layer should filter out duplicate advertising reports to the Host */
    XR_BLE_SCAN_DUPLICATE_MAX               = 0x2,  /*!< 0x02 – 0xFF, Reserved for future use */
} t_xr_ble_scan_duplicate;

/// Ble scan parameters
typedef struct {
    t_xr_ble_scan_type     scan_type;              /*!< Scan type */
    t_ble_addr_type     own_addr_type;          /*!< Owner address type */
    t_xr_ble_scan_filter   scan_filter_policy;     /*!< Scan filter policy */
    uint16_t                scan_interval;          /*!< Scan interval. This is defined as the time interval from
                                                      when the Controller started its last LE scan until it begins the subsequent LE scan.
                                                      Range: 0x0004 to 0x4000 Default: 0x0010 (10 ms)
                                                      Time = N * 0.625 msec
                                                      Time Range: 2.5 msec to 10.24 seconds*/
    uint16_t                scan_window;            /*!< Scan window. The duration of the LE scan. LE_Scan_Window
                                                      shall be less than or equal to LE_Scan_Interval
                                                      Range: 0x0004 to 0x4000 Default: 0x0010 (10 ms)
                                                      Time = N * 0.625 msec
                                                      Time Range: 2.5 msec to 10240 msec */
    t_xr_ble_scan_duplicate  scan_duplicate;       /*!< The Scan_Duplicates parameter controls whether the Link Layer should filter out 
                                                        duplicate advertising reports (BLE_SCAN_DUPLICATE_ENABLE) to the Host, or if the Link Layer should generate 
                                                        advertising reports for each packet received */
} t_xr_ble_scan_params;

/// Connection update parameters
typedef struct {
    t_xr_bd_addr bda;                              /*!< Bluetooth device address */
    uint16_t min_int;                               /*!< Min connection interval */
    uint16_t max_int;                               /*!< Max connection interval */
    uint16_t latency;                               /*!< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3 */
    uint16_t timeout;                               /*!< Supervision timeout for the LE Link. Range: 0x000A to 0x0C80.
                                                      Mandatory Range: 0x000A to 0x0C80 Time = N * 10 msec
                                                      Time Range: 100 msec to 32 seconds */
} t_xr_ble_conn_update_params;

/**
* @brief BLE pkt date length keys
*/
typedef struct
{
    uint16_t rx_len;                   /*!< pkt rx data length value */
    uint16_t tx_len;                   /*!< pkt tx data length value */
}t_xr_ble_pkt_data_length_params;

/**
* @brief BLE encryption keys
*/
typedef struct
{
    t_xr_bt_octet16     ltk;          /*!< The long term key*/
    t_xr_bt_octet8      rand;         /*!< The random number*/
    uint16_t             ediv;         /*!< The ediv value*/
    uint8_t              sec_level;    /*!< The security level of the security link*/
    uint8_t              key_size;     /*!< The key size(7~16) of the security link*/
} t_xr_ble_penc_keys;                 /*!< The key type*/

/**
* @brief  BLE CSRK keys
*/
typedef struct
{
    uint32_t            counter;      /*!< The counter */
    t_xr_bt_octet16    csrk;         /*!< The csrk key */
    uint8_t             sec_level;    /*!< The security level */
} t_xr_ble_pcsrk_keys;               /*!< The pcsrk key type */

/**
* @brief  BLE pid keys
*/
typedef struct
{
    t_xr_bt_octet16          irk;           /*!< The irk value */
    t_ble_addr_type       addr_type;     /*!< The address type */
    t_xr_bd_addr             static_addr;   /*!< The static address */
} t_xr_ble_pid_keys;                        /*!< The pid key type */

/**
* @brief  BLE Encryption reproduction keys
*/
typedef struct
{
    t_xr_bt_octet16  ltk;                  /*!< The long term key */
    uint16_t          div;                  /*!< The div value */
    uint8_t           key_size;             /*!< The key size of the security link */
    uint8_t           sec_level;            /*!< The security level of the security link */
} t_xr_ble_lenc_keys;                      /*!< The  key type */

/**
* @brief  BLE SRK keys
*/
typedef struct
{
    uint32_t          counter;              /*!< The counter value */
    uint16_t          div;                  /*!< The div value */
    uint8_t           sec_level;            /*!< The security level of the security link */
    t_xr_bt_octet16  csrk;                 /*!< The csrk key value */
} t_xr_ble_lcsrk_keys;                       /*!< The csrk key type */

/**
* @brief  Structure associated with XR_KEY_NOTIF_EVT
*/
typedef struct
{
    t_xr_bd_addr  bd_addr;        /*!< peer address */
    uint32_t       passkey;        /*!< the numeric value for comparison. If just_works, do not show this number to UI */
} t_xr_ble_sec_key_notif;         /*!< BLE key notify type*/

/**
* @brief  Structure of the security request
*/
typedef struct
{
    t_xr_bd_addr bd_addr;        /*!< peer address */
} t_xr_ble_sec_req;               /*!< BLE security request type*/

/**
* @brief  union type of the security key value
*/
typedef union
{
    t_xr_ble_penc_keys   penc_key;       /*!< received peer encryption key */
    t_xr_ble_pcsrk_keys  pcsrk_key;      /*!< received peer device SRK */
    t_xr_ble_pid_keys    pid_key;        /*!< peer device ID key */
    t_xr_ble_lenc_keys   lenc_key;       /*!< local encryption reproduction keys LTK = = d1(ER,DIV,0)*/
    t_xr_ble_lcsrk_keys    lcsrk_key;      /*!< local device CSRK = d1(ER,DIV,1)*/
} t_xr_ble_key_value;                    /*!< ble key value type*/

/**
* @brief  struct type of the bond key information value
*/
typedef struct
{
    t_xr_ble_key_mask    key_mask;       /*!< the key mask to indicate witch key is present */
    t_xr_ble_penc_keys   penc_key;       /*!< received peer encryption key */
    t_xr_ble_pcsrk_keys  pcsrk_key;      /*!< received peer device SRK */
    t_xr_ble_pid_keys    pid_key;        /*!< peer device ID key */
} t_xr_ble_bond_key_info;                /*!< ble bond key information value type */

/**
* @brief  struct type of the bond device value
*/
typedef struct
{
    t_xr_bd_addr  bd_addr;               /*!< peer address */
    t_xr_ble_bond_key_info bond_key;     /*!< the bond key information */
} t_xr_ble_bond_dev;                     /*!< the ble bond device type */


/**
* @brief  union type of the security key value
*/
typedef struct
{
    t_xr_bd_addr               bd_addr;        /*!< peer address */
    t_xr_ble_key_type          key_type;       /*!< key type of the security link */
    t_xr_ble_key_value         p_key_value;    /*!< the pointer to the key value */
} t_xr_ble_key;                                /*!< the union to the ble key value type*/

/**
* @brief  structure type of the ble local id keys value
*/
typedef struct {
    t_xr_bt_octet16       ir;                  /*!< the 16 bits of the ir value */
    t_xr_bt_octet16       irk;                 /*!< the 16 bits of the ir key value */
    t_xr_bt_octet16       dhk;                 /*!< the 16 bits of the dh key value */
} t_xr_ble_local_id_keys;                      /*!< the structure of the ble local id keys value type*/

/**
  * @brief Structure associated with XR_AUTH_CMPL_EVT
  */
typedef struct
{
    t_xr_bd_addr         bd_addr;               /*!< BD address peer device. */
    bool                  key_present;           /*!< Valid link key value in key element */
    xr_link_key          key;                   /*!< Link key associated with peer device. */
    uint8_t               key_type;              /*!< The type of Link Key */
    bool                  success;               /*!< TRUE of authentication succeeded, FALSE if failed. */
    uint8_t               fail_reason;           /*!< The HCI reason/error code for when success=FALSE */
    t_ble_addr_type   addr_type;             /*!< Peer device address type */
    t_bt_dev_type     dev_type;              /*!< Device type */
} t_xr_ble_auth_cmpl;                           /*!< The ble authentication complete cb type */

/**
  * @brief union associated with ble security
  */
typedef union
{
    t_xr_ble_sec_key_notif    key_notif;      /*!< passkey notification */
    t_xr_ble_sec_req          ble_req;        /*!< BLE SMP related request */
    t_xr_ble_key             ble_key;        /*!< BLE SMP keys used when pairing */
    t_xr_ble_local_id_keys    ble_id_keys;    /*!< BLE IR event */
    t_xr_ble_auth_cmpl        auth_cmpl;      /*!< Authentication complete indication. */
} t_xr_ble_sec;                               /*!< BLE security type */

/// Sub Event of XR_GAP_BLE_SCAN_RESULT_EVT
typedef enum {
    XR_GAP_SEARCH_INQ_RES_EVT             = 0,      /*!< Inquiry result for a peer device. */
    XR_GAP_SEARCH_INQ_CMPL_EVT            = 1,      /*!< Inquiry complete. */
    XR_GAP_SEARCH_DISC_RES_EVT            = 2,      /*!< Discovery result for a peer device. */
    XR_GAP_SEARCH_DISC_BLE_RES_EVT        = 3,      /*!< Discovery result for BLE GATT based service on a peer device. */
    XR_GAP_SEARCH_DISC_CMPL_EVT           = 4,      /*!< Discovery complete. */
    XR_GAP_SEARCH_DI_DISC_CMPL_EVT        = 5,      /*!< Discovery complete. */
    XR_GAP_SEARCH_SEARCH_CANCEL_CMPL_EVT  = 6,      /*!< Search cancelled */
} t_xr_gap_search_evt;

/**
 * @brief Ble scan result event type, to indicate the
 *        result is scan rt_xronse or advertising data or other
 */
typedef enum {
    XR_BLE_EVT_CONN_ADV         = 0x00,        /*!< Connectable undirected advertising (ADV_IND) */
    XR_BLE_EVT_CONN_DIR_ADV     = 0x01,        /*!< Connectable directed advertising (ADV_DIRECT_IND) */
    XR_BLE_EVT_DISC_ADV         = 0x02,        /*!< Scannable undirected advertising (ADV_SCAN_IND) */
    XR_BLE_EVT_NON_CONN_ADV     = 0x03,        /*!< Non connectable undirected advertising (ADV_NONCONN_IND) */
    XR_BLE_EVT_SCAN_RSP         = 0x04,        /*!< Scan Rt_xronse (SCAN_RSP) */
} t_xr_ble_evt_type;

typedef enum{
    XR_BLE_WHITELIST_REMOVE     = 0X00,    /*!< remove mac from whitelist */
    XR_BLE_WHITELIST_ADD        = 0X01,    /*!< add address to whitelist */
}t_xr_ble_wl_opration;

//brief Gap callback parameters union
typedef union {
    struct xr_ble_adv_data_cmpl_evt_param {	//XR_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the set advertising data operation success status */
    } adv_data_cmpl;                                /*!< Event parameter of XR_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT */
   
    struct xr_ble_scan_rsp_data_cmpl_evt_param {	//XR_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the set scan rt_xronse data operation success status */
    } scan_rsp_data_cmpl;                           /*!< Event parameter of XR_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT */
    
    struct xr_ble_scan_param_cmpl_evt_param {	//XR_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the set scan param operation success status */
    } scan_param_cmpl;                              /*!< Event parameter of XR_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT */
    
    struct xr_ble_scan_result_evt_param {	//XR_GAP_BLE_SCAN_RESULT_EVT
        t_xr_gap_search_evt search_evt;            /*!< Search event type */
        t_xr_bd_addr bda;                          /*!< Bluetooth device address which has been searched */
        t_bt_dev_type dev_type;                 /*!< Device type */
        t_ble_addr_type ble_addr_type;          /*!< Ble device address type */
        t_xr_ble_evt_type ble_evt_type;            /*!< Ble scan result event type */
        int rssi;                                   /*!< Searched device's RSSI */
        uint8_t  ble_adv[XR_BLE_ADV_DATA_LEN_MAX + XR_BLE_SCAN_RSP_DATA_LEN_MAX];     /*!< Received EIR */
        int flag;                                   /*!< Advertising data flag bit */
        int num_rt_xrs;                              /*!< Scan result number */
        uint8_t adv_data_len;                       /*!< Adv data length */
        uint8_t scan_rsp_len;                       /*!< Scan rt_xronse length */
    } scan_rst;                                     /*!< Event parameter of XR_GAP_BLE_SCAN_RESULT_EVT */
   
    struct xr_ble_adv_data_raw_cmpl_evt_param {	//XR_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the set raw advertising data operation success status */
    } adv_data_raw_cmpl;                            /*!< Event parameter of XR_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT */
    
    struct xr_ble_scan_rsp_data_raw_cmpl_evt_param {	//XR_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the set raw advertising data operation success status */
    } scan_rsp_data_raw_cmpl;                       /*!< Event parameter of XR_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT */
    
    struct xr_ble_adv_start_cmpl_evt_param {	//XR_GAP_BLE_ADV_START_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate advertising start operation success status */
    } adv_start_cmpl;                               /*!< Event parameter of XR_GAP_BLE_ADV_START_COMPLETE_EVT */
    
    struct xr_ble_scan_start_cmpl_evt_param {	//XR_GAP_BLE_SCAN_START_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate scan start operation success status */
    } scan_start_cmpl;                              /*!< Event parameter of XR_GAP_BLE_SCAN_START_COMPLETE_EVT */

    t_xr_ble_sec ble_security;                     /*!< ble gap security union type */
    
    struct xr_ble_scan_stop_cmpl_evt_param {	//XR_GAP_BLE_SCAN_STOP_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate scan stop operation success status */
    } scan_stop_cmpl;                               /*!< Event parameter of XR_GAP_BLE_SCAN_STOP_COMPLETE_EVT */
    
    struct xr_ble_adv_stop_cmpl_evt_param {	//XR_GAP_BLE_ADV_STOP_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate adv stop operation success status */
    } adv_stop_cmpl;                                /*!< Event parameter of XR_GAP_BLE_ADV_STOP_COMPLETE_EVT */
    
    struct xr_ble_set_rand_cmpl_evt_param {	//XR_GAP_BLE_SET_STATIC_RAND_ADDR_EVT
        t_bt_status status;                     /*!< Indicate set static rand address operation success status */
    } set_rand_addr_cmpl;                           /*!< Event parameter of XR_GAP_BLE_SET_STATIC_RAND_ADDR_EVT */
    
    struct xr_ble_update_conn_params_evt_param {	//XR_GAP_BLE_UPDATE_CONN_PARAMS_EVT
        t_bt_status status;                    /*!< Indicate update connection parameters success status */
        t_xr_bd_addr bda;                         /*!< Bluetooth device address */
        uint16_t min_int;                          /*!< Min connection interval */
        uint16_t max_int;                          /*!< Max connection interval */
        uint16_t latency;                          /*!< Slave latency for the connection in number of connection events. Range: 0x0000 to 0x01F3 */
        uint16_t conn_int;                         /*!< Current connection interval */
        uint16_t timeout;                          /*!< Supervision timeout for the LE Link. Range: 0x000A to 0x0C80.
                                                     Mandatory Range: 0x000A to 0x0C80 Time = N * 10 msec */
    }update_conn_params;                           /*!< Event parameter of XR_GAP_BLE_UPDATE_CONN_PARAMS_EVT */
    
    struct xr_ble_pkt_data_length_cmpl_evt_param {	//XR_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the set pkt data length operation success status */
        t_xr_ble_pkt_data_length_params params;    /*!<  pkt data length value */
    } pkt_data_lenth_cmpl;                          /*!< Event parameter of XR_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT */
   
    struct xr_ble_local_privacy_cmpl_evt_param {	//XR_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the set local privacy operation success status */
    } local_privacy_cmpl;                           /*!< Event parameter of XR_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT */
    
    struct xr_ble_remove_bond_dev_cmpl_evt_param {	//XR_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the remove bond device operation success status */
        t_xr_bd_addr bd_addr;                      /*!< The device address which has been remove from the bond list */
    }remove_bond_dev_cmpl;                          /*!< Event parameter of XR_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT */
    
    struct xr_ble_clear_bond_dev_cmpl_evt_param {	//XR_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the clear bond device operation success status */
    }clear_bond_dev_cmpl;                           /*!< Event parameter of XR_GAP_BLE_CLEAR_BOND_DEV_COMPLETE_EVT */
    
    struct xr_ble_get_bond_dev_cmpl_evt_param {	//XR_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the get bond device operation success status */
        uint8_t dev_num;                            /*!< Indicate the get number device in the bond list */
        t_xr_ble_bond_dev *bond_dev;               /*!< the pointer to the bond device Structure */
    }get_bond_dev_cmpl;                             /*!< Event parameter of XR_GAP_BLE_GET_BOND_DEV_COMPLETE_EVT */
    
    struct xr_ble_read_rssi_cmpl_evt_param {		//XR_GAP_BLE_READ_RSSI_COMPLETE_EVT
        t_bt_status status;                     	/*!< Indicate the read adv tx power operation success status */
        int8_t rssi;                                /*!< The ble remote device rssi value, the range is from -127 to 20, the unit is dbm,
                                                         if the RSSI cannot be read, the RSSI metric shall be set to 127. */
        t_xr_bd_addr remote_addr;                  	/*!< The remote device address */
    } read_rssi_cmpl;                               /*!< Event parameter of XR_GAP_BLE_READ_RSSI_COMPLETE_EVT */
    
    struct xr_ble_update_whitelist_cmpl_evt_param {	//XR_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT
        t_bt_status status;                     /*!< Indicate the add or remove whitelist operation success status */
        t_xr_ble_wl_opration wl_opration;          /*!< The value is XR_BLE_WHITELIST_ADD if add address to whitelist operation success, XR_BLE_WHITELIST_REMOVE if remove address from the whitelist operation success */
    } update_whitelist_cmpl;                        /*!< Event parameter of XR_GAP_BLE_UPDATE_WHITELIST_COMPLETE_EVT */
} t_xr_ble_gap_cb_param;

typedef void (* t_gap_ble_cb)(t_xr_gap_ble_cb_event event, t_xr_ble_gap_cb_param *param);

int32_t ble_gap_reg_cb(t_gap_ble_cb callback);

int32_t ble_gap_start_advertising (t_xr_ble_adv_params *adv_params);

int32_t ble_gap_config_adv_data(t_xr_ble_adv_data *adv_data);

int32_t ble_gap_update_conn_params(t_xr_ble_conn_update_params *params);

int32_t ble_gap_set_device_name(const char *name);

int32_t ble_gap_security_rsp(t_xr_bd_addr bd_addr, bool accept);

#ifdef __cplusplus
}
#endif

#endif /* __XR_GAP_BLE_API_H__ */
