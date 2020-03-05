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
#ifndef __AW_BT_DEFS_H__
#define __AW_BT_DEFS_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* relate to BT_STATUS_xxx in bt_def.h */
/// Status Return Value
typedef enum {
    XR_BT_STATUS_SUCCESS       = 0,            /* relate to BT_STATUS_SUCCESS in bt_def.h */
    XR_BT_STATUS_FAIL,                         /* relate to BT_STATUS_FAIL in bt_def.h */
    XR_BT_STATUS_NOT_READY,                    /* relate to BT_STATUS_NOT_READY in bt_def.h */
    XR_BT_STATUS_NOMEM,                        /* relate to BT_STATUS_NOMEM in bt_def.h */
    XR_BT_STATUS_BUSY,                         /* relate to BT_STATUS_BUSY in bt_def.h */
    XR_BT_STATUS_DONE          = 5,            /* relate to BT_STATUS_DONE in bt_def.h */
    XR_BT_STATUS_UNSUPPORTED,                  /* relate to BT_STATUS_UNSUPPORTED in bt_def.h */
    XR_BT_STATUS_PARM_INVALID,                 /* relate to BT_STATUS_PARM_INVALID in bt_def.h */
    XR_BT_STATUS_UNHANDLED,                    /* relate to BT_STATUS_UNHANDLED in bt_def.h */
    XR_BT_STATUS_AUTH_FAILURE,                 /* relate to BT_STATUS_AUTH_FAILURE in bt_def.h */
    XR_BT_STATUS_RMT_DEV_DOWN  = 10,           /* relate to BT_STATUS_RMT_DEV_DOWN in bt_def.h */
    XR_BT_STATUS_AUTH_REJECTED,                /* relate to BT_STATUS_AUTH_REJECTED in bt_def.h */
    XR_BT_STATUS_INVALID_STATIC_RAND_ADDR,     /* relate to BT_STATUS_INVALID_STATIC_RAND_ADDR in bt_def.h */
    XR_BT_STATUS_PENDING,                      /* relate to BT_STATUS_PENDING in bt_def.h */
    XR_BT_STATUS_UNACCEPT_CONN_INTERVAL,       /* relate to BT_UNACCEPT_CONN_INTERVAL in bt_def.h */
    XR_BT_STATUS_PARAM_OUT_OF_RANGE,           /* relate to BT_PARAM_OUT_OF_RANGE in bt_def.h */
    XR_BT_STATUS_TIMEOUT,                      /* relate to BT_STATUS_TIMEOUT in bt_def.h */
    XR_BT_STATUS_PEER_LE_DATA_LEN_UNSUPPORTED, /* relate to BTM_PEER_LE_DATA_LEN_UNSUPPORTED in stack/btm_api.h */
    XR_BT_STATUS_CONTROL_LE_DATA_LEN_UNSUPPORTED,/* relate to BTM_CONTROL_LE_DATA_LEN_UNSUPPORTED in stack/btm_api.h */
    XR_BT_STATUS_ERR_ILLEGAL_PARAMETER_FMT,    /* relate to HCI_ERR_ILLEGAL_PARAMETER_FMT in stack/hcidefs.h */
    XR_BT_STATUS_MEMORY_FULL,                  /* relate to BT_STATUS_MEMORY_FULL in bt_def.h */
} t_bt_status;

/*Define the bt octet 16 bit size*/
#define XR_BT_OCTET16_LEN    16
typedef uint8_t t_xr_bt_octet16[XR_BT_OCTET16_LEN];   /* octet array: size 16 */

#define XR_BT_OCTET8_LEN    8
typedef uint8_t t_xr_bt_octet8[XR_BT_OCTET8_LEN];   /* octet array: size 8 */

typedef uint8_t xr_link_key[XR_BT_OCTET16_LEN];      /* Link Key */

/// UUID type
typedef struct {
#define UUID_LEN_16     2
#define UUID_LEN_32     4
#define UUID_LEN_128    16
    uint16_t len;							/*!< UUID length, 16bit, 32bit or 128bit */
    union {
        uint16_t    uuid16;
        uint32_t    uuid32;
        uint8_t     uuid128[UUID_LEN_128];
    } uuid;									/*!< UUID */
} __attribute__((packed)) t_bt_uuid;

/// Bluetooth device type
typedef enum {
    XR_BT_DEVICE_TYPE_BREDR   = 0x01,
    XR_BT_DEVICE_TYPE_BLE     = 0x02,
    XR_BT_DEVICE_TYPE_DUMO    = 0x03,
} t_bt_dev_type;

/// Bluetooth address length
#define XR_BD_ADDR_LEN     6
/// Bluetooth device address
typedef uint8_t t_xr_bd_addr[XR_BD_ADDR_LEN];

/// BLE device address type
typedef enum {
    XR_BLE_ADDR_TYPE_PUBLIC        = 0x00,
    XR_BLE_ADDR_TYPE_RANDOM        = 0x01,
    XR_BLE_ADDR_TYPE_RPA_PUBLIC    = 0x02,
    XR_BLE_ADDR_TYPE_RPA_RANDOM    = 0x03,
} t_ble_addr_type;

/// Used to exchange the encryption key in the init key & response key
#define XR_BLE_ENC_KEY_MASK    (1 << 0)            /* relate to BTM_BLE_ENC_KEY_MASK in stack/btm_api.h */
/// Used to exchange the IRK key in the init key & response key
#define XR_BLE_ID_KEY_MASK     (1 << 1)            /* relate to BTM_BLE_ID_KEY_MASK in stack/btm_api.h */
/// Used to exchange the CSRK key in the init key & response key
#define XR_BLE_CSR_KEY_MASK    (1 << 2)            /* relate to BTM_BLE_CSR_KEY_MASK in stack/btm_api.h */
/// Used to exchange the link key(this key just used in the BLE & BR/EDR coexist mode) in the init key & response key
#define XR_BLE_LINK_KEY_MASK   (1 << 3)            /* relate to BTM_BLE_LINK_KEY_MASK in stack/btm_api.h */
typedef uint8_t t_xr_ble_key_mask;            /* the key mask type */


#ifdef __cplusplus
}
#endif

#endif 