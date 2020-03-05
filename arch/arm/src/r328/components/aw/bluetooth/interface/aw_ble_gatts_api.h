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

#ifndef __AW_BLE_GATTS_API_H__
#define __AW_BLE_GATTS_API_H__

#include "aw_bt_defs.h"
#include "aw_gatt_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/// GATT Server callback function events
typedef enum {
    BLE_GATTS_REG_EVT                = 0,       /*!< When register application id, the event comes */
    BLE_GATTS_READ_EVT               = 1,       /*!< When gatt client request read operation, the event comes */
    BLE_GATTS_WRITE_EVT              = 2,       /*!< When gatt client request write operation, the event comes */
    BLE_GATTS_EXEC_WRITE_EVT         = 3,       /*!< When gatt client request execute write, the event comes */
    BLE_GATTS_MTU_EVT                = 4,       /*!< When set mtu complete, the event comes */
    BLE_GATTS_CONF_EVT               = 5,       /*!< When receive confirm, the event comes */
    BLE_GATTS_UNREG_EVT              = 6,       /*!< When unregister application id, the event comes */
    BLE_GATTS_CREATE_EVT             = 7,       /*!< When create service complete, the event comes */
    BLE_GATTS_ADD_INCL_SRVC_EVT      = 8,       /*!< When add included service complete, the event comes */
    BLE_GATTS_ADD_CHAR_EVT           = 9,       /*!< When add characteristic complete, the event comes */
    BLE_GATTS_ADD_CHAR_DESCR_EVT     = 10,      /*!< When add descriptor complete, the event comes */
    BLE_GATTS_DELETE_EVT             = 11,      /*!< When delete service complete, the event comes */
    BLE_GATTS_START_EVT              = 12,      /*!< When start service complete, the event comes */
    BLE_GATTS_STOP_EVT               = 13,      /*!< When stop service complete, the event comes */
    BLE_GATTS_CONNECT_EVT            = 14,      /*!< When gatt client connect, the event comes */
    BLE_GATTS_DISCONNECT_EVT         = 15,      /*!< When gatt client disconnect, the event comes */
    BLE_GATTS_OPEN_EVT               = 16,      /*!< When connect to peer, the event comes */
    BLE_GATTS_CANCEL_OPEN_EVT        = 17,      /*!< When disconnect from peer, the event comes */
    BLE_GATTS_CLOSE_EVT              = 18,      /*!< When gatt server close, the event comes */
    BLE_GATTS_LISTEN_EVT             = 19,      /*!< When gatt listen to be connected the event comes */
    BLE_GATTS_CONGEST_EVT            = 20,      /*!< When congest happen, the event comes */
    /* following is extra event */
    BLE_GATTS_RESPONSE_EVT           = 21,      /*!< When gatt send response complete, the event comes */
    BLE_GATTS_CREAT_ATTR_TAB_EVT     = 22,
    BLE_GATTS_SET_ATTR_VAL_EVT       = 23,
} t_ble_gatts_cb_event;

/**
 * @brief Gatt server callback parameters union
 */

typedef union {
    struct gatts_reg_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t app_id;                /*!< Application id which input in register API */
    } reg;                              /*!< Gatt server callback param of BLE_GATTS_REG_EVT */

    struct gatts_read_evt {
        uint16_t conn_id;               /*!< Connection id */
        uint32_t trans_id;              /*!< Transfer id */
        t_xr_bd_addr bda;              /*!< The bluetooth device address which been read */
        uint16_t handle;                /*!< The attribute handle */
        uint16_t offset;                /*!< Offset of the value, if the value is too long */
        bool is_long;                   /*!< The value is too long or not */
        bool need_rsp;                  /*!< The read operation need to do response */
    } read;                             /*!< Gatt server callback param of BLE_GATTS_READ_EVT */

    struct gatts_write_evt {
        uint16_t conn_id;               /*!< Connection id */
        uint32_t trans_id;              /*!< Transfer id */
        t_xr_bd_addr bda;              /*!< The bluetooth device address which been written */
        uint16_t handle;                /*!< The attribute handle */
        uint16_t offset;                /*!< Offset of the value, if the value is too long */
        bool need_rsp;                  /*!< The write operation need to do response */
        bool is_prep;                   /*!< This write operation is prepare write */
        uint16_t len;                   /*!< The write attribute value length */
        uint8_t *value;                 /*!< The write attribute value */
    } write;                            /*!< Gatt server callback param of BLE_GATTS_WRITE_EVT */

    struct gatts_exec_write_evt {
        uint16_t conn_id;               /*!< Connection id */
        uint32_t trans_id;              /*!< Transfer id */
        t_xr_bd_addr bda;              /*!< The bluetooth device address which been written */
#define XR_GATT_PREP_WRITE_CANCEL 0x00 /*!< Prepare write flag to indicate cancel prepare write */
#define XR_GATT_PREP_WRITE_EXEC   0x01 /*!< Prepare write flag to indicate execute prepare write */
        uint8_t exec_write_flag;        /*!< Execute write flag */
    } exec_write;                       /*!< Gatt server callback param of BLE_GATTS_EXEC_WRITE_EVT */

    struct gatts_mtu_evt {
        uint16_t conn_id;               /*!< Connection id */
        uint16_t mtu;                   /*!< MTU size */
    } mtu;                              /*!< Gatt server callback param of BLE_GATTS_MTU_EVT */

    struct gatts_conf_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t conn_id;               /*!< Connection id */
        uint16_t len;                   /*!< The indication or notification value length, len is valid when send notification or indication failed */
        uint8_t *value;                 /*!< The indication or notification value , value is valid when send notification or indication failed */
    } conf;                             /*!< Gatt server callback param of BLE_GATTS_CONF_EVT (confirm) */

    struct gatts_create_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t service_handle;        /*!< Service attribute handle */
        t_gatt_srvc_id service_id;  	/*!< Service id, include service uuid and other information */
    } create;                           /*!< Gatt server callback param of BLE_GATTS_CREATE_EVT */

    struct gatts_add_incl_srvc_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t attr_handle;           /*!< Included service attribute handle */
        uint16_t service_handle;        /*!< Service attribute handle */
    } add_incl_srvc;                    /*!< Gatt server callback param of BLE_GATTS_ADD_INCL_SRVC_EVT */

    struct gatts_add_char_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t attr_handle;           /*!< Characteristic attribute handle */
        uint16_t service_handle;        /*!< Service attribute handle */
        t_bt_uuid char_uuid;        	/*!< Characteristic uuid */
    } add_char;                         /*!< Gatt server callback param of BLE_GATTS_ADD_CHAR_EVT */

    struct gatts_add_char_descr_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t attr_handle;           /*!< Descriptor attribute handle */
        uint16_t service_handle;        /*!< Service attribute handle */
        t_bt_uuid descr_uuid;       	/*!< Characteristic descriptor uuid */
    } add_char_descr;                   /*!< Gatt server callback param of BLE_GATTS_ADD_CHAR_DESCR_EVT */

    struct gatts_delete_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t service_handle;        /*!< Service attribute handle */
    } del;                              /*!< Gatt server callback param of BLE_GATTS_DELETE_EVT */

    struct gatts_start_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t service_handle;        /*!< Service attribute handle */
    } start;                            /*!< Gatt server callback param of BLE_GATTS_START_EVT */

    struct gatts_stop_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t service_handle;        /*!< Service attribute handle */
    } stop;                             /*!< Gatt server callback param of BLE_GATTS_STOP_EVT */

    struct gatts_connect_evt {
        uint16_t conn_id;               /*!< Connection id */
        t_xr_bd_addr remote_bda;       	/*!< Remote bluetooth device address */
    } connect;                          /*!< Gatt server callback param of BLE_GATTS_CONNECT_EVT */

    struct gatts_disconnect_evt {
        uint16_t conn_id;               /*!< Connection id */
        t_xr_bd_addr remote_bda;       	/*!< Remote bluetooth device address */
        t_gatt_conn_reason reason;  	/*!< Indicate the reason of disconnection */
    } disconnect;                       /*!< Gatt server callback param of BLE_GATTS_DISCONNECT_EVT */

    struct gatts_open_evt {
        t_gatt_status status;       	/*!< Operation status */
    } open;                             /*!< Gatt server callback param of BLE_GATTS_OPEN_EVT */

    struct gatts_cancel_open_evt {
        t_gatt_status status;       	/*!< Operation status */
    } cancel_open;                      /*!< Gatt server callback param of BLE_GATTS_CANCEL_OPEN_EVT */

    struct gatts_close_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t conn_id;               /*!< Connection id */
    } close;                            /*!< Gatt server callback param of BLE_GATTS_CLOSE_EVT */

    struct gatts_congest_evt {
        uint16_t conn_id;               /*!< Connection id */
        bool congested;                 /*!< Congested or not */
    } congest;                          /*!< Gatt server callback param of BLE_GATTS_CONGEST_EVT */

    struct gatts_rsp_evt {
        t_gatt_status status;       	/*!< Operation status */
        uint16_t handle;                /*!< Attribute handle which send response */
    } rsp;                              /*!< Gatt server callback param of BLE_GATTS_RESPONSE_EVT */

    struct gatts_add_attr_tab_evt {
        t_gatt_status status;       	/*!< Operation status */
        t_bt_uuid svc_uuid;         	/*!< Service uuid type */
        uint16_t num_handle;            /*!< The number of the attribute handle to be added to the gatts database */
        uint16_t *handles;              /*!< The number to the handles */
    } add_attr_tab;                     /*!< Gatt server callback param of BLE_GATTS_CREAT_ATTR_TAB_EVT */

    struct gatts_set_attr_val_evt{
        uint16_t srvc_handle;           /*!< The service handle */
        uint16_t attr_handle;           /*!< The attribute  handle */
        t_gatt_status status;       	/*!< Operation status*/
    } set_attr_val;                     /*!< Gatt server callback param of BLE_GATTS_SET_ATTR_VAL_EVT */

} t_ble_gatts_cb_arg;

typedef void ( *t_ble_gatts_cb)(t_ble_gatts_cb_event event, uint8_t gatts_itf, t_ble_gatts_cb_arg *arg);

int32_t ble_gatts_reg_cb(t_ble_gatts_cb callback);

int32_t ble_gatts_app_reg(uint16_t app_id);

int32_t ble_gatts_set_local_mtu(uint16_t setMTU);

int32_t ble_gatts_create_service(uint8_t gatts_if,
                                       t_gatt_service_pty *service_pty, uint16_t num_handle);

int32_t ble_gatts_start_service(uint16_t service_handle);

int32_t ble_gatts_add_char(uint16_t service_handle,  t_bt_uuid  *char_uuid,
                                 uint16_t perm, uint8_t property, t_attr_value *char_val,
                                 t_attr_control *control);

int32_t ble_gatts_add_char_descr(uint16_t service_handle, t_bt_uuid *descr_uuid,
                                        uint16_t perm, t_attr_value *char_descr_val,
                                        t_attr_control *control);

t_gatt_status ble_gatts_get_attr_value(uint16_t attr_handle, uint16_t *length, const uint8_t **value);

int32_t ble_gatts_read_value(t_ble_gatts_cb_arg *param, uint16_t *len, const void **value);

int32_t ble_gatts_send_indicate(uint8_t gatts_if, uint16_t conn_id, uint16_t attr_handle,
                                      uint16_t value_len, uint8_t *value, bool need_confirm);

int32_t ble_gatts_send_response(uint8_t gatts_if, uint16_t conn_id, uint32_t trans_id,
                                      t_gatt_status status, t_gatt_rsp *rsp);

#ifdef __cplusplus
}
#endif

#endif /* __AW_BLE_GATTS_API_H__ */
