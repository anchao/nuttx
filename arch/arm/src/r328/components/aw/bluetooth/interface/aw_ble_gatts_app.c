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
#include <string.h>
#include "kernel/os/os.h"

#include "aw_bt_defs.h"
#include "aw_gatt_defs.h"
#include "aw_ble_gatts_api.h"
#include "aw_ble_gap_api.h"
#include "sys_ctrl/config.h"
//#include "net/bluedroid/config.h"

#if CONFIG_GATTS_ENABLE	
#define loge(format, arg...) printf("Err : <%s : %d> " format, __func__, __LINE__, ##arg)
#define logw(format, arg...) printf("Warn: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logi(format, arg...) printf("Info: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logd(format, arg...) printf("Dbg : <%s : %d> " format, __func__, __LINE__, ##arg)

void print_hex_dump_bytes(const void *addr, unsigned int len);

#define GATTS_SERVICE_UUID_DEMO   	0x00FF
#define GATTS_CHAR_UUID_DEMO      	0xFF01
#define GATTS_NUM_HANDLE_TEST   	4

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define DEMO_DEVICE_NAME			"GATT_SERVICE_DEMO"

uint8_t char_str[] = {0x11,0x22,0x33};
uint8_t demo_property = 0;

t_xr_ble_conn_update_params conn_params = {{0}};

t_attr_value gatts_demo_char_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char_str),
    .attr_value   = char_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static t_xr_ble_adv_params adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = BLE_ADV_TYPE_IND,
    .own_addr_type      = XR_BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type    	=
    .channel_map        = BLE_ADV_CHNL_ALL,
    .adv_filter_policy 	= BLE_ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

//adv data
static t_xr_ble_adv_data adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, 			//TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, 	//&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (XR_BLE_ADV_FLAG_GEN_DISC | XR_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static t_xr_ble_adv_data scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, 			//TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, 	//&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (XR_BLE_ADV_FLAG_GEN_DISC | XR_BLE_ADV_FLAG_BREDR_NOT_SPT),
};


struct gatts_app_property {
	uint8_t 	app_id;
	uint8_t 	gatts_if;
	uint16_t 	conn_id;
	t_gatt_service_pty 		service_pty;
	t_gatt_char_pty 		char_pty;
	t_gatt_char_descr_pty 	char_descr_pty;
};

#define PROFILE_APP_ID	0
struct gatts_app_property gatts_app_A;


#define PREPARE_BUF_MAX_SIZE 1024
typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_envent;

static void gap_cb_event_handler(t_xr_gap_ble_cb_event event, t_xr_ble_gap_cb_param *param)
{
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case XR_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            ble_gap_start_advertising(&adv_params);
        }
        break;
    case XR_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case XR_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            ble_gap_start_advertising(&adv_params);
        }
        break;
    case XR_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case XR_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != XR_BT_STATUS_SUCCESS) {
            loge("Advertising start failed\n");
        }
        break;
    case XR_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != XR_BT_STATUS_SUCCESS) {
            loge("Advertising stop failed\n");
        }
        else {
            logi("Stop adv successfully\n");
        }
        break;
    case XR_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         logi("update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d\n",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
#if CONFIG_SMP_ENABLE
    case XR_GAP_BLE_SEC_REQ_EVT:
    {
    	int i = 0;
        for(i = 0; i < XR_BD_ADDR_LEN; i++) {
             logd("%x:\n",param->ble_security.ble_req.bd_addr[i]);
        }
        ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	    break;
	}
    case XR_GAP_BLE_AUTH_CMPL_EVT:
    {
        t_xr_bd_addr bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(t_xr_bd_addr));
        logi("remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        logi("address type = %d", param->ble_security.auth_cmpl.addr_type);
        logi("pair status = %s\n",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            loge("fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    }
#endif
    default:
        break;
    }
}

void gatts_demo_create_service(void)
{
	gatts_app_A.service_pty.service_id.is_primary = true;
	gatts_app_A.service_pty.service_id.inst_id = 0x00;
	gatts_app_A.service_pty.service_id.uuid.len = UUID_LEN_16;
	gatts_app_A.service_pty.service_id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_DEMO;

	int32_t res = ble_gap_config_adv_data(&adv_data);
	if (res){
		loge("config adv data failed, error code = %x\n", res);
	}
	adv_config_done |= adv_config_flag;

	res = ble_gap_config_adv_data(&scan_rsp_data);
	if (res){
		loge("config scan response data failed, error code = %x\n", res);
	}
	adv_config_done |= scan_rsp_config_flag;

	ble_gatts_create_service(gatts_app_A.gatts_if, &gatts_app_A.service_pty, GATTS_NUM_HANDLE_TEST);
}

void gatts_demo_add_char(void)
{
	gatts_app_A.char_pty.char_uuid.len = UUID_LEN_16;
	gatts_app_A.char_pty.char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_DEMO;

	demo_property = XR_GATT_CHAR_PROP_BIT_READ | XR_GATT_CHAR_PROP_BIT_WRITE | XR_GATT_CHAR_PROP_BIT_NOTIFY;

	int32_t res = ble_gatts_add_char(gatts_app_A.service_pty.service_handle, &gatts_app_A.char_pty.char_uuid,
									 XR_GATT_PERM_READ | XR_GATT_PERM_WRITE,
									 demo_property, &gatts_demo_char_val, NULL);
	if (res) {
		loge("add char failed, error code = %x\n", res);
	}
}

void gatts_demo_add_char_descr(void)
{
	gatts_app_A.char_descr_pty.desrc_uuid.len = UUID_LEN_16;
	gatts_app_A.char_descr_pty.desrc_uuid.uuid.uuid16 = XR_GATT_UUID_CHAR_CLIENT_CONFIG;

	int32_t res = ble_gatts_add_char_descr(gatts_app_A.service_pty.service_handle, &gatts_app_A.char_descr_pty.desrc_uuid,
											XR_GATT_PERM_READ | XR_GATT_PERM_WRITE, NULL, NULL);

	if (res) {
		loge("add char descr failed, error code = %x\n", res);
	}
}

void gap_update_conn_params(t_xr_bd_addr bda)
{
	t_xr_ble_conn_update_params xr_conn_params = {{0}};

	memcpy(xr_conn_params.bda, bda, sizeof(t_xr_bd_addr));
	/* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
	xr_conn_params.latency = 0;
    xr_conn_params.max_int = 0x20;    	// max_int = 0x20*1.25ms = 40ms
    xr_conn_params.min_int = 0x10;    	// min_int = 0x10*1.25ms = 20ms
    xr_conn_params.timeout = 400;    	// timeout = 400*10ms = 4000ms

	ble_gap_update_conn_params(&xr_conn_params);
}

void write_event_env_demo(uint8_t gatts_if, prepare_type_env_t *prepare_write_env, t_ble_gatts_cb_arg *param)
{
    t_gatt_status status = XR_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    loge("Gatt_server prep no mem\n");
                    status = XR_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = XR_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = XR_GATT_INVALID_ATTR_LEN;
                }
            }
			
            t_gatt_rsp *gatt_rsp = (t_gatt_rsp *)malloc(sizeof(t_gatt_rsp));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = XR_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            int16_t response_err = ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != 0){
               loge("Send response error\n");
            }
            free(gatt_rsp);
            if (status != XR_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}


static void ble_gatts_cb_event_handler(t_ble_gatts_cb_event cb_event, uint8_t gatts_if, t_ble_gatts_cb_arg *arg)
{
	switch(cb_event)
	{
		case BLE_GATTS_REG_EVT:
			OS_Sleep(1);
			logi("REGSITER_APP_EVT, status %d, app_id %d, gatts_if:%d\n", arg->reg.status, arg->reg.app_id, gatts_if);
			gatts_app_A.gatts_if = gatts_if;
			uint8_t res = ble_gap_set_device_name(DEMO_DEVICE_NAME);
			if (res) {
				loge("set device name failed, error code = %x\n", res);
			}
			else {
				logi("ble device name: %s\n", DEMO_DEVICE_NAME);
			}
			gatts_demo_create_service();
			break;
		case BLE_GATTS_CREATE_EVT:
			logi("CREATE_SERVICE_EVT, status %d, service_handle %d\n", arg->create.status, arg->create.service_handle);
			gatts_app_A.service_pty.service_handle = arg->create.service_handle;

			ble_gatts_start_service(gatts_app_A.service_pty.service_handle);
			gatts_demo_add_char();
			break;
		case BLE_GATTS_START_EVT:
			logi("SERVICE_START_EVT, status %d, service_handle %d\n", arg->start.status, arg->start.service_handle);
			break;
		case BLE_GATTS_ADD_CHAR_EVT:
		{	
			gatts_app_A.char_pty.char_handle = arg->add_char.attr_handle;
			uint16_t length = 0;
			const uint8_t *prf_char;
			int32_t ret = ble_gatts_get_attr_value(arg->add_char.attr_handle, &length, &prf_char);
			if (ret == -1){
				loge("ILLEGAL HANDLE");
			}
			logi("the gatts demo char length = %x\n", length);
			int i = 0;
			for (i = 0; i < length; i++){
				logi("prf_char[%x] = %x\n", i, prf_char[i]);
			}
			gatts_demo_add_char_descr();
			break;
		}
		case BLE_GATTS_ADD_CHAR_DESCR_EVT:
			gatts_app_A.char_descr_pty.descr_handle = arg->add_char_descr.attr_handle;
			logi("ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n", 
					arg->add_char_descr.status, arg->add_char_descr.attr_handle, arg->add_char_descr.service_handle);
			break;
		case BLE_GATTS_CONNECT_EVT:
			gatts_app_A.conn_id = arg->connect.conn_id;
			logi("GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:\n",
					arg->connect.conn_id,
                 	arg->connect.remote_bda[0], arg->connect.remote_bda[1], arg->connect.remote_bda[2],
                 	arg->connect.remote_bda[3], arg->connect.remote_bda[4], arg->connect.remote_bda[5]);
			gap_update_conn_params(arg->connect.remote_bda);
			break;
		case BLE_GATTS_DISCONNECT_EVT:
			logi("GATTS_DISCONNECT_EVT\n");
			break;
		case BLE_GATTS_WRITE_EVT:
			logi("GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", 
					arg->write.conn_id, arg->write.trans_id, arg->write.handle);
			if (!arg->write.is_prep){
				logd("Data:");
				uint16_t len = 0;
				const uint8_t *read_value;
				ble_gatts_read_value(arg, &len, &read_value);
				print_hex_dump_bytes(read_value, len);
				
				//print_hex_dump_bytes(arg->write.value, arg->write.len);
				if (gatts_app_A.char_descr_pty.descr_handle == arg->write.handle && arg->write.len == 2){
					uint16_t descr_value = arg->write.value[1]<<8 | arg->write.value[0];
					if (descr_value == 0x0001){
						if (demo_property & XR_GATT_CHAR_PROP_BIT_NOTIFY){
							logi("notify enable\n");
							uint8_t notify_data[15];
							int i = 0;
							for (i = 0; i < sizeof(notify_data); ++i)
                        	{
                            	notify_data[i] = i%0xff;
                        	}
							ble_gatts_send_indicate(gatts_if, arg->write.conn_id, gatts_app_A.char_pty.char_handle,
                                                sizeof(notify_data), notify_data, false);
						}
					}
					else if (descr_value == 0x0002){
						if (demo_property & XR_GATT_CHAR_PROP_BIT_INDICATE){
                        	logi("indicate enable\n");
                        	uint8_t indicate_data[15];
							int i = 0;
                        	for (i = 0; i < sizeof(indicate_data); ++i)
                        	{
                         	   indicate_data[i] = i%0xff;
                        	}
                       	 	//the size of indicate_data[] need less than MTU size
                        	ble_gatts_send_indicate(gatts_if, arg->write.conn_id, gatts_app_A.char_pty.char_handle,
                                                sizeof(indicate_data), indicate_data, true);
                    	}
					}
					else if (descr_value == 0x0000){
						logi("notify/indicate disable\n");
					}
					else{
						loge("unknown descr value\n");
						print_hex_dump_bytes(arg->write.value, arg->write.len);
					}
				}
			}
			write_event_env_demo(gatts_if, &prepare_write_envent, arg);
			break;
		case BLE_GATTS_MTU_EVT:
			logi("GATTS_MTU_EVT, MTU %d\n", arg->mtu.mtu);
			break;
		default:
			loge("cb_event not exist.\n");
	}
}

void ble_gatts_demo(void)
{
	int32_t res;

	OS_Sleep(2);
	
	res = ble_gatts_reg_cb(ble_gatts_cb_event_handler);
	if (res != 0) {
		loge("GATTS register error.\n");
		return;
	}
	
	res = ble_gap_reg_cb(gap_cb_event_handler);
	if (res != 0) {
		loge("GAP register error.\n");
		return;
	}

	gatts_app_A.app_id = PROFILE_APP_ID;
	res = ble_gatts_app_reg(gatts_app_A.app_id);
	if (res) {
		loge("GATTS app register error, error code = %x.\n", res);
		return;
	}

	gatts_demo_create_service();
	gatts_demo_add_char();
	gatts_demo_add_char_descr();

	res = ble_gatts_set_local_mtu(500);
	if (res) {
		loge("set local MTU failed, error code = %x\n", res);
	}
	
	return;
}

#endif
