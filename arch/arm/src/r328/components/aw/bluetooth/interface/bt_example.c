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

#include <stdlib.h>
#include <string.h>

#include "kernel/os/os.h"

#include "sys_ctrl/config.h"
#include "bt_example.h"
#include "bt_app_core.h"
#include "bt_app_gap.h"
#include "bt_app_audio.h"
//#include "driver/xr829/drv_xr829_bt.h"
#include "vendor.h"
//#include "../net_ctrl/sysinfo.h"

#include "aw_bt_a2dp_app.h"
#include "aw_bt_avrcp_app.h"
#include "aw_ble_gatts_app.h"
#include "aw_bt_common_api.h"

#define A2DP_SINK_EXAMPLE   (1)
#define A2DP_SOURCE_EXAMPLE (0)
#define AVRCP_EXAMPLE       (1)
#define HFP_EXAMPLE         (0)
#define GATTS_EXAMPLE       (1)
#define GATTC_EXAMPLE       (0)

#if ((GATTS_EXAMPLE && (!CONFIG_GATTS_ENABLE)) || (GATTC_EXAMPLE && (!CONFIG_GATTC_ENABLE)))
#error "gatt is not configed to enable now!"
#endif

#define loge(format, arg...) printf("Err : <%s : %d> " format, __func__, __LINE__, ##arg)
#define logw(format, arg...) printf("Warn: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logi(format, arg...) printf("Info: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logd(format, arg...) printf("Dbg : <%s : %d> " format, __func__, __LINE__, ##arg)

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/* handler for bluetooth stack enabled events */
static void bt_profile_hdl_stack_evt(uint16_t event, void *p_param);

static int vendor_send_command(vendor_opcode_t opcode, void *param)
{
	int ret = -1;
	switch (opcode)
	{
		case VENDOR_CHIP_POWER_CONTROL:
			break;
		case VENDOR_OPEN_USERIAL:
			break;
		case VENDOR_CLOSE_USERIAL:
			break;
		case VENDOR_GET_LPM_IDLE_TIMEOUT:
			//ret = DRV_XR829_Bluetooth_Op(DRV_BLUETOOTH_OP_LPM_GET_TIMEOUT, param);
			break;
		case VENDOR_SET_LPM_WAKE_STATE:
		{
			uint8_t *state = (uint8_t *) param;
			uint8_t wakeup_set = (*state == BT_VND_LPM_WAKE_ASSERT) ? 1 : 0;
			//ret = DRV_XR829_Bluetooth_Op(DRV_BLUETOOTH_OP_LPM_WAKE_SET_STATE, &wakeup_set);
		}
			break;
		case VENDOR_SET_AUDIO_STATE:
			break;
		default :
			break;
	}
	return ret;
}

static int vendor_send_async_command(vendor_async_opcode_t opcode, void *param)
{
	int ret = -1;
	switch (opcode)
	{
		case VENDOR_CONFIGURE_FIRMWARE:
			break;
		case VENDOR_CONFIGURE_SCO:
			break;
		case VENDOR_SET_LPM_MODE:
			//ret = DRV_XR829_Bluetooth_Op(DRV_BLUETOOTH_OP_LPM_MODE, param);
			break;
		case VENDOR_DO_EPILOG:
			break;
		default :
			break;
	}
	return ret;
}

void vendor_set_callback(vendor_async_opcode_t opcode, vendor_cb callback)
{
    if (opcode >= VENDOR_LAST_OP)
        return;

    switch (opcode)
    {
        case VENDOR_CONFIGURE_FIRMWARE:
        break;
    case VENDOR_CONFIGURE_SCO:
        break;
    case VENDOR_SET_LPM_MODE:
       // DRV_XR829_Bluetooth_Set_CB(DRV_BLUETOOTH_OP_LPM_MODE, callback);
        break;
    case VENDOR_DO_EPILOG:
        break;
    default :
        break;
    }
}
/*
static const vendor_t xr829_vendor = {
	.send_command = vendor_send_command,
	.send_async_command = vendor_send_async_command,
	.transmit = DRV_XR829_Bluetooth_Transmit,
	.receive = DRV_XR829_Bluetooth_Receive,
	.set_callback = vendor_set_callback,
};
*/
int bt_example(void)
{
	/* for showing the log, not flush by netos log. */
/*
	OS_MSleep(100);
	printf("\nBluedroid version %s\n\n", BLUEDROID_VERSION_STR);

	drv_bluetooth_config btcfg = {
			.sco_cfg = {DRV_BLUETOOTH_SCO_PCM, DRV_BLUETOOTH_SCO_CSVD},
			.lpm = DRV_BLUETOOTH_LPM_DISABLE,
			.result = NULL,
	};
	struct sysinfo *sys_info = sysinfo_get();
	memcpy(btcfg.mac, sys_info->mac_addr, DRV_BLUETOOTH_MAC_ADDR_LENGTH);
	btcfg.mac[0]++;
	btcfg.mac[0] = btcfg.mac[0]^btcfg.mac[5];
	btcfg.mac[1] = btcfg.mac[1]^btcfg.mac[4];
	if (btcfg.mac[2] == 0x9E)
		btcfg.mac[2]--;
	printf("!!!!!!!! bt device address: %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X !!!!!!!!\n",
		   btcfg.mac[5], btcfg.mac[4], btcfg.mac[3], btcfg.mac[2], btcfg.mac[1], btcfg.mac[0]);

	DRV_XR829_Bluetooth_Init(&btcfg);
	
	vendor_set_interface(&xr829_vendor);
	*/
	int32_t err;

	bt_app_audio_init();

	err = sys_ctrl_create();
	if(err) {
		loge("sys create failed.\n");
	}
	
	
    if ((err = bluedroid_init()) != 0) {
        loge("%s initialize bluedroid failed: %d\n", __func__, err);
    }

    if ((err = bluedroid_enable()) != 0) {
        loge("%s enable bluedroid failed: %d\n", __func__, err);
    }

#if (GATTS_EXAMPLE && CONFIG_GATTS_ENABLE)
	//void gatts_demo();
	//gatts_demo();
	//void ble_gatts_demo();
	ble_gatts_demo();
#endif

#if (GATTC_EXAMPLE && CONFIG_GATTC_ENABLE)
	void gattc_demo();
	gattc_demo();
#endif
	
    /* create application task */
    bt_app_task_start_up();

	printf("bt app started!\n");

    /* Bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_profile_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

	return 0;
}

static void bt_profile_hdl_stack_evt(uint16_t event, void *p_param)
{
    logd("%s evt %d\n", __func__, event);
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        /* set up device name */
        char dev_name[36];
		sprintf(dev_name, "RTT_BT_TEST_%02x:%02x:%02x:%02x:%02x:%02x", XR_BT_BD_ADDR_HEX(bt_dev_get_address()));
		printf("!!!!!!!! bt device name: %s !!!!!!!!\n", dev_name);
        bt_dev_set_device_name(dev_name);
		
#if A2DP_SINK_EXAMPLE
		//bt_app_a2dp_sink_enable();
		bt_app_a2dp_sink_demo();
#endif

#if AVRCP_EXAMPLE
		//bt_app_avrcp_ct_enable();
		bt_app_avrcp_demo();
#endif

#if HFP_EXAMPLE
		bt_app_hf_enable();
#endif

#if A2DP_SOURCE_EXAMPLE
       	bt_app_a2dp_source_enable();
#endif

#if A2DP_SINK_EXAMPLE || AVRCP_EXAMPLE || HFP_EXAMPLE || A2DP_SOURCE_EXAMPLE
        bt_app_gap_enable();
#endif
        break;
    }
    default:
        loge("%s unhandled evt %d", __func__, event);
        break;
    }
}

