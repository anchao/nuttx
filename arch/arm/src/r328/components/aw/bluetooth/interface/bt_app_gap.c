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
#include <stdio.h>

#include "bt_app_gap.h"
#include "aw_bt_gap_api.h"
#include "sys_ctrl/config.h"

#define loge(format, arg...) printf("Err : <%s : %d> " format, __func__, __LINE__, ##arg)
#define logw(format, arg...) printf("Warn: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logi(format, arg...) printf("Info: <%s : %d> " format, __func__, __LINE__, ##arg)
#define logd(format, arg...) printf("Dbg : <%s : %d> " format, __func__, __LINE__, ##arg)

static void bt_app_gap_cb(t_xr_bt_gap_cb_event event, t_xr_bt_gap_cb_param *param)
{
    switch (event) {
#if CONFIG_BT_SSP_ENABLE
    case XR_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == XR_BT_STATUS_SUCCESS) {

            logi("[gap_bt] authentication success: ");
            if (strlen((const char *)param->auth_cmpl.device_name) != 0) {
                printf("device_name[%s] ", param->auth_cmpl.device_name);
            }

            printf("bd_addr[%02x:%02x:%02x:%02x:%02x:%02x]\n",
                    param->auth_cmpl.bda[0], param->auth_cmpl.bda[1], param->auth_cmpl.bda[2],
                    param->auth_cmpl.bda[3], param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
        } else {
            loge("authentication failed, status:%d\n", param->auth_cmpl.stat);
        }
        break;
    }
    case XR_BT_GAP_CFM_REQ_EVT:
        logi("BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d\n", param->cfm_req.num_val);
        bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case XR_BT_GAP_KEY_NOTIF_EVT:
        logi("BT_GAP_KEY_NOTIF_EVT passkey:%d\n", param->key_notif.passkey);
        break;
    case XR_BT_GAP_KEY_REQ_EVT:
        logi("BT_GAP_KEY_REQ_EVT Please enter passkey!\n");
        break;
#endif ///CONFIG_BT_SSP_ENABLE
    default: {
        logi("event: %d", event);
        break;
    }
    }
    return;
}


void bt_app_gap_enable(void)
{
	bt_gap_reg_cb(bt_app_gap_cb);

#if CONFIG_BT_SSP_ENABLE
    t_xr_bt_sp_param param_type = XR_BT_SP_IOCAP_MODE;
    t_xr_bt_io_cap iocap = XR_BT_IO_CAP_NONE;
    bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif ///CONFIG_BT_SSP_ENABLE

    /* set discoverable and connectable mode, wait to be connected */
    bt_gap_set_scan_mode(XR_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
}

