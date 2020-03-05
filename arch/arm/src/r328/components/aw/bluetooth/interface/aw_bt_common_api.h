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
#ifndef __AW_BT_COMMON_API_H_
#define __AW_BT_COMMON_API_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLUEDROID_VERSION_STR "1.0.0"

#define XR_BT_BD_ADDR_STR         "%02x:%02x:%02x:%02x:%02x:%02x"
#define XR_BT_BD_ADDR_HEX(addr)   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]

//bluedroid stack status type
typedef enum {
    XR_BLUEDROID_STATUS_UNINITIALIZED   = 0,        /*!< Bluetooth not initialized */
    XR_BLUEDROID_STATUS_INITIALIZED,                /*!< Bluetooth initialized but not enabled */
    XR_BLUEDROID_STATUS_ENABLED                     /*!< Bluetooth initialized and enabled */
} xr_bluedroid_status;

xr_bluedroid_status bluedroid_get_status(void);

int32_t bluedroid_enable(void);

int32_t bluedroid_disable(void);

int32_t bluedroid_init(void);

int32_t bluedroid_deinit(void);

const uint8_t *bt_dev_get_address(void);

int32_t bt_dev_set_device_name(const char *name);

#ifdef __cplusplus
}
#endif

#endif 