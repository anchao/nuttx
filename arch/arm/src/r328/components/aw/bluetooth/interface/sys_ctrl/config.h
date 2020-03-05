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

#define CONFIG_LOG_COLORS 1
#define CONFIG_LOG_DEFAULT_LEVEL 3 //BT_TRACE_LEVEL_API

#define CONFIG_BLE_SCAN_DUPLICATE 0
#define CONFIG_BLUEDROID_PINNED_TO_CORE 0

#define CONFIG_BT_BLE_DYNAMIC_ENV_MEMORY 0
#define CONFIG_OSI_INITIAL_TRACE_LEVEL BT_TRACE_LEVEL_EVENT
#define CONFIG_BTTRC_DUMP_BUFFER 0      /* 0:disable hcidump, 1:parse mode, 2: raw mode, 3:write mode */
#define CONFIG_A2DP_ENABLE 1
#define CONFIG_BT_ACL_CONNECTIONS 2 /* origin 4, 2 is ok by try */
#define CONFIG_HFP_CLIENT_ENABLE 1
#define CONFIG_CLASSIC_BT_ENABLED 1
#define CONFIG_GATTS_ENABLE 1
#define CONFIG_GATTC_ENABLE 1
#define CONFIG_BT_ENABLED 1
#define CONFIG_BLE_ENABLE 1
#define CONFIG_SMP_ENABLE 1
#define CONFIG_BT_SSP_ENABLE 1
#define CONFIG_HFP_AUDIO_DATA_PATH_PCM 1
#define CONFIG_HFP_ENABLE 1
#define CONFIG_BLUEDROID_ENABLED 1
#define CONFIG_A2DP_SOURCE_TASK_STACK_SIZE 2048
#define CONFIG_BT_SPP_ENABLED 0
#define CONFIG_A2DP_SINK_TASK_STACK_SIZE 2048
#define CONFIG_BT_STACK_NO_LOG 0
#define CONFIG_HCI_LAYER_RX 1
#define CONFIG_HCILOG_ENABLE 1
#define BLUEDROID7 0
#define CONFIG_SBC_PCM_FRAME_NUM 8 
