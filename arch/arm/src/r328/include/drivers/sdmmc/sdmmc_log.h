/*
 * Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
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
 *    3. Neither the name of ALLWINNERTECH TECHNOLOGY CO., LTD. nor the names of
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

#ifndef __SDMMC_LOG__
#define __SDMMC_LOG__

#ifdef __cplusplus
extern "C" {
#endif

#include <debug.h>

#define SDMMC_LOG_TAG printf

#define ROM_DUMP_MASK   (1 << 0)
#define ROM_DBG_MASK    (1 << 1)
#define ROM_INF_MASK    (1 << 2)
#define ROM_WRN_MASK    (1 << 3)
#define ROM_ERR_MASK    (1 << 4)
#define ROM_TOTAL_MASKS (ROM_DUMP_MASK | ROM_DBG_MASK | ROM_INF_MASK | \
                         ROM_WRN_MASK | ROM_ERR_MASK)

void print_hex_dump_words(const void *addr, unsigned int len);
void print_hex_dump_bytes(const void *addr, unsigned int len);

#define SDMMC_LOG(level, mask, fmt, arg...)       \
	do {                                            \
		if (level & mask)                       \
			SDMMC_LOG_TAG(fmt,##arg);  \
	} while (0)

#define SDMMC_DUMP_BYTES(level, addr, len)                \
	do {                                            \
		if (level & ROM_DUMP_MASK)              \
			print_hex_dump_bytes(addr, len);\
	} while (0)

#define SDMMC_DUMP_WORDS(level, addr, len)                \
	do {                                            \
		if (level & ROM_DUMP_MASK)              \
			print_hex_dump_words((void*)addr, len);\
	} while (0)

#define SDMMC_IT_LOG 0
#if SDMMC_IT_LOG
#define SDMMC_IT_LOGD(fmt, args...) SDMMC_LOG_TAG("[INT:DBG]" fmt,##args)
#define SDMMC_IT_LOGW(fmt, args...) SDMMC_LOG_TAG("[INT:WRN]" fmt,##args)
#define SDMMC_IT_LOGI(fmt, args...) SDMMC_LOG_TAG("[INT:INF]" fmt,##args)
#define SDMMC_IT_LOGE(fmt, args...) SDMMC_LOG_TAG("[INT:ERR]" fmt,##args)
#else
#define SDMMC_IT_LOGD(fmt, args...)
#define SDMMC_IT_LOGW(fmt, args...)
#define SDMMC_IT_LOGI(fmt, args...)
#define SDMMC_IT_LOGE(fmt, args...)
#endif

/*sdmmc host log*/
#define SD_LOGD(format, args...)     SDMMC_LOG(card->debug_mask,ROM_DBG_MASK,"[SD:DBG]" format, ##args)
#define SD_LOGI(format, args...)     SDMMC_LOG(card->debug_mask,ROM_INF_MASK,"[SD:INF]" format, ##args)
#define SD_LOGW(format, args...)     SDMMC_LOG(card->debug_mask,ROM_WRN_MASK,"[SD:WRN]" format, ##args)
#define SD_LOGE(format, args...)     SDMMC_LOG(card->debug_mask,ROM_ERR_MASK,"[SD:ERR]" format, ##args)
#define SD_DUMP_WORDS(a, l)          SDMMC_DUMP_WORDS(card->debug_mask, a, l)
#define SD_DUMP_BYTES(a, l)          SDMMC_DUMP_BYTES(card->debug_mask, a, l)


/*sdc card host log*/
#define SDC_LOGD(format, args...) SDMMC_LOG(host->debug_mask,ROM_DBG_MASK,"[SDC:DBG]" format, ##args)
#define SDC_LOGI(format, args...) SDMMC_LOG(host->debug_mask,ROM_INF_MASK,"[SDC:INF]" format, ##args)
#define SDC_LOGW(format, args...) SDMMC_LOG(host->debug_mask,ROM_WRN_MASK,"[SDC:WRN]" format, ##args)
#define SDC_LOGE(format, args...) SDMMC_LOG(host->debug_mask,ROM_ERR_MASK,"[SDC:ERR]" format, ##args)
#define SDC_DUMP_WORD(a, l)		  SDMMC_DUMP_WORDS(host->debug_mask, a, l)

#define SD_ASEERT(v) do {if(v) {SDMMC_LOG_TAG("ASEERT at:%s:%d!!!\n", __func__,__LINE__);while(1);}} while(0)
#define SD_WARN(v)   do {if(v) {SDMMC_LOG_TAG("WARNING at %s:%d!!!\n", __func__, __LINE__);}} while (0)

#ifdef __cplusplus
}
#endif

#endif /* _ROM_DRIVER_CHIP_SDMMC__SD_DEFINE_H_ */
