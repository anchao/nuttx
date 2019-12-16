/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the people's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY¡¯S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS¡¯SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY¡¯S TECHNOLOGY.
*
*
* THIS SOFTWARE IS PROVIDED BY ALLWINNER"AS IS" AND TO THE MAXIMUM EXTENT
* PERMITTED BY LAW, ALLWINNER EXPRESSLY DISCLAIMS ALL WARRANTIES OF ANY KIND,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING WITHOUT LIMITATION REGARDING
* THE TITLE, NON-INFRINGEMENT, ACCURACY, CONDITION, COMPLETENESS, PERFORMANCE
* OR MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* IN NO EVENT SHALL ALLWINNER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __HAL_DMA_H__
#define __HAL_DMA_H__

#include <stdint.h>
#include <stdio.h>
#include "sunxi-dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/** This enum defines the DMA CHANNEL status. */
typedef enum {
    HAL_DMA_CHAN_STATUS_BUSY  = 0,              /* DMA channel status busy */
    HAL_DMA_CHAN_STATUS_FREE = 1               /* DMA channel status free */
} hal_dma_chan_status_t;

/** This enum defines the return type of GPIO API. */
typedef enum {
    HAL_DMA_STATUS_INVALID_PARAMETER         = -2,     /**< Invalid status. */
    HAL_DMA_STATUS_ERROR         = -1,     /**< Invalid input parameter. */
    HAL_DMA_STATUS_OK                = 0       /**< The DMA status ok. */
} hal_dma_status_t;


hal_dma_chan_status_t hal_dma_chan_request(unsigned long **hdma);
hal_dma_status_t hal_dma_prep_cyclic(unsigned long *hdma, uint32_t buf_addr, uint32_t buf_len, uint32_t period_len, enum dma_transfer_direction dir);
hal_dma_status_t hal_dma_cyclic_callback_install(unsigned long *hdma, dma_callback callback, void *callback_param);
hal_dma_status_t hal_dma_prep_memcpy(unsigned long *hdma, uint32_t dest, uint32_t src, uint32_t len);
hal_dma_status_t hal_dma_prep_device(unsigned long *hdma, uint32_t dest, uint32_t src, uint32_t len, enum dma_transfer_direction dir);
hal_dma_status_t hal_dma_slave_config(unsigned long *hdma, struct dma_slave_config*config);
enum dma_status hal_dma_tx_status(unsigned long *hdma, uint32_t *left_size);
hal_dma_status_t hal_dma_start(unsigned long *hdma);
hal_dma_status_t hal_dma_stop(unsigned long *hdma);
hal_dma_status_t hal_dma_chan_free(unsigned long *hdma);
void * dma_map_area(void * addr, size_t size, enum dma_transfer_direction dir);
void dma_unmap_area(void * addr, size_t size, enum dma_transfer_direction dir);
#ifdef __cplusplus
}
#endif

#endif
