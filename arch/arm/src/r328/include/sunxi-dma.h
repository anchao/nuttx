/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the People's Republic of China and other countries.
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

#ifndef __SUNXI_DMA_H__
#define __SUNXI_DMA_H__
#include <stdint.h>
#include <stdbool.h>
//#include "platform.h"
//#include "aw_types.h"
//#include <aos/kernel.h>
//#include "k_api.h"

//#ifdef  CONFIG_ARCH_SUN8IW18P1
//#include "sun8i/dma-sun8iw18.h"
#include "dma-sun8iw18.h"
#define SUNXI_DMAC_PBASE         0X03002000
#define DMA_IRQ_NUM	66
//typedef char bool;
#include <aw_common.h>

//#endif

#ifdef __cplusplus
extern "C" {
#endif


#define NR_MAX_CHAN		10				/* total of channels */

#define HIGH_CHAN		8

#define DMA_IRQ_EN(x)		(SUNXI_DMAC_PBASE + (0x00 + ((x) << 2)))		/* Interrupt enable register */
#define DMA_IRQ_STAT(x)		(SUNXI_DMAC_PBASE + (0x10 + ((x) << 2)))		/* Interrupt status register */
#define DMA_SECURE		(SUNXI_DMAC_PBASE + 0x20)				/* DMA security register */
#define DMA_GATE		(SUNXI_DMAC_PBASE + 0x28)				/* DMA gating register */
#define DMA_MCLK_GATE		0x04
#define DMA_COMMON_GATE		0x02
#define DMA_CHAN_GATE		0x01
#define DMA_STAT		(SUNXI_DMAC_PBASE + 0x30)			/* DMA Status Register RO */
#define DMA_ENABLE(x)		(SUNXI_DMAC_PBASE + (0x100 + ((x) << 6)))	/* Channels enable register */
#define DMA_PAUSE(x)		(SUNXI_DMAC_PBASE + (0x104 + ((x) << 6)))	/* DMA Channels pause register */
#define DMA_LLI_ADDR(x)		(SUNXI_DMAC_PBASE + (0x108 + ((x) << 6)))	/* Descriptor address register */
#define DMA_CFG(x)				(SUNXI_DMAC_PBASE + 0x10C + ((x) << 6))	/* Configuration register RO */
#define DMA_CUR_SRC(x)		(SUNXI_DMAC_PBASE + (0x110 + ((x) << 6)))	/* Current source address RO */
#define DMA_CUR_DST(x)		(SUNXI_DMAC_PBASE + (0x114 + ((x) << 6)))	/* Current destination address RO */
#define DMA_CNT(x)		(SUNXI_DMAC_PBASE + (0x118 + ((x) << 6)))	/* Byte counter left register RO */
#define DMA_PARA(x)		(SUNXI_DMAC_PBASE + (0x11C + ((x) << 6)))	/* Parameter register RO */
#define LINK_END		0xFFFFF800			/* lastest link must be 0xfffff800 */

/* DMA mode register */
#define DMA_OP_MODE(x)		(SUNXI_DMAC_PBASE + (0x128 + ((x) << 6)))		/* DMA mode register */
#define SRC_HS_MASK		(0x1 << 2)			/* bit 2: Source handshake mode */
#define DST_HS_MASK		(0x1 << 3)			/* bit 3: Destination handshake mode */

#define SET_OP_MODE(x, val)	({	\
		writel(val,DMA_OP_MODE(x));	\
		})


#define SHIFT_IRQ_MASK(val, ch) ({	\
		(ch) >= HIGH_CHAN	\
		? (val) << ((ch - HIGH_CHAN) << 2) \
		: (val) << ((ch) << 2);	\
		})

#define SHIFT_PENDING_MASK(val, ch) ({	\
		(ch) >= HIGH_CHAN	\
		? (val) << ((ch - HIGH_CHAN) << 2) \
		: (val) << ((ch) << 2);	\
		})

#define IRQ_HALF		0x01			/* Half package transfer interrupt pending */
#define IRQ_PKG			0x02			/* One package complete interrupt pending */
#define IRQ_QUEUE		0x04			/* All list complete transfer interrupt pending */
#define IRQ_ALL			(IRQ_HALF | IRQ_PKG | IRQ_QUEUE)

/* DMA channel configuration register */
/* The detail information of DMA configuration */
#define SRC_WIDTH(x)		((x) << 9)
#define SRC_BURST(x)		((x) << 6)
#define SRC_IO_MODE		(0x01 << 8)
#define SRC_LINEAR_MODE		(0x00 << 8)
#define SRC_DRQ(x)		((x) << 0)
#define DST_WIDTH(x)		((x) << 25)
#define DST_BURST(x)		((x) << 22)
#define DST_IO_MODE		(0x01 << 24)
#define DST_LINEAR_MODE		(0x00 << 24)
#define DST_DRQ(x)		((x) << 16)
#define CHAN_START		1
#define CHAN_STOP		0
#define CHAN_PAUSE		1
#define CHAN_RESUME		0
#define NORMAL_WAIT		(8 << 0)

#define sunxi_slave_id(d, s) 	(((d)<<16) | (s))
#define GET_SRC_DRQ(x)		((x) & 0x000000ff)
#define GET_DST_DRQ(x)		((x) & 0x00ff0000)


struct sunxi_dma_lli {
	uint32_t	cfg;
	uint32_t	src;
	uint32_t	dst;
	uint32_t	len;
	uint32_t	para;
	uint32_t	p_lln;
	struct sunxi_dma_lli *vlln;
};

struct sunxi_dma_chan_config {
	uint32_t	enable;
	uint32_t	pause;
	uint32_t	desc_addr;
	uint32_t	config;
	uint32_t	src_addr;
	uint32_t	dst_addr;
	uint32_t	left_byte;
	uint32_t	parameters;
};

typedef void (*dma_callback)(void *param);

/**
 * enum dma_slave_buswidth - defines bus width of the DMA slave
 * device, source or target buses
 */
enum dma_slave_buswidth {
	DMA_SLAVE_BUSWIDTH_UNDEFINED = 0,
	DMA_SLAVE_BUSWIDTH_1_BYTE = 1,
	DMA_SLAVE_BUSWIDTH_2_BYTES = 2,
	DMA_SLAVE_BUSWIDTH_3_BYTES = 3,
	DMA_SLAVE_BUSWIDTH_4_BYTES = 4,
	DMA_SLAVE_BUSWIDTH_8_BYTES = 8,
	DMA_SLAVE_BUSWIDTH_16_BYTES = 16,
	DMA_SLAVE_BUSWIDTH_32_BYTES = 32,
	DMA_SLAVE_BUSWIDTH_64_BYTES = 64,
};

enum dma_slave_burst {
	DMA_SLAVE_BURST_1 = 1,
	DMA_SLAVE_BURST_4 = 4,
	DMA_SLAVE_BURST_8 = 8,
	DMA_SLAVE_BURST_16 = 16,
};


/**
 * enum dma_transfer_direction - dma transfer mode and direction indicator
 * @DMA_MEM_TO_MEM: Async/Memcpy mode
 * @DMA_MEM_TO_DEV: Slave mode & From Memory to Device
 * @DMA_DEV_TO_MEM: Slave mode & From Device to Memory
 * @DMA_DEV_TO_DEV: Slave mode & From Device to Device
 */
enum dma_transfer_direction {
	DMA_MEM_TO_MEM = 0,
	DMA_MEM_TO_DEV = 1,
	DMA_DEV_TO_MEM = 2,
	DMA_DEV_TO_DEV = 3,
	DMA_TRANS_NONE,
};

/**
 * enum dma_status - DMA transaction status
 * @DMA_COMPLETE: transaction completed
 * @DMA_IN_PROGRESS: transaction not yet processed
 * @DMA_PAUSED: transaction is paused
 * @DMA_ERROR: transaction failed
 */
enum dma_status {
	DMA_INVALID_PARAMETER = -2,
	DMA_ERROR = -1,
	DMA_COMPLETE,
	DMA_IN_PROGRESS,
	DMA_PAUSED,
};

/**
 * struct dma_slave_config - dma slave channel runtime config
 * @direction: whether the data shall go in or out on this slave
 * channel, right now. DMA_MEM_TO_DEV and DMA_DEV_TO_MEM are
 * legal values. DEPRECATED, drivers should use the direction argument
 * to the device_prep_slave_sg and device_prep_dma_cyclic functions or
 * the dir field in the dma_interleaved_template structure.
 * @src_addr: this is the physical address where DMA slave data
 * should be read (RX), if the source is memory this argument is
 * ignored.
 * @dst_addr: this is the physical address where DMA slave data
 * should be written (TX), if the source is memory this argument
 * is ignored.
 * @src_addr_width: this is the width in bytes of the source (RX)
 * register where DMA data shall be read. If the source
 * is memory this may be ignored depending on architecture.
 * Legal values: 1, 2, 4, 8.
 * @dst_addr_width: same as src_addr_width but for destination
 * target (TX) mutatis mutandis.
 * @src_maxburst: the maximum number of words (note: words, as in
 * units of the src_addr_width member, not bytes) that can be sent
 * in one burst to the device. Typically something like half the
 * FIFO depth on I/O peripherals so you don't overflow it. This
 * may or may not be applicable on memory sources.
 * @dst_maxburst: same as src_maxburst but for destination target
 * mutatis mutandis.
 * @slave_id: Slave requester id. Only valid for slave channels. The dma
 * slave peripheral will have unique id as dma requester which need to be
 * pass as slave config.
 *
 * This struct is passed in as configuration data to a DMA engine
 * in order to set up a certain channel for DMA transport at runtime.
 * The DMA device/engine has to provide support for an additional
 * callback in the dma_device structure, device_config and this struct
 * will then be passed in as an argument to the function.
 *
 * The rationale for adding configuration information to this struct is as
 * follows: if it is likely that more than one DMA slave controllers in
 * the world will support the configuration option, then make it generic.
 * If not: if it is fixed so that it be sent in static from the platform
 * data, then prefer to do that.
 */
struct dma_slave_config {
	enum dma_transfer_direction direction;
	uint32_t src_addr;
	uint32_t dst_addr;
	enum dma_slave_buswidth src_addr_width;
	enum dma_slave_buswidth dst_addr_width;
	uint32_t src_maxburst;
	uint32_t dst_maxburst;
	uint32_t slave_id;
};

struct sunxi_dma_chan {
	uint8_t used:1;
	uint8_t chan_count:4;
	bool	cyclic:1;
	struct dma_slave_config  cfg;
	uint32_t periods_pos;
	uint32_t buf_len;
	struct sunxi_dma_lli *desc;
	uint32_t	irq_type;
	dma_callback callback;
	void *callback_param;
	kspinlock_t lock;
};

#ifdef __cplusplus
}
#endif

#endif
