/*
 * Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
 *
 * Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
 * the the People's Republic of China and other countries.
 * All Allwinner Technology Co.,Ltd. trademarks are used with permission.
 *
 * DISCLAIMER
 * THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
 * IF YOU NEED TO INTEGRATE THIRD PARTY¡¯S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR
 * MPEGLA, ETC.)
 * IN ALLWINNERS¡¯SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
 * ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
 * ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT
 * TO MATTERS
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <interrupt.h>
#include "hal_spi.h"
#include "hal_dma.h"


/******************fix me*********************/
#define hal_readl(addr)             (*((volatile unsigned long  *)(addr)))
#define hal_writel(v, addr)		(*((volatile unsigned long  *)(addr)) = (unsigned long)(v))
#define hal_readb(addr)             (*((volatile unsigned char  *)(addr)))
#define hal_writeb(v, addr)		(*((volatile unsigned char  *)(addr)) = (unsigned char)(v))

#define krhino_mm_alloc(size) malloc(size)
#define krhino_mm_free(ptr) 	free(ptr)
/******************fix me*********************/

#if (1)
#define CONFIG_SUNXI_SPI_CPU_XFER_ONLY
#endif

#define SPI_ERR(fmt, arg...) printf("%s()%d " fmt, __func__, __LINE__, ##arg)
#define SPI_INIT(fmt, arg...) printf("%s()%d " fmt, __func__, __LINE__, ##arg)

#if (0)
#define SPI_INFO_LEVEL
#endif

#if (0)
#define SPI_DATA_LEVEL
#endif

#if (0)
#define SPI_DUMPREG_LEVEL
#endif


#ifdef SPI_INFO_LEVEL
#define SPI_INFO(fmt, arg...) printf("%s()%d " fmt, __func__, __LINE__, ##arg)
#else
#define SPI_INFO(fmt, arg...)                                                  \
    do {                                                                   \
    } while (0);
#endif

static uint32_t g_spi_regbase[HAL_SPI_MASTER_MAX];
static uint32_t g_spi_irqnum[HAL_SPI_MASTER_MAX];
static sunxi_spi_t g_sunxi_spi[HAL_SPI_MASTER_MAX];


#define u32 uint32_t
//#define u8 uint8_t
struct sunxi_ccm_reg
{
    u32 pll_cpux_ctrl;      /*0x0*/
    u8 reserved_0x4[12];
    u32 pll_ddr0_ctrl;      /*0x10*/
    u8 reserved_0x14[4];
    u32 pll_ddr1_ctrl;      /*0x18*/
    u8 reserved_0x1C[4];
    u32 pll_peri0_ctrl;     /*0x20*/
    u8 reserved_0x24[4];
    u32 pll_peri1_ctrl;     /*0x28*/
    u8 reserved_0x2C[68];
    u32 pll_hsic_ctrl;      /*0x70*/
    u8 reserved_0x74[1164];
    u32 cpux_axi_cfg;       /*0x500*/
    u8 reserved_0x504[12];
    u32 psi_ahb1_ahb2_cfg;  /*0x510*/
    u8 reserved_0x514[8];
    u32 ahb3_cfg;           /*0x51c*/
    u32 apb1_cfg;           /*0x520*/
    u32 apb2_cfg;           /*0x524*/
    u8 reserved_0x528[24];
    u32 mbus_cfg;           /*0x540*/
    u8 reserved_0x544[316];
    u32 ce_clk;             /*0x680*/
    u8 reserved_0x684[8];
    u32 ce_bgr;             /*0x68c*/
    u32 ve_clk;             /*0x690*/
    u8 reserved_0x694[8];
    u32 ve_bgr;             /*0x69c*/
    u8 reserved_0x6A0[108];
    u32 dma_gate_reset;     /*0x70c*/
    u8 reserved_0x710[48];
    u32 avs_clk;            /*0x740*/
    u8 reserved_0x744[8];
    u32 avs_bgr;            /*0x74c*/
    u8 reserved_0x750[108];
    u32 iommu_bgr;          /*0x7bc*/
    u8 reserved_0x7C0[64];
    u32 dram_clk;           /*0x800*/
    u32 mbus_gate;/*0x804*/
    u32 pll_ddr_aux;        /*0x808*/
    u32 dram_bgr;           /*0x80c*/
    u32 nand_clk;           /*0x810*/
    u8 reserved_0x814[24];
    u32 nand_bgr;           /*0x82c*/
    u32 sdmmc0_clk;         /*0x830*/
    u32 sdmmc1_clk;         /*0x834*/
    u32 sdmmc2_clk;         /*0x838*/
    u8 reserved_0x83C[16];
    u32 smhc_bgr;           /*0x84c*/
    u8 reserved_0x850[188];
    u32 uart_bgr;           /*0x90c*/
    u8 reserved_0x910[12];
    u32 twi_bgr;            /*0x91c*/
    u8 reserved_0x920[28];
    u32 scr_bgr;            /*0x93c*/
    u32 spi0_clk_cfg;       /*0x940*/
    u32 spi1_clk_cfg;       /*0x944*/
    u8 reserved_0x948[36];
    u32 spi_gate_reset;     /*0x96c*/
    u8 reserved_0x970[256];
    u32 usb0_clk;           /*0xa70*/
    u8 reserved_0xA74[24];
    u32 usb_bgr;            /*0xa8c*/
};

static inline void dma_free_coherent(void *addr)
{
    void *malloc_ptr = NULL;
    if (!addr)
        return;
    malloc_ptr = (void *)(*(uint32_t *)((uint32_t *)addr - 1));
    krhino_mm_free(malloc_ptr);
}

static inline void *dma_alloc_coherent(size_t size)
{
    void *fake_ptr = NULL;
    void *malloc_ptr = NULL;

    malloc_ptr = krhino_mm_alloc(size + 64);
    if ((uint32_t)malloc_ptr & 0x3) {
        printf("error: krhino_mm_alloc not align to 4 byte\r\n");
    }
    fake_ptr = (void *)((uint32_t)(malloc_ptr + 64) & (~63));
    *(uint32_t *)((uint32_t *)fake_ptr - 1) = (uint32_t)malloc_ptr;

    return fake_ptr;
}

void spi_dump_reg(sunxi_spi_t *sspi, uint32_t offset, uint32_t len)
{
    uint32_t i;
    char buf[64], cnt = 0;

    for (i = 0; i < len; i = i + REG_INTERVAL)
    {
        if (i % HEXADECIMAL == 0)
            cnt += sprintf(buf + cnt, "0x%08lx: ",
                           (uint32_t)(sspi->base + offset + i));

        cnt +=
            sprintf(buf + cnt, "%08lx ", hal_readl(sspi->base + offset + i));

        if (i % HEXADECIMAL == REG_CL)
        {
            printf("%s\n", buf);
            cnt = 0;
        }
    }
}

/* config chip select */
static spi_master_status_t spi_set_cs(uint32_t chipselect, sunxi_spi_t *sspi)
{
    spi_master_status_t ret;
    uint32_t reg_val = hal_readl(sspi->base + SPI_TC_REG);

    if (chipselect < 4)
    {
        reg_val &= ~SPI_TC_SS_MASK; /* SS-chip select, clear two bits */
        reg_val |= chipselect
                   << SPI_TC_SS_BIT_POS; /* set chip select */
        hal_writel(reg_val, sspi->base + SPI_TC_REG);
        ret = SPI_MASTER_OK;
    }
    else
    {
        SPI_ERR("[spi%d] Chip Select set fail! cs = %ld\n", sspi->port,
                chipselect);
        ret = SPI_MASTER_INVALID_PARAMETER;
    }

    return ret;
}

/* config spi */
static void spi_config_tc(uint32_t config, sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_TC_REG);

    /*1. POL */
    if (config & SPI_POL_ACTIVE_)
    {
        reg_val |= SPI_TC_POL;    /*default POL = 1 */
    }
    else
    {
        reg_val &= ~SPI_TC_POL;
    }

    /*2. PHA */
    if (config & SPI_PHA_ACTIVE_)
    {
        reg_val |= SPI_TC_PHA;    /*default PHA = 1 */
    }
    else
    {
        reg_val &= ~SPI_TC_PHA;
    }

    /*3. SSPOL,chip select signal polarity */
    if (config & SPI_CS_HIGH_ACTIVE_)
    {
        reg_val &= ~SPI_TC_SPOL;
    }
    else
    {
        reg_val |= SPI_TC_SPOL;    /*default SSPOL = 1,Low level effect */
    }

    /*4. LMTF--LSB/MSB transfer first select */
    if (config & SPI_LSB_FIRST_ACTIVE_)
    {
        reg_val |= SPI_TC_FBS;
    }
    else
    {
        reg_val &= ~SPI_TC_FBS;    /*default LMTF =0, MSB first */
    }

    /* set DDB,DHB,SMC,SSCTL */
    /*5. dummy burst type */
    if (config & SPI_DUMMY_ONE_ACTIVE_)
    {
        reg_val |= SPI_TC_DDB;
    }
    else
    {
        reg_val &= ~SPI_TC_DDB;    /*default DDB =0, ZERO */
    }

    /*6.discard hash burst-DHB */
    if (config & SPI_RECEIVE_ALL_ACTIVE_)
    {
        reg_val &= ~SPI_TC_DHB;
    }
    else
    {
        reg_val |= SPI_TC_DHB;    /*default DHB =1, discard unused burst */
    }

    /*7. set SMC = 1 , SSCTL = 0 ,TPE = 1 */
    reg_val &= ~SPI_TC_SSCTL;

    hal_writel(reg_val, sspi->base + SPI_TC_REG);
}

/* delay internal read sample point*/
static void spi_sample_delay(uint32_t sdm, uint32_t sdc, sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_TC_REG);
    uint32_t org_val = reg_val;

    if (sdm)
    {
        reg_val |= SPI_TC_SDM;
    }
    else
    {
        reg_val &= ~SPI_TC_SDM;
    }

    if (sdc)
    {
        reg_val |= SPI_TC_SDC;
    }
    else
    {
        reg_val &= ~SPI_TC_SDC;
    }

    if (reg_val != org_val)
    {
        hal_writel(reg_val, sspi->base + SPI_TC_REG);
    }
}

/* start spi transfer */
static void spi_start_xfer(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_TC_REG);

    reg_val |= SPI_TC_XCH;
    hal_writel(reg_val, sspi->base + SPI_TC_REG);
}

/* enable spi bus */
static void spi_enable_bus(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_GC_REG);

    reg_val |= SPI_GC_EN;
    hal_writel(reg_val, sspi->base + SPI_GC_REG);
}

/* disbale spi bus */
static void spi_disable_bus(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_GC_REG);

    reg_val &= ~SPI_GC_EN;
    hal_writel(reg_val, sspi->base + SPI_GC_REG);
}

/* set master mode */
static void spi_set_master(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_GC_REG);

    reg_val |= SPI_GC_MODE;
    hal_writel(reg_val, sspi->base + SPI_GC_REG);
}

/* soft reset spi controller */
static void spi_soft_reset(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_GC_REG);

    reg_val |= SPI_GC_SRST;
    hal_writel(reg_val, sspi->base + SPI_GC_REG);
}

/* enable transmit pause */
static void spi_enable_tp(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_GC_REG);

    reg_val |= SPI_GC_TP_EN;
    hal_writel(reg_val, sspi->base + SPI_GC_REG);
}

/* set ss control */
static void spi_ss_owner(sunxi_spi_t *sspi, uint32_t on_off)
{
    u32 reg_val = hal_readl(sspi->base + SPI_TC_REG);

    on_off &= 0x1;
    if (on_off)
    {
        reg_val |= SPI_TC_SS_OWNER;
    }
    else
    {
        reg_val &= ~SPI_TC_SS_OWNER;
    }
    hal_writel(reg_val, sspi->base + SPI_TC_REG);
}

/* enable irq type */
static void spi_enable_irq(uint32_t bitmap, sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_INT_CTL_REG);

    bitmap &= SPI_INTEN_MASK;
    reg_val |= bitmap;
    hal_writel(reg_val, sspi->base + SPI_INT_CTL_REG);
}

/* disable irq type */
static void spi_disable_irq(uint32_t bitmap, sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_INT_CTL_REG);

    bitmap &= SPI_INTEN_MASK;
    reg_val &= ~bitmap;
    hal_writel(reg_val, sspi->base + SPI_INT_CTL_REG);
}

#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
/* enable dma irq */
static void spi_enable_dma_irq(uint32_t bitmap, sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_FIFO_CTL_REG);

    bitmap &= SPI_FIFO_CTL_DRQEN_MASK;
    reg_val |= bitmap;
    hal_writel(reg_val, sspi->base + SPI_FIFO_CTL_REG);
}

/* disable dma irq */
static void spi_disable_dma_irq(uint32_t bitmap, sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_FIFO_CTL_REG);

    bitmap &= SPI_FIFO_CTL_DRQEN_MASK;
    reg_val &= ~bitmap;
    hal_writel(reg_val, sspi->base + SPI_FIFO_CTL_REG);
}
#endif

/* query irq enable */
static uint32_t spi_qry_irq_enable(sunxi_spi_t *sspi)
{
    return (SPI_INTEN_MASK & hal_readl(sspi->base + SPI_INT_CTL_REG));
}

/* query irq pending */
static uint32_t spi_qry_irq_pending(sunxi_spi_t *sspi)
{
    return (SPI_INT_STA_MASK & hal_readl(sspi->base + SPI_INT_STA_REG));
}

/* clear irq pending */
static void spi_clr_irq_pending(uint32_t pending_bit, sunxi_spi_t *sspi)
{
    pending_bit &= SPI_INT_STA_MASK;
    hal_writel(pending_bit, sspi->base + SPI_INT_STA_REG);
}

/* query txfifo bytes */
static uint32_t spi_query_txfifo(sunxi_spi_t *sspi)
{
    uint32_t reg_val =
        (SPI_FIFO_STA_TX_CNT & hal_readl(sspi->base + SPI_FIFO_STA_REG));

    reg_val >>= SPI_TXCNT_BIT_POS;
    return reg_val;
}

/* query rxfifo bytes */
static uint32_t spi_query_rxfifo(sunxi_spi_t *sspi)
{
    uint32_t reg_val =
        (SPI_FIFO_STA_RX_CNT & hal_readl(sspi->base + SPI_FIFO_STA_REG));

    reg_val >>= SPI_RXCNT_BIT_POS;
    return reg_val;
}

/* reset fifo */
static void spi_reset_fifo(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_FIFO_CTL_REG);

    reg_val |= (SPI_FIFO_CTL_RX_RST | SPI_FIFO_CTL_TX_RST);
    /* Set the trigger level of RxFIFO/TxFIFO. */
    reg_val &= ~(SPI_FIFO_CTL_RX_LEVEL | SPI_FIFO_CTL_TX_LEVEL);
    reg_val |= (0x20 << 16) | 0x20;
    hal_writel(reg_val, sspi->base + SPI_FIFO_CTL_REG);
}

#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
static void spi_set_rx_trig(uint32_t val, sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_FIFO_CTL_REG);

    reg_val &= ~SPI_FIFO_CTL_RX_LEVEL;
    reg_val |= val & SPI_FIFO_CTL_RX_LEVEL;
    hal_writel(reg_val, sspi->base + SPI_FIFO_CTL_REG);
}

static void spi_set_tx_trig(uint32_t val, sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_FIFO_CTL_REG);

    reg_val &= ~SPI_FIFO_CTL_TX_LEVEL;
    reg_val |= (val << 16) & SPI_FIFO_CTL_TX_LEVEL;
    hal_writel(reg_val, sspi->base + SPI_FIFO_CTL_REG);
}
#endif

/* set transfer total length BC, transfer length TC and single transmit length
 * STC */
static void spi_set_bc_tc_stc(uint32_t tx_len, uint32_t rx_len,
                              uint32_t stc_len, uint32_t dummy_cnt,
                              sunxi_spi_t *sspi)
{
    uint32_t reg_val;

    /* set MBC(0x30) = tx_len + rx_len + dummy_cnt */
    reg_val = hal_readl(sspi->base + SPI_BURST_CNT_REG);
    reg_val &= ~SPI_BC_CNT_MASK;
    reg_val |= (SPI_BC_CNT_MASK & (tx_len + rx_len + dummy_cnt));
    hal_writel(reg_val, sspi->base + SPI_BURST_CNT_REG);

    /* set MTC(0x34) = tx_len */
    reg_val = hal_readl(sspi->base + SPI_TRANSMIT_CNT_REG);
    reg_val &= ~SPI_TC_CNT_MASK;
    reg_val |= (SPI_TC_CNT_MASK & tx_len);
    hal_writel(reg_val, sspi->base + SPI_TRANSMIT_CNT_REG);

    /* set BBC(0x38) = dummy cnt & single mode transmit counter */
    reg_val = hal_readl(sspi->base + SPI_BCC_REG);
    reg_val &= ~SPI_BCC_STC_MASK;
    reg_val |= (SPI_BCC_STC_MASK & stc_len);
    reg_val &= ~(0xf << 24);
    reg_val |= (dummy_cnt << 24);
    hal_writel(reg_val, sspi->base + SPI_BCC_REG);
}

/* set ss control */
static void spi_ss_ctrl(sunxi_spi_t *sspi, uint8_t on_off)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_TC_REG);

    on_off &= 0x1;
    if (on_off)
    {
        reg_val |= SPI_TC_SS_OWNER;
    }
    else
    {
        reg_val &= ~SPI_TC_SS_OWNER;
    }
    hal_writel(reg_val, sspi->base + SPI_TC_REG);
}

static void spi_disable_dual(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_BCC_REG);
    reg_val &= ~SPI_BCC_DUAL_MODE;
    hal_writel(reg_val, sspi->base + SPI_BCC_REG);
}

static void spi_enable_dual(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_BCC_REG);
    reg_val &= ~SPI_BCC_QUAD_MODE;
    reg_val |= SPI_BCC_DUAL_MODE;
    hal_writel(reg_val, sspi->base + SPI_BCC_REG);
}

static void spi_disable_quad(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_BCC_REG);

    reg_val &= ~SPI_BCC_QUAD_MODE;
    hal_writel(reg_val, sspi->base + SPI_BCC_REG);
}

static void spi_enable_quad(sunxi_spi_t *sspi)
{
    uint32_t reg_val = hal_readl(sspi->base + SPI_BCC_REG);

    reg_val |= SPI_BCC_QUAD_MODE;
    hal_writel(reg_val, sspi->base + SPI_BCC_REG);
}

/*  Platform-specific, SPI mode function */
int32_t spi_set_mode(hal_spi_master_port_t port, hal_spi_trans_mode_e mode)
{
	sunxi_spi_t *sspi = &g_sunxi_spi[port];
	uint32_t reg_val = hal_readl(sspi->base + SPI_TC_REG);

	switch(mode)
	{
		case SUNXI_SPIDEV_MODE0:
			reg_val &= ~SPI_TC_POL;
			reg_val &= ~SPI_TC_PHA;
			break;

		case SUNXI_SPIDEV_MODE1:
			reg_val &= ~SPI_TC_POL;
			reg_val |= SPI_TC_PHA;
			break;

		case SUNXI_SPIDEV_MODE2:
			reg_val |= SPI_TC_POL;
			reg_val &= ~SPI_TC_PHA;
			break;

		case SUNXI_SPIDEV_MODE3:
			reg_val |= SPI_TC_POL;
			reg_val |= SPI_TC_PHA;
			break;
		default:
			SPI_ERR("[spi%d] spi set mode error\n", sspi->port);
			return -1;
	}
	//other config.
	hal_writel(reg_val, sspi->base + SPI_TC_REG);
	return 0;
}

static spi_master_status_t spi_mode_check(sunxi_spi_t *sspi)
{
    if (sspi->mode_type != MODE_TYPE_NULL)
    {
        SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
        return SPI_MASTER_INVALID_PARAMETER;
    }

    /* full duplex */
    if (sspi->transfer->tx_buf && sspi->transfer->rx_buf)
    {
        spi_set_bc_tc_stc(sspi->transfer->tx_len,
                          sspi->transfer->rx_len,
                          sspi->transfer->tx_single_len,
                          sspi->transfer->dummy_byte, sspi);
        sspi->mode_type = FULL_DUPLEX_TX_RX;

        if (sspi->transfer->rx_nbits == SPI_NBITS_QUAD)
        {
            spi_disable_dual(sspi);
            spi_enable_quad(sspi);
            SPI_INFO("[spi%d] Quad mode Full duplex tx rx\n",
                     sspi->port);
        }
        else if (sspi->transfer->rx_nbits == SPI_NBITS_DUAL)
        {
            spi_disable_quad(sspi);
            spi_enable_dual(sspi);
            SPI_INFO("[spi%d] Dual mode Full duplex tx rx\n",
                     sspi->port);
        }
        else
        {
            spi_disable_quad(sspi);
            spi_disable_dual(sspi);
            SPI_INFO("[spi%d] Single mode Full duplex tx rx\n",
                     sspi->port);
        }
    } /* half duplex transmit */
    else if (sspi->transfer->tx_buf)
    {
        if (sspi->transfer->tx_nbits == SPI_NBITS_QUAD)
        {
            spi_disable_dual(sspi);
            spi_enable_quad(sspi);
            spi_set_bc_tc_stc(sspi->transfer->tx_len, 0,
                              sspi->transfer->tx_single_len,
                              sspi->transfer->dummy_byte, sspi);
            sspi->mode_type = QUAD_HALF_DUPLEX_TX;
            SPI_INFO("[spi%d] Quad mode Half duplex tx\n",
                     sspi->port);
        }
        else if (sspi->transfer->tx_nbits == SPI_NBITS_DUAL)
        {
            spi_disable_quad(sspi);
            spi_enable_dual(sspi);
            spi_set_bc_tc_stc(sspi->transfer->tx_len, 0,
                              sspi->transfer->tx_single_len,
                              sspi->transfer->dummy_byte, sspi);
            sspi->mode_type = DUAL_HALF_DUPLEX_TX;
            SPI_INFO("[spi%d] Dual mode Half duplex tx\n",
                     sspi->port);
        }
        else
        {
            spi_disable_quad(sspi);
            spi_disable_dual(sspi);
            spi_set_bc_tc_stc(sspi->transfer->tx_len, 0,
                              sspi->transfer->tx_len,
                              sspi->transfer->dummy_byte, sspi);
            sspi->mode_type = SGLE_HALF_DUPLEX_TX;
            SPI_INFO("[spi%d] Single mode Half duplex tx\n",
                     sspi->port);
        }
    } /* half duplex receive */
    else if (sspi->transfer->rx_buf)
    {
        if (sspi->transfer->rx_nbits == SPI_NBITS_QUAD)
        {
            spi_disable_dual(sspi);
            spi_enable_quad(sspi);
            sspi->mode_type = QUAD_HALF_DUPLEX_RX;
            SPI_INFO("[spi%d] Quad mode Half duplex rx\n",
                     sspi->port);
        }
        else if (sspi->transfer->rx_nbits == SPI_NBITS_DUAL)
        {
            spi_disable_quad(sspi);
            spi_enable_dual(sspi);
            sspi->mode_type = DUAL_HALF_DUPLEX_RX;
            SPI_INFO("[spi%d] Dual mode Half duplex rx\n",
                     sspi->port);
        }
        else
        {
            spi_disable_quad(sspi);
            spi_disable_dual(sspi);
            sspi->mode_type = SGLE_HALF_DUPLEX_RX;
            SPI_INFO("[spi%d] Single mode Half duplex rx\n",
                     sspi->port);
        }
        spi_set_bc_tc_stc(0, sspi->transfer->rx_len, 0,
                          sspi->transfer->dummy_byte, sspi);
    }

    return SPI_MASTER_OK;
}

static spi_master_status_t spi_cpu_write(sunxi_spi_t *sspi)
{
    uint32_t len = sspi->transfer->tx_len;
    const uint8_t *buf = sspi->transfer->tx_buf;
    uint32_t poll_time = 0xff;
#ifdef SPI_DATA_LEVEL
    uint32_t i, j;
    uint8_t dbuf[64], cnt = 0;
#endif

    if (NULL == buf)
    {
        SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
        return SPI_MASTER_INVALID_PARAMETER;
    }

#ifdef SPI_DUMPREG_LEVEL
    SPI_INFO("[spi%d] dump reg:\n", sspi->port);
    spi_dump_reg(sspi, 0, 0x60);
#endif

#ifdef SPI_DATA_LEVEL
    SPI_INFO("tx_len = %d\n", len);
    for (i = 0; i < len; i += 16)
    {
        cnt = 0;
        cnt += sprintf(dbuf + cnt, "%04x: ", i);
        for (j = 0; ((i + j) < len) && (j < 16); j++)
            cnt += sprintf(dbuf + cnt, "%02x ",
                           ((uint8_t *)(buf))[i + j]);
        printf("%s\n", dbuf);
    }
#endif

    for (; len > 0; --len)
    {
        hal_writeb(*buf++, sspi->base + SPI_TXDATA_REG);
        while ((spi_query_txfifo(sspi) >= MAX_FIFU) && poll_time--)
            ;
        if (poll_time <= 0)
        {
            SPI_ERR("[spi%d] cpu transfer data time out!\n",
                    sspi->port);
            SPI_INFO("[spi%d] dump reg:\n", sspi->port);
            spi_dump_reg(sspi, 0, 0x60);
            return SPI_MASTER_ERROR_TIMEOUT;
        }
    }

    return SPI_MASTER_OK;
}

static spi_master_status_t spi_cpu_read(sunxi_spi_t *sspi)
{
    uint32_t len = sspi->transfer->rx_len;
    uint8_t *buf = sspi->transfer->rx_buf;
    uint32_t poll_time = 0xff;
    uint32_t n;

#ifdef SPI_DATA_LEVEL
    uint32_t i, j;
    uint8_t dbuf[64], cnt = 0;
#endif

    if (NULL == buf)
    {
        SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
        return SPI_MASTER_INVALID_PARAMETER;
    }

#ifdef SPI_DUMPREG_LEVEL
    SPI_INFO("[spi%d] dump reg:\n", sspi->port);
    spi_dump_reg(sspi, 0, 0x60);
#endif

    for (n = 0; n < len; n++)
    {
        while (!spi_query_rxfifo(sspi) && poll_time--)
            ;
        if (poll_time <= 0)
        {
            SPI_ERR("[spi%d] cpu receive data time out!\n",
                    sspi->port);
            SPI_INFO("[spi%d] dump reg:\n", sspi->port);
            spi_dump_reg(sspi, 0, 0x60);
            return SPI_MASTER_ERROR_TIMEOUT;
        }

        *(buf + n) = hal_readb(sspi->base + SPI_RXDATA_REG);
    }

#ifdef SPI_DATA_LEVEL
    SPI_INFO("rx_len = %d\n", len);
    for (i = 0; i < len; i += 16)
    {
        cnt = 0;
        cnt += sprintf(dbuf + cnt, "%04x: ", i);
        for (j = 0; ((i + j) < len) && (j < 16); j++)
            cnt += sprintf(dbuf + cnt, "%02x ",
                           ((uint8_t *)(buf))[i + j]);
        printf("%s\n", dbuf);
    }
#endif

    return SPI_MASTER_OK;
}

#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
static spi_master_status_t spi_dma_tx_config(struct sunxi_spi *sspi)
{
    hal_dma_status_t ret;
    uint32_t len = sspi->transfer->tx_len;
    uint8_t *buf = sspi->transfer->tx_buf;
    struct dma_slave_config *config = &sspi->dma_tx.config;

#ifdef SPI_DATA_LEVEL
    unsigned int i, j;
    u8 dbuf[64], cnt = 0;
#endif

    if (NULL == buf)
    {
        SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
        return SPI_MASTER_INVALID_PARAMETER;
    }

    if (len <= ALIGN_DMA_BUF_SIZE)
    {
        memcpy(sspi->align_dma_buf, buf, len);
        buf = sspi->align_dma_buf;
    }
    else
    {
        SPI_ERR("[spi%d] tx size over dma align buf size\n",
                sspi->port);
        /* buffer on DMA must align to cache line */
        if ((uint32_t)buf & (64 - 1) || len & (64 - 1))
        {
            SPI_ERR("[spi%d] tx buf or len not align to 64\n",
                    sspi->port);
            return SPI_MASTER_INVALID_PARAMETER;
        }
    }
    //cpu_dcache_clean(buf, ALIGN(len, 64));

#ifdef SPI_DATA_LEVEL
    SPI_INFO("tx_len = %d\n", len);
    for (i = 0; i < len; i += 16)
    {
        cnt = 0;
        cnt += sprintf(dbuf + cnt, "%03x: ", i);
        for (j = 0; ((i + j) < len) && (j < 16); j++)
            cnt += sprintf(dbuf + cnt, "%02x ",
                           ((uint8_t *)(buf))[i + j]);
        printf("%s\n", dbuf);
    }
#endif

    ret = hal_dma_chan_request(&sspi->dma_tx.hdma);
    if (ret == HAL_DMA_CHAN_STATUS_BUSY)
    {
        SPI_ERR("[spi%d] request dma_rx failed\n", sspi->port);
        return SPI_MASTER_ERROR;
    }

    spi_set_tx_trig(0x20, sspi);
    spi_enable_dma_irq(SPI_FIFO_CTL_TX_DRQEN, sspi);

#ifdef SPI_DUMPREG_LEVEL
    SPI_INFO("[spi%d] dump reg:\n", sspi->port);
    spi_dump_reg(sspi, 0, 0x60);
#endif

    config->direction = DMA_MEM_TO_DEV;
    config->dst_addr = sspi->base + SPI_TXDATA_REG;
    config->src_addr = (int32_t)buf;

    if (len % DMA_SLAVE_BUSWIDTH_4_BYTES)
    {
        config->src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
        config->dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
        config->dst_maxburst = DMA_SLAVE_BURST_16;
        config->src_maxburst = DMA_SLAVE_BURST_16;
    }
    else
    {
        config->src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        config->dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        config->dst_maxburst = DMA_SLAVE_BURST_8;
        config->src_maxburst = DMA_SLAVE_BURST_8;
    }

    config->slave_id =
        sunxi_slave_id(SUNXI_SPI_DRQ_TX(sspi->port), DRQSRC_SDRAM);

    SPI_INFO("[spi%d] config:\n"
             "  direction: %d\n"
             "  src_addr: 0x%08x,"
             "  dst_addr: 0x%08x\n"
             "  src_addr_width: %d,"
             "  dst_addr_width: %d\n"
             "  src_maxburst: %u,"
             "  dst_maxburst: %u\n"
             "  slave_id: 0x%08x\n",
             sspi->port, config->direction, config->src_addr,
             config->dst_addr, config->src_addr_width,
             config->dst_addr_width, config->src_maxburst,
             config->dst_maxburst, config->slave_id);

    return SPI_MASTER_OK;
}

static spi_master_status_t spi_dma_tx_submit(struct sunxi_spi *sspi)
{
    hal_dma_status_t ret;
    uint32_t len = sspi->transfer->tx_len;
    unsigned long *hdma = sspi->dma_tx.hdma;
    struct dma_slave_config *config = &sspi->dma_tx.config;

    ret = hal_dma_slave_config(hdma, config);
    if (ret)
    {
        SPI_ERR("[spi%d] dma slave config failed! return %d\n",
                sspi->port, ret);
        return SPI_MASTER_ERROR;
    }

    ret = hal_dma_prep_device(hdma, config->dst_addr, config->src_addr, len,
                              config->direction);
    if (ret)
    {
        SPI_ERR("[spi%d] dma prep device failed! return %d\n",
                sspi->port, ret);
        return SPI_MASTER_ERROR;
    }

    ret = hal_dma_start(hdma);
    if (ret)
    {
        SPI_ERR("[spi%d] dma start error! return %d\n", sspi->port,
                ret);
        return SPI_MASTER_ERROR;
    }

    return SPI_MASTER_OK;
}

static void spi_dma_rx_callback(void *para)
{
    sunxi_spi_t *sspi = (sunxi_spi_t *)para;

    if (sem_post(&sspi->sem_rx) < 0)
    {
        SPI_ERR("[spi%d] xSemaphoreGive failed.\n", sspi->port);
    }
}

static spi_master_status_t spi_dma_rx_config(struct sunxi_spi *sspi)
{
    hal_dma_status_t ret;
    uint32_t len = sspi->transfer->rx_len, size = 0;
    uint8_t *buf = sspi->transfer->rx_buf;
    struct dma_slave_config *config = &sspi->dma_rx.config;

    if (NULL == buf)
    {
        SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
        return SPI_MASTER_INVALID_PARAMETER;
    }

    if (len <= ALIGN_DMA_BUF_SIZE)
    {
        buf = sspi->align_dma_buf;
        memset(buf, 0, ALIGN_DMA_BUF_SIZE);
    }
    else
    {
        SPI_ERR("[spi%d] rx size over dma align buf size\n",
                sspi->port);
        /* buffer on DMA must align to cache line */
        if ((uint32_t)buf & (64 - 1) || len & (64 - 1))
        {
            SPI_ERR("[spi%d] rx buf or len not align to 64\n",
                    sspi->port);
            return SPI_MASTER_INVALID_PARAMETER;
        }
    }
    //cpu_dcache_clean_invalidate(buf, ALIGN(len, 64));

    spi_enable_dma_irq(SPI_FIFO_CTL_RX_DRQEN, sspi);

#ifdef SPI_DUMPREG_LEVEL
    SPI_INFO("[spi%d] dump reg:\n", sspi->port);
    spi_dump_reg(sspi, 0, 0x60);
#endif

    ret = hal_dma_chan_request(&sspi->dma_rx.hdma);
    if (ret == HAL_DMA_CHAN_STATUS_BUSY)
    {
        SPI_ERR("[spi%d] request dma_rx failed\n", sspi->port);
        return SPI_MASTER_ERROR;
    }

    config->direction = DMA_DEV_TO_MEM;
    config->dst_addr = (uint32_t)buf;
    config->src_addr = sspi->base + SPI_RXDATA_REG;

    if (len % DMA_SLAVE_BUSWIDTH_4_BYTES)
    {
        config->src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
        config->dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
        config->dst_maxburst = DMA_SLAVE_BURST_16;
        config->src_maxburst = DMA_SLAVE_BURST_16;
    }
    else
    {
        config->src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        config->dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
        config->dst_maxburst = DMA_SLAVE_BURST_8;
        config->src_maxburst = DMA_SLAVE_BURST_8;
    }

    config->slave_id =
        sunxi_slave_id(DRQDST_SDRAM, SUNXI_SPI_DRQ_RX(sspi->port));

    SPI_INFO("[spi%d] config:\n"
             "  direction: %d\n"
             "  src_addr: 0x%08x,"
             "  dst_addr: 0x%08x\n"
             "  src_addr_width: %d,"
             "  dst_addr_width: %d\n"
             "  src_maxburst: %u,"
             "  dst_maxburst: %u\n"
             "  slave_id: 0x%08x\n",
             sspi->port, config->direction, config->src_addr,
             config->dst_addr, config->src_addr_width,
             config->dst_addr_width, config->src_maxburst,
             config->dst_maxburst, config->slave_id);

    return SPI_MASTER_OK;
}

static spi_master_status_t spi_dma_rx_submit(struct sunxi_spi *sspi)
{
    hal_dma_status_t ret;
    uint32_t len = sspi->transfer->rx_len, size = 0;
    unsigned long *hdma = sspi->dma_rx.hdma;
    struct dma_slave_config *config = &sspi->dma_rx.config;
    struct sunxi_dma_chan *chan = NULL;

    ret = hal_dma_slave_config(hdma, config);
    if (ret)
    {
        SPI_ERR("[spi%d] dma slave config failed! return %d\n",
                sspi->port, ret);
        return SPI_MASTER_ERROR;
    }

    ret = hal_dma_prep_device(hdma, config->dst_addr, config->src_addr, len,
                              config->direction);
    if (ret)
    {
        SPI_ERR("[spi%d] dma prep device failed! return %d\n",
                sspi->port, ret);
        return SPI_MASTER_ERROR;
    }

    chan = (struct sunxi_dma_chan *)hdma;
    chan->callback = spi_dma_rx_callback;
    chan->callback_param = sspi;

    ret = hal_dma_start(hdma);
    if (ret)
    {
        SPI_ERR("[spi%d] dma start error! return %d\n", sspi->port,
                ret);
        return SPI_MASTER_ERROR;
    }

    return SPI_MASTER_OK;
}

#endif

static spi_master_status_t spi_transfer(sunxi_spi_t *sspi)
{
    //hal_dma_status_t ret;

    switch (sspi->mode_type)
    {
        case SGLE_HALF_DUPLEX_RX:
        case DUAL_HALF_DUPLEX_RX:
        case QUAD_HALF_DUPLEX_RX:
        {
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            /* >64 use DMA transfer, or use cpu */
            if (sspi->transfer->rx_len > BULK_DATA_BOUNDARY)
            {
                SPI_INFO("[spi%d] rx by dma\n", sspi->port);
                /* For Rx mode, the DMA end(not TC flag) is real end. */
                spi_disable_irq(SPI_INTEN_TC, sspi);
                if (spi_dma_rx_config(sspi))
                {
                    return SPI_MASTER_ERROR;
                }
                if (spi_dma_rx_submit(sspi))
                {
                    return SPI_MASTER_ERROR;
                }
                spi_start_xfer(sspi);
            }
            else
            {
#endif
                SPI_INFO("[spi%d] rx by cpu\n", sspi->port);
                spi_clr_irq_pending(SPI_INT_STA_MASK, sspi);
                spi_start_xfer(sspi);
                spi_cpu_read(sspi);
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            }
#endif
            break;
        }
        case SGLE_HALF_DUPLEX_TX:
        case DUAL_HALF_DUPLEX_TX:
        case QUAD_HALF_DUPLEX_TX:
        {
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            /* >64 use DMA transfer, or use cpu */
            if (sspi->transfer->tx_len > BULK_DATA_BOUNDARY)
            {
                SPI_INFO("[spi%d] tx by dma\n", sspi->port);
                //sspi->sem = 1;
                spi_start_xfer(sspi);
                if (spi_dma_tx_config(sspi))
                {
                    return SPI_MASTER_ERROR;
                }
                if (spi_dma_tx_submit(sspi))
                {
                    return SPI_MASTER_ERROR;
                }
            }
            else
            {
#endif
                SPI_INFO("[spi%d] tx by cpu\n", sspi->port);
                spi_start_xfer(sspi);
                spi_cpu_write(sspi);
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            }
#endif
            break;
        }
        case FULL_DUPLEX_TX_RX:
        {
                SPI_INFO("[spi%d] tx and rx by cpu\n", BULK_DATA_BOUNDARY);
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            /* >64 use DMA transfer, or use cpu */
            if (sspi->transfer->rx_len > BULK_DATA_BOUNDARY)
            {
                SPI_INFO("[spi%d] tx and rx by dma\n", sspi->port);
                /* For Rx mode, the DMA end(not TC flag) is real end. */
                spi_disable_irq(SPI_INTEN_TC, sspi);
                spi_start_xfer(sspi);
                spi_cpu_write(sspi);
                if (spi_dma_rx_config(sspi))
                {
                    return SPI_MASTER_ERROR;
                }
                if (spi_dma_rx_submit(sspi))
                {
                    return SPI_MASTER_ERROR;
                }
            }
            else
            {
#endif
                SPI_INFO("[spi%d] tx and rx by cpu\n", sspi->port);
                spi_start_xfer(sspi);
                spi_cpu_write(sspi);
                spi_cpu_read(sspi);
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            }
#endif
            break;
        }
        default:
        {
            SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
            return SPI_MASTER_INVALID_PARAMETER;
        }
    }

    return SPI_MASTER_OK;
}

/*fix me*/
/* wake up the sleep thread, and give the result code */
static int spi_irq_handler(int irq, FAR void *context, void *ptr)
{
    uint32_t status = 0;
    sunxi_spi_t *sspi = (sunxi_spi_t *)ptr;

    //enable = spi_qry_irq_enable(sspi);
    status = spi_qry_irq_pending(sspi);
    spi_clr_irq_pending(status, sspi);

    sspi->result = SPI_XFER_OK;

    printf("=======spi irq handler====\n");
    /* master mode, Transfer Complete Interrupt */
    if (status & SPI_INT_STA_TC)
    {
        SPI_INFO("[spi%d] SPI TC COME\n", sspi->port);
        spi_disable_irq(SPI_INT_STA_TC | SPI_INT_STA_ERR, sspi);
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
        if (sspi->sem)
        {
            sspi->sem = 0;
            if (sem_post(&sspi->sem_tx) < 0)
                SPI_ERR("[spi%d] sem_post failed.\n", sspi->port);
        }
#endif
        return 0;
    }
    else if (status & SPI_INT_STA_ERR)     /* master mode:err */
    {
        SPI_ERR("[spi%d]  SPI ERR! status %#lx\n", sspi->port, status);
        SPI_INFO("[spi%d] dump reg:\n", sspi->port);
        spi_dump_reg(sspi, 0, 0x60);
        spi_disable_irq(SPI_INT_STA_TC | SPI_INT_STA_ERR, sspi);
        sspi->result = SPI_XFER_FAILED;
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
        if (sspi->sem)
        {
            sspi->sem = 0;
            if (sem_post(&sspi->sem_tx) < 0)
                SPI_ERR("[spi%d] sem_post failed.\n", sspi->port);
        }
#endif
        return 0;
    }
    return 0;
}

#if 0
static spi_master_status_t spi_pinctrl_init(sunxi_spi_t *sspi)
{
    uint8_t i;
    uint32_t flags = 0;
    drv_pinmux_status_t ret;

    switch (sspi->port)
    {
        case HAL_SPI_MASTER_0:
            sspi->pin_num = SPI0_NUM;
            sspi->pin_mux = SPI_MUXSEL;
            sspi->pin_drv = SPI_DRVSEL;
            sspi->pin = malloc(sizeof(uint32_t) * SPI0_NUM);
            sspi->pin[0] = SPI0_CLK;
            sspi->pin[1] = SPI0_MOSI;
            sspi->pin[2] = SPI0_CS0;
            sspi->pin[3] = SPI0_MISO;
            sspi->pin[4] = SPI0_HOLD;
            sspi->pin[5] = SPI0_WP;
            break;
        case HAL_SPI_MASTER_1:
            sspi->pin_num = SPI1_NUM;
            sspi->pin_mux = SPI_MUXSEL;
            sspi->pin_drv = SPI_DRVSEL;
            sspi->pin = malloc(sizeof(uint32_t) * SPI1_NUM);
            sspi->pin[0] = SPI1_CLK;
            sspi->pin[1] = SPI1_MOSI;
            sspi->pin[2] = SPI1_CS0;
            sspi->pin[3] = SPI1_MISO;
            break;
        case HAL_SPI_MASTER_2:
            sspi->pin_num = SPI2_NUM;
            sspi->pin_mux = SPI_MUXSEL;
            sspi->pin_drv = SPI_DRVSEL;
            sspi->pin = malloc(sizeof(uint32_t) * SPI2_NUM);
            sspi->pin[0] = SPI2_CLK;
            sspi->pin[1] = SPI2_MOSI;
            sspi->pin[2] = SPI2_CS0;
            sspi->pin[3] = SPI2_MISO;
            break;
        default:
            SPI_ERR("spi%d is invalid\n", sspi->port);
            return SPI_MASTER_INVALID_PARAMETER;
    }

    for (i = 0; i < sspi->pin_num; i++)
    {
        ret = drv_gpio_pinmux_set_function(sspi->pin[i], sspi->pin_mux);
        if (ret)
        {
            SPI_ERR(
                "[spi%d] PIN%lu set function failed! return %d\n",
                sspi->port, sspi->pin[i], ret);
            return SPI_MASTER_ERROR;
        }
        ret = drv_gpio_set_driving_level(sspi->pin[i], sspi->pin_drv);
        if (ret)
        {
            SPI_ERR(
                "[spi%d] PIN%lu set driving level failed! return %d\n",
                sspi->port, sspi->pin[i], ret);
            return SPI_MASTER_ERROR;
        }
        if (i == 2) //CS
        {
            ret = drv_gpio_set_pull_state(sspi->pin[i], DRV_GPIO_PULL_UP);
        }
        else
        {
            ret = drv_gpio_set_pull_state(sspi->pin[i], DRV_GPIO_PULL_DOWN_DISABLE);
        }
    }

    return SPI_MASTER_OK;
}
#endif

static uint32_t clock_get_pll6(void)
{
    return 600;
}

static int spi_clk_init(u32 bus, u32 mod_clk)
{
    struct sunxi_ccm_reg *const ccm =
        (struct sunxi_ccm_reg *)(0x03001000L);
    uint32_t mclk_base = (uint32_t)&ccm->spi0_clk_cfg + bus * 0x4;
    uint32_t rval, reg_val;
    uint32_t source_clk = 0;
    uint32_t m, n, div, factor_m;

    /* SCLK = src/M/N */
    /* N: 00:1 01:2 10:4 11:8 */
    /* M: factor_m + 1 */
#if 0
    n = 0;
    m = 1;
    div = 1;
    factor_m = m - 1;
    rval = (1U << 31) | (0x0 << 24) | (n << 8) | factor_m;
    source_clk = 24000000;
#else
    source_clk = clock_get_pll6() * 1000000;/*pll6, fix me*/
    SPI_INFO("source_clk: %d Hz\n", source_clk);

    div = (source_clk + mod_clk - 1) / mod_clk;
    div = div == 0 ? 1 : div;
    if (div > 128)
    {
        m = 1;
        n = 0;
        return -1;
    }
    else if (div > 64)
    {
        n = 3;
        m = div >> 3;
    }
    else if (div > 32)
    {
        n = 2;
        m = div >> 2;
    }
    else if (div > 16)
    {
        n = 1;
        m = div >> 1;
    }
    else
    {
        n = 0;
        m = div;
    }

    factor_m = m - 1;
    rval = (1U << 31) | (0x1 << 24) | (n << 8) | factor_m;
    /**sclk_freq = source_clk / (1 << n) / m; */
#endif
    SPI_INFO("source_clk : %d Hz, div =%d, n= %d, m=%d\n", source_clk, div, n, m);
    hal_writel(rval, mclk_base);
    /* spi reset */
    //setbits_le32(&ccm->spi_gate_reset, (0<<RESET_SHIFT));
    //setbits_le32(&ccm->spi_gate_reset, (1<<RESET_SHIFT));
    reg_val = hal_readl(&ccm->spi_gate_reset);
    reg_val = reg_val & (~(1 << 17)); //spi 1
    reg_val = reg_val & (~(1 << 16)); //spi 0
    hal_writel(reg_val, (uint32_t)&ccm->spi_gate_reset);

    reg_val = hal_readl(&ccm->spi_gate_reset);
    reg_val = reg_val | (1 << 17); //spi 1
    reg_val = reg_val | (1 << 16); //spi 0
    hal_writel(reg_val, (uint32_t)&ccm->spi_gate_reset);

    /* spi gating */
    //setbits_le32(&ccm->spi_gate_reset, (1<<GATING_SHIFT));
    reg_val = hal_readl(&ccm->spi_gate_reset);
    reg_val = reg_val | (1 << 1); //spi 1
    reg_val = reg_val | (1 << 0); //spi 0
    hal_writel(reg_val, (uint32_t)&ccm->spi_gate_reset);

    reg_val = hal_readl(&ccm->spi_gate_reset);
    return 0;
}
/*
spi_master_status_t spi_clk_exit(sunxi_spi_t *sspi)
{
    hal_clk_status_t ret;

    ret = hal_clock_disable(sspi->mclk);
    if (ret) {
        SPI_ERR("[spi%d] couldn't disable mlck! return %d\n",
                sspi->port, ret);
        return SPI_MASTER_ERROR;
    }

    return SPI_MASTER_OK;
}
*/

/*  Platform-specific, SPI frequency function */
uint32_t spi_set_freq(uint32_t port, uint32_t frequency)
{
	sunxi_spi_t *sspi = &g_sunxi_spi[port];

	if (frequency > SPI_MAX_FREQUENCY)
		SPI_ERR("[spi%d] invalid parameter! max_frequency is 100MHZ\n",sspi->port);

	if (spi_clk_init(port, frequency))
	{
		SPI_ERR("[spi%d] init clk error\n", sspi->port);
		return SPI_MASTER_ERROR;
	}
	if (frequency >= SPI_HIGH_FREQUENCY)
		spi_sample_delay(0, 1, sspi);
	else if (frequency <= SPI_LOW_FREQUENCY)
		spi_sample_delay(1, 0, sspi);
	else
		spi_sample_delay(0, 0, sspi);

	return 0;
}

static spi_master_status_t spi_cpu_complete(sunxi_spi_t *sspi)
{
    uint32_t timeout = 0xffff;

    while (!sspi->result && timeout--)
        ;
    if (timeout <= 0)
    {
        SPI_ERR("[spi%d] xfer timeout\n", sspi->port);
        SPI_INFO("[spi%d] dump reg:\n", sspi->port);
        spi_dump_reg(sspi, 0, 0x60);
        return SPI_MASTER_ERROR_TIMEOUT;
    }
    else if (SPI_XFER_FAILED == sspi->result)
    {
        SPI_ERR("[spi%d] xfer failed...\n", sspi->port);
        SPI_INFO("[spi%d] dump reg:\n", sspi->port);
        spi_dump_reg(sspi, 0, 0x60);
        return SPI_MASTER_ERROR;
    }

    return SPI_MASTER_OK;
}

#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
static spi_master_status_t spi_dma_tx_complete(sunxi_spi_t *sspi)
{
    uint32_t sem_ret;
    uint32_t timeout = 0x7fffffff;
    unsigned long *hdma = sspi->dma_tx.hdma;
    hal_dma_status_t dma_ret;
    spi_master_status_t ret;

    sem_ret = sem_wait(&sspi->sem_tx);
    if (sem_ret == 0)
    {
        if (SPI_XFER_OK == sspi->result)
        {
            SPI_INFO("ok\n");
            ret = SPI_MASTER_OK;
        }
        else if (SPI_XFER_FAILED == sspi->result)
        {
            SPI_ERR("[spi%d] xfer failed...\n", sspi->port);
            SPI_INFO("[spi%d] dump reg:\n", sspi->port);
            spi_dump_reg(sspi, 0, 0x60);
            ret = SPI_MASTER_ERROR;
        }
    }
    else
    {
        SPI_ERR("[spi%d] dma xfer timeout\n", sspi->port);
        SPI_INFO("[spi%d] dump reg:\n", sspi->port);
        spi_dump_reg(sspi, 0, 0x60);

        sspi->result = SPI_XFER_FAILED;

        if (sem_post(&sspi->sem_tx) < 0)
        {
            SPI_ERR("[spi%d] xSemaphoreGive failed.\n", sspi->port);
        }

        ret = SPI_MASTER_ERROR_TIMEOUT;
    }

end:

    dma_ret = hal_dma_stop(hdma);
    if (dma_ret)
    {
        SPI_ERR("[spi%d] dma stop error! ret %d\n", sspi->port,
                dma_ret);
        return SPI_MASTER_ERROR;
    }

    dma_ret = hal_dma_chan_free(hdma);
    if (dma_ret)
    {
        SPI_ERR("[spi%d] free dma_tx failed, ret %d\n", sspi->port,
                dma_ret);
        return SPI_MASTER_ERROR;
    }

    return ret;
}

static spi_master_status_t spi_dma_rx_complete(sunxi_spi_t *sspi)
{
    uint32_t sem_ret;
    uint32_t len = sspi->transfer->rx_len, size = 0;
    uint8_t *buf = sspi->transfer->rx_buf;
    unsigned long *hdma = sspi->dma_rx.hdma;
    hal_dma_status_t dma_ret;
    spi_master_status_t ret;

#ifdef SPI_DATA_LEVEL
    unsigned int i, j;
    u8 dbuf[64], cnt = 0;
#endif

    sem_ret = sem_wait(&sspi->sem_rx);
    if (sem_ret < 0)
    {
        SPI_ERR("[spi%d] dma xfer timeout\n", sspi->port);
        SPI_INFO("[spi%d] dump reg:\n", sspi->port);
        spi_dump_reg(sspi, 0, 0x40);

        sspi->result = SPI_XFER_FAILED;

        ret = SPI_MASTER_ERROR_TIMEOUT;
        goto end;
    }

    //cpu_dcache_invalidate(sspi->align_dma_buf, ALIGN(len, 64));
    sspi->result = SPI_XFER_OK;
    if (len <= ALIGN_DMA_BUF_SIZE)
    {
        memcpy(buf, sspi->align_dma_buf, len);
    }
    ret = SPI_MASTER_OK;
    SPI_INFO("ok\n");

#ifdef SPI_DATA_LEVEL
    SPI_INFO("rx_len = %d\n", len);
    for (i = 0; i < len; i += 16)
    {
        cnt = 0;
        cnt += sprintf(dbuf + cnt, "%03x: ", i);
        for (j = 0; ((i + j) < len) && (j < 16); j++)
            cnt += sprintf(dbuf + cnt, "%02x ",
                           ((uint8_t *)(buf))[i + j]);
        printf("%s\n", dbuf);
    }
#endif

end:
    spi_disable_irq(SPI_INT_STA_TC | SPI_INT_STA_ERR, sspi);

    dma_ret = hal_dma_stop(hdma);
    if (dma_ret)
    {
        SPI_ERR("[spi%d] dma stop error! ret %d\n", sspi->port,
                dma_ret);
        ret = SPI_MASTER_ERROR;
    }

    dma_ret = hal_dma_chan_free(hdma);
    if (dma_ret)
    {
        SPI_ERR("[spi%d] free dma_rx failed, ret %d\n", sspi->port,
                dma_ret);
        return SPI_MASTER_ERROR;
    }

    return ret;
}
#endif


/*
 * < 64 : cpu ;  >= 64 : dma
 * wait for done completion in this function, wakup in the irq hanlder
 */
static spi_master_status_t sunxi_spi_xfer(hal_spi_master_port_t port,
        hal_spi_master_transfer_t *transfer)
{
    spi_master_status_t ret = 0;
    sunxi_spi_t *sspi = &g_sunxi_spi[port];

    if (NULL == transfer)
    {
        SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
        ret = SPI_MASTER_INVALID_PARAMETER;
        goto end;
    }

    SPI_INFO("[spi%d] tl=%u rl=%u, tsl=%u\n", sspi->port, transfer->tx_len,
             transfer->rx_len, transfer->tx_single_len);

    if ((!transfer->tx_buf && !transfer->rx_buf) ||
        (!transfer->tx_len && !transfer->rx_buf))
    {
        SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
        ret = SPI_MASTER_INVALID_PARAMETER;
        goto end;
    }

    sspi->result = SPI_XFER_READY;
    sspi->transfer = transfer;

    if (spi_mode_check(sspi))
    {
        SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
        ret = SPI_MASTER_INVALID_PARAMETER;
        goto end;
    }

    spi_clr_irq_pending(SPI_INT_STA_MASK, sspi);
    spi_reset_fifo(sspi);
    spi_enable_irq(SPI_INTEN_TC | SPI_INTEN_ERR, sspi);
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
    spi_disable_dma_irq(SPI_FIFO_CTL_DRQEN_MASK, sspi);
#endif

    if (spi_transfer(sspi))
    {
        ret = SPI_MASTER_ERROR;
        goto end;
    }

    switch (sspi->mode_type)
    {
        case SGLE_HALF_DUPLEX_RX:
        case DUAL_HALF_DUPLEX_RX:
        case QUAD_HALF_DUPLEX_RX:
        case FULL_DUPLEX_TX_RX:
        {
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            /* >64 use DMA transfer, or use cpu */
            if (sspi->transfer->rx_len > BULK_DATA_BOUNDARY)
            {
                if (spi_dma_rx_complete(sspi))
                {
                    ret = SPI_MASTER_ERROR;
                    goto end;
                }
            }
            else
            {
#endif
                if (spi_cpu_complete(sspi))
                {
                    ret = SPI_MASTER_ERROR;
                    goto end;
                }
                else
                {
                    ret = SPI_MASTER_OK;
                }
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            }
#endif
            break;
        }
        case SGLE_HALF_DUPLEX_TX:
        case DUAL_HALF_DUPLEX_TX:
        case QUAD_HALF_DUPLEX_TX:
        {
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            /* >64 use DMA transfer, or use cpu */
            if (sspi->transfer->tx_len > BULK_DATA_BOUNDARY)
            {
                if (spi_dma_tx_complete(sspi))
                {
                    ret = SPI_MASTER_ERROR;
                    goto end;
                }
            }
            else
            {
#endif
                if (spi_cpu_complete(sspi))
                {
                    ret = SPI_MASTER_ERROR;
                    goto end;
                }
                else
                {
                    ret = SPI_MASTER_OK;
                }
#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
            }
#endif
            break;
        }
        default:
        {
            SPI_ERR("[spi%d] invalid parameter\n", sspi->port);
            ret = SPI_MASTER_INVALID_PARAMETER;
        }
    }

end:
    if (sspi->mode_type != MODE_TYPE_NULL)
    {
        sspi->mode_type = MODE_TYPE_NULL;
    }

    return ret;
}

void sunxi_spi_workqueue(void *arg)
{
	sunxi_spi_t *sspi = (sunxi_spi_t *)arg;
	hal_spi_master_transfer_t *tr, *tr_last = NULL;

	list_for_each_entry(tr, &sspi->queue, node, hal_spi_master_transfer_t)
	{
		//delete the last transfer that has trans complete.
		//free the memory, which malloc in read/write/control func.
		if (tr_last != NULL)
		{
			list_del(&tr_last->node);
			free(tr_last);
		}
		tr_last = tr;
		sunxi_spi_xfer(sspi->port, tr);
		if (tr->mode == SUNXI_SPI_SYNC)
		{
			if (sem_post(&sspi->complete) == 0)
			{
				SPI_INFO("[spi%d] spi workqueue trans complete\n");
			}
			else
				SPI_ERR("[spi%d] xSemaphoreGive failed.\n", sspi->port);
		}
	}

	//delete last one.
	if (tr_last != NULL)
	{
		list_del(&tr_last->node);
		free(tr_last);
	}
}

static spi_master_status_t sunxi_spi_async(hal_spi_master_port_t port,
        hal_spi_master_transfer_t *transfer)
{
    SPI_INFO("\t %s \n", __func__);
    sunxi_spi_t *sspi = &g_sunxi_spi[port];
    hal_spi_master_transfer_t *tr = malloc(sizeof(hal_spi_master_transfer_t));

    memcpy(tr, transfer, sizeof(hal_spi_master_transfer_t));

    list_add_tail(&tr->node, &sspi->queue);
    /* work_queue(LPWORK, sspi->work, sunxi_spi_workqueue, sspi, 0); */
    sunxi_spi_workqueue(sspi);

    return SPI_MASTER_OK;
}

static spi_master_status_t sunxi_spi_sync(hal_spi_master_port_t port,
        hal_spi_master_transfer_t *transfer)
{
    SPI_INFO("\t %s \n", __func__);
    sunxi_spi_t *sspi = &g_sunxi_spi[port];
    hal_spi_master_transfer_t *tr = malloc(sizeof(hal_spi_master_transfer_t));

    memcpy(tr, transfer, sizeof(hal_spi_master_transfer_t));

    list_add_tail(&tr->node, &sspi->queue);
    /* work_queue(LPWORK, sspi->work, sunxi_spi_workqueue, sspi, 0);*/
    sunxi_spi_workqueue(sspi);
    if (sem_wait(&sspi->complete) < 0)
	    return SPI_MASTER_ERROR;

    return SPI_MASTER_OK;

}

#define SUNXI_CCM_BASE                  (0x03001000L)
static int sunxi_get_spic_clk(int bus)
{
    struct sunxi_ccm_reg *const ccm =
        (struct sunxi_ccm_reg *)SUNXI_CCM_BASE;
    u32 mclk_base = (u32)&ccm->spi0_clk_cfg + bus * 0x4;
    u32 reg_val = 0;
    u32 src = 0, clk = 0, sclk_freq = 0;
    u32 n, m;

    reg_val = hal_readl(mclk_base);
    src = (reg_val >> 24) & 0x7;
    n = (reg_val >> 8) & 0x3;
    m = ((reg_val >> 0) & 0xf) + 1;

    switch (src)
    {
        case 0:
            clk = 24000000;
            break;
        case 1:
            clk = clock_get_pll6() * 1000000;
            break;
        default:
            clk = 0;
            break;
    }
    sclk_freq = clk / (1 << n) / m;
    SPI_INFO("addr is %p, sclk_freq= %d Hz,reg_val: %x , n=%d, m=%d\n", mclk_base,  sclk_freq, reg_val, n, m);
    return sclk_freq;

}

/* set spi clock */
static void spi_set_clk(u32 spi_clk, u32 ahb_clk, uint32_t base_addr, u32 cdr)
{
    u32 reg_val = 0;
    u32 div_clk = 0;

    SPI_INFO("set spi clock %d, mclk %d\n", spi_clk, ahb_clk);
    reg_val = hal_readl(base_addr + SPI_CLK_CTL_REG);

    /* CDR2 */
    if (cdr)
    {
        div_clk = ahb_clk / (spi_clk * 2) - 1;
        reg_val &= ~SPI_CLK_CTL_CDR2;
        reg_val |= (div_clk | SPI_CLK_CTL_DRS);
        SPI_INFO("CDR2 - n = %d\n", div_clk);
    }
    else     /* CDR1 */
    {
        while (ahb_clk > spi_clk)
        {
            div_clk++;
            ahb_clk >>= 1;
        }
        reg_val &= ~(SPI_CLK_CTL_CDR1 | SPI_CLK_CTL_DRS);
        reg_val |= (div_clk << 8);
        SPI_INFO("CDR1 - n = %d\n", div_clk);
    }

    hal_writel(reg_val, base_addr + SPI_CLK_CTL_REG);
}


static spi_master_status_t sunxi_spi_hw_config(hal_spi_master_port_t port, hal_spi_master_config_t *spi_config)
{
    uint sclk_freq = 0;
    uint32_t clock_frequency;
    sunxi_spi_t *sspi = &g_sunxi_spi[port];

    if (NULL == spi_config)
    {
        SPI_ERR("[spi%d] invalid parameter\n", port);
        return SPI_MASTER_INVALID_PARAMETER;
    }

    sspi->base = g_spi_regbase[port];
    sspi->port = port;
    sspi->mode_type = MODE_TYPE_NULL;

    if (spi_config->clock_frequency)
    {
        clock_frequency = spi_config->clock_frequency;
    }
    else
    {
        clock_frequency = SPI_MOD_CLK;
    }

    if (clock_frequency > SPI_MAX_FREQUENCY)
        SPI_ERR("[spi%d] invalid parameter! max_frequency is 100MHZ\n",
                sspi->port);
    else
        SPI_INIT("[spi%d] clock_frequency = %ldHZ\n", sspi->port,
                 clock_frequency);

    if (clock_frequency >= SPI_HIGH_FREQUENCY)
    {
        spi_sample_delay(0, 1, sspi);
    }
    else if (clock_frequency <= SPI_LOW_FREQUENCY)
    {
        spi_sample_delay(1, 0, sspi);
    }
    else
    {
        spi_sample_delay(0, 0, sspi);
    }

    sclk_freq = sunxi_get_spic_clk(sspi->port);
    if (!sclk_freq)
    {
        SPI_INFO("spi clk error ! \n");
    }

    spi_set_clk(clock_frequency, sclk_freq, sspi->base, 0);

    if (spi_config->slave_port)
    {
        SPI_ERR("[spi%d] software control cs isn't support \n",
                sspi->port);
        return SPI_MASTER_INVALID_PARAMETER;
    }
    else
    {
        spi_set_cs(spi_config->slave_port, sspi);
    }

    if (spi_config->bit_order)
    {
        spi_config_tc(SPI_LSB_FIRST_ACTIVE_, sspi);
    }

    spi_config_tc((spi_config->cpol) | (spi_config->cpha), sspi);

    return SPI_MASTER_OK;

}

static hal_spi_master_config_t cfg =
{
    //.clock_frequency = SPI_MAX_FREQUENCY,
    .clock_frequency = 50 * 1000 * 1000,
    .slave_port = HAL_SPI_MASTER_SLAVE_0,
    .cpha = HAL_SPI_MASTER_CLOCK_PHASE0,
    .cpol = HAL_SPI_MASTER_CLOCK_POLARITY0,
    .bit_order = HAL_SPI_MASTER_LSB_FIRST,
};//default SPI configure

spi_master_status_t sunxi_spi_init(hal_spi_master_port_t port)
{
    uint8_t i;
    sunxi_spi_t *sspi = &g_sunxi_spi[port];

    for (i = 0; i < HAL_SPI_MASTER_MAX; i++)
    {
        g_spi_regbase[i] = SUNXI_SPI0_PBASE + i * SUNXI_SPI_REG_SIZE;
        g_spi_irqnum[i] = SUNXI_IRQ_SPI0 + i;
    }

    sspi->base = g_spi_regbase[port];
    sspi->irqnum = g_spi_irqnum[port];
    sspi->port = port;
    sspi->mode_type = MODE_TYPE_NULL;

    SPI_INFO("spi[%d] init ,reg base is %p \n", port, sspi->base);

 //   if (request_irq(sspi->irqnum, spi_irq_handler, 0, NULL, sspi) < 0)
 //   {
 //       SPI_ERR("[spi%d] request irq error\n", sspi->port);
 //       return SPI_MASTER_ERROR;
 //   }
 //   enable_irq(sspi->irqnum);
    irq_attach(86, spi_irq_handler, sspi);

    up_enable_irq(86);

#if 0
    if (spi_pinctrl_init(sspi))
    {
        SPI_ERR("[spi%d] init pinctrl error\n", sspi->port);
        return SPI_MASTER_ERROR;
    }
#endif

    if (spi_clk_init(sspi->port, SPI_MAX_FREQUENCY))
    {
        SPI_ERR("[spi%d] init clk error\n", sspi->port);
        return SPI_MASTER_ERROR;
    }

    spi_soft_reset(sspi);

    sunxi_spi_hw_config(port, &cfg);

    spi_enable_bus(sspi);
    spi_set_master(sspi);
    spi_enable_tp(sspi);
    /*spi controller sends ss signal automatically*/
    spi_ss_owner(sspi, 0);

    /* reset fifo */
    spi_reset_fifo(sspi);

#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
    uint8_t ret;
    ret = sem_init(&sspi->sem_tx, 0, 0);
    if (ret < 0) {
        SPI_ERR("[spi%d] creating semaphore_tx failed.\n", sspi->port);
    }

    ret = sem_init(&sspi->sem_rx, 0, 0);
    if (ret < 0) {
        SPI_ERR("[spi%d] creating semaphore_rx failed.\n", sspi->port);
    }
    sspi->align_dma_buf = dma_alloc_coherent(ALIGN_DMA_BUF_SIZE);
    if (!sspi->align_dma_buf)
    {
        SPI_ERR("[spi%d] alloc memory failed\n", sspi->port);
        return SPI_MASTER_ERROR_NOMEM;
    }
    SPI_INIT("[spi%d] DMA xfer enable\n", sspi->port);
#else
    SPI_INIT("[spi%d] CPU xfer only\n", sspi->port);
#endif

    INIT_LIST_HEAD(&sspi->queue);
    sspi->work = malloc(sizeof(struct work_s));
    if (sspi->work == NULL)
    {
        SPI_ERR("[spi%d] malloc for queue failed\n", sspi->port);
        return SPI_MASTER_ERROR_NOMEM;
    }


    return SPI_MASTER_OK;
}

static spi_master_status_t sunxi_spi_uninit(hal_spi_master_port_t port)
{
    uint8_t i;
    sunxi_spi_t *sspi = &g_sunxi_spi[port];

    spi_disable_bus(sspi);

    //disable_irq(sspi->irqnum);

#ifndef CONFIG_SUNXI_SPI_CPU_XFER_ONLY
    dma_free_coherent(sspi->align_dma_buf);
    sem_destroy(&sspi->sem_tx);
    sem_destroy(&sspi->sem_rx);
#endif
#if 0
    //TODO clk exit
    if (spi_clk_exit(sspi))
    {
        SPI_ERR("[spi%d] exit clk error\n", sspi->port);
        return SPI_MASTER_ERROR;
    }
#endif
    //free_irq(sspi->irqnum, sspi);

    return SPI_MASTER_OK;
}

static spi_master_status_t sunxi_spi_write(hal_spi_master_port_t port,
        const void *buf, uint32_t size, hal_spi_sync_mode_e mode)
{
    spi_master_status_t ret;
    hal_spi_master_transfer_t *tr = malloc(sizeof(hal_spi_master_transfer_t));

    tr->tx_buf = buf;
    tr->tx_len = size;
    tr->rx_buf = NULL;
    tr->rx_len = 0;
    tr->dummy_byte = 0;
    tr->tx_single_len = size;
    tr->tx_nbits = SPI_NBITS_SINGLE;
    tr->mode = mode;

    SPI_INFO("spi[%d] write data,len is %d \n", port, size);
    if (mode == SUNXI_SPI_SYNC)
	    ret = sunxi_spi_sync(port, tr);
    else
	    ret = sunxi_spi_async(port, tr);
    return ret;
}

static spi_master_status_t sunxi_spi_read(hal_spi_master_port_t port,
        void *buf, uint32_t size, hal_spi_sync_mode_e mode)
{
    spi_master_status_t ret;
    hal_spi_master_transfer_t *tr = malloc(sizeof(hal_spi_master_transfer_t));

    tr->rx_buf = buf;
    tr->rx_len = size;
    tr->tx_buf = NULL;
    tr->tx_len = 0;
    tr->dummy_byte = 0;
    tr->tx_single_len = size;
    tr->rx_nbits = SPI_NBITS_SINGLE;

    SPI_INFO("spi[%d] read data,len is %d \n", port, size);
    if (mode == SUNXI_SPI_SYNC)
	    ret = sunxi_spi_sync(port, tr);
    else
	    ret = sunxi_spi_async(port, tr);
    return ret;
}

spi_master_status_t sunxi_spi_control(hal_spi_master_port_t port,
        hal_spi_transfer_cmd_t cmd, void *args)
{
    spi_master_status_t ret;
    hal_spi_master_transfer_t *tr;
    SPI_INFO("control cmd  is %d \n");
    switch (cmd)
    {
        case SPI_WRITE_READ:
            tr = (hal_spi_master_transfer_t *)args;
            SPI_INFO("spi[%d] tlen is %d, rlen is %d, dummy_byte is %d,\
			tx_single_len is %d, tx_nbits is %d, rx_nbits is %d\n", port, tr->tx_len,
                     tr->rx_len, tr->dummy_byte, tr->tx_single_len, tr->tx_nbits,
		     tr->rx_nbits);
	    if (tr->mode == SUNXI_SPI_SYNC)
		    ret = sunxi_spi_sync(port, tr);
	    else
		    ret = sunxi_spi_async(port, tr);
            break;
        case SPI_CONFIG:
            ret = sunxi_spi_hw_config(port, (hal_spi_master_config_t *)args);
            break;
	case SPI_MODE:
	    ret = spi_set_mode(port, *(uint32_t *)args);

	case SPI_BUS_SPEED:
	    ret = spi_set_freq(port, *(uint32_t *)args);

#if 0
	case SPI_CONTROL_SS:
	    ret = sunxi_spi_set_cs(port, (bool)args);
#endif
        default:
            ret = SPI_MASTER_ERROR;
            SPI_ERR("CMD can not identify \n");
    }

    return ret;
}

const sunxi_hal_driver_spi_t sunxi_hal_spi_driver =
{
    .initialize     = sunxi_spi_init,
    .uninitialize   = sunxi_spi_uninit,
    .send           = sunxi_spi_write,
    .receive        = sunxi_spi_read,
    .control        = sunxi_spi_control,
};
