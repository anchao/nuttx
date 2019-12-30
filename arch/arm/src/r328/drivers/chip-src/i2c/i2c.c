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

#include <stdio.h>
#include <semaphore.h>
#include <interrupt.h>
#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

#include <r328_irq.h>
#include <hal_clk.h>
#include <hal_gpio.h>
#include "i2c.h"
#include <hal_i2c.h>

//#define DEBUG
#ifdef DEBUG
#define I2C_INFO(fmt, arg...) printf("%s()%d "fmt, __func__, __LINE__, ##arg)
#else
#define I2C_INFO(fmt, arg...)
#endif

#define I2C_ERR(fmt, arg...) printf(":%d "fmt, __LINE__, ##arg)

#define I2C_SEM_MAX_COUNT 0xFFFFFFFFUL

#define SUNXI_I2C_OK      0
#define SUNXI_I2C_FAIL   -1
#define SUNXI_I2C_RETRY  -2
#define SUNXI_I2C_SFAIL  -3  /* start fail */
#define SUNXI_I2C_TFAIL  -4  /* stop  fail */

#define DMA_THRESHOLD	32
#define MAX_FIFO	32
#define DMA_TIMEOUT	1000

#define pdPASS   0
#define pdTRUE  -1
#define pdFALSE -1
/* I2C transfer status */
enum {
	I2C_XFER_IDLE    = 0x1,
	I2C_XFER_START   = 0x2,
	I2C_XFER_RUNNING = 0x4,
};


typedef struct i2c_gpio {
	int gpio;
	uint8_t         enable_mux;
	uint8_t         disable_mux;
}i2c_gpio_t;

typedef struct i2c_pinctrl {
	i2c_gpio_t	*clk;
	i2c_gpio_t	*sda;

	int (*enable)(uint8_t port);
	int (*disable)(uint8_t port);
}i2c_pinctrl_t;

typedef struct sunxi_i2c {
//	unsigned int		status; /* start, running, idle */

//	spinlock_t		lock; /* syn */
//	struct completion	cmd_complete;
	uint8_t			port;
	uint32_t		freq;
	i2c_pinctrl_t		pinctrl;

//	xSemaphoreHandle	sem;
	sem_t			sem;
	uint32_t		timeout;
	int			status;
	uint8_t			result;

	i2c_msg_t		*msgs;
	uint32_t		msgs_num;
	uint32_t		msgs_idx;
	uint32_t		msgs_ptr;

	uint32_t		base_addr;
	uint32_t		irqnum;

	uint8_t			twi_drv_used;
	uint8_t			pkt_interval;
	struct i2c_master_s	*dev_s;
//	struct sunxi_i2c_dma	*dma_tx;
//	struct sunxi_i2c_dma	*dma_rx;
//	struct sunxi_i2c_dma	*dma_using;
} sunxi_i2c_t;

static const uint32_t g_i2c_frequency[HAL_I2C_FREQUENCY_MAX] = {100000, 200000, 400000};
//static uint32_t g_i2c_regbase[SUNXI_I2C_NUM];
//static uint32_t g_i2c_irqnum[SUNXI_I2C_NUM];
static sunxi_i2c_t g_sunxi_i2c[SUNXI_I2C_NUM];

/* set twi clock
 *
 * clk_n: clock divider factor n
 * clk_m: clock divider factor m
 */
static void twi_clk_write_reg(struct sunxi_i2c *i2c, unsigned int reg_clk,
		unsigned int clk_m, unsigned int clk_n,
		unsigned int mask_clk_m, unsigned int mask_clk_n)
{
	const uint32_t base_addr = i2c->base_addr;
	unsigned int reg_val = readl(base_addr + reg_clk);

	I2C_INFO("[i2c%d] reg_clk = 0x%x, clk_m = %u, clk_n = %u,"
			"mask_clk_m = %x, mask_clk_n = %x\n", i2c->port,
			reg_clk, clk_m, clk_n, mask_clk_m, mask_clk_n);
	if (reg_clk == TWI_DRIVER_BUSC) {
		reg_val &= ~(mask_clk_m | mask_clk_n);
		reg_val |= ((clk_m | (clk_n << 4)) << 8);
		writel(reg_val, base_addr + reg_clk);
		I2C_INFO("[i2c%d] reg: 0x%x value: 0x%x\n",
				i2c->port, reg_clk,
				readl(base_addr + reg_clk));
	} else {
		reg_val &= ~(mask_clk_m | mask_clk_n);
		reg_val |= ((clk_m  << 3) | clk_n);
		writel(reg_val, base_addr + reg_clk);
		I2C_INFO("[i2c%d] reg: 0x%x value: 0x%x\n",
				i2c->port, reg_clk,
				readl(base_addr + reg_clk));
	}
}

/*
* Fin is APB CLOCK INPUT;
* Fsample = F0 = Fin/2^CLK_N;
* F1 = F0/(CLK_M+1);
* Foscl = F1/10 = Fin/(2^CLK_N * (CLK_M+1)*10);
* Foscl is clock SCL;100KHz or 400KHz
*
* clk_in: apb clk clock
* sclk_req: freqence to set in HZ
*/
static int twi_set_clock(struct sunxi_i2c *i2c, unsigned int reg_clk,
		unsigned int clk_in, unsigned int sclk_req,
		unsigned int mask_clk_m, unsigned int mask_clk_n)
{
	unsigned int clk_m = 0;
	unsigned int clk_n = 0;
	unsigned int _2_pow_clk_n = 1;
	unsigned int src_clk      = clk_in/10;
	unsigned int divider      = src_clk/sclk_req;  /* 400khz or 100khz */
	unsigned int sclk_real    = 0;      /* the real clock frequency */

	if (divider == 0) {
		clk_m = 1;
		goto set_clk;
	}

	/*
	 * search clk_n and clk_m,from large to small value so
	 * that can quickly find suitable m & n.
	 */
	while (clk_n < 8) { /* 3bits max value is 8 */
		/* (m+1)*2^n = divider -->m = divider/2^n -1 */
		clk_m = (divider/_2_pow_clk_n) - 1;
		/* clk_m = (divider >> (_2_pow_clk_n>>1))-1 */
		while (clk_m < 16) { /* 4bits max value is 16 */
			/* src_clk/((m+1)*2^n) */
			sclk_real = src_clk/(clk_m + 1)/_2_pow_clk_n;
			if (sclk_real <= sclk_req)
				goto set_clk;
			 else
				clk_m++;
		}
		clk_n++;
		_2_pow_clk_n *= 2; /* mutilple by 2 */
	}

set_clk:
	twi_clk_write_reg(i2c, reg_clk, clk_m, clk_n, mask_clk_m, mask_clk_n);
	return 0;
}




/*************************************** TWI ENGINE XFER REG CONTROL begin****************************/

/* clear the interrupt flag */
static inline void twi_clear_irq_flag(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);
	/* start and stop bit should be 0 */
	reg_val |= TWI_CTL_INTFLG;
	reg_val &= ~(TWI_CTL_STA | TWI_CTL_STP);
	writel(reg_val, base_addr + TWI_CTL_REG);
	/* read two more times to make sure that */
	/* interrupt flag does really be cleared */
	{
		unsigned int temp;

		temp = readl(base_addr + TWI_CTL_REG);
		temp |= readl(base_addr + TWI_CTL_REG);
	}
}

/* get data first, then clear flag */
static inline void
twi_get_byte(const uint32_t base_addr, unsigned char  *buffer)
{
	*buffer = (unsigned char)(TWI_DATA_MASK &
				readl(base_addr + TWI_DATA_REG));
	twi_clear_irq_flag(base_addr);
}

/* only get data, we will clear the flag when stop */
static inline void
twi_get_last_byte(const uint32_t base_addr, unsigned char  *buffer)
{
	*buffer = (unsigned char)(TWI_DATA_MASK &
			readl(base_addr + TWI_DATA_REG));
}

/* write data and clear irq flag to trigger send flow */
static inline void
twi_put_byte(const uint32_t base_addr, const unsigned char *buffer)
{
	writel((unsigned int)*buffer, base_addr + TWI_DATA_REG);
	twi_clear_irq_flag(base_addr);
}

static inline void twi_enable_irq(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	/*
	 * 1 when enable irq for next operation, set intflag to 0 to prevent
	 * to clear it by a mistake (intflag bit is write-1-to-clear bit)
	 * 2 Similarly, mask START bit and STOP bit to prevent to set it
	 * twice by a mistake (START bit and STOP bit are self-clear-to-0 bits)
	 */
	reg_val |= TWI_CTL_INTEN;
	reg_val &= ~(TWI_CTL_STA | TWI_CTL_STP | TWI_CTL_INTFLG);
	writel(reg_val, base_addr + TWI_CTL_REG);
}

static inline void twi_disable_irq(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val &= ~TWI_CTL_INTEN;
	reg_val &= ~(TWI_CTL_STA | TWI_CTL_STP | TWI_CTL_INTFLG);
	writel(reg_val, base_addr + TWI_CTL_REG);
}

static inline void
twi_disable(const uint32_t base_addr, unsigned int reg, unsigned int mask)
{
	unsigned int reg_val = readl(base_addr + reg);

	reg_val &= ~mask;
	writel(reg_val, base_addr + reg);
	I2C_INFO("offset: 0x%x value: 0x%x\n", reg,
			readl(base_addr + reg));
}

static inline void
twi_enable(const uint32_t base_addr, unsigned int reg, unsigned int mask)
{
	unsigned int reg_val = readl(base_addr + reg);

	reg_val |= mask;
	writel(reg_val, base_addr + reg);
	I2C_INFO("offset: 0x%x value: 0x%x\n", reg,
			readl(base_addr + reg));
}


/* trigger start signal, the start bit will be cleared automatically */
static inline void twi_set_start(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val |= TWI_CTL_STA;
	reg_val &= ~TWI_CTL_INTFLG;
	writel(reg_val, base_addr + TWI_CTL_REG);
}

/* get start bit status, poll if start signal is sent */
static inline unsigned int twi_get_start(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val >>= 5;
	return reg_val & 1;
}

/* trigger stop signal, the stop bit will be cleared automatically */
static inline void twi_set_stop(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val |= TWI_CTL_STP;
	reg_val &= ~TWI_CTL_INTFLG;
	writel(reg_val, base_addr + TWI_CTL_REG);
}

/* get stop bit status, poll if stop signal is sent */
static inline unsigned int twi_get_stop(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val >>= 4;
	return reg_val & 1;
}

static inline void twi_disable_ack(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val &= ~TWI_CTL_ACK;
	reg_val &= ~TWI_CTL_INTFLG;
	writel(reg_val, base_addr + TWI_CTL_REG);
}

/* when sending ack or nack, it will send ack automatically */
static inline void twi_enable_ack(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	reg_val |= TWI_CTL_ACK;
	reg_val &= ~TWI_CTL_INTFLG;
	writel(reg_val, base_addr + TWI_CTL_REG);
}

/* get the interrupt flag */
static inline unsigned int twi_query_irq_flag(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_CTL_REG);

	return (reg_val & TWI_CTL_INTFLG);/* 0x 0000_1000 */
}

/* get interrupt status */
static inline unsigned int twi_query_irq_status(const uint32_t base_addr)
{
	unsigned int reg_val = readl(base_addr + TWI_STAT_REG);

	return (reg_val & TWI_STAT_MASK);
}

/* soft reset twi */
static inline void
twi_soft_reset(const uint32_t base_addr, unsigned int reg, unsigned int mask)
{
	unsigned int reg_val = readl(base_addr + reg);

	reg_val |= mask;
	writel(reg_val, base_addr + reg);
}

/* Enhanced Feature Register */
static inline void twi_set_efr(const uint32_t base_addr, unsigned int efr)
{
	unsigned int reg_val = readl(base_addr + TWI_EFR_REG);

	reg_val &= ~TWI_EFR_MASK;
	efr     &= TWI_EFR_MASK;
	reg_val |= efr;
	writel(reg_val, base_addr + TWI_EFR_REG);
}



/* function  */
static int twi_start(const uint32_t base_addr, int port)
{
	unsigned int timeout = 0xff;

	I2C_INFO("twi start\n");
	twi_set_start(base_addr);
	while ((twi_get_start(base_addr) == 1) && (--timeout))
		;
	if (timeout == 0) {
		I2C_ERR("[i2c%d] START can't sendout!\n", port);
		return SUNXI_I2C_FAIL;
	}

	return SUNXI_I2C_OK;
}

static int twi_restart(const uint32_t base_addr, int port)
{
	unsigned int timeout = 0xff;

	I2C_INFO("twi restart\n");
	twi_set_start(base_addr);
	twi_clear_irq_flag(base_addr);
	while ((twi_get_start(base_addr) == 1) && (--timeout))
		;
	if (timeout == 0) {
		I2C_ERR("[i2c%d] Restart can't sendout!\n", port);
		return SUNXI_I2C_FAIL;
	}

	return SUNXI_I2C_OK;
}

static int twi_stop(const uint32_t base_addr, int port)
{
	unsigned int timeout = 0xff;

	twi_set_stop(base_addr);
	//unsigned int reg_val = readl(base_addr + TWI_CTL_REG);
	twi_clear_irq_flag(base_addr);

	twi_get_stop(base_addr);/* it must delay 1 nop to check stop bit */
	while ((twi_get_stop(base_addr) == 1) && (--timeout))
		;
	if (timeout == 0) {
		I2C_ERR("[i2c%d] STOP can't sendout!\n", port);
		return SUNXI_I2C_TFAIL;
	}

	//twi_clear_irq_flag(base_addr);
	timeout = 0xff;
	while ((readl(base_addr + TWI_STAT_REG) != TWI_STAT_IDLE)
		&& (--timeout))
		;
	if (timeout == 0) {
		I2C_ERR("[i2c%d] i2c state(0x%0x) isn't idle(0xf8)\n",
				port, readl(base_addr + TWI_STAT_REG));
		return SUNXI_I2C_TFAIL;
	}

	timeout = 0xff;
	while ((readl(base_addr + TWI_LCR_REG) != TWI_LCR_IDLE_STATUS
		&& readl(base_addr + TWI_LCR_REG) != TWI_LCR_NORM_STATUS)
		&& (--timeout))
		;

	if (timeout == 0) {
		I2C_ERR("[i2c%d] i2c lcr(0x%0x) isn't idle(0x3a)\n",
				port, readl(base_addr + TWI_LCR_REG));
		return SUNXI_I2C_TFAIL;
	}

	//twi_clear_irq_flag(base_addr);
	I2C_INFO("twi stop end\n");

	return SUNXI_I2C_OK;
}

/* get SDA state */
static unsigned int twi_get_sda(const uint32_t base_addr)
{
	unsigned int status = 0;

	status = TWI_LCR_SDA_STATE_MASK & readl(base_addr + TWI_LCR_REG);
	status >>= 4;
	return (status&0x1);
}

/* set SCL level(high/low), only when SCL enable */
static void twi_set_scl(const uint32_t base_addr, unsigned int hi_lo)
{
	unsigned int reg_val = readl(base_addr + TWI_LCR_REG);

	reg_val &= ~TWI_LCR_SCL_CTL;
	hi_lo   &= 0x01;
	reg_val |= (hi_lo<<3);
	writel(reg_val, base_addr + TWI_LCR_REG);
}

/* enable SDA or SCL */
static void twi_enable_lcr(const uint32_t base_addr, unsigned int sda_scl)
{
	unsigned int reg_val = readl(base_addr + TWI_LCR_REG);

	sda_scl &= 0x01;
	if (sda_scl)
		reg_val |= TWI_LCR_SCL_EN;/* enable scl line control */
	else
		reg_val |= TWI_LCR_SDA_EN;/* enable sda line control */

	writel(reg_val, base_addr + TWI_LCR_REG);
}

/* disable SDA or SCL */
static void twi_disable_lcr(const uint32_t base_addr, unsigned int sda_scl)
{
	unsigned int reg_val = readl(base_addr + TWI_LCR_REG);

	sda_scl &= 0x01;
	if (sda_scl)
		reg_val &= ~TWI_LCR_SCL_EN;/* disable scl line control */
	else
		reg_val &= ~TWI_LCR_SDA_EN;/* disable sda line control */

	writel(reg_val, base_addr + TWI_LCR_REG);
}

/* send 9 clock to release sda */
static int twi_send_clk_9pulse(const uint32_t base_addr, int port)
{
	int twi_scl = 1;
	int low = 0;
	int high = 1;
	int cycle = 0;
	unsigned char status;

	/* enable scl control */
	twi_enable_lcr(base_addr, twi_scl);

	while (cycle < 9) {
		if (twi_get_sda(base_addr)
		    && twi_get_sda(base_addr)
		    && twi_get_sda(base_addr)) {
			break;
		}
		/* twi_scl -> low */
		twi_set_scl(base_addr, low);
		up_udelay(1000);

		/* twi_scl -> high */
		twi_set_scl(base_addr, high);
		up_udelay(1000);
		cycle++;
	}

	if (twi_get_sda(base_addr)) {
		twi_disable_lcr(base_addr, twi_scl);
		status =  SUNXI_I2C_OK;
	} else {
		I2C_ERR("[i2c%d] SDA is still Stuck Low, failed.\n", port);
		twi_disable_lcr(base_addr, twi_scl);
		status =  SUNXI_I2C_FAIL;
	}

	return status;
}

/*************************************** TWI ENGINE XFER REG CONTROL end****************************/

#if 0
/* set twi clock
 *
 * clk_n: clock divider factor n
 * clk_m: clock divider factor m
 */
static void twi_clk_write_reg(const uint32_t base_addr, uint32_t sclk_freq,
		uint8_t clk_m, uint8_t clk_n)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_BUSC);
#if defined(CONFIG_ARCH_SUN50IW10)
	uint32_t duty;
#endif
//	I2C_INFO("reg_clk = 0x%x, clk_m = %u, clk_n = %u,"
//			"mask_clk_m = %x, mask_clk_n = %x\n",
//			reg_clk, clk_m, clk_n, mask_clk_m, mask_clk_n);

		reg_val &= ~(TWI_DRV_CLK_M | TWI_DRV_CLK_N);
		reg_val |= ((clk_m | (clk_n << 4)) << 8);
#if defined(CONFIG_ARCH_SUN50IW10)
		duty = TWI_DRV_CLK_DUTY;
		if (sclk_freq > STANDDARD_FREQ)
			reg_val |= duty;
		else
			reg_val &= ~duty;
#endif
		writel(reg_val, base_addr + TWI_DRIVER_BUSC);
}

/*
* Fin is APB CLOCK INPUT;
* Fsample = F0 = Fin/2^CLK_N;
* F1 = F0/(CLK_M+1);
* Foscl = F1/10 = Fin/(2^CLK_N * (CLK_M+1)*10);
* Foscl is clock SCL;100KHz or 400KHz
*
* clk_in: apb clk clock
* sclk_freq: freqence to set in HZ
*/
static int32_t twi_set_clock(hal_i2c_port_t i2c_port,
		uint32_t clk_in, uint32_t sclk_freq)
{
	const uint32_t base_addr = g_i2c_regbase[i2c->port];

	uint8_t clk_m = 0, clk_n = 0, _2_pow_clk_n = 1;
	uint32_t src_clk      = clk_in/10;
	uint32_t divider      = src_clk/sclk_freq;  /* 400khz or 100khz */
	uint32_t sclk_real    = 0;      /* the real clock frequency */

	if (divider == 0) {
		clk_m = 1;
		goto set_clk;
	}

	/*
	 * search clk_n and clk_m,from large to small value so
	 * that can quickly find suitable m & n.
	 */
	while (clk_n < 8) { /* 3bits max value is 8 */
		/* (m+1)*2^n = divider -->m = divider/2^n -1 */
		clk_m = (divider/_2_pow_clk_n) - 1;
		/* clk_m = (divider >> (_2_pow_clk_n>>1))-1 */
		while (clk_m < 16) { /* 4bits max value is 16 */
			/* src_clk/((m+1)*2^n) */
			sclk_real = src_clk/(clk_m + 1)/_2_pow_clk_n;
			if (sclk_real <= sclk_freq)
				goto set_clk;
			 else
				clk_m++;
		}
		clk_n++;
		_2_pow_clk_n *= 2; /* mutilple by 2 */
	}

set_clk:
	twi_clk_write_reg(base_addr, sclk_freq, clk_m, clk_n);
	return 0;
}
#endif


/*************************************** TWI DRV XFER REG CONTROL begin****************************/

static uint32_t i2c_query_irq_status(const uint32_t base_addr)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_INTC);

	return (reg_val & TWI_DRV_STAT_MASK);
}

static void i2c_clear_irq_flag(uint32_t pending_bit, const uint32_t base_addr)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_INTC);

	pending_bit &= TWI_DRV_STAT_MASK;
	reg_val |= pending_bit;
	writel(reg_val, base_addr + TWI_DRIVER_INTC);
}

static void i2c_clear_pending(const uint32_t base_addr)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_INTC);

	reg_val |= TWI_DRV_STAT_MASK;
	writel(reg_val, base_addr + TWI_DRIVER_INTC);
}

/* start i2c transfer */
static void i2c_start_xfer(const uint32_t base_addr)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_CTRL);

	reg_val |= START_TRAN;
	writel(reg_val, base_addr + TWI_DRIVER_CTRL);
}

/*
 * send DMA RX Req when the data byte number in RECV_FIFO reaches RX_TRIG
 * or Read Packet Tansmission completed with RECV_FIFO not empty
 */
static void i2c_set_rx_trig_level(uint32_t val, const uint32_t base_addr)
{
	uint32_t mask = TRIG_MASK;
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_DMAC);

	val = (val & mask) << 16;
	reg_val &= ~(mask << 16);
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_DMAC);
}

/* bytes be send as slave device reg address */
static void i2c_set_packet_addr_byte(uint32_t val, const uint32_t base_addr)
{
	uint32_t mask = ADDR_BYTE;
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_FMT);

	reg_val &= ~mask;
	val = (val << 16) & mask;
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_FMT);
}

/* bytes be send/received as data */
static void i2c_set_packet_data_byte(uint32_t val, const uint32_t base_addr)
{
	uint32_t mask = DATA_BYTE;
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_FMT);

	reg_val &= ~mask;
	val &= mask;
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_FMT);
}

#if 0
/* interval between each packet in 32*Fscl cycles */
static void i2c_set_packet_interval(uint32_t val, const uint32_t base_addr)
{
	uint32_t mask = INTERVAL_MASK;
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_CFG);

	reg_val &= ~mask;
	val <<= 16;
	val &= mask;
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_CFG);
}
#endif

/* FIFO data be transmitted as PACKET_CNT packets in current format */
static void i2c_set_packet_cnt(uint32_t val, const uint32_t base_addr)
{
	uint32_t mask = PACKET_MASK;
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_CFG);

	reg_val &= ~mask;
	val &= mask;
	reg_val |= val;
	writel(reg_val, base_addr + TWI_DRIVER_CFG);
}

/* do not send slave_id +W */
static void i2c_enable_read_tran_mode(const uint32_t base_addr)
{
	uint32_t mask = READ_TRAN;
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_CTRL);

	reg_val |= mask;
	writel(reg_val, base_addr + TWI_DRIVER_CTRL);
}

/* send slave_id + W */
static void i2c_disable_read_tran_mode(const uint32_t base_addr)
{
	uint32_t mask = READ_TRAN;
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_CTRL);

	reg_val &= ~mask;
	writel(reg_val, base_addr + TWI_DRIVER_CTRL);
}

static void i2c_enable_tran_irq(uint32_t bitmap, const uint32_t base_addr)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_INTC);

	reg_val |= bitmap;
	reg_val &= ~TWI_DRV_STAT_MASK;
	writel(reg_val, base_addr + TWI_DRIVER_INTC);
}

static void i2c_disable_tran_irq(uint32_t bitmap, const uint32_t base_addr)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_INTC);

	reg_val &= ~bitmap;
	reg_val &= ~TWI_DRV_STAT_MASK;
	writel(reg_val, base_addr + TWI_DRIVER_INTC);
}

#if 0
static void i2c_enable_dma_irq(uint32_t bitmap, const uint32_t base_addr)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_DMAC);

	bitmap &= I2C_DRQEN_MASK;
	reg_val |= bitmap;
	writel(reg_val, base_addr + TWI_DRIVER_DMAC);
}
#endif

static void i2c_disable_dma_irq(uint32_t bitmap, const uint32_t base_addr)
{
	uint32_t reg_val = readl(base_addr + TWI_DRIVER_DMAC);

	bitmap &= I2C_DRQEN_MASK;
	reg_val &= ~bitmap;
	writel(reg_val, base_addr + TWI_DRIVER_DMAC);
}

static void i2c_slave_addr(const uint32_t base_addr, i2c_msg_t *msgs)
{
	uint32_t val = 0, cmd = 0;

	/* read, default value is write */
	if (msgs->flags & I2C_M_RD)
		cmd = SLV_RD_CMD;

	if (msgs->flags & I2C_M_TEN) {
		/* SLV_ID | CMD | SLV_ID_X */
		val = ((0x78 | ((msgs->addr >> 8) & 0x03)) << 9) | cmd
			| (msgs->addr & 0xff);
//		I2C_INFO("10bit addr\n");
	} else {
		val = ((msgs->addr & 0x7f) << 9) | cmd;
//		I2C_INFO("7bit addr\n");
	}

	writel(val, base_addr + TWI_DRIVER_SLV);
}

/* the number of data in SEND_FIFO */
static int32_t i2c_query_txfifo(const uint32_t base_addr)
{
	uint32_t reg_val;

	reg_val = readl(base_addr + TWI_DRIVER_FIFOC) & SEND_FIFO_CONT;

	return reg_val;
}

/* the number of data in RECV_FIFO */
static int32_t i2c_query_rxfifo(const uint32_t base_addr)
{
	uint32_t reg_val;

	reg_val = readl(base_addr + TWI_DRIVER_FIFOC) & RECV_FIFO_CONT;
	reg_val >>= 16;

	return reg_val;
}

static void i2c_clear_txfifo(const uint32_t base_addr)
{
	uint32_t reg_val;

	reg_val = readl(base_addr + TWI_DRIVER_FIFOC);
	reg_val |= SEND_FIFO_CLEAR;
	writel(reg_val, base_addr + TWI_DRIVER_FIFOC);
}

static void i2c_clear_rxfifo(const uint32_t base_addr)
{
	uint32_t reg_val;

	reg_val = readl(base_addr + TWI_DRIVER_FIFOC);
	reg_val |= RECV_FIFO_CLEAR;
	writel(reg_val, base_addr + TWI_DRIVER_FIFOC);
}

static int i2c_send_msgs(sunxi_i2c_t *i2c, i2c_msg_t *msgs)
{
	uint16_t i;
	uint8_t time = 0xff;

	I2C_INFO("i2c[%d] msgs->len = %d\n", i2c->port, msgs->len);

	for (i = 0; i < msgs->len; i++) {
		while ((i2c_query_txfifo(i2c->base_addr) >= MAX_FIFO) && time--)
			;
		if (time) {
			writeb(msgs->buf[i], i2c->base_addr + TWI_DRIVER_SENDF);
		} else {
			I2C_ERR("[i2c%d] SEND FIFO overflow. timeout\n", i2c->port);
			return SUNXI_I2C_FAIL;
		}
	}

	return SUNXI_I2C_OK;
}

static uint32_t i2c_recv_msgs(sunxi_i2c_t *i2c, i2c_msg_t *msgs)
{
	uint16_t i;
	uint8_t time = 0xff;

	I2C_INFO("i2c[%d] msgs->len = %d\n", i2c->port, msgs->len);

	for (i = 0; i < msgs->len; i++) {
		while (!i2c_query_rxfifo(i2c->base_addr) && time--)
			;
		if (time) {
			msgs->buf[i] = readb(i2c->base_addr + TWI_DRIVER_RECVF);
		} else
			return 0;
	}
	return msgs->len;
}

static int i2c_drv_write(sunxi_i2c_t *i2c, i2c_msg_t *msgs)
{
	I2C_INFO("i2c write\n");
	i2c->msgs = msgs;

	i2c_slave_addr(i2c->base_addr, msgs);
	if (msgs->len == 1) {
		i2c_set_packet_addr_byte(0, i2c->base_addr);
		i2c_set_packet_data_byte(msgs->len, i2c->base_addr);
	} else {
		i2c_set_packet_addr_byte(1, i2c->base_addr);
		i2c_set_packet_data_byte(msgs->len - 1, i2c->base_addr);
	}
	i2c_set_packet_cnt(1, i2c->base_addr);

	i2c_clear_pending(i2c->base_addr);
	i2c_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, i2c->base_addr);
	i2c_start_xfer(i2c->base_addr);

	return i2c_send_msgs(i2c, msgs);
}
/*
static int32_t i2c_dma_write(hal_i2c_port_t i2c_port, i2c_msg_t *msgs)
{
	int32_t ret = 0;
	const uint32_t base_addr = g_i2c_regbase[i2c->port];

	i2c->msgs = msgs;

	i2c_slave_addr(i2c, msgs);
	i2c_set_packet_addr_byte(1, base_addr);
	i2c_set_packet_data_byte(msgs->len - 1, base_addr);
	i2c_set_packet_cnt(1, base_addr);

	i2c_clear_pending(base_addr);
	i2c_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, base_addr);
	i2c_enable_dma_irq(DMA_TX, base_addr);
	i2c_start_xfer(base_addr);

	i2c->dma_using = i2c->dma_tx;
	i2c->dma_using->dma_transfer_dir = DMA_MEM_TO_DEV;
	i2c->dma_using->dma_data_dir = DMA_TO_DEVICE;
	i2c->dma_using->dma_len = msgs->len;

	ret = i2c_dma_xfer(i2c, msgs->buf);

	return ret;
}
*/

/*************************************** TWI DRV XFER REG CONTROL end****************************/


static int i2c_drv_read(sunxi_i2c_t *i2c, i2c_msg_t *msgs, int32_t num)
{
	I2C_INFO("i2c read\n");
	i2c_msg_t *wmsgs = NULL, *rmsgs = NULL;

	if (num == 1) {
		wmsgs = NULL;
		rmsgs = msgs;
	} else if (num == 2) {
		wmsgs = msgs;
		rmsgs = msgs + 1;
	} else {
		I2C_ERR("msg num err\n");
		return -1;
	}
	I2C_INFO("rmsgs->len : %d\n", rmsgs->len);

	i2c->msgs = rmsgs;

	i2c_slave_addr(i2c->base_addr, rmsgs);
	i2c_set_packet_cnt(1, i2c->base_addr);
	i2c_set_packet_data_byte(rmsgs->len, i2c->base_addr);
	if (rmsgs->len > MAX_FIFO)
		i2c_set_rx_trig_level(MAX_FIFO, i2c->base_addr);
	else
		i2c_set_rx_trig_level(rmsgs->len, i2c->base_addr);
	if (i2c_query_rxfifo(i2c->base_addr))
		i2c_clear_rxfifo(i2c->base_addr);

	i2c_clear_pending(i2c->base_addr);
	i2c_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, i2c->base_addr);
	i2c_start_xfer(i2c->base_addr);

	if (wmsgs)
		return i2c_send_msgs(i2c, wmsgs);

	return 0;
}
/*
static int32_t
i2c_dma_read(hal_i2c_port_t i2c_port, i2c_msg_t *msgs, int32_t num)
{
	int32_t ret = 0;
	i2c_msg_t *wmsgs, *rmsgs;
	const uint32_t base_addr = g_i2c_regbase[i2c->port];

	if (num == 1) {
		wmsgs = NULL;
		rmsgs = msgs;
	} else if (num == 2) {
		wmsgs = msgs;
		rmsgs = msgs + 1;
	}

	i2c->msgs = rmsgs;

	i2c_slave_addr(i2c, rmsgs);
	i2c_set_packet_data_byte(rmsgs->len, base_addr);
	i2c_set_packet_cnt(1, base_addr);
	i2c_set_rx_trig_level(MAX_FIFO/2, base_addr);
	if (i2c_query_rxfifo(base_addr))
		i2c_clear_rxfifo(base_addr);

	i2c_clear_pending(base_addr);
	i2c_enable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT, base_addr);
	i2c_enable_dma_irq(DMA_RX, base_addr);
	i2c_start_xfer(base_addr);
	if (wmsgs)
		i2c_send_msgs(i2c, wmsgs);

	i2c->dma_using = i2c->dma_rx;
	i2c->dma_using->dma_transfer_dir = DMA_DEV_TO_MEM;
	i2c->dma_using->dma_data_dir = DMA_FROM_DEVICE;
	i2c->dma_using->dma_len = rmsgs->len;

	ret = i2c_dma_xfer(i2c, rmsgs->buf);

	return ret;
}
*/

static int sunxi_i2c_drv_complete(sunxi_i2c_t *i2c)
{
//	BaseType_t sem_ret;
	int sem_ret = 0;

//	sem_ret = xSemaphoreTake(i2c->sem, (i2c->timeout *1000)/portTICK_RATE_MS);
	sem_ret = sem_wait(&i2c->sem);
	if (sem_ret != pdPASS) {
		I2C_ERR("[i2c%d] twi driver xfer timeout (dev addr:0x%x)\n", i2c->port, i2c->msgs->addr);
		//dump_reg(i2c, 0x200, 0x20);
		i2c_disable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT
				| RX_REQ_INT | TX_REQ_INT, i2c->base_addr);
		i2c_disable_dma_irq(DMA_TX | DMA_RX, i2c->base_addr);
		return SUNXI_I2C_FAIL;
	} else if (i2c->result == RESULT_ERR) {
		I2C_ERR("[i2c%d]twi drv xfer incomplete xfer"
				"(status: 0x%x, dev addr: 0x%x)\n",
				i2c->port, i2c->msgs_idx, i2c->msgs->addr);
		i2c_disable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT
				| RX_REQ_INT | TX_REQ_INT, i2c->base_addr);
		i2c_disable_dma_irq(DMA_TX | DMA_RX, i2c->base_addr);
		return SUNXI_I2C_FAIL;
	}

	I2C_INFO("twi drv xfer complete\n");

//	spin_lock_irqsave(&i2c->lock, flags);
	i2c->result = RESULT_COMPLETE;
//	spin_unlock_irqrestore(&i2c->lock, flags);

	return SUNXI_I2C_OK;
}

static int sunxi_i2c_engine_complete(struct sunxi_i2c *i2c, int code)
{
	int ret = SUNXI_I2C_OK;
//	BaseType_t sem_ret, taskwoken = pdFALSE;
	int sem_ret = pdFALSE;

	i2c->msgs     = NULL;
	i2c->msgs_num = 0;
	i2c->msgs_ptr = 0;
	i2c->status  = I2C_XFER_IDLE;

	/* i2c->msgs_idx  store the information */
	if (code == SUNXI_I2C_FAIL) {
		I2C_ERR("[i2c%d] Maybe Logic Error, debug it!\n", i2c->port);
		i2c->msgs_idx = code;
		ret = SUNXI_I2C_FAIL;
		i2c->result = RESULT_ERR;
	} else if (code != SUNXI_I2C_OK) {
		i2c->msgs_idx = code;
		ret = SUNXI_I2C_FAIL;
		i2c->result = RESULT_ERR;
	}

//	sem_ret = xSemaphoreGiveFromISR(i2c->sem, &taskwoken);
	sem_ret = sem_post(&i2c->sem);
	if (sem_ret != pdPASS) {
		ret = SUNXI_I2C_FAIL;
		I2C_ERR(" evdev give semaphore err\n");
	}
	/*if (pdTRUE == taskwoken) {
		//switch isr contex
	}*/
	I2C_INFO("complete\n");

	return ret;
}

/*
 ****************************************************************************
 *
 *  FunctionName:           sunxi_i2c_addr_byte
 *
 *  Description:
 *         7bits addr: 7-1bits addr+0 bit r/w
 *         10bits addr: 1111_11xx_xxxx_xxxx-->1111_0xx_rw,xxxx_xxxx
 *         send the 7 bits addr,or the first part of 10 bits addr
 *  Parameters:
 *
 *
 *  Return value:
 *           ��
 *  Notes:
 *
 ****************************************************************************
 */
static void sunxi_i2c_addr_byte(sunxi_i2c_t *i2c)
{
	unsigned char addr = 0;
	unsigned char tmp  = 0;

	if (i2c->msgs[i2c->msgs_idx].flags & I2C_M_TEN) {
		/* 0111_10xx,ten bits address--9:8bits */
		tmp = 0x78 | (((i2c->msgs[i2c->msgs_idx].addr)>>8) & 0x03);
		addr = tmp << 1;	/*1111_0xx0*/
		/* how about the second part of ten bits addr? */
		/* Answer: deal at twi_core_process() */
	} else
		/* 7-1bits addr, xxxx_xxx0 */
		addr = (i2c->msgs[i2c->msgs_idx].addr & 0x7f) << 1;

	/* read, default value is write */
	if (i2c->msgs[i2c->msgs_idx].flags & I2C_M_RD)
		addr |= 1;

	if (i2c->msgs[i2c->msgs_idx].flags & I2C_M_TEN) {
		I2C_INFO("[i2c%d] first part of 10bits = 0x%x\n",
				i2c->port, addr);
	} else
		I2C_INFO("[i2c%d] 7bits+r/w = 0x%x\n",
				i2c->port, addr);

	/* send 7bits+r/w or the first part of 10bits */
	twi_put_byte(i2c->base_addr, &addr);
}

static int sunxi_i2c_core_process(struct sunxi_i2c *i2c)
{
	const uint32_t base_addr = i2c->base_addr;
	int  ret        = SUNXI_I2C_OK;
	int  err_code   = 0;
	unsigned char  state = 0;
	unsigned char  tmp   = 0;

	state = twi_query_irq_status(base_addr);

	//spin_lock_irqsave(&i2c->lock, flags);
	I2C_INFO("[i2c%d][slave address = (0x%x), state = (0x%x)]\n",
			i2c->port, i2c->msgs->addr, state);
	if (i2c->msgs == NULL) {
		I2C_ERR("[i2c%d] i2c message is NULL, err_code = 0xfe\n",
				i2c->port);
		err_code = 0xfe;
		goto msg_null;
	}

	switch (state) {
	case 0xf8:
		/* On reset or stop the bus is idle, use only at poll method */
		err_code = 0xf8;
		goto err_out;
	case 0x08: /* A START condition has been transmitted */
	case 0x10: /* A repeated start condition has been transmitted */
		sunxi_i2c_addr_byte(i2c);/* send slave address */
		break;
	case 0xd8: /* second addr has transmitted, ACK not received!    */
	case 0x20: /* SLA+W has been transmitted; NOT ACK has been received */
		err_code = 0x20;
		goto err_out;
	case 0x18: /* SLA+W has been transmitted; ACK has been received */
		/* if any, send second part of 10 bits addr */
		if (i2c->msgs[i2c->msgs_idx].flags & I2C_M_TEN) {
			/* the remaining 8 bits of address */
			tmp = i2c->msgs[i2c->msgs_idx].addr & 0xff;
			twi_put_byte(base_addr, &tmp); /* case 0xd0: */
			break;
		}
		/* for 7 bit addr, then directly send data byte--case 0xd0:  */
	case 0xd0: /* second addr has transmitted,ACK received!     */
	case 0x28: /* Data byte in DATA REG has been transmitted; */
		   /*  ACK has been received */
		/* after send register address then START send write data  */
		if (i2c->msgs_ptr < i2c->msgs[i2c->msgs_idx].len) {
			I2C_INFO("write : %0x\n", (i2c->msgs[i2c->msgs_idx].buf[i2c->msgs_ptr]));
			twi_put_byte(base_addr,
				&(i2c->msgs[i2c->msgs_idx].buf[i2c->msgs_ptr]));
			i2c->msgs_ptr++;
			break;
		}

		i2c->msgs_idx++; /* the other msg */
		i2c->msgs_ptr = 0;
		if (i2c->msgs_idx == i2c->msgs_num) {
			err_code = SUNXI_I2C_OK;/* Success,wakeup */
			goto ok_out;
		} else if (i2c->msgs_idx < i2c->msgs_num) {
			/* for restart pattern, read spec, two msgs */
			ret = twi_restart(base_addr, i2c->port);
			if (ret == SUNXI_I2C_FAIL) {
				I2C_ERR("[i2c%d] twi restart fail\n", i2c->port);
				err_code = SUNXI_I2C_FAIL;
				goto err_out;/* START can't sendout */
			}
		} else {
			err_code = SUNXI_I2C_FAIL;
			goto err_out;
		}
		break;
	case 0x30: /* Data byte in I2CDAT has been transmitted; */
		   /* NOT ACK has been received */
		err_code = 0x30;	/*err,wakeup the thread*/
		goto err_out;
	case 0x38: /* Arbitration lost during SLA+W, SLA+R or data bytes */
		err_code = 0x38;	/*err,wakeup the thread*/
		goto err_out;
	case 0x40: /* SLA+R has been transmitted; ACK has been received */
		/* with Restart,needn't to send second part of 10 bits addr */
		/* refer-"I2C-SPEC v2.1" */
		/* enable A_ACK need it(receive data len) more than 1. */
		if (i2c->msgs[i2c->msgs_idx].len > 1) {
			/* send register addr complete,then enable the A_ACK */
			/* and get ready for receiving data */
			twi_enable_ack(base_addr);
			twi_clear_irq_flag(base_addr);/* jump to case 0x50 */
		} else if (i2c->msgs[i2c->msgs_idx].len == 1) {
			twi_clear_irq_flag(base_addr);/* jump to case 0x58 */
		}
		break;
	case 0x48: /* SLA+R has been transmitted; NOT ACK has been received */
		err_code = 0x48;	/*err,wakeup the thread*/
		goto err_out;
	case 0x50: /* Data bytes has been received; ACK has been transmitted */
		/* receive first data byte */
		if (i2c->msgs_ptr < i2c->msgs[i2c->msgs_idx].len) {
			/* more than 2 bytes, the last byte need not to send ACK */
			if ((i2c->msgs_ptr + 2) == i2c->msgs[i2c->msgs_idx].len)
				/* last byte no ACK */
				twi_disable_ack(base_addr);

			/* get data then clear flag,then next data coming */
			twi_get_byte(base_addr,
				&i2c->msgs[i2c->msgs_idx].buf[i2c->msgs_ptr]);
			I2C_INFO("read : %0x\n", (i2c->msgs[i2c->msgs_idx].buf[i2c->msgs_ptr]));
			i2c->msgs_ptr++;
			break;
		}
		/* err process, the last byte should be @case 0x58 */
		err_code = SUNXI_I2C_FAIL;/* err, wakeup */
		goto err_out;
	case 0x58:
		/* Data byte has been received; NOT ACK has been transmitted */
		/* received the last byte  */
		if (i2c->msgs_ptr == i2c->msgs[i2c->msgs_idx].len - 1) {
			twi_get_last_byte(base_addr,
				&i2c->msgs[i2c->msgs_idx].buf[i2c->msgs_ptr]);
			I2C_INFO("read : %0x\n", (i2c->msgs[i2c->msgs_idx].buf[i2c->msgs_ptr]));
			i2c->msgs_idx++;
			i2c->msgs_ptr = 0;
			if (i2c->msgs_idx == i2c->msgs_num) {
				/* succeed,wakeup the thread */
				err_code = SUNXI_I2C_OK;
				goto ok_out;
			} else if (i2c->msgs_idx < i2c->msgs_num) {
				/* repeat start */
				ret = twi_restart(base_addr, i2c->port);
				if (ret == SUNXI_I2C_FAIL) {/* START fail */
					I2C_ERR("[i2c%d] twi restart fail\n", i2c->port);
					err_code = SUNXI_I2C_FAIL;
					goto err_out;
				}
				break;
			}
		} else {
			err_code = 0x58;
			goto err_out;
		}
	case 0x00: /* Bus error during master or slave mode due to illegal level condition */
		err_code = 0xff;
		goto err_out;
	default:
		err_code = state;
		goto err_out;
	}
	//spin_unlock_irqrestore(&i2c->lock, flags);
	return ret;

ok_out:
err_out:
	if (twi_stop(base_addr, i2c->port) == SUNXI_I2C_FAIL)
		I2C_ERR("[i2c%d] STOP failed!\n", i2c->port);


msg_null:
	ret = sunxi_i2c_engine_complete(i2c, err_code);/* wake up */
	//spin_unlock_irqrestore(&i2c->lock, flags);
	return ret;
}

static int sunxi_i2c_drv_core_process(sunxi_i2c_t *i2c)
{
	int ret = SUNXI_I2C_OK;
	uint32_t status, code = 0;
//	BaseType_t sem_ret, taskwoken = pdFALSE;
	int sem_ret = pdFALSE;

//	spin_lock_irqsave(&i2c->lock, flags);

	status = i2c_query_irq_status(i2c->base_addr);
	i2c_clear_irq_flag(status, i2c->base_addr);

	I2C_INFO("twi drv irq status : %d\n", status);
	if (status & TRAN_COM_PD) {
		I2C_INFO("twi drv complete\n");
		i2c_disable_tran_irq(TRAN_COM_INT, i2c->base_addr);
		i2c->result = RESULT_COMPLETE;

		if ((status & RX_REQ_PD) && (i2c->msgs->len < DMA_THRESHOLD)) {
			i2c_recv_msgs(i2c, i2c->msgs);
		}
		goto ok_out;
	}

	if (status & TRAN_ERR_PD) {
		I2C_INFO("twi drv error\n");
		i2c_disable_tran_irq(TRAN_ERR_INT, i2c->base_addr);
		code = readl(i2c->base_addr + TWI_DRIVER_CTRL);
		code = (code & TWI_DRV_STA) >> 16;
		I2C_INFO("err code : %0x\n", code);
		switch (code) {
		case 0x00:
			I2C_ERR("[i2c%d] bus error\n", i2c->port);
			break;
		case 0x01:
			I2C_ERR("[i2c%d] Timeout when sending 9th SCL clk\n", i2c->port);
			break;
		case 0x20:
			I2C_ERR("[i2c%d] Address + Write bit transmitted,"
					"ACK not received\n", i2c->port);
			break;
		case 0x30:
			I2C_ERR("[i2c%d] Data byte transmitted in master mode,"
					"ACK not received\n", i2c->port);
			break;
		case 0x38:
			I2C_ERR("[i2c%d] Arbitration lost in address"
					"or data byte\n", i2c->port);
			break;
		case 0x48:
			I2C_ERR("[i2c%d] Address + Read bit transmitted,"
					"ACK not received\n", i2c->port);
			break;
		case 0x58:
			I2C_ERR("[i2c%d] Data byte received in master mode,"
					"ACK not received\n", i2c->port);
			break;
		default:
			I2C_ERR("[i2c%d] unknown error\n", i2c->port);
			break;
		}
		goto err_out;
	}

err_out:
	i2c->msgs_idx = code;
	i2c->result = RESULT_ERR;
	I2C_INFO("packet transmission failed , status : 0x%0x\n", code);

ok_out:
	//wake up
//	sem_ret = xSemaphoreGiveFromISR(i2c->sem, &taskwoken);
	sem_ret = sem_post(&i2c->sem);
	if (sem_ret != pdPASS) {
		ret = SUNXI_I2C_FAIL;
		I2C_ERR(" evdev give semaphore err\n");
	}
	/*if (pdTRUE == taskwoken) {
		//switch isr contex
	}*/

	return ret;
//	spin_unlock_irqrestore(&i2c->lock, flags);
}


int sunxi_i2c_handler(int irq, void *centext, void *dev)
{
	struct sunxi_i2c *i2c = (sunxi_i2c_t *)dev;

	if (i2c->twi_drv_used)
		sunxi_i2c_drv_core_process(i2c);
	else {
		if (!twi_query_irq_flag(i2c->base_addr)) {
			I2C_ERR("unknown interrupt!\n");
			return -1;
		}

		/* disable irq */
		twi_disable_irq(i2c->base_addr);

		/* twi core process */
		sunxi_i2c_core_process(i2c);

		/*
		 * enable irq only when twi is transferring,
		 * otherwise disable irq
		 */
		if (i2c->status != I2C_XFER_IDLE)
			twi_enable_irq(i2c->base_addr);
	}
	return 0;
}


/**
 * i2c_do_xfer - twi driver transmission control
 */
static int sunxi_i2c_drv_do_xfer(struct sunxi_i2c *i2c, struct i2c_msg *msgs, int num)
{
//	uint64_t flags = 0;
	int ret = -1;

//	spin_lock_irqsave(&i2c->lock, flags);
//	i2c->result = 0;
//	spin_unlock_irqrestore(&i2c->lock, flags);

	i2c_clear_pending(i2c->base_addr);
	i2c_disable_tran_irq(TRAN_COM_INT | TRAN_ERR_INT
			| RX_REQ_INT | TX_REQ_INT, i2c->base_addr);
	i2c_disable_dma_irq(DMA_TX | DMA_RX, i2c->base_addr);
	if (i2c_query_txfifo(i2c->base_addr))
		i2c_clear_txfifo(i2c->base_addr);

	if (num == 1) {
		if (msgs->flags & I2C_M_RD) {
			I2C_INFO("1 msgs read \n");
			/* 1 msgs read */
			i2c_enable_read_tran_mode(i2c->base_addr);
			i2c_set_packet_addr_byte(0, i2c->base_addr);
#ifdef DMA_XXX
			if (i2c->dma_rx && (msgs->len >= DMA_THRESHOLD)) {
				I2C_INFO("i2c[%d] master dma read\n", i2c->port);
				ret =  i2c_dma_read(i2c, msgs, num);
			} else {
#endif
				I2C_INFO("i2c[%d] master cpu read\n", i2c->port);
				ret = i2c_drv_read(i2c, msgs, num);
#ifdef DMA_XXX
			}
#endif
		} else {
			/* 1 msgs write */
			i2c_disable_read_tran_mode(i2c->base_addr);
#ifdef DMA_XXX
			if (i2c->dma_tx && (msgs->len >= DMA_THRESHOLD)) {
//				dprintk(DEBUG_INFO,
//						"master dma write\n",
//						i2c->port);
				ret = i2c_dma_write(i2c, msgs);
			} else {
#endif
//				dprintk(DEBUG_INFO,
//						"master cpu write\n",
//						i2c->port);
				ret = i2c_drv_write(i2c, msgs);
#ifdef DMA_XXX
			}
#endif
		}
	} else if ((num == 2) && ((msgs + 1)->flags & I2C_M_RD)) {
		/* 2 msgs read */
		I2C_INFO("2 msgs read\n");
		i2c_disable_read_tran_mode(i2c->base_addr);
		i2c_set_packet_addr_byte(msgs->len, i2c->base_addr);
#ifdef DMA_XXX
		if (i2c->dma_rx && ((msgs + 1)->len >= DMA_THRESHOLD)) {
//			dprintk(DEBUG_INFO, "master dma read\n",
//					i2c->port);
			ret =  i2c_dma_read(i2c, msgs, num);
		} else {
#endif
//			dprintk(DEBUG_INFO, "master cpu read\n",
//					i2c->port);
			ret = i2c_drv_read(i2c, msgs, num);
#ifdef DMA_XXX
		}
#endif
	} else {
			/* 1 msgs write */
			i2c_disable_read_tran_mode(i2c->base_addr);
#ifdef DMA_XXX
			if (i2c->dma_tx && (msgs->len >= DMA_THRESHOLD)) {
//				dprintk(DEBUG_INFO,
//						"master dma write\n",
//						i2c->port);
				ret = i2c_dma_write(i2c, msgs);
			} else {
#endif
//				dprintk(DEBUG_INFO,
//						"master cpu write\n",
//						i2c->port);
				ret = i2c_drv_write(i2c, msgs);
#ifdef DMA_XXX
			}
#endif
		}

	if (ret)
		return ret;

	return sunxi_i2c_drv_complete(i2c);
}


static int sunxi_i2c_engine_do_xfer(struct sunxi_i2c *i2c, struct i2c_msg *msgs, int num)
{
	int ret;

	const uint32_t base_addr = i2c->base_addr;
//	BaseType_t sem_ret;
	int sem_ret;

	I2C_INFO("sunxi_i2c_engine_do_xfer\n");
	twi_soft_reset(base_addr, TWI_SRST_REG, TWI_SRST_SRST);
	//udelay(100);

	/* test the bus is free,already protect by the semaphore at DEV layer */
	while (twi_query_irq_status(base_addr) != TWI_STAT_IDLE &&
		twi_query_irq_status(base_addr) != TWI_STAT_BUS_ERR &&
		twi_query_irq_status(base_addr) != TWI_STAT_ARBLOST_SLAR_ACK) {
		I2C_ERR("[i2c%d] bus is busy, status = %x\n",
			i2c->port, twi_query_irq_status(base_addr));
		if (twi_send_clk_9pulse(base_addr, i2c->port) != SUNXI_I2C_OK) {
			ret = SUNXI_I2C_RETRY;
			goto out;
		} else
			break;
	}

	/* may conflict with xfer_complete */
	//spin_lock_irqsave(&i2c->lock, flags);
	i2c->msgs    = msgs;
	i2c->msgs_num = num;
	i2c->msgs_ptr = 0;
	i2c->msgs_idx = 0;
	twi_enable_irq(base_addr);  /* enable irq */
	twi_disable_ack(base_addr); /* disabe ACK */
	/* set the special function register,default:0. */
	twi_set_efr(base_addr, 0);
	//spin_unlock_irqrestore(&i2c->lock, flags);

	/* START signal, needn't clear int flag */
	i2c->status  = I2C_XFER_START;
	ret = twi_start(base_addr, i2c->port);
	if (ret == SUNXI_I2C_FAIL) {
		I2C_ERR("[i2c%d] twi start fail\n", i2c->port);
		twi_soft_reset(base_addr, TWI_SRST_REG, TWI_SRST_SRST);
		twi_disable_irq(base_addr);  /* disable irq */
		ret = SUNXI_I2C_RETRY;
		goto out;
	}


	i2c->status  = I2C_XFER_RUNNING;
	/* sleep and wait,do the transfer at interrupt handler,timeout = 5*HZ */
//	sem_ret = xSemaphoreTake(i2c->sem, (i2c->timeout *1000)/portTICK_RATE_MS);
	sem_ret = sem_wait(&i2c->sem);
	/* return code,if(msgs_idx == num) succeed */
	ret = i2c->msgs_idx;
	if (sem_ret != pdPASS) {
		I2C_ERR("[i2c%d] xfer timeout (dev addr:0x%x)\n",
				i2c->port, msgs->addr);
		//spin_lock_irqsave(&i2c->lock, flags);
		i2c->msgs = NULL;
		//spin_unlock_irqrestore(&i2c->lock, flags);
		ret = SUNXI_I2C_FAIL;
	} else if (ret != num) {
		I2C_ERR("[i2c%d] incomplete xfer (status: 0x%x, dev addr: 0x%x)\n",
				i2c->port, ret, msgs->addr);
		ret = SUNXI_I2C_FAIL;
	}

	I2C_INFO("sunxi_i2c_engine_do_xfer end\n");
out:
	return ret;
}

hal_i2c_status_t sunxi_i2c_xfer(hal_i2c_port_t port, i2c_msg_t *msgs, int32_t num)
{
	sunxi_i2c_t *i2c = &g_sunxi_i2c[port];
	int ret;

	if ((msgs == NULL) || (num <= 0)) {
		I2C_ERR("[i2c%d] invalid argument\n", port);
		return HAL_I2C_STATUS_INVALID_PARAMETER;
	}


#if 0
	I2C_INFO("num = %d\n", num);
	if (debug_mask & DEBUG_INFO2) {
		for (n = 0; n < num; n++) {
			printk("num: %d, data: ", n);
			if ((msgs + n)->buf) {
				for (m = 0; m < (msgs + n)->len; m++)
					printk("%02x ", *((msgs + n)->buf + m));
				printk("\n");
			} else
				printk("null\n");
		}
	}
#endif

	//spin_lock_irq();
	//i2c->result = I2C_XFER_OK;
	//spin_unlock_irq();

	if (i2c->twi_drv_used) {
		I2C_INFO("[i2c%d] twi driver xfer\n", i2c->port);
		ret = sunxi_i2c_drv_do_xfer(i2c, msgs, num);
		if (ret < 0)
			return HAL_I2C_STATUS_ERROR;
	} else {
		I2C_INFO("[i2c%d] twi engine xfer\n", i2c->port);
		ret = sunxi_i2c_engine_do_xfer(i2c, msgs, num);
		if (ret < 0)
			return HAL_I2C_STATUS_ERROR;
	}

	if (ret == num)
		return HAL_I2C_STATUS_OK;
	else
		return HAL_I2C_STATUS_ERROR;
}

static int sunxi_i2c_gpio_enable(uint8_t port)
{
	sunxi_i2c_t *i2c = &g_sunxi_i2c[port];

	i2c_gpio_t *clk = i2c->pinctrl.clk;
	i2c_gpio_t *sda = i2c->pinctrl.sda;

	hal_gpio_pinmux_set_function((hal_gpio_pin_t)clk->gpio, (hal_gpio_direction_t)clk->enable_mux);
	hal_gpio_pinmux_set_function((hal_gpio_pin_t)sda->gpio, (hal_gpio_direction_t)sda->enable_mux);

	return 0;
}

static int sunxi_i2c_gpio_disable(uint8_t port)
{
	sunxi_i2c_t *i2c = &g_sunxi_i2c[port];

	i2c_gpio_t *clk = i2c->pinctrl.clk;
	i2c_gpio_t *sda = i2c->pinctrl.sda;

	hal_gpio_pinmux_set_function((hal_gpio_pin_t)clk->gpio, (hal_gpio_direction_t)clk->disable_mux);
	hal_gpio_pinmux_set_function((hal_gpio_pin_t)sda->gpio, (hal_gpio_direction_t)sda->disable_mux);

	return 0;
}

static int sunxi_i2c_clk_init(sunxi_i2c_t *i2c)
{
	const uint32_t base_addr = i2c->base_addr;
	unsigned int clk_apb = 0;

	hal_clock_enable((hal_clk_id_t)(HAL_CLK_PERIPH_TWI0 + i2c->port));

	//source clk : apb2. default : 24000000
	clk_apb = hal_clk_get_rate(HAL_CLK_BUS_APB2);
	I2C_INFO("i2c clk_src(apb) : %d\n", clk_apb);

	/* enable twi engine or twi driver */
	//twi_set_clock(i2c->port, 24000000, i2c->freq);
	twi_set_clock(i2c, TWI_CLK_REG, clk_apb, i2c->freq,
			TWI_CLK_DIV_M, TWI_CLK_DIV_N);

	if (i2c->twi_drv_used) {
		twi_set_clock(i2c, TWI_DRIVER_BUSC, 24000000, i2c->freq,
				TWI_DRV_CLK_M, TWI_DRV_CLK_N);
		twi_enable(base_addr, TWI_DRIVER_CTRL, TWI_DRV_EN);
	}
	else {
		twi_set_clock(i2c, TWI_CLK_REG, clk_apb, i2c->freq,
				TWI_CLK_DIV_M, TWI_CLK_DIV_N);
		twi_enable(i2c->base_addr, TWI_CTL_REG, TWI_CTL_BUSEN);
	}


	return 0;
}

static int sunxi_i2c_clk_exit(sunxi_i2c_t *i2c)
{
	const uint32_t base_addr = i2c->base_addr;

	/* disable twi bus */
	twi_disable(base_addr, TWI_DRIVER_CTRL, TWI_DRV_EN);


	return 0;
}

static int sunxi_i2c_hw_init(sunxi_i2c_t *i2c)
{
	uint32_t port = i2c->port;
	i2c_pinctrl_t *pinctrl = &i2c->pinctrl;

	//gpio init
	pinctrl->enable(port);

	//clk init
	sunxi_i2c_clk_init(i2c);

	if (!(i2c->twi_drv_used))
		twi_soft_reset(i2c->base_addr, TWI_SRST_REG, TWI_SRST_SRST);

	return 0;
}

hal_i2c_status_t sunxi_i2c_init(hal_i2c_config_t *i2c_config)
{
	int sem_ret;
	sunxi_i2c_t *i2c = &g_sunxi_i2c[i2c_config->port];

	i2c->port = i2c_config->port;
	i2c->freq = g_i2c_frequency[i2c_config->freq];
	i2c->twi_drv_used = i2c_config->trans_mode == HAL_TWI_DRV_XFER ? 1 : 0;
	i2c->pinctrl.clk = (i2c_gpio_t *)&i2c_config->clk;
	i2c->pinctrl.sda = (i2c_gpio_t *)&i2c_config->sda;

	i2c->pinctrl.enable = sunxi_i2c_gpio_enable;
	i2c->pinctrl.disable = sunxi_i2c_gpio_disable;

	i2c->base_addr = SUNXI_TWI0_PBASE + (i2c->port * SUNXI_I2C_REG_SIZE);
	i2c->irqnum = R328_IRQ_TWI0 + i2c->port;

	i2c->status       = I2C_XFER_IDLE;
	i2c->timeout = 5;

	//spin_unlock_irq();

	/*i2c->sem = (xSemaphoreHandle)xSemaphoreCreateCounting(I2C_SEM_MAX_COUNT, 0);
	if (i2c->sem == NULL) {
		I2C_ERR("[i2c%d] request init sem error\n", i2c->port);
		return HAL_I2C_STATUS_ERROR;
	}*/
	sem_ret = sem_init(&i2c->sem, 0, 0);
	if (sem_ret != 0) {
		I2C_ERR("[i2c%d] request init sem error\n", i2c->port);
		return HAL_I2C_STATUS_ERROR;
	}

	/*if (irq_request(i2c->irqnum, sunxi_i2c_handler, i2c) < 0) {
		I2C_ERR("[i2c%d] request irq error\n", i2c->port);
		return HAL_I2C_STATUS_ERROR;
	}

	if (irq_enable(i2c->irqnum) < 0) {
		I2C_ERR("[i2c%d] enable irq error\n", i2c->port);
		return HAL_I2C_STATUS_ERROR;
	}*/
        if (irq_attach(i2c->irqnum, sunxi_i2c_handler, i2c) < 0) {
                I2C_ERR("[i2c%d] request irq error\n", i2c->port);
                return HAL_I2C_STATUS_ERROR;
        }
        up_enable_irq(i2c->irqnum);

	if (sunxi_i2c_hw_init(i2c) < 0) {
		I2C_ERR("[i2c%d] hardware init error\n", i2c->port);
		return HAL_I2C_STATUS_ERROR;
	}
	/*
	i2c->dev_s = &sunxi_i2c_master;

	ret = i2c_register(i2c->dev_s, i2c->port);
	if (ret != 0) {
		sinfo("[i2c%d] register failed\n", i2c->port);
		return HAL_I2C_STATUS_ERROR;
	}*/
	I2C_INFO("[i2c%d] init success\n", i2c->port);
	return HAL_I2C_STATUS_OK;
}

hal_i2c_status_t sunxi_i2c_deinit(hal_i2c_port_t port)
{
	sunxi_i2c_t *i2c = &g_sunxi_i2c[port];

	sunxi_i2c_clk_exit(i2c);

	return HAL_I2C_STATUS_OK;
}


