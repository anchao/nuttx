/*
 * =====================================================================================
 *
 *       Filename:  secondary.c
 *
 *    Description:  bootup secondary core on R328.
 *
 *        Version:  1.0
 *        Created:  2019年07月10日 14时32分37秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  czl init file
 *   Organization:  
 *
 * =====================================================================================
 */
#include <stdint.h>

#define RTC_BASE              0x07000000
#define sunxi_cpucfg_base     0x09010000
#define CPU1_SOFT_ENT_REG     (0x258)
#define C0_RST_CTRL           (0x00)
#define C0_CTRL_REG0          (0x10)
#define C0_CPU_STATUS         (0X80)
#define CORE_RESET_OFFSET     (0x00)
#define L1_RST_DISABLE_OFFSET (0x00)
#define STANDBYWFI_OFFSET     (0x10)

#define GENERNIC_TIMRE_REQ	(24000000)
#define SPC_BASE		(0x03008000)
#define SUNXI_CCM_BASE		(0x03001000)
#define SUNXI_DMA_BASE  	(0x03002000)
#define SPC_STA_REG(x)		(SPC_BASE+0x10*(x)+0x0)
#define SPC_SET_REG(x)		(SPC_BASE+0x10*(x)+0x4)
#define SPC_CLR_REG(x)		(SPC_BASE+0x10*(x)+0x8)

#define portNOP() __asm volatile( "NOP" )

static inline uint64_t read_cntpct(void)
{
    uint64_t val;
    asm volatile("mrrc p15, 0, %Q0, %R0, c14" : "=r" (val));
    return val;
}

static inline uint32_t read_cntfrq(void)
{      
    unsigned int cntfrq;
    /* write */
    unsigned int temp=24000000;
    asm volatile ("mcr p15, 0, %0, c14, c0, 0" :: "r"(temp));
    /* read */
    asm volatile ("mrc  p15, 0, %0, c14, c0, 0" : "=r"(cntfrq));

    return cntfrq;
}

static inline void putreg32(uint32_t val, uint32_t addr)
{
    *(volatile uint32_t *)addr = val;    
}

static inline uint32_t read32(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

void udelay(uint32_t us)
{                 
    uint64_t start, target;
    uint64_t frequency;

    frequency = read_cntfrq();

    start = read_cntpct();
    target = frequency / 1000000ULL * us;

    while (read_cntpct() - start <= target) ;  
}

void mdelay(uint32_t ms)
{
    udelay(ms * 1000);
}

void sdelay(uint32_t sec)
{
    mdelay(sec * 1000);
}

void r328_set_cpu1_boot_entry(uint32_t entry)
{
    putreg32(entry, RTC_BASE + CPU1_SOFT_ENT_REG);
    mdelay(10);

    asm volatile("isb");
    asm volatile("dsb");
    asm volatile("dmb");
}

static void r328_enable_cpu1(int cpu)
{
    unsigned int value;
    
    /* assert cpu core reset low */
    value = read32(sunxi_cpucfg_base + C0_RST_CTRL);
    value &= (~(0x1 << (CORE_RESET_OFFSET + cpu)));
    putreg32(value, sunxi_cpucfg_base + C0_RST_CTRL);
    
    mdelay(10);
    /* L1RSTDISABLE hold low */
    value = read32(sunxi_cpucfg_base + C0_CTRL_REG0);
    value &= (~(0x1 << (L1_RST_DISABLE_OFFSET + cpu)));
    putreg32(value, sunxi_cpucfg_base + C0_CTRL_REG0);
    
    mdelay(20);
    
    /* Deassert core reset high */
    value = read32(sunxi_cpucfg_base + C0_RST_CTRL);
    value |= (0x1 << (CORE_RESET_OFFSET + cpu));
    putreg32(value, sunxi_cpucfg_base + C0_RST_CTRL);
    mdelay(10);

    asm volatile("isb");
    asm volatile("dsb");
    asm volatile("dmb");
}

void sunxi_spc_set_to_ns(void)
{
    /* set master0~13 to non_secure */
    putreg32(0x1801e023, SPC_SET_REG(0));
    putreg32(0x00000005, SPC_SET_REG(2));
    putreg32(0x00100319, SPC_SET_REG(4));
    putreg32(0x00000000, SPC_SET_REG(5));
    putreg32(0x017D3703, SPC_SET_REG(6));
    putreg32(0x00000000, SPC_SET_REG(7));
    putreg32(0x0000030F, SPC_SET_REG(8));
    putreg32(0x00000000, SPC_SET_REG(9));
    putreg32(0x03590303, SPC_SET_REG(10));
    putreg32(0x00000000, SPC_SET_REG(11));
    putreg32(0x00000000, SPC_SET_REG(12));
 
    /*dma reset*/
    putreg32(read32(SUNXI_CCM_BASE+0x70c) | (1 << 16), (SUNXI_CCM_BASE+0x70c));
    udelay(20);
    /*gating clock for dma pass*/
    putreg32(read32(SUNXI_CCM_BASE+0x70c) | (1 << 0), (SUNXI_CCM_BASE+0x70c));
 
    /* set ccmu security switch: set mbus_sec bus_sec pll_sec to non-sec */
    putreg32(0x7, SUNXI_CCM_BASE+0xf00);
 
    /* set dma security switch: set DMA channel0-7 to non-sec */
    putreg32(0xfff, SUNXI_DMA_BASE+0x20);
}


void sencond_cpu_bootup(void)
{
    //r328_set_cpu1_boot_entry((uint32_t)secondary_cpu_start);
    //portNOP();
    asm volatile( "NOP" );
    asm volatile( "NOP" );
    asm volatile( "NOP" );
    r328_enable_cpu1(1);
    asm volatile("isb");
    asm volatile("dmb");
    asm volatile("dmb");
}
