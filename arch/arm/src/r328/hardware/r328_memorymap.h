/************************************************************************************
 * arch/arm/src/r328/hardware/r328_memorymap.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_R328_HARDWARE_R328_MEMORYMAP_H
#define __ARCH_ARM_SRC_R328_HARDWARE_R328_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/r328/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Decimal configuration values may exceed 2Gb and, hence, overflow to negative
 * values unless we force them to unsigned long:
 */

#define __CONCAT(a,b) a ## b
#define MKULONG(a) __CONCAT(a,ul)

/* r328 physical section base addresses (aligned to 1MB boundaries) */
#define R328_INTMEM_PSECTION  0x00000000  /* Internal memory 0x0002:0000-0x0003:ffff */
#define R328_CE_PSECTION      0x01900000  /* CE              0x0190:4000-0x0190:4fff */
#define R328_SYS_PSECTION     0x03000000  /* system          0x0300:0000-0x0302:ffff */
#define R328_RTC_PSECTION     0x07000000  /* Rtc             0x0700:0000-0x0700:03ff */
#define R328_STORAGE_PSECTION 0x04000000  /* Storage         0x0401:1000-0x0400:5fff */
#define R328_PERIPH_PSECTION  0x05000000  /* Peripherals     0x0500:0000-0x0549:ffff */
#define R328_LEDC_PSECTION    0x06700000  /* Ledc            0x0670:0000-0x0670:03ff */
#define R328_CPUX_PSECTION    0x08100000  /* Cpux            0x0810:0000-0x0902:0fff */
#define R328_DRAM_PSECTION    0x40000000  /* Dram            0x4000:0000-0xffff:ffff */
#define R328_BROM_PSECTION    0x00000000  /* BROM            0x0000:0000-0x0000:7fff */

/* R328 Offsets from the internal memory section base address */
#define R328_NBROM_OFFSET    0x00000000 /* N-BROM         0x0000:0000-0x0000:7fff 32K  */
#define R328_SBROM_OFFSET    0x00000000 /* S-BROM         0x0000:0000-0x0000:7fff 32K  */
#define R328_SRAMA1_OFFSET   0x00020000 /* SRAM A1        0x0002:0000-0x0003:ffff 128K */

/* R328 offsets from the system section base address */
#define R328_SYS_CFG_OFFSET     0x00000000 /* SYS_CFG    0x0300:0000-0x0300:0fff 4K */
#define R328_CCMU_OFFSET        0x00001000 /* CCMU       0x0300:1000-0x0300:1fff 4K */
#define R328_DMAC_OFFSET        0x00002000 /* DMAC       0x0300:2000-0x0300:2fff 4K */
#define R328_HSTIMER_OFFSET     0x00005000 /* HSTIMER    0x0300:5000-0x0300:5fff 4K */
#define R328_SID_OFFSET         0x00006000 /* SID        0x0300:6000-0x0300:6fff 4K */
#define R328_SMC_OFFSET         0x00007000 /* SMC        0x0300:7000-0x0300:7fff 4K */
#define R328_SPC_OFFSET         0x00008000 /* SPC        0x0300:8000-0x0300:83ff 1K */
#define R328_TIMER_OFFSET       0x00009000 /* TIMER      0x0300:9000-0x0300:93ff 1K */
#define R328_PWM_OFFSET         0x0000A000 /* PWM        0x0300:A000-0x0300:A3ff 1K */
#define R328_GPIO_OFFSET        0x0000B000 /* GPIO       0x0300:B000-0x0300:B3ff 1K */
#define R328_PSI_OFFSET         0x0000C000 /* PSI        0x0300:C000-0x0300:C3ff 1K */
#define R328_DCU_OFFSET         0x00010000 /* DCU        0x0301:0000-0x0301:ffff 64K */
#define R328_GIC_OFFSET         0x00020000 /* GIC        0x0302:0000-0x0301:ffff 64K */

/* R328 offsets from the rtc section base address */
#define R328_RTC_OFFSET         0x00000000 /* RTC        0x0700:0000-0x0700:03ff 1K */

/* R328 offsets from the storage section base address */
#define R328_NAND_OFFSET        0x00011000 /* NAND0      0x0401:1000-0x0401:1fff 4K  */
#define R328_SMHC_OFFSET        0x00021000 /* SMHC1      0x0402:1000-0x0402:1fff 4K  */
#define R328_MSI_CTRL_OFFSET    0x00002000 /* MSI_CTRL   0x0400:2000-0x0400:2fff 4K  */
#define R328_DRAM_PHY_OFFSET    0x00003000 /* DRAM_PHY   0x0400:3000-0x0400:5fff 12K */

/* R328 offsets from the periph section base address */
#define R328_UART0_OFFSET       0x00000000 /* UART0      0x0500:0000-0x0500:03ff 1K  */
#define R328_UART1_OFFSET       0x00000400 /* UART1      0x0500:0400-0x0500:07ff 1K  */
#define R328_UART2_OFFSET       0x00000800 /* UART2      0x0500:0800-0x0500:0bff 1K  */
#define R328_UART3_OFFSET       0x00000C00 /* UART3      0x0500:0C00-0x0500:0fff 1K  */
#define R328_TWI0_OFFSET        0x00002000 /* TWI0       0x0500:2000-0x0500:23ff 1K  */
#define R328_TWI1_OFFSET        0x00002400 /* TWI1       0x0500:2400-0x0500:27ff 1K  */
#define R328_SPI0_OFFSET        0x00010000 /* SPI0       0x0501:0000-0x0501:0fff 4K  */
#define R328_SPI1_OFFSET        0x00011000 /* SPI1       0x0501:1000-0x0501:1fff 4K  */
#define R328_GPADC_OFFSET       0x00070000 /* GPADC      0x0507:0000-0x0507:03ff 1K  */
#define R328_THS_OFFSET         0x00070400 /* THS        0x0507:0400-0x0507:07ff 1K  */
#define R328_LRADC_OFFSET       0x00070800 /* LRADC      0x0507:0800-0x0507:0bff 1K  */
#define R328_I2S0_OFFSET        0x00090000 /* I2S0       0x0509:0000-0x0509:0fff 4K  */
#define R328_I2S1_OFFSET        0x00091000 /* I2S1       0x0509:1000-0x0509:1fff 4K  */
#define R328_I2S2_OFFSET        0x00092000 /* I2S1       0x0509:2000-0x0509:2fff 4K  */
#define R328_SPDIF_OFFSET       0x00093000 /* SPDIF      0x0509:3000-0x0509:33ff 1K  */
#define R328_DMIC_OFFSET        0x00095000 /* DMIC       0x0509:5000-0x0509:53ff 1K  */
#define R328_AUDIOCODEC_OFFSET  0x00096000 /* AUDIOCODEC 0x0509:6000-0x0509:6fff 4K  */
#define R328_USB0_OFFSET        0x00100000 /* USB0       0x0510:0000-0x051f:ffff 1M  */
#define R328_MAD_OFFSET         0x00400000 /* USB0       0x0540:0000-0x0540:0fff 4K  */
#define R328_MAD_SRAM_OFFSET    0x00480000 /* MAD_SRAM   0x0548:0000-0x0549:ffff 128K  */

/* R328 offsets from the ledc section base address */
#define R328_LEDC_OFFSET       0x00000000  /* LEDC       0x0670:0000-0x0670:3ff0 1K  */

/* R328 offsets from the cpux section base address */
#define R328_CPU_SYS_OFFSET    0x00000000  /* CPU_SYS    0x0810:0000-0x0810:03ff 1K  */
#define R328_TIMESTAMPS_OFFSET 0x00010000  /* TIMESTAMPS 0x0811:0000-0x0811:0fff 4K  */
#define R328_TIMESTAMPC_OFFSET 0x00020000  /* TIMESTAMPS 0x0812:0000-0x0812:0fff 4K  */
#define R328_IDC_OFFSET        0x00030000  /* IDC        0x0813:0000-0x0813:0fff 4K  */

/* R328 offsets from the dram section base address */
#define R328_DRAM_OFFSET       0x00000000  /* DRAM       0x4000:0000-0xffff:ffff 3G  */


/* R328 internal memory physical base address */
#define R328_NBROM_PADDR    (R328_INTMEM_PSECTION + R328_NBROM_OFFSET)
#define R328_SBROM_PADDR    (R328_INTMEM_PSECTION + R328_SBROM_OFFSET)
#define R328_SRAMA1_PADDR   (R328_INTMEM_PSECTION + R328_SRAMA1_OFFSET)


/* R328 system physical base address */
#define R328_SYS_CFG_PADDR  (R328_SYS_PSECTION + R328_SYS_CFG_OFFSET)
#define R328_CCMU_PADDR     (R328_SYS_PSECTION + R328_CCMU_OFFSET)
#define R328_DMAC_PADDR     (R328_SYS_PSECTION + R328_DMAC_OFFSET)
#define R328_HSTIMER_PADDR  (R328_SYS_PSECTION + R328_HSTIMER_OFFSET)
#define R328_SID_PADDR      (R328_SYS_PSECTION + R328_SID_OFFSET)
#define R328_SMC_PADDR      (R328_SYS_PSECTION + R328_SMC_OFFSET)
#define R328_SPC_PADDR      (R328_SYS_PSECTION + R328_SPC_OFFSET)
#define R328_TIMER_PADDR    (R328_SYS_PSECTION + R328_TIMER_OFFSET)
#define R328_PWM_PADDR      (R328_SYS_PSECTION + R328_PWM_OFFSET)
#define R328_GPIO_PADDR     (R328_SYS_PSECTION + R328_GPIO_OFFSET)
#define R328_PSI_PADDR      (R328_SYS_PSECTION + R328_PSI_OFFSET)
#define R328_DCU_PADDR      (R328_SYS_PSECTION + R328_DCU_OFFSET)
#define R328_GIC_PADDR      (R328_SYS_PSECTION + R328_GIC_OFFSET)

/* R328 rtc physical base address */
#define R328_RTC_PADDR      (R328_RTC_PSECTION + R328_RTC_OFFSET)


/* R328 storage physical base address */
#define R328_NAND_PADDR     (R328_STORAGE_PSECTION + R328_NAND_OFFSET)
#define R328_SMHC_PADDR     (R328_STORAGE_PSECTION + R328_SMHC_OFFSET)
#define R328_MSI_CTRL_PADDR (R328_STORAGE_PSECTION + R328_MSI_CTRL_OFFSET)
#define R328_DRAM_PHY_PADDR (R328_STORAGE_PSECTION + R328_DRAM_PHY_OFFSET)

/* R328 periph physical base address */
#define R328_UART0_PADDR    (R328_PERIPH_PSECTION + R328_UART0_OFFSET)
#define R328_UART1_PADDR    (R328_PERIPH_PSECTION + R328_UART1_OFFSET)
#define R328_UART2_PADDR    (R328_PERIPH_PSECTION + R328_UART2_OFFSET)
#define R328_UART3_PADDR    (R328_PERIPH_PSECTION + R328_UART3_OFFSET)
#define R328_TWI0_PADDR     (R328_PERIPH_PSECTION + R328_TWI0_OFFSET)
#define R328_TWI1_PADDR     (R328_PERIPH_PSECTION + R328_TWI1_OFFSET)
#define R328_SPI0_PADDR     (R328_PERIPH_PSECTION + R328_SPI0_OFFSET)
#define R328_SPI1_PADDR     (R328_PERIPH_PSECTION + R328_SPI1_OFFSET)
#define R328_GPADC_PADDR    (R328_PERIPH_PSECTION + R328_GPADC_OFFSET)
#define R328_THS_PADDR      (R328_PERIPH_PSECTION + R328_THS_OFFSET)
#define R328_LRADC_PADDR    (R328_PERIPH_PSECTION + R328_LRADC_OFFSET)
#define R328_I2S0_PADDR     (R328_PERIPH_PSECTION + R328_I2S0_OFFSET)
#define R328_I2S1_PADDR     (R328_PERIPH_PSECTION + R328_I2S1_OFFSET)
#define R328_I2S2_PADDR     (R328_PERIPH_PSECTION + R328_I2S2_OFFSET)
#define R328_SPDIF_PADDR    (R328_PERIPH_PSECTION + R328_SPDIF_OFFSET)
#define R328_DMIC_PADDR     (R328_PERIPH_PSECTION + R328_DMIC_OFFSET)
#define R328_AUDIOCODEC_PADDR    (R328_PERIPH_PSECTION + R328_AUDIOCODEC_OFFSET)
#define R328_USB0_PADDR    (R328_PERIPH_PSECTION + R328_USB0_OFFSET)
#define R328_MAD_PADDR     (R328_PERIPH_PSECTION + R328_MAD_OFFSET)
#define R328_MAD_SRAM_PADDR    (R328_PERIPH_PSECTION + R328_MAD_SRAM_OFFSET)


/* R328 ledc memory physical base addresses */
#define R328_LEDC_PADDR    (R328_LEDC_PSECTION + R328_LEDC_OFFSET)

/* R328 dram memory physical base addresses */
#define R328_DRAM_PADDR    (R328_DRAM_PSECTION + R328_DRAM_OFFSET)

/* Sizes of memory regions in bytes.
 *
 * These sizes exclude the undefined addresses at the end of the memory
 * region.  The implemented sizes of the external memory regions are
 * not known apriori and must be specified with configuration settings.
 */
#define R328_INTMEM_SIZE      0X00020000  /* Internal memory 0x0002:0000-0x0003:ffff */
#define R328_CE_SIZE          0x00001000  /* CE              0x0190:4000-0x0190:4fff */
#define R328_SYS_SIZE         0x00030000  /* system          0x0300:0000-0x0302:ffff */
#define R328_RTC_SIZE         0x00000400  /* Rtc             0x0700:0000-0x0700:03ff */
#define R328_STORAGE_SIZE     0x00005000  /* Storage         0x0401:1000-0x0400:5fff */
#define R328_PERIPH_SIZE      0x004A0000  /* Peripherals     0x0500:0000-0x0549:ffff */
#define R328_LEDC_SIZE        0x00000400  /* Ledc            0x0670:0000-0x0670:03ff */
#define R328_CPUX_SIZE        0x01000000  /* Cpux            0x0810:0000-0x0902:0fff */
#define R328_BROM_SIZE        0x00008000  /* BROM            0x0000:0000-0x0000:7fff */
#define R328_DDR_SIZE         0x04000000  /* DDR             0x0000:0000-0x0000:7fff */


/* Force configured sizes that might exceed 2GB to be unsigned long */

#define R328_DDR_MAPOFFSET    MKULONG(CONFIG_R328_DDR_MAPOFFSET)
#define R328_DDR_MAPSIZE      MKULONG(CONFIG_R328_DDR_MAPSIZE)
#define R328_DDR_HEAP_OFFSET  MKULONG(CONFIG_R328_DDR_HEAP_OFFSET)
#define R328_DDR_HEAP_SIZE    MKULONG(CONFIG_R328_DDR_HEAP_SIZE)

/* Convert size in bytes to number of sections (in Mb). */

#define _NSECTIONS(b)        (((b)+0x000fffff) >> 20)

/* Sizes of memory regions in sections.
 *
 * The boot logic in A1X_boot.c, will select 1Mb level 1 MMU mappings to
 * span the entire physical address space.  The definitions below specify
 * the number of 1Mb entries that are required to span a particular address
 * region.
 *
 * NOTE: the size of the mapped SDRAM region depends on the configured size
 * of DRAM, not on the size of the address space assigned to DRAM.
 */
#define R328_INTMEM_NSECTIONS    _NSECTIONS(R328_INTMEM_SIZE)
#define R328_CE_NSECTIONS        _NSECTIONS(R328_CE_SIZE)
#define R328_SYS_NSECTIONS       _NSECTIONS(R328_SYS_SIZE)
#define R328_RTC_NSECTIONS       _NSECTIONS(R328_RTC_SIZE)
#define R328_STORAGE_NSECTIONS   _NSECTIONS(R328_STORAGE_SIZE)
#define R328_PERIPH_NSECTIONS    _NSECTIONS(R328_PERIPH_SIZE)
#define R328_LEDC_NSECTIONS      _NSECTIONS(R328_LEDC_SIZE)
#define R328_CPUX_NSECTIONS      _NSECTIONS(R328_CPUX_SIZE)
#define R328_BROM_NSECTIONS      _NSECTIONS(R328_BROM_SIZE)
#define R328_DDR_NSECTIONS      _NSECTIONS(R328_DDR_SIZE)


/* Section MMU Flags */

#define R328_INTMEM_MMUFLAGS      MMU_MEMFLAGS
#define R328_CE_MMUFLAGS          MMU_IOFLAGS
#define R328_SYS_MMUFLAGS         MMU_IOFLAGS
#define R328_RTC_MMUFLAGS         MMU_IOFLAGS
#define R328_STORAGE_MMUFLAGS     MMU_IOFLAGS
#define R328_PERIPH_MMUFLAGS      MMU_IOFLAGS
#define R328_LEDC_MMUFLAGS        MMU_IOFLAGS
#define R328_CPUX_MMUFLAGS        MMU_IOFLAGS
#define R328_BROM_MMUFLAGS        MMU_IOFLAGS
#define R328_DDR_MMUFLAGS         MMU_MEMFLAGS


/* A1X Virtual (mapped) Memory Map
 *
 * board_memorymap.h contains special mappings that are needed when a ROM
 * memory map is used.  It is included in this odd location becaue it depends
 * on some the virtual address definitions provided above.
 */

#include <arch/board/board_memorymap.h>

/* A1X Virtual (mapped) Memory Map.  These are the mappings that will
 * be created if the page table lies in RAM.  If the platform has another,
 * read-only, pre-initialized page table (perhaps in ROM), then the board.h
 * file must provide these definitions.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE

/* Notice that these mappings are a simple 1-to-1 mappings */

#define R328_INTMEM_VSECTION  0x00000000  /* Internal memory 0x0002:0000-0x0003:ffff */
#define R328_CE_VSECTION      0x01900000  /* CE              0x0190:4000-0x0190:4fff */
#define R328_SYS_VSECTION     0x03000000  /* system          0x0300:0000-0x0302:ffff */
#define R328_RTC_VSECTION     0x07000000  /* Rtc             0x0700:0000-0x0700:03ff */
#define R328_STORAGE_VSECTION 0x04000000  /* Storage         0x0401:1000-0x0400:5fff */
#define R328_PERIPH_VSECTION  0x05000000  /* Peripherals     0x0500:0000-0x0549:ffff */
#define R328_LEDC_VSECTION    0x06700000  /* Ledc            0x0670:0000-0x0670:03ff */
#define R328_CPUX_VSECTION    0x08100000  /* Cpux            0x0810:0000-0x0902:0fff */
#define R328_DRAM_VSECTION    0x40000000  /* Dram            0x4000:0000-0xffff:ffff */
#define R328_BROM_VSECTION    0x00000000  /* BROM            0x0000:0000-0x0000:7fff */

#endif


/* R328 internal memory physical base address */
#define R328_NBROM_VADDR    (R328_INTMEM_VSECTION + R328_NBROM_OFFSET)
#define R328_SBROM_VADDR    (R328_INTMEM_VSECTION + R328_SBROM_OFFSET)
#define R328_SRAMA1_VADDR   (R328_INTMEM_VSECTION + R328_SRAMA1_OFFSET)


/* R328 system physical base address */
#define R328_SYS_CFG_VADDR  (R328_SYS_VSECTION + R328_SYS_CFG_OFFSET)
#define R328_CCMU_VADDR     (R328_SYS_VSECTION + R328_CCMU_OFFSET)
#define R328_DMAC_VADDR     (R328_SYS_VSECTION + R328_DMAC_OFFSET)
#define R328_HSTIMER_VADDR  (R328_SYS_VSECTION + R328_HSTIMER_OFFSET)
#define R328_SID_VADDR      (R328_SYS_VSECTION + R328_SID_OFFSET)
#define R328_SMC_VADDR      (R328_SYS_VSECTION + R328_SMC_OFFSET)
#define R328_SPC_VADDR      (R328_SYS_VSECTION + R328_SPC_OFFSET)
#define R328_TIMER_VADDR    (R328_SYS_VSECTION + R328_TIMER_OFFSET)
#define R328_PWM_VADDR      (R328_SYS_VSECTION + R328_PWM_OFFSET)
#define R328_GPIO_VADDR     (R328_SYS_VSECTION + R328_GPIO_OFFSET)
#define R328_PSI_VADDR      (R328_SYS_VSECTION + R328_PSI_OFFSET)
#define R328_DCU_VADDR      (R328_SYS_VSECTION + R328_DCU_OFFSET)
#define R328_GIC_VADDR      (R328_SYS_VSECTION + R328_GIC_OFFSET)

/* R328 rtc physical base address */
#define R328_RTC_VADDR      (R328_RTC_VSECTION + R328_RTC_OFFSET)


/* R328 storage physical base address */
#define R328_NAND_VADDR     (R328_STORAGE_VSECTION + R328_NAND_OFFSET)
#define R328_SMHC_VADDR     (R328_STORAGE_VSECTION + R328_SMHC_OFFSET)
#define R328_MSI_CTRL_VADDR (R328_STORAGE_VSECTION + R328_MSI_CTRL_OFFSET)
#define R328_DRAM_PHY_VADDR (R328_STORAGE_VSECTION + R328_DRAM_PHY_OFFSET)

/* R328 periph physical base address */
#define R328_UART0_VADDR    (R328_PERIPH_VSECTION + R328_UART0_OFFSET)
#define R328_UART1_VADDR    (R328_PERIPH_VSECTION + R328_UART1_OFFSET)
#define R328_UART2_VADDR    (R328_PERIPH_VSECTION + R328_UART2_OFFSET)
#define R328_UART3_VADDR    (R328_PERIPH_VSECTION + R328_UART3_OFFSET)
#define R328_TWI0_VADDR     (R328_PERIPH_VSECTION + R328_TWI0_OFFSET)
#define R328_TWI1_VADDR     (R328_PERIPH_VSECTION + R328_TWI1_OFFSET)
#define R328_SPI0_VADDR     (R328_PERIPH_VSECTION + R328_SPI0_OFFSET)
#define R328_SPI1_VADDR     (R328_PERIPH_VSECTION + R328_SPI1_OFFSET)
#define R328_GPADC_VADDR    (R328_PERIPH_VSECTION + R328_GPADC_OFFSET)
#define R328_THS_VADDR      (R328_PERIPH_VSECTION + R328_THS_OFFSET)
#define R328_LRADC_VADDR    (R328_PERIPH_VSECTION + R328_LRADC_OFFSET)
#define R328_I2S0_VADDR     (R328_PERIPH_VSECTION + R328_I2S0_OFFSET)
#define R328_I2S1_VADDR     (R328_PERIPH_VSECTION + R328_I2S1_OFFSET)
#define R328_I2S2_VADDR     (R328_PERIPH_VSECTION + R328_I2S2_OFFSET)
#define R328_SPDIF_VADDR    (R328_PERIPH_VSECTION + R328_SPDIF_OFFSET)
#define R328_DMIC_VADDR     (R328_PERIPH_VSECTION + R328_DMIC_OFFSET)
#define R328_AUDIOCODEC_VADDR    (R328_PERIPH_VSECTION + R328_AUDIOCODEC_OFFSET)
#define R328_USB0_VADDR    (R328_PERIPH_VSECTION + R328_USB0_OFFSET)
#define R328_MAD_VADDR     (R328_PERIPH_VSECTION + R328_MAD_OFFSET)
#define R328_MAD_SRAM_VADDR    (R328_PERIPH_VSECTION + R328_MAD_SRAM_OFFSET)

#define CHIP_MPCORE_VBASE  (R328_GIC_VADDR)


/* R328 ledc memory physical base addresses */
#define R328_LEDC_VADDR    (R328_LEDC_VSECTION + R328_LEDC_OFFSET)

/* R328 dram memory physical base addresses */
#define R328_DRAM_VADDR    (R328_DRAM_VSECTION + R328_DRAM_OFFSET)

/* Offset SDRAM address */
#define R328_DDR_MAPPADDR     (R328_DRAM_PSECTION+R328_DDR_MAPOFFSET)
#define R328_DDR_MAPVADDR     (R328_DRAM_VSECTION+R328_DDR_MAPOFFSET)

#define R328_UART_VADDR(n)    (R328_UART0_VADDR+ 0x400*n)

/* NuttX virtual base address
 *
 * The boot logic will create a temporarily mapping based on where NuttX is
 * executing in memory.  In this case, NuttX will be running from either
 * internal SRAM or external SDRAM.
 *
 * Setup the RAM region as the NUTTX .txt, .bss, and .data region.
 */

#define NUTTX_TEXT_VADDR     (CONFIG_RAM_VSTART & 0xfff00000)
#define NUTTX_TEXT_PADDR     (CONFIG_RAM_START & 0xfff00000)
#define NUTTX_TEXT_PEND      ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#define NUTTX_TEXT_SIZE      (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

/* MMU Page Table
 *
 * Determine the address of the MMU page table.  Regardless of the memory
 * configuration, we will keep the page table in the A1X's internal SRAM.
 */

#if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)

  /* Sanity check.. if one is undefined, both should be undefined */

#  if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif

  /* A sanity check, if the configuration says that the page table is read-only
   * and pre-initialized (maybe ROM), then it should have also defined both of
   * the page table base addresses.
   */

#  ifdef CONFIG_ARCH_ROMPGTABLE
#    error "CONFIG_ARCH_ROMPGTABLE defined; PGTABLE_BASE_P/VADDR not defined"
#  endif

#else /* PGTABLE_BASE_PADDR || PGTABLE_BASE_VADDR */

  /* If CONFIG_PAGING is selected, then parts of the 1-to-1 virtual memory
   * map probably do not apply because paging logic will probably partition
   * the SRAM section differently.  In particular, if the page table is located
   * at the end of SRAM, then the virtual page table address defined below
   * will probably be in error.  In that case PGTABLE_BASE_VADDR is defined
   * in the file mmu.h
   *
   * We must declare the page table at the bottom or at the top of internal
   * SRAM.  We pick the bottom of internal SRAM *unless* there are vectors
   * in the way at that position.
   */

#  if defined(CONFIG_ARCH_LOWVECTORS)
  /* In this case, table must lie in SRAM A2 after the vectors in SRAM A1 */

#    define PGTABLE_BASE_PADDR  R328_SRAMA1_PADDR
#    define PGTABLE_BASE_VADDR  R328_SRAMA1_VADDR

#  else /* CONFIG_ARCH_LOWVECTORS */

  /* Otherwise, the vectors lie at another location.  The page table will
   * then be positioned at the beginning of SRAM A1.
   */

#    define PGTABLE_BASE_PADDR  R328_SRAMA1_PADDR
#    define PGTABLE_BASE_VADDR  R328_SRAMA1_VADDR

#  endif /* CONFIG_ARCH_LOWVECTORS */

  /* Note that the page table does not lie in the same address space as does the
   * mapped RAM in either case.  So we will need to create a special mapping for
   * the page table at boot time.
   */

#  define ARMV7A_PGTABLE_MAPPING 1

#endif /* PGTABLE_BASE_PADDR || PGTABLE_BASE_VADDR */

/* Level 2 Page table start addresses.
 *
 * 16Kb of memory is reserved hold the page table for the virtual mappings.  A
 * portion of this table is not accessible in the virtual address space (for
 * normal operation).   There are several large holes in the physical address
 * space for which there will never be level 1 mappings:
 *
 *                                    LI PAGE TABLE
 *   ADDRESS RANGE           SIZE     ENTRIES       SECTIONS
 *   ----------------------- ------- -------------- ---------
 *   0x0003:0000-0x01eb:ffff 275MB   0x0004-0x006c 26
 *                                  *(none usable) 0
 *   0x01ec:0000-0x3fff:ffff 993MB   0x0078-0x0ffc 993
 *                                  *0x0400-0x0ffc 767
 *
 * And the largest is probably from the end of SDRAM through 0xfff0:0000.
 * But the size of that region varies with the size of the installed SDRAM.
 * It is at least:
 *
 *                                    LI PAGE TABLE
 *   ADDRESS RANGE           SIZE     ENTRIES       SECTIONS
 *   ----------------------- ------- -------------- ---------
 *   0xc000:0000-0xffef:ffff 1022MB  *0x3000-0x3ff8 1022
 *
 * And probably much larger.
 *
 *   * NOTE that the L2 page table entries must be aligned 1KB address
 *     boundaries.
 *
 * These two larger regions is where L2 page tables will positioned.  Up to
 * two L2 page tables may be used:
 *
 * 1) One mapping the vector table (only when CONFIG_ARCH_LOWVECTORS is not
 *    defined).
 * 2) If on-demand paging is supported (CONFIG_PAGING=y), than an additional
 *    L2 page table is needed.
 */

#ifndef CONFIG_ARCH_LOWVECTORS
/* Vector L2 page table offset/size */

#  define VECTOR_L2_OFFSET        0x000000400
#  define VECTOR_L2_SIZE          0x000000bfc

/* Vector L2 page table base addresses */

#  define VECTOR_L2_PBASE         (PGTABLE_BASE_PADDR+VECTOR_L2_OFFSET)
#  define VECTOR_L2_VBASE         (PGTABLE_BASE_VADDR+VECTOR_L2_OFFSET)

/* Vector L2 page table end addresses */

#  define VECTOR_L2_END_PADDR     (VECTOR_L2_PBASE+VECTOR_L2_SIZE)
#  define VECTOR_L2_END_VADDR     (VECTOR_L2_VBASE+VECTOR_L2_SIZE)

#endif /* !CONFIG_ARCH_LOWVECTORS */

/* Paging L2 page table offset/size */

#define PGTABLE_L2_START_PADDR    (R328_DRAM_PSECTION+R328_DDR_MAPOFFSET+R328_DDR_MAPSIZE)
#define PGTABLE_BROM_OFFSET       0x3ffc

#define PGTABLE_L2_OFFSET         ((PGTABLE_L2_START_PADDR >> 18) & ~3)
#define PGTABLE_L2_SIZE           (PGTABLE_BROM_OFFSET - PGTABLE_L2_OFFSET)

/* Paging L2 page table base addresses
 *
 * NOTE: If CONFIG_PAGING is defined, mmu.h will re-assign the virtual
 * address of the page table.
 */

#define PGTABLE_L2_PBASE          (PGTABLE_BASE_PADDR+PGTABLE_L2_OFFSET)
#define PGTABLE_L2_VBASE          (PGTABLE_BASE_VADDR+PGTABLE_L2_OFFSET)

/* Paging L2 page table end addresses */

#define PGTABLE_L2_END_PADDR      (PGTABLE_L2_PBASE+PGTABLE_L2_SIZE)
#define PGTABLE_L2_END_VADDR      (PGTABLE_L2_VBASE+PGTABLE_L2_SIZE)

/* Base address of the interrupt vector table.
 *
 *   A1X_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
 *   A1X_VECTOR_VSRAM - Virtual address of vector table in SRAM
 *   A1X_VECTOR_VADDR - Virtual address of vector table (0x00000000 or 0xffff0000)
 *
 * NOTE: When using LOWVECTORS, the actual base of the vectors appears to be
 * offset to address 0x0000:0040
 */

#define VECTOR_TABLE_SIZE         0x00010000
#define VECTOR_TABLE_OFFSET       0x00000040

#ifdef CONFIG_ARCH_LOWVECTORS  /* Vectors located at 0x0000:0000  */

#  define R328_VECTOR_PADDR        R328_SRAMA1_PADDR
#  define R328_VECTOR_VSRAM        R328_SRAMA1_VADDR
#  define R328_VECTOR_VADDR        0x00000000

#else  /* Vectors located at 0xffff:0000 -- this probably does not work */

#if 0
#  ifdef A1X_ISRAM1_SIZE >= VECTOR_TABLE_SIZE
#    define A1X_VECTOR_PADDR      (A1X_SRAMA1_PADDR+A1X_ISRAM1_SIZE-VECTOR_TABLE_SIZE)
#    define A1X_VECTOR_VSRAM      (A1X_SRAMA1_VADDR+A1X_ISRAM1_SIZE-VECTOR_TABLE_SIZE)
#  else
#    define A1X_VECTOR_PADDR      (A1X_SRAMA1_PADDR+A1X_ISRAM0_SIZE-VECTOR_TABLE_SIZE)
#    define A1X_VECTOR_VSRAM      (A1X_SRAMA1_VADDR+A1X_ISRAM0_SIZE-VECTOR_TABLE_SIZE)
#  endif
#  define A1X_VECTOR_VADDR        0xffff0000
#endif

#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_A1X_HARDWARE_A10_MEMORYMAP_H */
