/****************************************************************************
 * arch/arm/src/r328/r328_boot.c
 *
 *   Copyright (C) 2013, 2017 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include "chip.h"
#include "arm.h"
#include "mmu.h"
#include "fpu.h"
#include "up_internal.h"
#include "up_arch.h"

#include "r328_lowputc.h"
#include "r328_boot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The vectors are, by default, positioned at the beginning of the text
 * section.  They will always have to be copied to the correct location.
 *
 * If we are using high vectors (CONFIG_ARCH_LOWVECTORS=n).  In this case,
 * the vectors will lie at virtual address 0xffff:000 and we will need
 * to a) copy the vectors to another location, and b) map the vectors
 * to that address, and
 *
 * For the case of CONFIG_ARCH_LOWVECTORS=y, defined.  Vectors will be
 * copied to SRAM A1 at address 0x0000:0000
 */

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_ARCH_ROMPGTABLE)
#  error High vector remap cannot be performed if we are using a ROM page table
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This table describes how to map a set of 1Mb pages to space the physical
 * address space of the r328.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
static const struct section_mapping_s section_mapping[] =
{
  { R328_INTMEM_PSECTION,  R328_INTMEM_VSECTION,  /* Includes vectors and page table */
    R328_INTMEM_MMUFLAGS,  R328_INTMEM_NSECTIONS
  },
  { R328_SYS_PSECTION,     R328_SYS_VSECTION,  /* Includes vectors and page table */
    R328_SYS_MMUFLAGS,     R328_SYS_NSECTIONS
  },
  { R328_SDHC_PSECTION,    R328_SDHC_VSECTION,  /* Includes vectors and page table */
    R328_SYS_MMUFLAGS,     R328_SDHC_NSECTIONS
  },
  { R328_RTC_PSECTION,     R328_RTC_VSECTION,  /* Includes vectors and page table */
    R328_RTC_MMUFLAGS,     R328_RTC_NSECTIONS
  },
  { R328_PERIPH_PSECTION,  R328_PERIPH_VSECTION,
    R328_PERIPH_MMUFLAGS,  R328_PERIPH_NSECTIONS
  },
  { R328_CPUX_PSECTION,  R328_CPUX_VSECTION,
    R328_CPUX_MMUFLAGS,  R328_CPUX_NSECTIONS
  },
  { R328_DDR_MAPPADDR,     R328_DDR_MAPVADDR,
    R328_DDR_MMUFLAGS,     R328_DDR_NSECTIONS
  },
  { R328_BROM_PSECTION,    R328_BROM_VSECTION,
    R328_BROM_MMUFLAGS,    R328_BROM_NSECTIONS
  }
};

#define NMAPPINGS \
  (sizeof(section_mapping) / sizeof(struct section_mapping_s))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r328_setupmappings
 *
 * Description:
 *   Map all of the initial memory regions defined in section_mapping[]
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline void r328_setupmappings(void)
{
  mmu_l1_map_regions(section_mapping, NMAPPINGS);
}
#endif

/****************************************************************************
 * Name: r328_vectorpermissions
 *
 * Description:
 *   Set permissions on the vector mapping.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && \
     defined(CONFIG_PAGING)
static void r328_vectorpermissions(uint32_t mmuflags)
{
  /* The PTE for the beginning of ISRAM is at the base of the L2 page table */

  uint32_t pte = mmu_l2_getentry(PG_L2_VECT_VADDR, 0);

  /* Mask out the old MMU flags from the page table entry.
   *
   * The pte might be zero the first time this function is called.
   */

  if (pte == 0)
    {
      pte = PG_VECT_PBASE;
    }
  else
    {
      pte &= PG_L1_PADDRMASK;
    }

  /* Update the page table entry with the MMU flags and save */

  mmu_l2_setentry(PG_L2_VECT_VADDR, pte, 0, mmuflags);
}
#endif

/****************************************************************************
 * Name: r328_vectormapping
 *
 * Description:
 *   Setup a special mapping for the interrupt vectors when (1) the
 *   interrupt vectors are not positioned in ROM, and when (2) the interrupt
 *   vectors are located at the high address, 0xffff0000.  When the
 *   interrupt vectors are located in ROM, we just have to assume that they
 *   were set up correctly;  When vectors  are located in low memory,
 *   0x00000000, the mapping for the ROM memory region will be suppressed.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && !defined(CONFIG_ARCH_LOWVECTORS)
static void r328_vectormapping(void)
{
  uint32_t vector_paddr = r328_VECTOR_PADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_vaddr = r328_VECTOR_VADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_size  = (uint32_t)&_vector_end - (uint32_t)&_vector_start;
  uint32_t end_paddr    = r328_VECTOR_PADDR + vector_size;

  /* REVISIT:  Cannot really assert in this context */

  DEBUGASSERT (vector_size <= VECTOR_TABLE_SIZE);

  /* We want to keep our interrupt vectors and interrupt-related logic in
   * zero-wait state internal SRAM (ISRAM).  The r328 has 128Kb of ISRAM
   * positioned at physical address 0x0300:0000; we need to map this to
   * 0xffff:0000.
   */

  while (vector_paddr < end_paddr)
    {
      mmu_l2_setentry(VECTOR_L2_VBASE,  vector_paddr, vector_vaddr,
                      MMU_L2_VECTORFLAGS);
      vector_paddr += 4096;
      vector_vaddr += 4096;
    }

  /* Now set the level 1 descriptor to refer to the level 2 page table. */

  mmu_l1_setentry(VECTOR_L2_PBASE & PMD_PTE_PADDR_MASK,
                  r328_VECTOR_VADDR & PMD_PTE_PADDR_MASK,
                  MMU_L1_VECTORFLAGS);
}
#else
  /* No vector remap */

#  define r328_vectormapping()
#endif

/****************************************************************************
 * Name: r328_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.  Vectors are already
 *   positioned at the beginning of the text region and only need to be
 *   copied in the case where we are using high vectors.
 *
 ****************************************************************************/

static void r328_copyvectorblock(void)
{
    return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 *   This logic will be executing in SDRAM.  This boot logic was started by
 *   the A10 boot logic.  At this point in time, clocking and SDRAM have
 *   already be initialized (they must be because we are executing out of
 *   SDRAM).  So all that must be done here is to:
 *
 *     1) Refine the memory mapping,
 *     2) Configure the serial console, and
 *     3) Perform board-specific initializations.
 *
 ****************************************************************************/
#ifdef CONFIG_SMP
extern void r328_cpu_enable(void);
#endif

#ifdef  CONFIG_DRIVERS_CCMU
//#include <hal_clk.h>
extern int  hal_clock_init(void);

int r328_clock_init(void)
{
	return hal_clock_init();
}
#endif


#define PROGRESS 
void arm_boot(void)
{
#ifndef CONFIG_ARCH_ROMPGTABLE
  /* __start provided the basic MMU mappings for SRAM.  Now provide mappings
   * for all IO regions (Including the vector region).
   */

  r328_setupmappings();
  up_lowputc('A');

  /* Provide a special mapping for the IRAM interrupt vector positioned in
   * high memory.
   */

  r328_vectormapping();
  up_lowputc('B');

#endif /* CONFIG_ARCH_ROMPGTABLE */

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  r328_copyvectorblock();

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU */

  arm_fpuconfig();
#endif

#ifdef CONFIG_BOOT_SDRAM_DATA
  /* This setting is inappropriate for the r328 because the code is *always*
   * executing from SDRAM.  If CONFIG_BOOT_SDRAM_DATA happens to be set,
   * let's try to do the right thing and initialize the .data and .bss
   * sections.
   */

  arm_data_initialize();
  up_lowputc('C');
#endif

  /* Perform common, low-level chip initialization (might do nothing) */

  //r328_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  //up_earlyserialinit();
#endif

  /* Perform board-specific initialization,  This must include:
   *
   * - Initialization of board-specific memory resources (e.g., SDRAM)
   * - Configuration of board specific resources (PIOs, LEDs, etc).
   */
  up_lowputc('D');

#ifdef CONFIG_SMP

  r328_cpu_enable();

  up_lowputc('E');

#endif

#ifdef CONFIG_DRIVERS_CCMU
  r328_clock_init();
#endif

  //r328_boardinitialize();
}

void board_early_initialize(void)
{

}

void board_late_initialize(void)
{
#ifdef CONFIG_DRIVERS_SOUND
extern int sunxi_soundcard_init(void);
	sunxi_soundcard_init();
#endif
}
