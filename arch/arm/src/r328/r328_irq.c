/****************************************************************************
 * arch/arm/src/r328/r328_irq.c
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"
#include "up_internal.h"
#include "sctlr.h"

#include "r328_pio.h"
#include "r328_irq.h"
#include "gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size of the interrupt stack allocation */

#define INTSTACK_ALLOC (CONFIG_SMP_NCPUS * INTSTACK_SIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

#ifdef CONFIG_SMP
/* For the case of configurations with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];
#else
volatile uint32_t *g_current_regs[1];
#endif

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
/* In the SMP configuration, we will need custom IRQ and FIQ stacks.
 * These definitions provide the aligned stack allocations.
 */

uint64_t g_irqstack_alloc[INTSTACK_ALLOC >> 3];
uint64_t g_fiqstack_alloc[INTSTACK_ALLOC >> 3];

/* These are arrays that point to the top of each interrupt stack */

uintptr_t g_irqstack_top[CONFIG_SMP_NCPUS] =
{
  (uintptr_t)g_irqstack_alloc + INTSTACK_SIZE,
#if CONFIG_SMP_NCPUS > 1
  (uintptr_t)g_irqstack_alloc + 2 * INTSTACK_SIZE,
#endif
#if CONFIG_SMP_NCPUS > 2
  (uintptr_t)g_irqstack_alloc + 3 * INTSTACK_SIZE,
#endif
#if CONFIG_SMP_NCPUS > 3
  (uintptr_t)g_irqstack_alloc + 4 * INTSTACK_SIZE
#endif
};

uintptr_t g_fiqstack_top[CONFIG_SMP_NCPUS] =
{
  (uintptr_t)g_fiqstack_alloc + INTSTACK_SIZE,
#if CONFIG_SMP_NCPUS > 1
  (uintptr_t)g_fiqstack_alloc + 2 * INTSTACK_SIZE,
#endif
#if CONFIG_SMP_NCPUS > 2
  (uintptr_t)g_fiqstack_alloc + 3 * INTSTACK_SIZE,
#endif
#if CONFIG_SMP_NCPUS > 3
  (uintptr_t)g_fiqstack_alloc + 4 * INTSTACK_SIZE
#endif
};

#endif

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

#include "debug.h"
#include "interrupt.h"

#if 0

#define readb(addr)             (*((volatile unsigned char  *)(addr)))
#define readw(addr)             (*((volatile unsigned short *)(addr)))
#define readl(addr)             (*((volatile unsigned long  *)(addr)))
#define writeb(v, addr) 	(*((volatile unsigned char  *)(addr)) = (unsigned char)(v))
#define writew(v, addr) 	(*((volatile unsigned short *)(addr)) = (unsigned short)(v))
#define writel(v, addr) 	(*((volatile unsigned long  *)(addr)) = (unsigned long)(v))
#define GICC_IAR_INT_ID_MASK	(0x3ff)


/* Symbols defined via the linker script */

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

static __inline void sr32(unsigned int addr, unsigned int start_bit, unsigned int num_bits, unsigned int value)
{
    unsigned int tmp, msk = (1 << num_bits) - 1;
    tmp = readl(addr) & ~(msk << start_bit);
    tmp |= value << start_bit;
    writel(tmp, addr);
}

void gic_distributor_init(void)
{
	unsigned int cpumask = 0x01010101;
	unsigned int gic_irqs;
	unsigned int i;

	/*SMP_DBG("GIC_DIST_CON=0x%08x.\n", GIC_DIST_CON);*/

	writel(0, GIC_DIST_CON);

	/* check GIC hardware configutation */
	gic_irqs = ((readl(GIC_CON_TYPE) & 0x1f) + 1) * 32;
	if (gic_irqs > 1020)
		gic_irqs = 1020;
	else if (gic_irqs < GIC_IRQ_NUM) {
		/*SMP_DBG("GIC parameter config xx error, only support %d"*/
				/*" irqs < %d(spec define)!!\n", gic_irqs, GIC_IRQ_NUM);*/
		return ;
	}

	/*SMP_DBG("GIC Support max %d interrupts.\n", gic_irqs);*/

	/*  Set ALL interrupts as group1(non-secure) interrupts */
	unsigned int max_irq = readl(GIC_DIST_BASE + 0x004) & 0x1f;
	unsigned int it_lines_number = max_irq;

	for(i = 0; i <= it_lines_number; i++) {
		writel(0xffffffff, GIC_DIST_BASE + 0x80 + i * 4);
	}

	/* set trigger type to be level-triggered, active low */
	for (i=GIC_SRC_SGI(0); i<GIC_IRQ_NUM; i+=16)
		writel(0, GIC_IRQ_MOD_CFG(i>>4));				//��16
	/* set priority */
	for (i=GIC_SRC_SGI(0); i<GIC_IRQ_NUM; i+=4)
		writel(0xa0a0a0a0, GIC_SPI_PRIO((i-32)>>2));	//��4
	/* set processor target */
	for (i=GIC_SRC_SGI(0); i<GIC_IRQ_NUM; i+=4)
		writel(cpumask, GIC_SPI_PROC_TARG((i-32)>>2));	//��4

	/* disable all interrupts */
	for (i=GIC_SRC_SGI(0); i<GIC_IRQ_NUM; i+=32)
		writel(0xffffffff, GIC_CLR_EN(i>>5));	//��32

	/* clear all interrupt active state */
	for (i=GIC_SRC_SGI(0); i<GIC_IRQ_NUM; i+=32)
		writel(0xffffffff, GIC_ACT_CLR(i>>5));	//��32


	writel(0x3, GIC_DIST_CON);
	/*soft_break();*/
}

void gic_cpuif_init(void)
{
    	int i = 0;

	writel(0, GIC_CPU_IF_CTRL);

	writel(0xffffffff, GIC_DIST_BASE + 0x80);

	/*
	 * Deal with the banked PPI and SGI interrupts - disable all
	 * PPI interrupts, ensure all SGI interrupts are enabled.
	 */
	writel(0xffff0000, GIC_CLR_EN0);
	writel(0x0000ffff, GIC_SET_EN0);

	/* Set priority on PPI and SGI interrupts */
	for (i=0; i<16; i+=4)
	{
		writel(0xa0a0a0a0, GIC_SGI_PRIO(i>>2));
	}
	for (i=16; i<32; i+=4)
	{
		writel(0xa0a0a0a0, GIC_PPI_PRIO((i-16)>>2));
	}

#define GICC_CTLR_ENABLEGRP0    (1 << 0)
#define GICC_CTLR_ENABLEGRP1    (1 << 1)
#define GICC_CTLR_FIQEN         (1 << 3)
	writel(0xff, GIC_INT_PRIO_MASK);
	writel(0xf, GIC_CPU_IF_CTRL);
	/*soft_break();*/

}

void up_irqinitialize(void)
{
   gic_cpuif_init();
   gic_distributor_init();

   DEBUGASSERT((((uintptr_t)&_vector_start) & ~VBAR_MASK) == 0);
   cp15_wrvbar((uint32_t)&_vector_start);

   (void)up_irq_enable();
}
#if 0
void up_irqinitialize(void)
{
  int i;

  sinfo("r328 irq initialize\n");

  arm_gic0_initialize();  /* Initialization unique to CPU0 */
  arm_gic_initialize();   /* Initialization common to all CPUs */

  /* And finally, enable interrupts */

  (void)up_irq_enable();


  //r328_dumpintc("initial", 0);
}
#endif


void up_enable_irq(int irq_no)
{
	unsigned int base;
	unsigned int base_os;
	unsigned int bit_os;

  sinfo("enable irq number=%d.\n", irq_no);
	if (irq_no >= GIC_IRQ_NUM) {
		sinfo("irq NO.(%d) > GIC_IRQ_NUM(%d) !!\n", irq_no, GIC_IRQ_NUM);
		return -1;
	}

	base_os = irq_no >> 5; // ��32
	base = GIC_SET_EN(base_os);
	bit_os = irq_no & 0x1f; // %32
  sinfo("base=%x. bit_os=%x\n", base, bit_os);
	sr32(base, bit_os, 1, 1);

	return 0;
}

void up_disable_irq(int irq_no)
{
	unsigned int base;
	unsigned int base_os;
	unsigned int bit_os;

	if (irq_no >= GIC_IRQ_NUM) {
		sinfo("irq NO.(%d) > GIC_IRQ_NUM(%d) !!\n", irq_no, GIC_IRQ_NUM);
		return -1;
	}

	base_os = irq_no >> 5; // ��32
	base = GIC_CLR_EN(base_os);
	bit_os = irq_no & 0x1f; // %32
	sr32(base, bit_os, 1, 1);

	return 0;
}


uint32_t *arm_decodeirq(uint32_t *regs)
{
  uint32_t regval;
  int irq;

  //sinfo(" >>>> decode irq.\n");
  /* Read the interrupt acknowledge register and get the interrupt ID */

  regval = getreg32(GIC_ICCIAR);
  irq    = (regval & GIC_ICCIAR_INTID_MASK) >> GIC_ICCIAR_INTID_SHIFT;

  //sinfo("---->>>>>>>>irq=%d\n", irq);

  /* Ignore spurions IRQs.  ICCIAR will report 1023 if there is no pending
   * interrupt.
   */

  //DEBUGASSERT(irq < NR_IRQS || irq == 1023);
  if (irq < NR_IRQS)
    {
      /* Dispatch the interrupt */

      regs = arm_doirq(irq, regs);
    }

  /* Write to the end-of-interrupt register */

  putreg32(regval, GIC_ICCEOIR);
  return regs;
}
#endif /* if 0 */

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

void up_irqinitialize(void)
{
  /* The following operations need to be atomic, but since this function is
   * called early in the initialization sequence, we expect to have exclusive
   * access to the GIC.
   */

  /* Initialize the Generic Interrupt Controller (GIC) for CPU0 */

  arm_gic0_initialize();  /* Initialization unique to CPU0 */
  arm_gic_initialize();   /* Initialization common to all CPUs */

#ifdef CONFIG_ARCH_LOWVECTORS
  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are two ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. Set the Cortex-A5 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *  The second method is used by this logic.
   */

  /* Set the VBAR register to the address of the vector table */
   //DEBUGASSERT((((uintptr_t)&_vector_start) & ~VBAR_MASK) == 0);
   //cp15_wrvbar((uint32_t)&_vector_start);

#endif /* CONFIG_ARCH_LOWVECTORS */

   DEBUGASSERT((((uintptr_t)&_vector_start) & ~VBAR_MASK) == 0);
   cp15_wrvbar((uint32_t)&_vector_start);
  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

  /* And finally, enable interrupts */

  (void)up_irq_enable();
}
