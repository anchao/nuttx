/****************************************************************************
 * arch/arm/src/r328/r328_lowputc.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"


#define	UART_REG_ADDR	(0x05000000)
#define UART_REG_RBR 	(UART_REG_ADDR + 0x00)
#define UART_REG_THR 	(UART_REG_ADDR + 0x00)
#define UART_REG_DLL 	(UART_REG_ADDR + 0x00)
#define UART_REG_DLH 	(UART_REG_ADDR + 0x04)
#define UART_REG_IER 	(UART_REG_ADDR + 0x04)
#define UART_REG_IIR 	(UART_REG_ADDR + 0x08)
#define UART_REG_FCR 	(UART_REG_ADDR + 0x08)
#define UART_REG_LCR 	(UART_REG_ADDR + 0x0c)
#define UART_REG_MCR 	(UART_REG_ADDR + 0x10)
#define UART_REG_LSR 	(UART_REG_ADDR + 0x14)
#define UART_REG_MSR 	(UART_REG_ADDR + 0x18)
#define UART_REG_SCH 	(UART_REG_ADDR + 0x1c)
#define UART_REG_USR 	(UART_REG_ADDR + 0x7c)
#define UART_REG_TFL 	(UART_REG_ADDR + 0x80)
#define UART_REG_RFL 	(UART_REG_ADDR + 0x84)
#define UART_REG_HALT	(UART_REG_ADDR + 0xa4)

//uart config
#define UART_BAUDRATE	(115200)


#define uart_readb(addr)             (*((volatile unsigned char  *)(addr)))
#define uart_readw(addr)             (*((volatile unsigned short *)(addr)))
#define uart_readl(addr)             (*((volatile unsigned long  *)(addr)))
#define uart_writeb(v, addr) (*((volatile unsigned char  *)(addr)) = (unsigned char)(v))
#define uart_writew(v, addr) (*((volatile unsigned short *)(addr)) = (unsigned short)(v))
#define uart_writel(v, addr) (*((volatile unsigned long  *)(addr)) = (unsigned long)(v))


void up_lowputc(char ch)
{
	while (!(uart_readl(UART_REG_USR) & 2)) {
		//fifo is full, check again.
		;
	}

	//write out charset to transmit fifo
	uart_writeb(ch, UART_REG_THR);
	
}

