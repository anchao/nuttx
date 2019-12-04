/****************************************************************************************
 * arch/arm/include/r328/r328_irq.h
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
 ****************************************************************************************/

/* This file should never be included directed but, rather, only indirectly through
 * nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_R328_R328_IRQ_H
#define __ARCH_ARM_INCLUDE_R328_R328_IRQ_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* External interrupts numbers */

#define R328_IRQ_NMI         0 /* External Non-Mask Interrupt */
#  define R328_IRQ_POWER     0 /* Power module */
#  define R328_IRQ_BATTERY   0 /* Brownout detect */
#  define R328_IRQ_BROWNOUT  0 /* Brownout */
#define R328_IRQ_UART0       108 /* UART 0 interrupt */
#define R328_IRQ_UART1       109 /* UART 1 interrupt */
#define R328_IRQ_UART2       3 /* UART 2 interrupt */
#define R328_IRQ_UART3       4 /* UART 3 interrupt */
#define R328_IRQ_IR0         5 /* IR 0 interrupt */
#define R328_IRQ_IR1         6 /* IR 1 interrupt */
#define R328_IRQ_TWI0        7 /* TWI 0 interrupt */
#define R328_IRQ_TWI1        8 /* TWI 1 interrupt */
#define R328_IRQ_TWI2        9 /* TWI 2 interrupt */
#define R328_IRQ_SPI0       10 /* SPI 0 interrupt */
#define R328_IRQ_SPI1       11 /* SPI 1 interrupt */
#define R328_IRQ_SPI2       12 /* SPI 2 interrupt */
#define R328_IRQ_NC         13 /* NC */
#define R328_IRQ_AC97       14 /* AC97 interrupt */
#define R328_IRQ_TS         15 /* TS interrupt */
#define R328_IRQ_IIS        16 /* Digital Audio Controller interrupt */
#define R328_IRQ_UART4      17 /* UART 4 interrupt */
#define R328_IRQ_UART5      18 /* UART 5 interrupt */
#define R328_IRQ_UART6      19 /* UART 6 interrupt */
#define R328_IRQ_UART7      20 /* UART 7 interrupt */
#define R328_IRQ_KEYPAD     21 /* Keypad interrupt */
#define R328_IRQ_TIMER0     22 /* Timer port 0 */
#define R328_IRQ_TIMER1     23 /* Timer port 1 */
#define R328_IRQ_TIMER2     24 /* Timer 2 */
#  define R328_IRQ_ALARM    24 /* Alarm */
#  define R328_IRQ_WD       24 /* Watchdog */
#define R328_IRQ_TIMER3     25 /* Timer 3 interrupt */
#define R328_IRQ_CAN        26 /* CAN Bus controller interrupt */
#define R328_IRQ_DMA        27 /* DMA channel interrupt */
#define R328_IRQ_PIO        28 /* PIO interrupt */
#define R328_IRQ_TOUCH      29 /* Touch Panel interrupt */
#define R328_IRQ_AUDIO      30 /* Analog Audio Codec interrupt */
#define R328_IRQ_LRADC      31 /* LRADC interrupt */
#define R328_IRQ_SDMMC0     32 /* SD/MMC Host Controller 0 interrupt */
#define R328_IRQ_SDMMC1     33 /* SD/MMC Host Controller 1 interrupt */
#define R328_IRQ_SDMMC2     34 /* SD/MMC Host Controller 2 interrupt */
#define R328_IRQ_SDMMC3     35 /* SD/MMC Host Controller 3 interrupt */
#define R328_IRQ_RESERVED36 36
#define R328_IRQ_NAND       37 /* NAND Flash Controller (NFC) interrupt */
#define R328_IRQ_USB0       38 /* USB 0 wakeup, connect, disconnect interrupt */
#define R328_IRQ_USB1       39 /* USB 1 wakeup, connect, disconnect interrupt */
#define R328_IRQ_USB2       40 /* USB 2 wakeup, connect, disconnect interrupt */
#define R328_IRQ_SCR        41 /* SCR interrupt */
#define R328_IRQ_CSI0       42 /* CSI 0 interrupt */
#define R328_IRQ_CSI1       43 /* CSI 1 interrupt */
#define R328_IRQ_LCDC0      44 /* LCD Controller 0 interrupt */
#define R328_IRQ_LCDC1      45 /* LCD Controller 1 interrupt */
#define R328_IRQ_MP         46 /* MP interrupt */
#define R328_IRQ_DEFE0      47 /* DE-FE0 interrupt */
#  define R328_IRQ_DEBE0    47 /* DE-BE0 interrupt */
#define R328_IRQ_DEFE1      48 /* DE-FE1 interrupt */
#  define R328_IRQ_DEBE1    48 /* DE-BE1 interrupt */
#define R328_IRQ_PMU        49 /* PMU interrupt */
#define R328_IRQ_SPI3       50 /* SPI3 interrupt */
#define R328_IRQ_TZASC      51 /* TZASC interrupt */
#define R328_IRQ_PATA       52 /* PATA interrupt */
#define R328_IRQ_VE         53 /* VE interrupt */
#define R328_IRQ_SS         54 /* Security System interrupt */
#define R328_IRQ_EMAC       55 /* EMAC interrupt */
#define R328_IRQ_RESERVED56 56
#define R328_IRQ_RESERVED57 57
#define R328_IRQ_HDMI       58 /* HDMI interrupt */
#define R328_IRQ_TVE        59 /* TV encoder 0/1 interrupt */
#define R328_IRQ_ACE        60 /* ACE interrupt */
#define R328_IRQ_TVD        61 /* TV decoder interrupt */
#define R328_IRQ_PS20       62 /* PS2-0 interrupt */
#define R328_IRQ_PS21       63 /* PS2-1 interrupt */
#define R328_IRQ_USB3       64 /* USB 3 wakeup, connect, disconnect interrupt */
#define R328_IRQ_USB4       65 /* USB 4 wakeup, connect, disconnect interrupt */
#define R328_IRQ_PLE        66 /* PLE interrupts */
#  define R328_IRQ_PERFMU   66 /* Performance monitor interrupt */
#define R328_IRQ_TIMER4     67 /* Timer 4 interrupt */
#define R328_IRQ_TIMER5     68 /* Timer 5 interrupt */
#define R328_IRQ_GPU_GP     69
#define R328_IRQ_GPU_GPMMU  70
#define R328_IRQ_GPU_PP0    71
#define R328_IRQ_GPU_PPMMU0 72
#define R328_IRQ_GPU_PMU    73
#define R328_IRQ_GPU_RSV0   74
#define R328_IRQ_GPU_RSV1   75
#define R328_IRQ_GPU_RSV2   76
#define R328_IRQ_GPU_RSV3   77
#define R328_IRQ_GPU_RSV4   78
#define R328_IRQ_GPU_RSV5   79
#define R328_IRQ_GPU_RSV6   80

/* Total number of interrupts */

#define R328_IRQ_NINT       320

/* Up to 32 external PIO interrupts */

#ifdef CONFIG_R328_PIO_IRQ
#  define R328_PIO_EINT0    (R328_IRQ_NINT+0)
#  define R328_PIO_EINT1    (R328_IRQ_NINT+1)
#  define R328_PIO_EINT2    (R328_IRQ_NINT+2)
#  define R328_PIO_EINT3    (R328_IRQ_NINT+3)
#  define R328_PIO_EINT4    (R328_IRQ_NINT+4)
#  define R328_PIO_EINT5    (R328_IRQ_NINT+5)
#  define R328_PIO_EINT6    (R328_IRQ_NINT+6)
#  define R328_PIO_EINT7    (R328_IRQ_NINT+7)
#  define R328_PIO_EINT8    (R328_IRQ_NINT+8)
#  define R328_PIO_EINT9    (R328_IRQ_NINT+9)
#  define R328_PIO_EINT10   (R328_IRQ_NINT+10)
#  define R328_PIO_EINT11   (R328_IRQ_NINT+11)
#  define R328_PIO_EINT12   (R328_IRQ_NINT+12)
#  define R328_PIO_EINT13   (R328_IRQ_NINT+13)
#  define R328_PIO_EINT14   (R328_IRQ_NINT+14)
#  define R328_PIO_EINT15   (R328_IRQ_NINT+15)
#  define R328_PIO_EINT16   (R328_IRQ_NINT+16)
#  define R328_PIO_EINT17   (R328_IRQ_NINT+17)
#  define R328_PIO_EINT18   (R328_IRQ_NINT+18)
#  define R328_PIO_EINT19   (R328_IRQ_NINT+19)
#  define R328_PIO_EINT20   (R328_IRQ_NINT+20)
#  define R328_PIO_EINT21   (R328_IRQ_NINT+21)
#  define R328_PIO_EINT22   (R328_IRQ_NINT+22)
#  define R328_PIO_EINT23   (R328_IRQ_NINT+23)
#  define R328_PIO_EINT24   (R328_IRQ_NINT+24)
#  define R328_PIO_EINT25   (R328_IRQ_NINT+25)
#  define R328_PIO_EINT26   (R328_IRQ_NINT+26)
#  define R328_PIO_EINT27   (R328_IRQ_NINT+27)
#  define R328_PIO_EINT28   (R328_IRQ_NINT+28)
#  define R328_PIO_EINT29   (R328_IRQ_NINT+29)
#  define R328_PIO_EINT30   (R328_IRQ_NINT+30)
#  define R328_PIO_EINT31   (R328_IRQ_NINT+31)
#  define R328_PIO_NINT     32
#else
#  define R328_PIO_NINT     0
#endif

/* Total number of IRQ numbers */

#define NR_IRQS            (R328_IRQ_NINT + R328_PIO_NINT)

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Inline functions
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_r328_R328_IRQ_H */

