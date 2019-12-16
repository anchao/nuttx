/*
 * The interface function of controlling the CE register.
 *
 * Copyright (C) 2013 Allwinner.
 *
 * Mintow <duanmintao@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <stdio.h>
#include <string.h>
#include "aw_types.h"
//#include "io.h"
#include "ce_common.h"
#include "ce_reg.h"

#define readb(addr)             (*((volatile unsigned char  *)(addr)))
#define readw(addr)             (*((volatile unsigned short *)(addr)))
#define readl(addr)             (*((volatile unsigned long  *)(addr)))
#define writeb(v, addr) 	(*((volatile unsigned char  *)(addr)) = (unsigned char)(v))
#define writew(v, addr) 	(*((volatile unsigned short *)(addr)) = (unsigned short)(v))
#define writel(v, addr) 	(*((volatile unsigned long  *)(addr)) = (unsigned long)(v))

#define SUNXI_CCM_BASE						(0x03001000)
#define CCMU_CE_CLK_REG						(SUNXI_CCM_BASE + 0x680)
#define CCMU_CE_BGR_REG						(SUNXI_CCM_BASE + 0x68C)
#define MBUS_MAT_CLK_GATING_REG				(SUNXI_CCM_BASE + 0x804)

#define CE_CLK_SRC_MASK						(0x1)
#define CE_CLK_SRC_SEL_BIT					(24)
#define CE_CLK_SRC							(0x01)
#define CE_CLK_DIV_RATION_N_BIT				(8)
#define CE_CLK_DIV_RATION_N_MASK			(0x3)
#define CE_CLK_DIV_RATION_N					(0)

#define CE_CLK_DIV_RATION_M_BIT				(0)
#define CE_CLK_DIV_RATION_M_MASK			(0xF)
#define CE_CLK_DIV_RATION_M					(3)

#define CE_SCLK_ONOFF_BIT					(31)
#define CE_SCLK_ON							(1)

#define CE_GATING_PASS						(1)
#define CE_GATING_BIT						(0)
#define CE_RST_BIT							(16)
#define CE_DEASSERT							(1)


void ce_clock_init(void)
{
	u32  reg_val;

	reg_val = readl(CCMU_CE_CLK_REG);

	/*set div n*/
	reg_val &= ~(CE_CLK_DIV_RATION_N_MASK << CE_CLK_DIV_RATION_N_BIT);
	reg_val |= CE_CLK_DIV_RATION_N << CE_CLK_DIV_RATION_N_BIT;

	/*set div m*/
	reg_val &= ~(CE_CLK_DIV_RATION_M_MASK << CE_CLK_DIV_RATION_M_BIT);
	reg_val |= CE_CLK_DIV_RATION_M << CE_CLK_DIV_RATION_M_BIT;

	writel(reg_val, CCMU_CE_CLK_REG);

	/*set CE src clock*/
	reg_val &= ~(CE_CLK_SRC_MASK << CE_CLK_SRC_SEL_BIT);

	/* PLL_PERI0(2X) */
	reg_val |= CE_CLK_SRC << CE_CLK_SRC_SEL_BIT;

	/*set src clock on*/
	reg_val |= CE_SCLK_ON << CE_SCLK_ONOFF_BIT;

	writel(reg_val, CCMU_CE_CLK_REG);

	/*open CE gating*/
	reg_val = readl(CCMU_CE_BGR_REG);
	reg_val |= CE_GATING_PASS << CE_GATING_BIT;
	writel(reg_val, CCMU_CE_BGR_REG);

	/*de-assert*/
	reg_val = readl(CCMU_CE_BGR_REG);
	reg_val |= CE_DEASSERT << CE_RST_BIT;
	writel(reg_val, CCMU_CE_BGR_REG);

	/*set mbus clock gating*/
	reg_val = readl(MBUS_MAT_CLK_GATING_REG);
	reg_val |= 1 << 2;
	writel(reg_val, MBUS_MAT_CLK_GATING_REG);
}


u32 ce_readl(u32 offset)
{
	return readl(CE_S_BASE_REG + offset);
}

static void ce_writel(u32 offset, u32 val)
{
	writel(val, CE_S_BASE_REG + offset);
}

u32 ce_reg_rd(u32 offset)
{
	return ce_readl(offset);
}

void ce_reg_wr(u32 offset, u32 val)
{
	ce_writel(offset, val);
}

void ce_keyselect_set(int select, ce_task_desc_t *task)
{
	task->sym_ctl |= select << CE_SYM_CTL_KEY_SELECT_SHIFT;
}

void ce_keysize_set(int size, ce_task_desc_t *task)
{
	int type = CE_AES_KEY_SIZE_128;

	switch (size) {
	case AES_KEYSIZE_192:
		type = CE_AES_KEY_SIZE_192;
		break;
	case AES_KEYSIZE_256:
		type = CE_AES_KEY_SIZE_256;
		break;
	default:
		break;
	}

	task->sym_ctl |= (type << CE_SYM_CTL_KEY_SIZE_SHIFT);
}

void ce_rsa_width_set(int size, ce_task_desc_t *task)
{
	int width_type = 0;

	switch (size) {
	case 512:
		width_type = CE_RSA_PUB_MODULUS_WIDTH_512;
		break;
	case 1024:
		width_type = CE_RSA_PUB_MODULUS_WIDTH_1024;
		break;
	case 2048:
		width_type = CE_RSA_PUB_MODULUS_WIDTH_2048;
		break;
	case 3072:
		width_type = CE_RSA_PUB_MODULUS_WIDTH_3072;
		break;
	case 4096:
		width_type = CE_RSA_PUB_MODULUS_WIDTH_4096;
		break;
	default:
		break;
	}

	task->asym_ctl |= width_type << CE_ASYM_CTL_RSA_PM_WIDTH_SHIFT;

}

/* key: phsical address. */
void ce_key_set(char *key, int size, ce_task_desc_t *task)
{
	int i = 0;
	int key_sel = CE_KEY_SELECT_INPUT;
	struct {
		int type;
		char desc[AES_MIN_KEY_SIZE];
	} keys[] = {
		{CE_KEY_SELECT_SSK,			CE_KS_SSK},
		{CE_KEY_SELECT_HUK,			CE_KS_HUK},
		{CE_KEY_SELECT_RSSK,		CE_KS_RSSK},
		{CE_KEY_SELECT_INTERNAL_0, CE_KS_INTERNAL_0},
		{CE_KEY_SELECT_INTERNAL_1, CE_KS_INTERNAL_1},
		{CE_KEY_SELECT_INTERNAL_2, CE_KS_INTERNAL_2},
		{CE_KEY_SELECT_INTERNAL_3, CE_KS_INTERNAL_3},
		{CE_KEY_SELECT_INTERNAL_4, CE_KS_INTERNAL_4},
		{CE_KEY_SELECT_INTERNAL_5, CE_KS_INTERNAL_5},
		{CE_KEY_SELECT_INTERNAL_6, CE_KS_INTERNAL_6},
		{CE_KEY_SELECT_INTERNAL_7, CE_KS_INTERNAL_7},
		{CE_KEY_SELECT_INPUT, ""} };

	while (keys[i].type != CE_KEY_SELECT_INPUT) {
		if (strncasecmp(key, keys[i].desc, AES_MIN_KEY_SIZE) == 0) {
			key_sel = keys[i].type;
			memset(key, 0, size);
			break;
		}
		i++;
	}
	CE_DBG("The key select: %d\n", key_sel);

	ce_keyselect_set(key_sel, task);
	ce_keysize_set(size, task);
	task->key_addr = (u32)key;
}

void ce_pending_clear(int flow)
{
	int val = CE_CHAN_PENDING << flow;

	ce_writel(CE_REG_ISR, val);
}

int ce_pending_get(void)
{
	return ce_readl(CE_REG_ISR);
}

void ce_irq_enable(int flow)
{
	int val = ce_readl(CE_REG_ICR);

	val |= CE_CHAN_INT_ENABLE << flow;
	ce_writel(CE_REG_ICR, val);
}

void ce_irq_disable(int flow)
{
	int val = ce_readl(CE_REG_ICR);

	val &= ~(CE_CHAN_INT_ENABLE << flow);
	ce_writel(CE_REG_ICR, val);
}

void ce_md_get(char *dst, char *src, int size)
{
	memcpy(dst, src, size);
}


void ce_iv_set(char *iv, int size, ce_task_desc_t *task)
{
	task->iv_addr = (u32)iv;
}

void ce_iv_mode_set(int mode, ce_task_desc_t *task)
{
	task->comm_ctl |= mode << CE_COMM_CTL_IV_MODE_SHIFT;
}

void ce_cntsize_set(int size, ce_task_desc_t *task)
{
	task->sym_ctl |= size << CE_SYM_CTL_CTR_SIZE_SHIFT;
}

void ce_cnt_set(char *cnt, int size, ce_task_desc_t *task)
{
	task->ctr_addr = (u32)cnt;
	ce_cntsize_set(CE_CTR_SIZE_128, task);
}

void ce_cts_last(ce_task_desc_t *task)
{
	task->sym_ctl |= CE_SYM_CTL_AES_CTS_LAST;
}

#ifndef CE_SUPPORT_CE_V3_1

void ce_xts_first(ce_task_desc_t *task)
{
	task->sym_ctl |= CE_SYM_CTL_AES_XTS_FIRST;
}

void ce_xts_last(ce_task_desc_t *task)
{
	task->sym_ctl |= CE_SYM_CTL_AES_XTS_LAST;
}

#endif


void ce_method_set(int dir, int type, ce_task_desc_t *task)
{
	task->comm_ctl |= dir << CE_COMM_CTL_OP_DIR_SHIFT;
	task->comm_ctl |= type << CE_COMM_CTL_METHOD_SHIFT;
}

void ce_aes_mode_set(int mode, ce_task_desc_t *task)
{
	task->sym_ctl |= mode << CE_SYM_CTL_OP_MODE_SHIFT;
}

void ce_cfb_bitwidth_set(int bitwidth, ce_task_desc_t *task)
{
	int val = 0;

	switch (bitwidth) {
	case 1:
		val = CE_CFB_WIDTH_1;
		break;
	case 8:
		val = CE_CFB_WIDTH_8;
		break;
	case 64:
		val = CE_CFB_WIDTH_64;
		break;
	case 128:
		val = CE_CFB_WIDTH_128;
		break;
	default:
		break;
	}
	task->sym_ctl |= val << CE_SYM_CTL_CFB_WIDTH_SHIFT;
}

void ce_set_tsk(u32 task_addr)
{
	ce_writel(CE_REG_TSK, task_addr);
}

void ce_ctrl_start(void)
{
	u32 val = ce_readl(CE_REG_TLR);
	val = val | (0x1 << 0);
	ce_writel(CE_REG_TLR, val);
}

int ce_flow_err(int flow)
{
	return ce_readl(CE_REG_ERR) & CE_REG_ESR_CHAN_MASK(flow);
}


void ce_data_len_set(int len, ce_task_desc_t *task)
{
	task->data_len = len;
}

void ce_wait_finish(u32 flow)
{
	u32 int_en;
	int_en = ce_readl(CE_REG_ICR) & 0xf;
	int_en = int_en & (0x01 << flow);
	if (int_en != 0) {
		while ((ce_readl(CE_REG_ISR) & (0x01 << flow)) == 0) {
			;
		}
	}
}

void ce_reg_printf(void)
{
	CE_ERR("The ce control register:\n");
	CE_ERR("[TSK] = 0x%08x\n", ce_readl(CE_REG_TSK));
	CE_ERR("[CTL] = 0x%08x\n", ce_readl(CE_REG_CTL));
	CE_ERR("[ICR] = 0x%08x\n", ce_readl(CE_REG_ICR));
	CE_ERR("[ISR] = 0x%08x\n", ce_readl(CE_REG_ISR));
	CE_ERR("[TLR] = 0x%08x\n", ce_readl(CE_REG_TLR));
	CE_ERR("[TSR] = 0x%08x\n", ce_readl(CE_REG_TSR));
	CE_ERR("[ERR] = 0x%08x\n", ce_readl(CE_REG_ERR));
	CE_ERR("[CSA] = 0x%08x\n", ce_readl(CE_REG_CSA));
	CE_ERR("[CDA] = 0x%08x\n", ce_readl(CE_REG_CDA));
	CE_ERR("[VER] = 0x%08x\n", ce_readl(CE_REG_VER));
}
