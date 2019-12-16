/* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.

 * Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
 * the the People's Republic of China and other countries.
 * All Allwinner Technology Co.,Ltd. trademarks are used with permission.

 * DISCLAIMER
 * THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
 * IF YOU NEED TO INTEGRATE THIRD PARTY’S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
 * IN ALLWINNERS’SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
 * ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
 * ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
 * COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
 * YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY’S TECHNOLOGY.


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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <irq.h>
#include <interrupt.h>
#include <nuttx/spinlock.h>
#include "ce_reg.h"
#include "ce_common.h"
#include "hal_aes.h"
#include <nuttx/mm/mm.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/cache.h>

#define IRQ_DONE		(0x5A5A)
#define CE_WAIT_TIME	(500000)

u32 irq_done;
extern unsigned int mdelay(unsigned int counter);

void ce_print_hex(char *_data, int _len, void *_addr)
{
	int i;

	CE_DBG("---------------- The valid len = %d ----------------\n",
		_len);
	for (i = 0; i < _len/8; i++) {
		CE_DBG("0x%p: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			i*8 + _addr,
			_data[i*8+0], _data[i*8+1], _data[i*8+2], _data[i*8+3],
			_data[i*8+4], _data[i*8+5], _data[i*8+6], _data[i*8+7]);
	}
	CE_DBG("----------------------------------------------------\n");
}

void ce_print_task_info(ce_task_desc_t *task)
{
	CE_DBG("-----------task_info------\n");
	CE_DBG("task = 0x%x\n", (uint32_t)task);
	CE_DBG("task->comm_ctl = 0x%x\n", task->comm_ctl);
	CE_DBG("task->sym_ctl = 0x%x\n", task->sym_ctl);
	CE_DBG("task->asym_ctl = 0x%x\n", task->asym_ctl);
	CE_DBG("task->chan_id = 0x%x\n", task->chan_id);
	CE_DBG("task->ctr_addr = 0x%x\n", task->ctr_addr);
	CE_DBG("task->data_len = 0x%x\n", task->data_len);
	CE_DBG("task->iv_addr = 0x%x\n", task->iv_addr);
	CE_DBG("task->key_addr = 0x%x\n", task->key_addr);
	CE_DBG("task->src[0].addr = 0x%x\n", task->src[0].addr);
	CE_DBG("task->src[0].len = 0x%x\n", task->src[0].len);
	CE_DBG("task->dst[0].addr = 0x%x\n", task->dst[0].addr);
	CE_DBG("task->dst[0].len = 0x%x\n", task->dst[0].len);
}


int wait_for_completion_timeout(s32 time)
{
	s32 counter = 0;

	while(counter <= time) {
		if (irq_done == IRQ_DONE) {
			return 0;
		}
		mdelay(1);
		counter ++;
	}
	return -1;
}

static int ce_irq_handler(int irq, FAR void *context, FAR void *arg)
{
	int i;
	int pending = 0;
	irqstate_t flags = 0;

	flags = spin_lock_irqsave();

	pending = ce_pending_get();
	CE_DBG("pending: %#x\n", pending);
	for (i = 0; i < CE_FLOW_NUM; i++) {
		if (pending & (CE_CHAN_PENDING << i)) {
			CE_DBG("Chan %d completed. pending: %#x\n", i, pending);
			ce_pending_clear(i);
			irq_done = IRQ_DONE;
		}
	}

	spin_unlock_irqrestore(flags);

	return OK;
}

static int ce_irq_request(void)
{
//	u32 irqn = R328_IRQ_SS;
	u32 irqn = 98;

	if (irq_attach(irqn, ce_irq_handler, NULL) < 0) {
		CE_ERR("Cannot request IRQ\n");
		return -1;
	}

	up_enable_irq(irqn);

	return 0;
}

int sunxi_ce_init(void)
{
	int ret = 0;

	ce_clock_init();
	ret = ce_irq_request();
	if (ret < 0) {
		return -1;
	}
	irq_done = 0x0;
	return 0;
}

static void ce_task_desc_init(ce_task_desc_t *task, u32 flow)
{
	memset((void *)task, 0x0, sizeof(ce_task_desc_t));
	task->chan_id = flow;
	task->comm_ctl |= CE_COMM_CTL_TASK_INT_MASK;
}

static int aes_crypt_start(u8 *src_buf,
                          u32 src_length,
                          u8 *dest_buf,
                          u32 dest_length,
                          ce_task_desc_t *task)
{
	int ret = 0;
	int data_word_len = 0;

	ce_pending_clear(task->chan_id);

	if ((src_length & 0x3) == 0) {
		data_word_len = src_length >> 2;
	} else {
		data_word_len = (src_length >> 2) + 1;
	}

	ce_data_len_set((src_length >> 2), task);

	task->src[0].addr = (u32)src_buf;
	task->src[0].len = data_word_len;

	task->dst[0].addr = (u32)dest_buf;
	task->dst[0].len = data_word_len;
	task->next = 0;
//	FlushCacheAll();
	cpu_icache_invalidate_all();
	cpu_dcache_clean_all();
	//ce_print_task_info(task);
	ce_set_tsk((u32)task);
	ce_irq_enable(task->chan_id);
	ce_ctrl_start();

#ifdef CE_NO_IRQ
	ce_wait_finish(task->chan_id);
#else
	ret = wait_for_completion_timeout(CE_WAIT_TIME);
	if (ret != 0) {
		CE_ERR("Timed out\n");
		ret = AES_TIME_OUT;
	}
#endif
	//ce_print_hex((char *)task->dst[0].addr, (task->dst[0].len * 4), (char *)task->dst[0].addr);
	/*ce_reg_printf();*/

	ce_irq_disable(task->chan_id);



	return AES_STATUS_OK;
}

static ce_task_desc_t *ce_aes_config(u8 dir, u8 type, u8 mode, u8 *key_buf, u32 key_length)
{
	ce_task_desc_t *task = NULL;
	u32 flow = 0;

	task = (ce_task_desc_t *)kmm_memalign(CE_ALIGN_SIZE, sizeof(ce_task_desc_t));
	if (task == NULL) {
		CE_ERR("kmm_memalign fail\n");
		return NULL;
	}
	CE_DBG("task addr = 0x%x\n", (u32)task);
	ce_task_desc_init(task, flow);
	ce_method_set(dir, type, task);
	ce_aes_mode_set(mode, task);
	ce_key_set(key_buf, key_length, task);

	return task;
}

int do_aes_crypt(hal_aes_buffer_t *src_text,
				hal_aes_buffer_t *dest_text,
				hal_aes_buffer_t *key,
				u8 *init_vector,
				u8 mode,
				u8 dir)

{
	u32 last_block_size = 0;
	u32 block_num = 0;
	u32 padding_size = 0;
	u32 first_encypt_size = 0;
	u8 data_block[AES_BLOCK_SIZE] = {0};
	u8 *iv;
	u8 *init_vector2;
	ce_task_desc_t *task;


	if ((src_text == NULL)
			|| (dest_text == NULL)
			|| (key == NULL)
			|| (init_vector == NULL)) {
			CE_ERR("input is NULL\n");
			return AES_INPUT_ERROR;
    }

	if ((key->length != AES_KEYSIZE_128)
			&& (key->length != AES_KEYSIZE_192)
			&& (key->length != AES_KEYSIZE_256)) {
			CE_ERR("key length is %d, invalid\n", key->length);
			return AES_INPUT_ERROR;
    }

	if ((((u32)src_text->buffer & (CE_ALIGN_SIZE -1 )) != 0)
			|| (((u32)dest_text->buffer & (CE_ALIGN_SIZE - 1 )) != 0)
			|| (((u32)key->buffer & (CE_ALIGN_SIZE - 1)) != 0)) {
			CE_ERR("input buffer is not %d align\n", CE_ALIGN_SIZE);
			return AES_INPUT_ERROR;
	}

	/*ce config*/
	task = ce_aes_config(dir, CE_METHOD_AES, mode, key->buffer, key->length);
	if (task == NULL) {
		return AES_INPUT_ERROR;
	}

	if (dir == CRYPT_DIR_DECRYPT) {
		CE_ERR("CRYPT_DIR_DECRYPT = %d\n", dir);
		if ((src_text->length % AES_BLOCK_SIZE) != 0) {
			CE_ERR("encrypted_text length is %d, invalid\n", src_text->length);
			return AES_INPUT_ERROR;
		}
		ce_iv_set(init_vector, 0, task);
		if (AES_STATUS_OK != aes_crypt_start(src_text->buffer,
												src_text->length,
												dest_text->buffer,
												dest_text->length,
												task)) {
			CE_ERR("aes decrypt fail\n");
			kmm_free(task);
			return AES_CRYPTO_ERROR;
		}

		dest_text->length = src_text->length;
		kmm_free(task);
		return AES_STATUS_OK;

	} else {
		block_num = src_text->length / AES_BLOCK_SIZE;
		last_block_size = src_text->length % AES_BLOCK_SIZE;
		padding_size = AES_BLOCK_SIZE - last_block_size;

		if (block_num > 0) {
			CE_DBG("block_num = %d\n", block_num);
			first_encypt_size = block_num * AES_BLOCK_SIZE;
			ce_iv_set(init_vector, 0, task);
			if (AES_STATUS_OK != aes_crypt_start(src_text->buffer,
												first_encypt_size,
												dest_text->buffer,
												dest_text->length,
												task)) {
				CE_ERR("do_aes_encrypt fail.\n");
				kmm_free(task);
				return AES_CRYPTO_ERROR;
			 }
			dest_text->length = block_num * AES_BLOCK_SIZE;

			if (last_block_size) {
				CE_DBG("last_block_size = %d\n", last_block_size);
				CE_DBG("padding_size = %d\n", padding_size);
				memcpy(data_block, src_text->buffer + first_encypt_size, last_block_size);
				memset(data_block + last_block_size, padding_size, padding_size);
				if (AES_TYPE_CBC == mode) {
					init_vector2 = dest_text->buffer + first_encypt_size - AES_BLOCK_SIZE;
					iv = init_vector2;
				} else {
					iv = init_vector;
				}
				ce_iv_set(iv, 0, task);

				if (HAL_AES_STATUS_OK != aes_crypt_start(data_block,
														AES_BLOCK_SIZE,
														dest_text->buffer + first_encypt_size,
														AES_BLOCK_SIZE,
														task)) {
					CE_ERR("do_aes_encrypt fail\n");
					kmm_free(task);
					return AES_CRYPTO_ERROR;
				}
				dest_text->length = dest_text->length + AES_BLOCK_SIZE;
			}
		} else {
			CE_DBG("padding_size = %d\n", padding_size);
			memcpy(data_block, src_text->buffer, src_text->length);
			memset(data_block + last_block_size, padding_size, padding_size);
			ce_iv_set(init_vector, 0, task);
			if (HAL_AES_STATUS_OK != aes_crypt_start(data_block,
													AES_BLOCK_SIZE,
													dest_text->buffer,
													AES_BLOCK_SIZE,
													task)) {
				CE_ERR("do_aes_encrypt fail\n");
				return AES_CRYPTO_ERROR;
			}
			dest_text->length = (block_num + 1) * AES_BLOCK_SIZE;
		}

		CE_DBG("dest_text->length = %d\n", dest_text->length);
		kmm_free(task);
		return AES_STATUS_OK;
	}
}

static uint32_t aw_endian4(uint32_t data)
{
	uint32_t d1, d2, d3, d4;

	d1 = (data & 0xff) << 24;
	d2 = (data & 0xff00) << 8;
	d3 = (data & 0xff0000) >> 8;
	d4 = (data & 0xff000000) >> 24;

	return (d1 | d2 | d3 | d4);
}

//SHA1/MD5/SHA224/SHA256/SHA384/SHA512 padding
static uint32_t hash_padding(uint32_t hash_mode, uint64_t data_size, uint8_t* text)
{
	uint32_t i;
	uint32_t k, q;
	uint32_t size;
	uint32_t padding_buf[32];
    uint8_t *ptext;

	if (hash_mode < 4) {
		k = data_size / 64;
		q = data_size % 64;

		ptext = (uint8_t*)padding_buf;
		if (q == 0) {
			for (i = 0; i < 16; i++)
				padding_buf[i] = 0x0;
			padding_buf[0] = 0x00000080;
			if (hash_mode == 0) {
				padding_buf[14] = data_size << 3;
				padding_buf[15] = data_size >> 29;
			} else {
				padding_buf[14] = data_size >> 29;
				padding_buf[15] = data_size << 3;
				padding_buf[14] = aw_endian4(padding_buf[14]);
				padding_buf[15] = aw_endian4(padding_buf[15]);
			}
			for (i = 0; i < 64; i++)
				text[k * 64 + i] = ptext[i];
			size = (k + 1) * 64;
		} else if (q < 56) {
			for (i = 0; i < 16; i++)
				padding_buf[i] = 0x0;
			for (i = 0; i < q; i++)
				ptext[i] = text[k * 64 + i];
			ptext[q] = 0x80;
			if (hash_mode == 0) {
				padding_buf[14] = data_size << 3;
				padding_buf[15] = data_size >> 29;
			} else {
				padding_buf[14] = data_size >> 29;
				padding_buf[15] = data_size << 3;
				padding_buf[14] = aw_endian4(padding_buf[14]);
				padding_buf[15] = aw_endian4(padding_buf[15]);
			}
			for (i = 0; i < 64; i++)
				text[k * 64 + i] = ptext[i];
			size = (k + 1) * 64;
		} else {
			for (i = 0; i < 16; i++)
				padding_buf[i] = 0x0;
				for (i = 0; i < q; i++)
					ptext[i] = text[k * 64 + i];
				ptext[q] = 0x80;
				for(i = 0; i < 64; i++)
					text[k * 64 + i] = ptext[i];

				//send last 512-bits text to SHA1/MD5
				for (i = 0; i < 16; i++)
					padding_buf[i] = 0x0;
				if (hash_mode == 0) {
					padding_buf[14] = data_size << 3;
					padding_buf[15] = data_size >> 29;
				} else {
					padding_buf[14] = data_size >> 29;
					padding_buf[15] = data_size << 3;
					padding_buf[14] = aw_endian4(padding_buf[14]);
					padding_buf[15] = aw_endian4(padding_buf[15]);
				}
				for (i = 0; i < 64; i++)
					text[(k + 1) * 64 + i] = ptext[i];
				size = (k + 2) * 64;
		}
	} else {
		k = data_size / 128;
		q = data_size % 128;
		ptext = (uint8_t *)padding_buf;
		if (q == 0) {
			for (i = 0; i < 32; i++)
				padding_buf[i] = 0x0;

			padding_buf[0] = 0x00000080;
			padding_buf[29] = data_size >> 61;
			padding_buf[30] = data_size >> 29;
			padding_buf[31] = data_size << 3;
			padding_buf[29] = aw_endian4(padding_buf[29]);
			padding_buf[30] = aw_endian4(padding_buf[30]);
			padding_buf[31] = aw_endian4(padding_buf[31]);

			for (i = 0; i < 128; i++)
					text[k * 128 + i] = ptext[i];
			size = (k + 1) * 128;
		 } else if (q <= 112) {
			for (i = 0; i < 32; i++)
				padding_buf[i] = 0x0;
			for (i = 0; i < q; i++)
					ptext[i] = text[k * 128 + i];
			ptext[q] = 0x80;
			padding_buf[29] = data_size >> 61;
			padding_buf[30] = data_size >> 29;
			padding_buf[31] = data_size << 3;
			padding_buf[29] = aw_endian4(padding_buf[29]);
			padding_buf[30] = aw_endian4(padding_buf[30]);
			padding_buf[31] = aw_endian4(padding_buf[31]);

			for (i = 0; i < 128; i++)
				text[k * 128 + i] = ptext[i];
			size = (k + 1) * 128;
		} else {
			for (i = 0; i < 32; i++)
				padding_buf[i] = 0x0;
			for (i = 0; i < q; i++)
				ptext[i] = text[k * 128 + i];
				ptext[q] = 0x80;
			for (i = 0; i < 128; i++)
				text[k * 128 + i] = ptext[i];

			/*send last 1024-bits text to SHA384/512*/
			for (i = 0; i < 32; i++)
				padding_buf[i] = 0x0;
			padding_buf[29] = data_size >> 61;
			padding_buf[30] = data_size >> 29;
			padding_buf[31] = data_size << 3;
			padding_buf[29] = aw_endian4(padding_buf[29]);
			padding_buf[30] = aw_endian4(padding_buf[30]);
			padding_buf[31] = aw_endian4(padding_buf[31]);

			for (i = 0; i < 128; i++)
				text[(k + 1) * 128 + i] = ptext[i];
			size = (k + 2) * 128;
		}
	}
	return size;
}

int ce_hash_start(crypto_hash_req_ctx *req_ctx)
{
	int ret = 0;
	uint8_t chan_id = 1;
	ce_task_desc_t *task = NULL;
	uint32_t src_word_len = 0;

	src_word_len = req_ctx->src_length >> 2;

	task = (ce_task_desc_t *)kmm_memalign(CE_ALIGN_SIZE, sizeof(ce_task_desc_t));
	if (task == NULL) {
		CE_ERR("kmm_memalign fail\n");
		return -1;
	}
	CE_DBG("task addr = 0x%x\n", (uint32_t)task);

	ce_task_desc_init(task, chan_id);
	ce_pending_clear(chan_id);
	ce_method_set(req_ctx->dir, req_ctx->type, task);
	ce_data_len_set(src_word_len, task);
	if (req_ctx->md_size) {
		ce_iv_set(req_ctx->md, req_ctx->md_size, task);
		ce_iv_mode_set(CE_HASH_IV_INPUT, task);
	}

	task->src[0].addr = (uint32_t)req_ctx->src_buffer;
	task->src[0].len = src_word_len;

	task->dst[0].addr = (uint32_t)req_ctx->dst_buffer;
	task->dst[0].len = req_ctx->dst_length >> 2;
	task->next = 0;
	//FlushCacheAll();
	cpu_icache_invalidate_all();
	cpu_dcache_clean_all();
	//ce_print_task_info(task);
	ce_set_tsk((uint32_t)task);
	ce_irq_enable(task->chan_id);
	ce_ctrl_start();

#ifdef CE_NO_IRQ
	ce_wait_finish(task->chan_id);
#else
	ret = wait_for_completion_timeout(CE_WAIT_TIME);
	if (ret != 0) {
		CE_ERR("Timed out\n");
		kmm_free(task);
		ret = SHA_TIME_OUT;
	}
#endif
	//ce_print_hex((char *)task->dst[0].addr, (task->dst[0].len * 4), (char *)task->dst[0].addr);
	/*ce_reg_printf();*/

	ce_irq_disable(task->chan_id);
	memcpy(req_ctx->md, req_ctx->dst_buffer, req_ctx->dst_length);
	req_ctx->md_size = req_ctx->dst_length;
	kmm_free(task);
	return SHA_STATUS_OK;

}

int do_hash_crypt(crypto_hash_req_ctx *req_ctx)
{
	uint8_t	*src_tmp = NULL;
	uint32_t src_align_len = 0;
	int ret = 0;

	if (req_ctx == NULL) {
			CE_ERR("sha req_ctx is NULL\n");
			return SHA_INPUT_ERROR;
    }

	if (((u32)req_ctx->dst_buffer & (CE_ALIGN_SIZE - 1)) != 0) {
			printf("digest buffer is not %d align\n", CE_ALIGN_SIZE);
			return SHA_INPUT_ERROR;
	}

	if (req_ctx->type == CE_METHOD_MD5) {
		req_ctx->padding_mode = MD5_PADDING_MODE;
	} else if (req_ctx->type == CE_METHOD_SHA1) {
		req_ctx->padding_mode = SHA1_PADDING_MODE;
	} else if (req_ctx->type == CE_METHOD_SHA224) {
		req_ctx->padding_mode = SHA224_PADDING_MODE;
	} else if (req_ctx->type == CE_METHOD_SHA256) {
		req_ctx->padding_mode = SHA256_PADDING_MODE;
	} else if (req_ctx->type == CE_METHOD_SHA384) {
		req_ctx->padding_mode = SHA384_PADDING_MODE;
	} else if (req_ctx->type == CE_METHOD_SHA512) {
		req_ctx->padding_mode = SHA512_PADDING_MODE;
	} else {
		CE_ERR("ce don't support hash mode\n");
		return SHA_INPUT_ERROR;
	}

	src_tmp = (u8 *)kmm_memalign(CE_ALIGN_SIZE, req_ctx->src_length + 2*CE_ALIGN_SIZE);
	if (src_tmp == NULL) {
		printf (" malloc src_tmp buffer fail\n");
		return SHA_MALLOC_ERROR;
	}
	memset(src_tmp, 0x0, (req_ctx->src_length + CE_ALIGN_SIZE));
	memcpy(src_tmp, req_ctx->src_buffer, req_ctx->src_length);

	/*ce hash padding*/
	src_align_len = hash_padding(req_ctx->padding_mode, req_ctx->src_length, src_tmp);
	req_ctx->src_length = src_align_len;
	req_ctx->src_buffer = src_tmp;

	ret = ce_hash_start(req_ctx);
	if (ret < 0) {
		CE_ERR("caclu hash erro num is %d\n", ret);
		kmm_free(src_tmp);
		return SHA_CRYPTO_ERROR;
	}

	kmm_free(src_tmp);
	return SHA_STATUS_OK;
}

static void __rsa_padding(uint8_t *dst_buf, uint8_t *src_buf, uint32_t data_len, uint32_t group_len)
{
	int i = 0;

	memset(dst_buf, 0, group_len);
	for (i = group_len - data_len; i < group_len; i++) {
		dst_buf[i] = src_buf[group_len - 1 - i];
	}
}

int ce_rsa_start(crypto_rsa_req_ctx *req_ctx)
{
	int ret = 0;
	uint8_t chan_id = 2;
	uint8_t *p_dst = NULL;
	ce_task_desc_t *task = NULL;
	uint32_t data_word_len = 0;

	data_word_len = req_ctx->bitwidth >> 5;

	p_dst = kmm_memalign(CE_ALIGN_SIZE, data_word_len);
	if (p_dst == NULL) {
		CE_ERR("kmm_memalign fail\n");
		return RSA_MALLOC_ERROR;
	}
	memset(p_dst, 0x0, data_word_len);

	task = (ce_task_desc_t *)kmm_memalign(CE_ALIGN_SIZE, sizeof(ce_task_desc_t));
	if (task == NULL) {
		CE_ERR("kmm_memalign fail\n");
		return -1;
	}
	CE_DBG("task addr = 0x%x\n", (uint32_t)task);

	ce_task_desc_init(task, chan_id);
	ce_pending_clear(chan_id);
	ce_method_set(req_ctx->dir, req_ctx->type, task);
	ce_rsa_width_set(req_ctx->bitwidth, task);
	task->iv_addr = (uint32_t) req_ctx->key_n;
	task->key_addr = (uint32_t) req_ctx->key_e;
	ce_data_len_set(data_word_len, task);

	task->src[0].addr = (uint32_t)req_ctx->src_buffer;
	task->src[0].len = data_word_len;

	task->dst[0].addr = (uint32_t)p_dst;
	task->dst[0].len = data_word_len;
	task->next = 0;
//	FlushCacheAll();
	cpu_icache_invalidate_all();
	cpu_dcache_clean_all();
	/*ce_print_task_info(task);*/
	ce_set_tsk((uint32_t)task);
	ce_irq_enable(task->chan_id);
	ce_ctrl_start();

#ifdef CE_NO_IRQ
	ce_wait_finish(task->chan_id);
#else
	ret = wait_for_completion_timeout(CE_WAIT_TIME);
	if (ret != 0) {
		CE_ERR("Timed out\n");
		kmm_free(task);
		ret = RSA_TIME_OUT;
	}
#endif
	/*ce_reg_printf();*/

	ce_irq_disable(task->chan_id);
	__rsa_padding(req_ctx->dst_buffer, p_dst, req_ctx->dst_length, req_ctx->dst_length);

	kmm_free(p_dst);
	kmm_free(task);
	return RSA_STATUS_OK;

}

int do_rsa_crypt(crypto_rsa_req_ctx *req_ctx)
{
	int ret = 0;
	uint8_t	*p_src = NULL;
	uint8_t	*p_n = NULL;
	uint32_t tmp_len = 0;
	uint32_t bitwidth_byte_len = 0;

	if (req_ctx == NULL) {
			CE_ERR("rsa req_ctx is NULL\n");
			return RSA_INPUT_ERROR;
    }

	if ((((u32)req_ctx->key_n & (CE_ALIGN_SIZE - 1)) != 0)
		&& (((u32)req_ctx->key_e & (CE_ALIGN_SIZE - 1)) != 0)
		&& (((u32)req_ctx->src_buffer & (CE_ALIGN_SIZE - 1)) != 0)
		&& (((u32)req_ctx->dst_buffer & (CE_ALIGN_SIZE - 1)) != 0)) {
			printf("rsa req_ctx buffer is not %d align\n", CE_ALIGN_SIZE);
			return RSA_INPUT_ERROR;
	}

	bitwidth_byte_len = req_ctx->bitwidth >> 3;
	p_src = kmm_memalign(CE_ALIGN_SIZE, bitwidth_byte_len);
	if (p_src == NULL) {
		CE_ERR("kmm_memalign fail\n");
		return RSA_MALLOC_ERROR;
	}
	memset(p_src, 0x0, bitwidth_byte_len);
	__rsa_padding(p_src, req_ctx->src_buffer, req_ctx->src_length, bitwidth_byte_len);
	req_ctx->src_buffer = p_src;

	p_n = kmm_memalign(CE_ALIGN_SIZE, bitwidth_byte_len);
	if (p_n == NULL) {
		CE_ERR("kmm_memalign fail\n");
		kmm_free(p_src);
		return RSA_MALLOC_ERROR;
	}
	memset(p_n, 0x0, bitwidth_byte_len);
	__rsa_padding(p_n, req_ctx->key_n, req_ctx->n_len, bitwidth_byte_len);
	req_ctx->key_n = p_n;

	ret = ce_rsa_start(req_ctx);
	if (ret < 0) {
		CE_ERR("caclu rsa erro num is %d\n", ret);
		kmm_free(p_src);
		kmm_free(p_n);
		return RSA_CRYPTO_ERROR;
	}
	kmm_free(p_src);
	kmm_free(p_n);
	return RSA_STATUS_OK;
}

