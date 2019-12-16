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
#include <string.h>
#include <stdio.h>
#include <nuttx/mm/mm.h>
#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include "hal_aes.h"
#include "aw_types.h"

#define CE_ALIGN_SIZE 0x20

extern int sunxi_ce_init(void);
extern void *kmm_memalign(size_t size, size_t align);
extern int do_aes_crypt(hal_aes_buffer_t *src_text,
				hal_aes_buffer_t *dest_text,
				hal_aes_buffer_t *key,
				u8 *init_vector,
				u8 mode,
				u8 dir);
extern int do_hash_crypt(crypto_hash_req_ctx *req_ctx);



/*aes_cbc_encrypt*/
hal_aes_status_t hal_aes_cbc_encrypt(hal_aes_buffer_t *src_text,
                                     hal_aes_buffer_t *dest_text,
                                     hal_aes_buffer_t *key,
                                     u8 *iv)
{
    return do_aes_crypt(src_text, dest_text, key, iv, AES_MODE_CBC, CRYPT_DIR_ENCRYPT);
}

/*aes_cbc_decrypt*/
hal_aes_status_t hal_aes_cbc_decrypt(hal_aes_buffer_t *src_text,
                                     hal_aes_buffer_t *dest_text,
                                     hal_aes_buffer_t *key,
                                     u8 *iv)
{
    return do_aes_crypt(src_text, dest_text, key, iv, AES_MODE_CBC, CRYPT_DIR_DECRYPT);
}

/*aes_ecb_encrypt*/
hal_aes_status_t hal_aes_ecb_encrypt(hal_aes_buffer_t *src_text,
                                     hal_aes_buffer_t *dest_text,
                                     hal_aes_buffer_t *key)
{
    u8 init_vector[AES_IV_LENGTH] = {0};

    return do_aes_crypt(src_text, dest_text, key, init_vector, AES_MODE_ECB, CRYPT_DIR_ENCRYPT);
}

/*aes_ecb_decrypt*/
hal_aes_status_t hal_aes_ecb_decrypt(hal_aes_buffer_t *src_text,
                                     hal_aes_buffer_t *dest_text,
                                     hal_aes_buffer_t *key)
{
    u8 init_vector[AES_IV_LENGTH] = {0};

    return do_aes_crypt(src_text, dest_text, key, init_vector, AES_MODE_ECB, CRYPT_DIR_DECRYPT);
}

/*sha crypto*/
hal_hash_status_t hal_hash_crypt(uint8_t *src_buffer, uint32_t src_length, uint8_t *digest, uint32_t digest_length, uint32_t hash_type)
{
    hal_hash_status_t status;
	crypto_hash_req_ctx *req_ctx = NULL;
	uint8_t *src_tmp = NULL;
	int ret = 0;

    if ((src_buffer == NULL) && (digest == NULL)) {
        printf("input buffer is NULL\n");
        return HAL_HASH_INPUT_ERROR;
    }

	if ((hash_type != HASH_METHOD_MD5)
		&& (hash_type != HASH_METHOD_SHA1)
		&& (hash_type != HASH_METHOD_SHA224)
		&& (hash_type != HASH_METHOD_SHA256)
		&& (hash_type != HASH_METHOD_SHA384)
		&& (hash_type != HASH_METHOD_SHA384)) {
			printf("ce don't support %d this hash type\n", hash_type);
			return HAL_HASH_INPUT_ERROR;
	}

	if ((digest_length != SHA1_DIGEST_SIZE)
			&& (digest_length != SHA224_DIGEST_SIZE)
			&& (digest_length != SHA256_DIGEST_SIZE)
			&& (digest_length != SHA384_DIGEST_SIZE)
			&& (digest_length != SHA512_DIGEST_SIZE)) {
			printf("key length is %d, invalid\n", digest_length);
			return HAL_HASH_INPUT_ERROR;
    }

	req_ctx = (u8 *)kmm_memalign(CE_ALIGN_SIZE, sizeof(crypto_hash_req_ctx));
	if (req_ctx == NULL) {
		printf (" malloc req_ctx buffer fail\n");
		return HAL_HASH_MALLOC_ERROR;
	}
	memset(req_ctx, 0x0, sizeof(crypto_hash_req_ctx));

	req_ctx->src_buffer = src_buffer;
	req_ctx->src_length = src_length;
	req_ctx->dst_buffer = digest;
	req_ctx->dst_length = digest_length;
	req_ctx->md_size	= 0;
	req_ctx->type		= hash_type;

	ret = do_hash_crypt(req_ctx);
	if (ret < 0) {
		printf (" do_digest_crypt fail\n");
		kmm_free(req_ctx);
		return HAL_HASH_CRYPTO_ERROR;
	}

	kmm_free(req_ctx);
	return HAL_HASH_STATUS_OK;
}

/*sha crypto*/
hal_rsa_status_t hal_rsa_crypt(uint8_t *src_buffer, uint32_t src_length, uint8_t *dst_buffer, uint32_t dst_length, rsa_key_t *rsa_key)
{
    hal_rsa_status_t status;
	crypto_rsa_req_ctx *req_ctx = NULL;
	uint8_t *src_tmp = NULL;
	int ret = 0;

    if ((src_buffer == NULL)
		&& (dst_buffer == NULL)
		&& (rsa_key == NULL)) {
        printf("input buffer is NULL\n");
        return HAL_RSA_INPUT_ERROR;
    }

	if ((rsa_key->bitwidth > 2048)) {
			printf("ce max support 2048 key bitwidth\n");
			return HAL_RSA_INPUT_ERROR;
	}

	if ((rsa_key->key_n == NULL)
			&& (rsa_key->key_e == NULL)
			&& (rsa_key->key_d == NULL)) {
			printf("key buf is NULL\n");
			return HAL_RSA_INPUT_ERROR;
    }

	req_ctx = (u8 *)kmm_memalign(CE_ALIGN_SIZE, sizeof(crypto_rsa_req_ctx));
	if (req_ctx == NULL) {
		printf (" malloc req_ctx buffer fail\n");
		return HAL_RSA_MALLOC_ERROR;
	}
	memset(req_ctx, 0x0, sizeof(crypto_rsa_req_ctx));

	req_ctx->key_n = rsa_key->key_n;
	req_ctx->n_len = rsa_key->n_len;
	req_ctx->key_e = rsa_key->key_e;
	req_ctx->e_len = rsa_key->e_len;
	req_ctx->key_d = rsa_key->key_d;
	req_ctx->d_len = rsa_key->d_len;
	req_ctx->src_buffer = src_buffer;
	req_ctx->src_length = src_length;
	req_ctx->dst_buffer = dst_buffer;
	req_ctx->dst_length = dst_length;
	req_ctx->bitwidth	= rsa_key->bitwidth;
	req_ctx->bitwidth	= CRYPTO_TYPE_RSA;

	ret = do_rsa_crypt(req_ctx);
	if (ret < 0) {
		printf (" do_rsa_crypt fail\n");
		kmm_free(req_ctx);
		return HAL_RSA_CRYPTO_ERROR;
	}

	kmm_free(req_ctx);
	return HAL_RSA_STATUS_OK;
}

