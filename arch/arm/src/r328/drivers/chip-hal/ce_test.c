/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the people's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY’S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS’SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY’S TECHNOLOGY.
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
#include <string.h>
#include <aw_types.h>
#include <hal_aes.h>
#include <nuttx/kmalloc.h>

extern hal_aes_status_t hal_aes_ecb_encrypt(hal_aes_buffer_t *src_text,
                                     hal_aes_buffer_t *dest_text,
                                     hal_aes_buffer_t *key);
extern hal_aes_status_t hal_aes_ecb_decrypt(hal_aes_buffer_t *src_text,
                                     hal_aes_buffer_t *dest_text,
                                     hal_aes_buffer_t *key);
extern hal_aes_status_t hal_aes_cbc_encrypt(hal_aes_buffer_t *src_text,
                                     hal_aes_buffer_t *dest_text,
                                     hal_aes_buffer_t *key,
                                     u8 *iv);
extern hal_aes_status_t hal_aes_cbc_decrypt(hal_aes_buffer_t *src_text,
                                     hal_aes_buffer_t *dest_text,
                                     hal_aes_buffer_t *key,
                                     u8 *iv);
extern hal_hash_status_t hal_hash_crypt(uint8_t *src_buffer,
										uint32_t src_length,
										uint8_t *digest,
										uint32_t digest_length,
										uint32_t hash_type);

extern int sunxi_ce_init(void);

#define DATA_SIZE		512
#define CE_ALIGN_SIZE	0x20


#define HASH_METHOD_MD5				16
#define HASH_METHOD_SHA1			17
#define HASH_METHOD_SHA224			18
#define HASH_METHOD_SHA256			19
#define HASH_METHOD_SHA384			20
#define HASH_METHOD_SHA512			21

int aes_ecb_test(hal_aes_buffer_t *src_text, hal_aes_buffer_t *dest_text, hal_aes_buffer_t *key_text)
{
	u8 *tmp = NULL;
	int ret = 0;
	hal_aes_buffer_t tmp_text;

	printf ("#######aes_ecb_size test %d################\n", src_text->length);
	tmp = (u8 *)kmm_memalign(CE_ALIGN_SIZE, DATA_SIZE);
	if (tmp == NULL) {
		printf (" malloc data buffer fail\n");
		return -1;
	}
	memset(tmp, 0x0, DATA_SIZE);
	tmp_text.buffer = tmp;
	tmp_text.length = DATA_SIZE;

	ret = hal_aes_ecb_encrypt(src_text, dest_text, key_text);
	if (ret < 0) {
		printf (" aes ecb_encrypt fail %d\n", ret);
		return -1;
	}

	if (hal_aes_ecb_decrypt(dest_text, &tmp_text, key_text) < 0) {
		printf (" aes ecb_decrypt fail fail\n");
		return -1;
	}

	if (memcmp(src_text->buffer, tmp_text.buffer, src_text->length)) {
		printf (" aes ecb memcmp fail\n");
		return -1;
	}
	printf ("############aes_ecb_test pass: %d#############\n\n\n", src_text->length);
	return 0;
}

int aes_cbc_test(hal_aes_buffer_t *src_text, hal_aes_buffer_t *dest_text, hal_aes_buffer_t *key_text)
{
	u8 *tmp = NULL;
	int ret = 0;
	u8 iv[16] = {
	0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
	0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
	};
	hal_aes_buffer_t tmp_text;

	printf ("#######aes_cbc_size test %d################\n", src_text->length);
	tmp = (u8 *)kmm_memalign(CE_ALIGN_SIZE, DATA_SIZE);
	if (tmp == NULL) {
		printf (" malloc data buffer fail\n");
		return -1;
	}
	memset(tmp, 0x0, DATA_SIZE);
	tmp_text.buffer = tmp;
	tmp_text.length = DATA_SIZE;

	ret = hal_aes_cbc_encrypt(src_text, dest_text, key_text, iv);
	if (ret < 0) {
		printf (" aes cbc_encrypt fail %d\n", ret);
		return -1;
	}

	if (hal_aes_cbc_decrypt(dest_text, &tmp_text, key_text, iv) < 0) {
		printf (" aes cbc_decrypt fail fail\n");
		return -1;
	}

	if (memcmp(src_text->buffer, tmp_text.buffer, src_text->length)) {
		printf (" aes cbc memcmp fail\n");
		return -1;
	}
	printf ("############aes_cbc_test pass: %d#############\n\n\n", src_text->length);
	return 0;
}

int hash_test(void)
{
	uint8_t digest[SHA256_DIGEST_SIZE] = {0xF9, 0x3A, 0xC1, 0x74, 0xAC, 0xD9, 0x7B, 0x23,
										0x45, 0x8C, 0x57, 0x1F, 0x52, 0xC9, 0x73, 0x47,
										0xDD, 0x85, 0x6E, 0xCD, 0xB6, 0x46, 0x97, 0xE8,
										0x6F, 0x71, 0xFB, 0xE8, 0x8B, 0xDF, 0xED, 0x19
										};
	uint8_t *src_data = NULL;
	uint8_t *dest_data = NULL;
	u32 data_size = DATA_SIZE;
	int ret = -1;
	sunxi_ce_init();
	/*malloc dest buf*/
	src_data = (u8 *)kmm_memalign(CE_ALIGN_SIZE, data_size);
	if (src_data == NULL) {
		printf (" malloc dest buffer fail\n");
		ret = -1;
		goto out;
	}
	memset(src_data, 0x55, data_size);

	/*malloc dest buf*/
	dest_data = (u8 *)kmm_memalign(CE_ALIGN_SIZE, SHA256_DIGEST_SIZE);
	if (dest_data == NULL) {
		printf (" malloc dest buffer fail\n");
		ret = -1;
		goto out;
	}

	memset(dest_data, 0x0, SHA256_DIGEST_SIZE);
	ret = hal_hash_crypt(src_data, data_size, dest_data, SHA256_DIGEST_SIZE, HASH_METHOD_SHA256);
	if (ret < 0) {
		printf (" hal_hash_crypt fail\n");
		ret = -1;
	}

	ret = memcmp(dest_data, digest, SHA256_DIGEST_SIZE);
	if (ret != 0) {
		printf (" hash compare fail\n");
		ret = -1;
	}
	printf("sha256 test ok!!!\n");
out:
	if (dest_data != NULL) {
		kmm_free(dest_data);
	}

	if (src_data != NULL) {
		kmm_free(src_data);
	}

	return ret;
}


int aes_test(void)
{
	int ret = -1;
	int i = 0;
	u32 data_size = DATA_SIZE;
	u32 size_test[4] = {13, 16, 223, 256};
	u32 algin_size = 0;
	u8 *data = NULL;
	u8 key[16] = {
	0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88,
	0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00
	};
	hal_aes_buffer_t src_text;
	hal_aes_buffer_t dest_text;
	hal_aes_buffer_t key_text;

	sunxi_ce_init();

	/*malloc key buf*/
	data = (u8 *)kmm_memalign(CE_ALIGN_SIZE, sizeof(key));
	if (data == NULL) {
		printf (" malloc key buffer fail\n");
		return -1;
	}
	memcpy(data, key, sizeof(key));
	key_text.buffer = data;
	key_text.length = sizeof(key);

	/*malloc dest buf*/
	data = (u8 *)kmm_memalign(CE_ALIGN_SIZE, data_size);
	if (data == NULL) {
		printf (" malloc dest buffer fail\n");
		ret = -1;
		goto out;
	}
	dest_text.buffer = data;
	dest_text.length = data_size;

	/*malloc src buf*/
	data = (u8 *)kmm_memalign(CE_ALIGN_SIZE, data_size);
	if (data == NULL) {
		printf (" malloc data buffer fail\n");
		ret = -1;
		goto out;
	}
	memset(data, 0x78, data_size);
	src_text.buffer = data;
	src_text.length = data_size;

	for (i = 0; i< 4; i++) {
		src_text.length = size_test[i];
		if (aes_ecb_test(&src_text, &dest_text, &key_text) < 0) {
			printf("aes_ecb_test fail\n");
			return -1;
		}
	}

	for (i = 0; i< 4; i++) {
		src_text.length = size_test[i];
		if (aes_cbc_test(&src_text, &dest_text, &key_text) < 0) {
			printf("aes_cbc_test fail\n");
			return -1;
		}
	}
	ret = 0;

out:
	if (key_text.buffer != NULL) {
		kmm_free(key_text.buffer);
	}
	if (dest_text.buffer != NULL) {
		kmm_free(dest_text.buffer);
	}
	if (src_text.buffer != NULL) {
		kmm_free(src_text.buffer);
	}
	return ret;
}

void ce_test(void)
{
	hash_test();
	aes_test();
}


