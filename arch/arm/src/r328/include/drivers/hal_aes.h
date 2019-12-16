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

#ifndef __HAL_AES_H__
#define __HAL_AES_H__

//#include "types.h"


typedef unsigned long long  u64;
typedef unsigned int        u32;
typedef unsigned short      u16;
typedef unsigned char       u8;
typedef signed long long    s64;
typedef signed int          s32;
typedef signed short        s16;
typedef signed char         s8;
//typedef signed char         bool;
typedef unsigned int        size_t;
typedef unsigned int        uint;

#define ADDR_ALIGN_SIZE		0x4

#define AES_MODE_ECB	0
#define AES_MODE_CBC	1

#define CRYPT_DIR_ENCRYPT	0
#define CRYPT_DIR_DECRYPT	1

#define AES_IV_LENGTH	16

/*define the return value for hal_aes*/
typedef enum{
	HAL_AES_STATUS_ERROR = -1,
	HAL_AES_STATUS_OK = 0
} hal_aes_status_t;

/*define the data buffer for hal_aes*/
typedef struct {
	u8 *buffer;
	u32 length;
} hal_aes_buffer_t;

/*define the ctx for aes requtest*/
typedef struct {
	u32 dir;
	u32 type;
	u32 mode;
	u32 bitwidth;
} aes_req_ctx_t;


#define HASH_METHOD_MD5				16
#define HASH_METHOD_SHA1			17
#define HASH_METHOD_SHA224			18
#define HASH_METHOD_SHA256			19
#define HASH_METHOD_SHA384			20
#define HASH_METHOD_SHA512			21

#define MD5_DIGEST_SIZE			(20)
#define SHA1_DIGEST_SIZE		(20)
#define SHA224_DIGEST_SIZE		(28)
#define SHA256_DIGEST_SIZE		(32)
#define SHA384_DIGEST_SIZE		(48)
#define SHA512_DIGEST_SIZE		(64)

#define SHA_MAX_DIGEST_SIZE	 (64)

/*define the return value for hal_hash*/
typedef enum{
	HAL_HASH_INPUT_ERROR = -1,
	HAL_HASH_MALLOC_ERROR = -2,
	HAL_HASH_CRYPTO_ERROR = -3,
	HAL_HASH_STATUS_OK = 0
} hal_hash_status_t;

/*define the ctx for hash requtest*/
typedef struct {
	uint8_t *src_buffer;
	uint32_t src_length;
	uint8_t *dst_buffer;
	uint32_t dst_length;
	uint8_t md[SHA_MAX_DIGEST_SIZE];
	uint32_t md_size;
	uint32_t type;
	uint32_t dir;
	uint32_t padding_mode;
} crypto_hash_req_ctx;


#define CRYPTO_TYPE_RSA				32

/*define the return value for hal_rsa*/
typedef enum{
	HAL_RSA_INPUT_ERROR = -1,
	HAL_RSA_MALLOC_ERROR = -2,
	HAL_RSA_CRYPTO_ERROR = -3,
	HAL_RSA_STATUS_OK = 0
} hal_rsa_status_t;

typedef struct {
	uint8_t *key_n;
	uint32_t n_len;
	uint8_t *key_e;
	uint32_t e_len;
	uint8_t *key_d;
	uint32_t d_len;
	uint32_t bitwidth;
} rsa_key_t;

/*define the ctx for rsa requtest*/
typedef struct {
	uint8_t *key_n;
	uint32_t n_len;
	uint8_t *key_e;
	uint32_t e_len;
	uint8_t *key_d;
	uint32_t d_len;
	uint8_t *src_buffer;
	uint32_t src_length;
	uint8_t *dst_buffer;
	uint32_t dst_length;
	uint32_t dir;
	uint32_t type;
	uint32_t bitwidth;
} crypto_rsa_req_ctx;

#endif /*__HAL_AES_H__*/

