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

#ifndef __SUNXI_CE_H__
#define __SUNXI_CE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* For debug */
#if 0
#define CE_EXIT()			CE_DBG("%s\n", "Exit")
#define CE_ENTER()			CE_DBG("%s\n", "Enter ...")
#else
#define CE_DBG(fmt, arg...)
#define CE_ERR(fmt, arg...)
#define CE_EXIT()			CE_DBG("%s\n", "Exit")
#define CE_ENTER()			CE_DBG("%s\n", "Enter ...")
#endif

#define CE_ALIGN_SIZE	0x20

#define AES_KEYSIZE_128		16
#define AES_KEYSIZE_192		24
#define AES_KEYSIZE_256		32
#define AES_BLOCK_SIZE		16
#define AES_MIN_KEY_SIZE	16
#define AES_MAX_KEY_SIZE	32


#define AES_TYPE_ECB	0
#define AES_TYPE_CBC	1



/*define the return value for aes*/
typedef enum{
	AES_STATUS_OK = 0,
	AES_INPUT_ERROR = -1,
	AES_KEY_ERROR = -2,
	AES_CRYPTO_ERROR = -3,
	AES_TIME_OUT = -4,
} aes_status_t;

/*define the return value for sha*/
typedef enum{
	SHA_STATUS_OK = 0,
	SHA_INPUT_ERROR = -1,
	SHA_MALLOC_ERROR = -2,
	SHA_CRYPTO_ERROR = -3,
	SHA_TIME_OUT = -4,
} sha_status_t;

/*define digest size*/
#define SHA1_BLOCK_SIZE      (64)  /**<  512 bits = 64  bytes */
#define SHA1_DIGEST_SIZE     (20)  /**<  160 bits = 20  bytes */
#define SHA224_BLOCK_SIZE    (64)  /**<  512 bits = 64  bytes */
#define SHA224_DIGEST_SIZE   (28)  /**<  224 bits = 28  bytes */
#define SHA256_BLOCK_SIZE    (64)  /**<  512 bits = 64  bytes */
#define SHA256_DIGEST_SIZE   (32)  /**<  256 bits = 32  bytes */
#define SHA384_BLOCK_SIZE    (128) /**< 1024 bits = 128 bytes */
#define SHA384_DIGEST_SIZE   (48)  /**<  384 bits = 48  bytes */
#define SHA512_BLOCK_SIZE    (128) /**< 1024 bits = 128 bytes */
#define SHA512_DIGEST_SIZE   (64)  /**<  512 bits = 64  bytes */

#define SHA_MAX_DIGEST_SIZE	 (64)
#define CE_HASH_PAD_SIZE	(SHA512_BLOCK_SIZE * 2)

#define MD5_PADDING_MODE		0x0
#define SHA1_PADDING_MODE		0x1
#define SHA224_PADDING_MODE		0x2
#define SHA256_PADDING_MODE		0x3
#define SHA384_PADDING_MODE		0x4
#define SHA512_PADDING_MODE		0x5

/*define the return value for rsa*/
typedef enum{
	RSA_STATUS_OK = 0,
	RSA_INPUT_ERROR = -1,
	RSA_MALLOC_ERROR = -2,
	RSA_CRYPTO_ERROR = -3,
	RSA_TIME_OUT = -4,
} rsa_status_t;

/*define CE version*/
#define CE_SUPPORT_CE_V3_1

/*ce flow*/
#define CE_FLOW_NUM				4
#define CE_FLOW_AVAILABLE		0
#define CE_FLOW_UNAVAILABLE		1



#ifdef __cplusplus
}
#endif

#endif /* __SUNXI_CE_H__ */
