/* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.

 * Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
 * the the People's Republic of China and other countries.
 * All Allwinner Technology Co.,Ltd. trademarks are used with permission.

 * DISCLAIMER
 * THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
 * IF YOU NEED TO INTEGRATE THIRD PARTY¡¯S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
 * IN ALLWINNERS¡¯SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
 * ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
 * ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
 * COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
 * YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY¡¯S TECHNOLOGY.


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

#include "../../drivers/chip-src/gpadc/r328_gpadc.h"

#ifndef __HAL_GPADC_H__
#define __HAL_GPADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Enums
 *****************************************************************************/
typedef enum{
	HAL_GPADC_STATUS_ERROR_PARAMETER = -3,
	HAL_GPADC_STATUS_ERROR_CHANNEL = -2,
	HAL_GPADC_STATUS_ERROR = -1,
	HAL_GPADC_STATUS_OK = 0
}hal_gpadc_status_t;

/*****************************************************************************
 * Functions
 *****************************************************************************/

/* @brief      GPADC init function
 * @return
 * #HAL_ADC_STATUS_OK, ADC init success. \n
 */
hal_gpadc_status_t hal_gpadc_init(void);

/* @brief      GPADC deinit function. reset the GPADC.
 * @return
 * #
 * #
 */
hal_gpadc_status_t hal_gpadc_deinit(void);

/* @brief      GPADC register callback function.
 * @para
 * #[in] the callback function.
 * @return
 * #HAL_ADC_STATUS_OK, ADC init success. \n
 * #HAL_ADC_STATUS_ERROR_CHANNEL, channel is error. \n
 * #HAL_ADC_STATUS_ERROR_PARAMETER, error para.
 */
hal_gpadc_status_t hal_gpadc_register_callback(gpadc_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_GPADC_H__ */
