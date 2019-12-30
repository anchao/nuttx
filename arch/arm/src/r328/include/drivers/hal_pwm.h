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


#ifndef __HAL_PWM_H__
#define __HAL_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Enums
 *****************************************************************************/
typedef enum {
	HAL_PWM_STATUS_ERROR_PARAMETER = -3,
	HAL_PWM_STATUS_ERROR_CHANNEL = -2,
	HAL_PWM_STATUS_ERROR = -1,
	HAL_PWM_STATUS_OK = 0
} hal_pwm_status_t;

typedef enum {
	PWM_POLARITY_INVERSED,
	PWM_POLARITY_NORMAL,
} hal_pwm_polarity;

typedef enum {
	PWM_CLK_OSC,
	PWM_CLK_APB
} hal_pwm_clk_src;

/*****************************************************************************
 * Functions
 *****************************************************************************/
hal_pwm_status_t hal_pwm_init(void);
hal_pwm_status_t hal_pwm_config(uint32_t channel, uint32_t duty_ns, uint32_t period_ns);
hal_pwm_status_t hal_pwm_enable(uint32_t channel);
hal_pwm_status_t hal_pwm_disable(uint32_t channel);
hal_pwm_status_t hal_pwm_set_polarity(uint32_t channel, hal_pwm_polarity polarity);


#ifdef CONFIG_USING_PWM_DRIVER_OPS
typedef struct sunxi_hal_driver_pwm
{
	hal_pwm_status_t (* initialize)(void);
	hal_pwm_status_t (* config)(uint32_t channel, uint32_t duty_ns, uint32_t period_ns);
	hal_pwm_status_t (* enable)(uint32_t channel);
	hal_pwm_status_t (* disable)(uint32_t channel);
	hal_pwm_status_t (* set_polarity)(uint32_t channel, hal_pwm_polarity polarity);
} const sunxi_hal_driver_pwm_t;


extern const sunxi_hal_driver_pwm_t sunxi_hal_pwm_driver ;
#endif

#ifdef __cplusplus
}
#endif

#endif /* __HAL_PWM_H__ */
