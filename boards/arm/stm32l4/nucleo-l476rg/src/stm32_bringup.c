/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>

#include <arch/board/board.h>

#include "nucleo-l476rg.h"

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32l4_rtc.h"
#endif

#include "stm32l4_i2c.h"

#ifdef CONFIG_SENSORS_BMP280
#include "stm32_bmp280.h"
#endif

#ifdef CONFIG_SENSORS_MPU9250
#include "stm32_mpu9250.h"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#  define MMCSD_MINOR       CONFIG_NSH_MMCSDMINOR
#else
#  define MMCSD_MINOR       0
#endif

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32l4_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                bus, ret);
          stm32l4_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
  stm32_i2c_register(1);
#if 0
  stm32_i2c_register(1);
  stm32_i2c_register(2);
#endif
}
#else
#  define stm32_i2ctool()
#endif

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *rtclower;
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2ctool();
#endif

#ifdef CONFIG_SENSORS_QENCODER
  int index;
  char buf[9];
#endif
  int ret = OK;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32L4 lower-half RTC driver */

  rtclower = stm32l4_rtc_lowerhalf();
  if (!rtclower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, rtclower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef HAVE_MMCSD_SDIO
  /* First, get an instance of the SDIO interface */

  g_sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!g_sdio)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_sdio);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
             ret);
      return ret;
    }

  /* Then let's guess and say that there is a card in the slot. There is no
   * card detect GPIO.
   */

  sdio_mediachange(g_sdio, true);

  syslog(LOG_INFO, "[boot] Initialized SDIO\n");
#endif

#ifdef CONFIG_SENSORS_AS726X
  ret = stm32_as726xinitialize("/dev/spectr0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize AS726X, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_BMP180
  ret = stm32_bmp180initialize("/dev/press0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP180, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32l4_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32l4_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN
  ret = stm32l4_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_can_setup failed: %d\n", ret);
      return ret;
    }
#endif

  /* Initialize MMC and register the MMC driver. */

#ifdef HAVE_MMCSD_SPI
  ret = stm32l4_mmcsd_initialize(MMCSD_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_INPUT_AJOYSTICK
  /* Initialize and register the joystick driver */

  ret = board_ajoy_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the joystick driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_TIMER
  /* Initialize and register the timer driver */

  ret = board_timer_driver_initialize("/dev/timer0", 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  index = 0;

#ifdef CONFIG_STM32L4_TIM1_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM2_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM3_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM4_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 4);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM5_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 5);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM8_QE
  snprintf(buf, sizeof(buf), "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 8);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif
#endif /* CONFIG_SENSORS_QENCODER */

#ifdef CONFIG_SENSORS_HTS221
  ret = stm32l4_hts221_initialize("/dev/hts221");
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize HTC221 driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_LSM6DSL
  ret = stm32l4_lsm6dsl_initialize("/dev/lsm6dsl0");
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize LSM6DSL driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_LSM303AGR
  ret = stm32l4_lsm303agr_initialize("/dev/lsm303mag0");
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize LSM303AGR driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = stm32l4_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_WL_CC1101
  /* Initialize and register the cc1101 radio */

  ret = stm32l4_cc1101_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_cc1101_initialize failed: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_BMP280
  /* Try to register BMP280 device in I2C1 */

  ret = board_bmp280_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP280 driver: %d\n", ret);
    }
  else
    {
      syslog(LOG_ERR, "Initialized BMP280 driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MPU9250
  /* Try to register MPU9250 device in I2C1 */

  ret = board_mpu9250_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize MPU9250 driver: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "Initialized MPU9250 driver: %d\n", ret);
    }
#endif

  return ret;
}
