/****************************************************************************
 * arch/arm/src/r328/r328_wdt.c
 *
 *   Copyright (C) 2018 Zglue Inc. All rights reserved.
 *   Author: Levin Li <zhiqiang@zglue.com>
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>

#include "chip.h"

#include "sunxi_hal_wdt.h"

#if defined(CONFIG_WATCHDOG) && defined(CONFIG_R328_WATCHDOG)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WDT_FMIN       (1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * well-known watchdog_lowerhalf_s structure.
 */

struct r328_wdt_lowerhalf_s
{
  FAR const struct watchdog_ops_s  *ops;  /* Lower half operations */
  struct sunxi_wdt_dev_t *wdt_dev;
  uint32_t timeout;   /* The (actual) timeout(milliseconds) */
  uint32_t lastreset; /* The last reset time */
  bool     started;   /* true: The watchdog timer has been started */
  uint16_t reload;    /* Timer reload value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* "Lower half" driver methods **********************************************/

static int r328_start(FAR struct watchdog_lowerhalf_s *lower);
static int r328_stop(FAR struct watchdog_lowerhalf_s *lower);
static int r328_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int r328_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status);
static int r328_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                            uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct watchdog_ops_s g_wdtops =
{
  .start      = r328_start,
  .stop       = r328_stop,
  .keepalive  = r328_keepalive,
  .getstatus  = r328_getstatus,
  .settimeout = r328_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

/* "Lower half" driver state */

static struct r328_wdt_lowerhalf_s g_wdtdev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r328_start
 *
 * Description:
 *   Start the watchdog timer, resetting the time to the current timeout,
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int r328_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct r328_wdt_lowerhalf_s *priv = (FAR struct r328_wdt_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  if (!priv->started)
  {
      if (sunxi_wdt_start(priv->wdt_dev))
      {
          wdinfo("sunxi_wdt_start faile\n");
          return -1;
      }
      priv->lastreset = clock_systimer();
      priv->started   = true;
  }

  return OK;
}

/****************************************************************************
 * Name: r328_stop
 *
 * Description:
 *   Stop the watchdog timer
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int r328_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct r328_wdt_lowerhalf_s *priv = (FAR struct r328_wdt_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  if (sunxi_wdt_stop(priv->wdt_dev))
  {
      wdinfo("sunxi_wdt_stop faile\n");
      return -1;
  } else {
      priv->started = false;
      priv->lastreset = 0;
  }

  return OK;
}

/****************************************************************************
 * Name: r328_keepalive
 *
 * Description:
 *   Reset the watchdog timer to the current timeout value, prevent any
 *   imminent watchdog timeouts.  This is sometimes referred as "pinging"
 *   the watchdog timer or "petting the dog".
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of the "lower-half"
 *           driver state structure.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int r328_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct r328_wdt_lowerhalf_s *priv = (FAR struct r328_wdt_lowerhalf_s *)lower;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Reload the WDT timer */
  if (sunxi_wdt_ping(priv->wdt_dev))
  {
       wdinfo("sunxi_wdt_ping faile\n");
       return -1;
  } else {
       priv->lastreset = clock_systimer();
  }

  return OK;
}

/****************************************************************************
 * Name: r328_getstatus
 *
 * Description:
 *   Get the current watchdog timer status
 *
 * Input Parameters:
 *   lower  - A pointer the publicly visible representation of the "lower-half"
 *            driver state structure.
 *   status - The location to return the watchdog status information.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int r328_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                           FAR struct watchdog_status_s *status)
{
  FAR struct r328_wdt_lowerhalf_s *priv = (FAR struct r328_wdt_lowerhalf_s *)lower;
  uint32_t ticks;
  uint32_t elapsed;

  wdinfo("Entry\n");
  DEBUGASSERT(priv);

  /* Return the status bit */

  status->flags = WDFLAGS_RESET;
  if (priv->started)
  {
      status->flags |= WDFLAGS_ACTIVE;
  }

  /* Return the actual timeout in milliseconds */

  status->timeout = priv->timeout;

  /* Get the elapsed time since the last ping */
  ticks   = clock_systimer() - priv->lastreset;
  elapsed = (int32_t)TICK2MSEC(ticks);

  if (elapsed > priv->timeout)
  {
      elapsed = priv->timeout;
  }

  /* Return the approximate time until the watchdog timer expiration */

  status->timeleft = priv->timeout - elapsed;

  wdinfo("Status     :\n");
  wdinfo("  flags    : %08x\n", status->flags);
  wdinfo("  timeout  : %d\n", status->timeout);
  wdinfo("  timeleft : %d\n", status->timeleft);
  return OK;
}

/****************************************************************************
 * Name: r328_settimeout
 *
 * Description:
 *   Set a new timeout value (and reset the watchdog timer)
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the "lower-half"
 *             driver state structure.
 *   timeout - The new timeout value in milliseconds.
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int r328_settimeout(FAR struct watchdog_lowerhalf_s *lower, uint32_t timeout)
{
  FAR struct r328_wdt_lowerhalf_s *priv = (FAR struct r328_wdt_lowerhalf_s *)lower;

  wdinfo("Entry: timeout=%d milliseconds\n", timeout);
  DEBUGASSERT(priv);

  /* Can this timeout be represented? */

  if (timeout < (priv->wdt_dev->min_timeout * 1000) ||
		 timeout > (priv->wdt_dev->max_timeout * 1000))
  {
      wderr("ERROR: Cannot represent timeout=%d > %d\n",
			  timeout, (priv->wdt_dev->max_timeout * 1000));
      return -ERANGE;
  }

  if (priv->started)
  {
      wdwarn("WARNING: Watchdog is already started, timeout=%d\n", priv->timeout);
      wdwarn("         If need to reset timeout, need to stop Watchdog first\n");
      return -EBUSY;
  }

  priv->timeout = timeout;
  priv->wdt_dev->timeout = (timeout / 1000);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: r328_wdt_initialize
 *
 * Description:
 *   Initialize the WDT watchdog time.  The watchdog timer is initialized and
 *   registers as 'devpath.  The initial state of the watchdog time is
 *   disabled.
 *
 * Input Parameters:
 *   devpath    - The full path to the watchdog.  This should be of the form
 *                /dev/watchdog0
 *
 * Returned Values:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int r328_wdt_initialize(FAR const char *devpath)
{
  FAR struct r328_wdt_lowerhalf_s *priv = &g_wdtdev;
  FAR void *handle = NULL;

  wdinfo("Entry: devpath=%s\n", devpath);
  FAR struct sunxi_wdt_dev_t *sunxi_wdt_dev = NULL;
  FAR struct sunxi_wdt_info_t *sunxi_wdt_info = NULL;

#ifdef CONFIG_R328_WDOG_BOOTON
  if (sunxi_wdt_init(&sunxi_wdt_dev, 1)) {
#else
  if (sunxi_wdt_init(&sunxi_wdt_dev, 0)) {
#endif
    wdinfo("sunxi_wdt_init faile\n");
    return -ENODEV;
  }

  if (sunxi_wdt_get_info(sunxi_wdt_dev, &sunxi_wdt_info)) {
    wdinfo("sunxi_wdt_get_info faile\n");
    return -ENODEV;
  }

  /* Initialize the driver state structure. */

  priv->ops     = &g_wdtops;
  /* timeout is milliseconds */
  priv->timeout = sunxi_wdt_dev->timeout * 1000;
  priv->wdt_dev = sunxi_wdt_dev;
  priv->started = false;

  /* Register the watchdog driver as /dev/watchdog0 */

  handle = watchdog_register(devpath, (FAR struct watchdog_lowerhalf_s *)priv);

  if (handle != NULL) {
      wdinfo("watchdog init success!\n");
      wdinfo("watchdog dev name: %s\n", sunxi_wdt_info->dev_name);
      wdinfo("watchdog drv name: %s\n", sunxi_wdt_info->drv_name);
      wdinfo("watchdog drv version: %s\n", sunxi_wdt_info->drv_version);
  } else {
      wdinfo("watchdog register faile\n");
      return -ENODEV;
  }
  return (handle != NULL) ? OK : -ENODEV;
}

#endif /* CONFIG_WATCHDOG && CONFIG_R328_WATCHDOG */
