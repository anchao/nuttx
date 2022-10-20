/****************************************************************************
 * net/netdev/netdev_ifconf.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>

#include <net/if.h>
#include <nuttx/net/netdev.h>

#include <nuttx/mm/iob.h>

#include "inet/inet.h"
#include "utils/utils.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: prepare_pktbuf
 *
 * Description:
 *   prepare data buffer for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

static void netdev_pktbuf_prepare(FAR struct net_driver_s *dev,
                                  bool throttled, unsigned int timeout)
{
  if (dev->d_iob == NULL)
    {
      dev->d_iob = net_iobtimedalloc(throttled, timeout);
      dev->d_iob->io_data    = dev->d_iob->io_data;
    }

    dev->d_len = 0;
}

/****************************************************************************
 * Name: set_pktbuf
 *
 * Description:
 *   Set an I/O buffer for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

static void netdev_pktbuf_set(FAR struct net_driver_s *dev,
                              FAR struct iob_s *iob)
{
  dev->d_iob = iob;
  dev->d_iob->io_data    = iob->io_data;
  dev->d_len    = iob->io_pktlen;
}

/****************************************************************************
 * Name: clear_pktbuf
 *
 * Description:
 *   Clean up buffer resources for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network and dev->d_iob has been
 *   released or taken away.
 *
 ****************************************************************************/

static void netdev_pktbuf_clear(FAR struct net_driver_s *dev)
{
  dev->d_iob = NULL;
  dev->d_iob->io_data    = NULL;
  dev->d_len    = 0;
}

/****************************************************************************
 * Name: release_pktbuf
 *
 * Description:
 *   Release buffer resources for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

static void netdev_pktbuf_release(FAR struct net_driver_s *dev)
{
  if (dev->d_iob != NULL)
    {
      iob_free_chain(dev->d_iob);
      dev->d_iob = NULL;
    }

  dev->d_iob->io_data    = NULL;
  dev->d_len    = 0;
}

/****************************************************************************
 * Name: netdev_pktbuf_update
 *
 * Description:
 *   Update the offset and length value to a given I/O buffer
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

static void netdev_pktbuf_update(FAR struct net_drivers_s *dev,
                                 uint16_t off, uint16_t len)
{
  if (dev->d_iob != NULL)
    {
      dev->d_iob->io_offset = off;
      dev->d_iob->io_len    = len;
      dev->d_iob->io_pktlen = len;
    }
}

