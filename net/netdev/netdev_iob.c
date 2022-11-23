/****************************************************************************
 * net/netdev/netdev_iob.c
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

#include <debug.h>
#include <errno.h>

#include <nuttx/net/netdev.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_iob_update
 *
 * Description:
 *   Update the offset and length to a given I/O buffer,
 *   The buffer layout as follow:
 *  -------------------------------------------------------------------------
 *  |          iob entry 0           |          iob entry 1           | ... |
 *  -------------------------------------------------------------------------
 *  |<-- io_offset -->|<-- io_len -->|<---------- io_len ------------>| ... |
 *  |     (offset)    |<------------- io_pktlen (datalen) ----------------->|
 *  -------------------------------------------------------------------------
 *
 * Assumptions:
 *   None
 *
 ****************************************************************************/

void netdev_iob_update(FAR struct iob_s *iob, uint16_t offset,
                       uint16_t datalen)
{
  FAR struct iob_s *penultimate;
  FAR struct iob_s *last;
  FAR struct iob_s *next;
  uint32_t ninqueue = 0;
  uint32_t nrequire;
  uint16_t len;

  /* The data offset must be less than CONFIG_IOB_BUFSIZE */

  if (iob == NULL || offset >= CONFIG_IOB_BUFSIZE)
    {
      return;
    }

  /* Calculate the total entries of the data in the I/O buffer chain */

  next = iob;
  while (next != NULL)
    {
      ninqueue++;
      next = next->io_flink;
    }

  /* Trim inqueue entries if needed */

  nrequire = (offset + datalen) / CONFIG_IOB_BUFSIZE + 1;

  if (nrequire < ninqueue)
    {
      /* Loop until complete the trim */

      nrequire = ninqueue - nrequire;

      while (nrequire--)
        {
          last = NULL;
          penultimate = NULL;

          /* find the last entry in the chain.  */

          for (next = iob; next; next = next->io_flink)
            {
              penultimate = last;
              last = next;
            }

          /* Free the last, empty buffer in the list */

          iob_free(last);

          /* Unlink the penultimate from the freed buffer */

          if (penultimate)
            {
              penultimate->io_flink = NULL;
            }
        }
    }

  iob->io_offset = offset;
  iob->io_pktlen = datalen;

  /* Update size of each iob */

  while (iob != NULL && datalen > 0)
    {
      if (datalen + offset >= CONFIG_IOB_BUFSIZE)
        {
          len = CONFIG_IOB_BUFSIZE - offset;
          offset = 0;
        }
      else
        {
          len = datalen;
        }

      iob->io_len = len;
      datalen    -= len;
      iob         = iob->io_flink;
    }
}

/****************************************************************************
 * Name: netdev_iob_prepare
 *
 * Description:
 *   Prepare data buffer for a given NIC
 *   The iob offset will be updated to l2 gruard size by default:
 *  ----------------------------------------------------------------
 *  |                     iob entry                                |
 *  ---------------------------------------------------------------|
 *  |<-- CONFIG_NET_LL_GRUARDSIZE -->|<--- io_len/io_pktlen(0) --->|
 *  ---------------------------------------------------------------|
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 * Returned Value:
 *   A non-zero copy is returned on success.
 *
 ****************************************************************************/

int netdev_iob_prepare(FAR struct net_driver_s *dev, bool throttled,
                       unsigned int timeout)
{
  /* Prepare iob buffer */

  if (dev->d_iob == NULL)
    {
      dev->d_iob = net_iobtimedalloc(throttled, timeout);
      if (dev->d_iob == NULL)
        {
          return -ENOMEM;
        }
    }

  /* Set the device buffer to l2 */

  dev->d_buf = (FAR void *)LLBUF;

  /* Update l2 gruard size */

  netdev_iob_update(dev->d_iob, CONFIG_NET_LL_GRUARDSIZE, 0);

  return OK;
}

/****************************************************************************
 * Name: netdev_iob_clear
 *
 * Description:
 *   Clean up buffer resources for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network and dev->d_iob has been
 *   released or taken away.
 *
 ****************************************************************************/

void netdev_iob_clear(FAR struct net_driver_s *dev)
{
  /* Clear the device buffer */

  dev->d_iob = NULL;
  dev->d_buf = NULL;
  dev->d_len = 0;
}

/****************************************************************************
 * Name: netdev_iob_release
 *
 * Description:
 *   Release buffer resources for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

void netdev_iob_release(FAR struct net_driver_s *dev)
{
  /* Release device buffer */

  if (dev->d_iob != NULL)
    {
      iob_free_chain(dev->d_iob);
      dev->d_iob = NULL;
    }
}
