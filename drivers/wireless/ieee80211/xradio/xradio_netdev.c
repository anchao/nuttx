/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Author: laumy <liumingyuan@allwinnertech.com>
* Date  : 2020-01-09
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



#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include <nuttx/net/arp.h>
#include <nuttx/net/dns.h>
#include <nuttx/net/netdev.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/net/pkt.h>
#include <nuttx/net/net.h>

#include <aw/wifi/wifi_adapter.h>

#include <xradio_netif.h>
#include "xradio_netdev.h"

#define BUF ((struct eth_hdr_s *)priv->xr_dev.d_buf)

#define XR_DEV_DBG printf
#define XR_DEV_ERR printf

#if (USE_TX_GET_BUFF == 0)
static uint8_t g_pktbuf[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];
#endif

static struct xradio_dev_s *g_xradio_priv = NULL;

#define XRADIO_DRV_WDDELAY      (1*CLK_TCK)

static int xradio_drv_ifup(FAR struct net_driver_s *dev);
static int xradio_drv_ifdown(FAR struct net_driver_s *dev);
static int xradio_drv_txavail(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int xradio_drv_addmac(FAR struct net_driver_s *dev,
                                FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NET_IGMP
static int xradio_drv_rmmac(FAR struct net_driver_s *dev,
                               FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int xradio_drv_ioctl(FAR struct net_driver_s *dev, int cmd,
                               unsigned long arg);
#endif
static void xradio_drv_poll_expiry(int argc, wdparm_t arg, ...);

static void xradio_drv_poll_work(FAR void *arg);

#if USE_TX_GET_BUFF
static int xradio_get_tx_payload_buffer(struct xradio_dev_s *priv)
{
	if(priv->cur_tx_frame != NULL) { //TX complete,clear.
		return OK;
	}

	priv->cur_tx_frame = xradio_tx_buff_get();
	if(NULL == priv->cur_tx_frame) {
		XR_DEV_ERR("Cannot allocate TX frame.\n");
		return -1;
	}
	return OK;
}
#endif

static void xradio_netdev_notify_tx_done(FAR struct xradio_dev_s *priv)
{
	work_queue(LPWORK,&priv->xr_pollwork,xradio_drv_poll_work,priv,0);
}

static int xradio_drv_transmit(struct xradio_dev_s *priv)
{
	int ret;

	ret = ethernetif_linkoutput(priv->xr_dev.d_buf,priv->xr_dev.d_len);
	if(ret) {
		XR_DEV_ERR("failed to transmit frame.\n");
		return -1;
	}
	return 0;
}


static int xradio_drv_txpoll(FAR struct net_driver_s *dev)
{

  FAR struct xradio_dev_s *priv = (struct xradio_dev_s *) dev->d_private;
  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */
  if (priv->xr_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */
#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->xr_dev.d_flags))
#endif
        {
          arp_out(&priv->xr_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->xr_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->xr_dev))
        {
          /* Send the packet */

          net_lock();

		  if(priv->cur_tx_frame == NULL) {
			net_unlock();
			xradio_netdev_notify_tx_done(priv);
			return 0;
		  }

          xradio_drv_transmit(priv);

          /* TODO: Check if there is room in the device to hold another packet.
           * If not, return a non-zero value to terminate the poll.
           */
		  priv->xr_dev.d_buf = NULL;
#if USE_TX_GET_BUFF
		  xradio_tx_buff_free();
		  priv->cur_tx_frame = NULL;
#endif
          net_unlock();
          NETDEV_TXPACKETS(&priv->xr_dev);
		  xradio_netdev_notify_tx_done(priv);
          return 1;
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

static void xradio_drv_poll_work(FAR void *arg)
{
	FAR struct net_driver_s *dev = (struct net_driver_s *)arg;
	FAR struct xradio_dev_s *priv = (struct xradio_dev_s *)dev->d_private;

	/* Lock the network and serialize driver operations if necessary.
	 * NOTE: Serialization is only required in the case where the driver work
	 * is performed on an LP worker thread and where more than one LP worker
	 * thread has been configured.
	 */

	net_lock();
	if(IFF_IS_UP(dev->d_flags)) {
		if(priv->cur_tx_frame == NULL) {
#if USE_TX_GET_BUFF
	/* Perform the poll */

	/* Check if there is room in the send another TX packet.  We cannot perform
	 * the TX poll if he are unable to accept another packet for transmission.
	 */
			if(xradio_get_tx_payload_buffer(priv)) {
				goto exit_unlock;
			}

			dev->d_buf = priv->cur_tx_frame->data;
			dev->d_len = 0;
#endif
		}
		if (dev->d_buf) {
	   /* If so, update TCP timing states and poll the network for new XMIT data.
       * Hmmm.. might be bug here.  Does this mean if there is a transmit in
       * progress, we will missing TCP time state updates?
       */

			devif_timer(dev, xradio_drv_txpoll);
  /* Setup the watchdog poll timer again */

			wd_start(priv->xr_txpolldog, XRADIO_DRV_WDDELAY, xradio_drv_poll_expiry, 1,
				(wdparm_t)dev);
		}
	}
#if USE_TX_GET_BUFF
exit_unlock:
#endif

	net_unlock();
}


static void xradio_drv_txavail_work(FAR void *arg)
{
	FAR struct net_driver_s *dev = arg;
	FAR struct xradio_dev_s *priv = (struct xradio_dev_s *)dev->d_private;

	/* Lock the network and serialize driver operations if necessary.
	 * NOTE: Serialization is only required in the case where the driver work
	 * is performed on an LP worker thread and where more than one LP worker
	 * thread has been configured.
	 */
	net_lock();
	if(priv->if_up) {
#if USE_TX_GET_BUFF
		if(priv->cur_tx_frame == NULL) {
	/* Ignore the notification if the interface is not yet up */
			if(xradio_get_tx_payload_buffer(priv)) {
				goto exit_unlock;
			}

			dev->d_buf = priv->cur_tx_frame->data;
			dev->d_len = 0;
#endif
		}
		if(dev->d_buf)
			(void)devif_poll(&priv->xr_dev,xradio_drv_txpoll);
	}
#if USE_TX_GET_BUFF
exit_unlock:
#endif
	net_unlock();

}

static void xradio_drv_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct net_driver_s *dev = (FAR struct net_driver_s *)arg;
  FAR struct xradio_dev_s *priv = (struct xradio_dev_s *)dev->d_private;

  /* Schedule to perform the interrupt processing on the worker thread. */

  work_queue(LPWORK, &priv->xr_pollwork, xradio_drv_poll_work, dev, 0);
}


extern void net_hex_dump(char *pref, int width, unsigned char *buf, int len);
static int xradio_drv_txavail(FAR struct net_driver_s *dev)
{
	FAR struct xradio_dev_s *priv = (struct xradio_dev_s *)dev->d_private;

	/* Is our single work structure available?  It may not be if there are
	 * pending interrupt actions and we will have to ignore the Tx
	 * availability action.
	 */
	if (work_available(&priv->xr_pollwork)) {
	    /* Schedule to serialize the poll on the worker thread. */
	    work_queue(LPWORK, &priv->xr_pollwork, xradio_drv_txavail_work, dev, 0);
	}

	return OK;
}
#if 0
static void xradio_receive(FAR struct xradio_dev_s *priv)
{
  do
    {
      /* Request frame buffer from bus interface */
	  //net_hex_dump(__func__,20,priv->xr_dev.d_buf,priv->xr_dev.d_len);
#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap */

       pkt_input(&priv->xr_dev);
#endif

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(&priv->xr_dev);

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&priv->xr_dev);
          ipv4_input(&priv->xr_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->xr_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
              if (IFF_IS_IPv4(priv->xr_dev.d_flags))
#endif
                {
                  arp_out(&priv->xr_dev);
                }
#ifdef CONFIG_NET_IPv6
              else
                {
                  neighbor_out(&kel->xr_dev);
                }
#endif

              /* And send the packet */

              xradio_drv_transmit(priv);
            }
          else
            {
              /* Release RX frame buffer */
				;
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("Iv6 frame\n");
          NETDEV_RXIPV6(&priv->xr_dev);

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->xr_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->xr_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
              if (IFF_IS_IPv4(priv->xr_dev.d_flags))
                {
                  arp_out(&priv->xr_dev);
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
                {
                  neighbor_out(&priv->xr_dev);
                }
#endif

              /* And send the packet */

              xradio_drv_transmit(priv);
            }
          else
            {
              /* Release RX frame buffer */
				;
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == htons(ETHTYPE_ARP))
        {
          arp_arpin(&priv->xr_dev);
          NETDEV_RXARP(&priv->xr_dev);

          /* If the above function invocation resulted in data that should be
           * sent out on the network, the field  d_len will set to a value > 0.
           */

          if (priv->xr_dev.d_len > 0)
            {
              xradio_drv_transmit(priv);
            }
          else
            {
              /* Release RX frame buffer */
				;
            }
        }
      else
#endif
        {
          /* On some routers, it may constantly receive mysterious packet...
           * https://www.wireshark.org/docs/wsar_html/epan/etypes_8h.html
           * for more etypes definitions.
           */

          NETDEV_RXDROPPED(&priv->xr_dev);
		  ;
        }
    }
  while (0); /* While there are more packets to be processed */
}
#else
static void xradio_reply(FAR struct xradio_dev_s *priv)
{
	if(priv->xr_dev.d_len > 0) {
#ifdef CONFIG_NET_IPv6
		if(IFF_IS_IPv4(priv->dev.d_flags))
#endif
		{
			arp_out(&priv->xr_dev);
		}
#ifdef CONFIG_NET_IPv6
		else {
			neighbor_out(&priv->xr_dev);
		}
#endif
		xradio_drv_transmit(priv);
	}
}
static void xradio_receive(FAR struct xradio_dev_s *priv)
{

#ifdef CONFIG_NET_PKT
	pkt_input(&priv->xr_dev);
#endif
	if(BUF->type == HTONS(TPID_8021QVLAN)) {
		XR_DEV_ERR("ERROR:Not implement!!!!\n");
	}

#ifdef CONFIG_NET_IPv4
	if(BUF->type == (HTONS(ETHTYPE_IP))) {   //case IP Packet:
        ninfo("IPv4 frame\n");
        NETDEV_RXIPV6(&priv->xr_dev);
		arp_ipin(&priv->xr_dev); //update send mac address based on ip address.
		ipv4_input(&priv->xr_dev);
		xradio_reply(priv);
	}else
#endif

#ifdef CONFIG_NET_IPv6
    if (BUF->type == HTONS(ETHTYPE_IP6))
	{
        NETDEV_RXIPV6(&priv->xr_dev);

        ipv6_input(&priv->xr_dev);

        xradio_reply(priv);
	}
    else
#endif
#ifdef CONFIG_NET_ARP
	if (BUF->type == htons(ETHTYPE_ARP)) //case ARP Packet:
	{
        ninfo("ARP frame\n");
		arp_arpin(&priv->xr_dev);
		NETDEV_RXARP(&priv->xr_dev);

		if (priv->xr_dev.d_len > 0)
        {
            xradio_drv_transmit(priv);
        }
    } else
#endif
	{
		NETDEV_RXDROPPED(&priv->xr_dev);
	}
}

#endif
void xradio_rx_notify_rx(uint8_t *data, uint16_t len)
{
	FAR struct xradio_dev_s *priv = g_xradio_priv;
	void *old_buff = NULL;

	net_lock();
	old_buff = priv->xr_dev.d_buf;
#if USE_TX_GET_BUFF
	priv->xr_dev.d_buf = data;
#else
	memcpy(priv->xr_dev.d_buf,data,len);
#endif
	priv->xr_dev.d_len = len;

	xradio_receive(priv);

	priv->xr_dev.d_buf = old_buff;
	net_unlock();

}

struct net_driver_s* xradio_get_net_dev(void)
{
	FAR struct xradio_dev_s *priv = g_xradio_priv;
	return &priv->xr_dev;
}

static int xradio_drv_ifup(FAR struct net_driver_s *dev)
{
	FAR struct xradio_dev_s *priv = (struct xradio_dev_s *)dev->d_private;

	XR_DEV_DBG("priv addr:%p,dev addr:%p\n",priv,&priv->xr_dev);

#ifdef CONFIG_NET_IPv4
	ninfo("Bringing up: %d.%d.%d.%d\n",
		dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
		(dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
	ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
		dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
		dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
		dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

	xradio_wlan_init(WLAN_MODE_STA,dev); //init xradio device
  /* Set and activate a timer process */

	wd_start(priv->xr_txpolldog, XRADIO_DRV_WDDELAY, xradio_drv_poll_expiry, 1,
           (wdparm_t)dev);

    if ((dev->d_flags & IFF_UP) == 0) {
          /* No, bring the interface up now */
		dev->d_flags |= IFF_UP;
    }

	priv->if_up = true;
	return 0;
}

static int xradio_drv_ifdown(FAR struct net_driver_s *dev)
{
	FAR struct xradio_dev_s *priv = (struct xradio_dev_s *)dev->d_private;
	irqstate_t flags;

	xradio_wlan_deinit(); //deinit xradio device

  /* Disable the interrupt */
	flags = enter_critical_section();

  /* Cancel the TX poll timer and work */

	wd_cancel(priv->xr_txpolldog);
	work_cancel(LPWORK, &priv->xr_pollwork);

	priv->if_up = false;

	leave_critical_section(flags);
	return 0;
}

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int xradio_drv_addmac(FAR struct net_driver_s *dev,
                                FAR const uint8_t *mac)
{
	return 0;
}
#endif

#ifdef CONFIG_NET_IGMP
static int xradio_drv_rmmac(FAR struct net_driver_s *dev,
                               FAR const uint8_t *mac)
{
	return 0;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int xradio_drv_ioctl(FAR struct net_driver_s *dev, int cmd,
                               unsigned long arg)
{
	int ret;
	FAR struct xradio_dev_s *priv = (struct xradio_dev_s *)dev->d_private;

	switch(cmd) {
		case SIOCSIWSCAN:
			ret = xradio_wl_start_scan(priv,(void *)arg);
			break;

		case SIOCGIWSCAN:
			ret = xradio_wl_get_scan_results(priv,(void *)arg);
			break;

		case SIOCSIFHWADDR:    /* Set device MAC address */
			ret = xradio_wl_set_mac_address(priv, (void *)arg);
	        break;

		case SIOCSIWENCODEEXT: //set password
			ret = xradio_wl_set_encode_ext(priv,(void *)arg);
			break;

		case SIOCSIWESSID: /* Set ESSID (network name) */
			ret = xradio_wl_set_ssid(priv,(void *)arg);
			break;

		case SIOCSIWMODE:
			ret = xradio_wl_set_mode(priv,(void *)arg);
			break;
		case SIOCGIWFREQ:     /* Get channel/frequency (Hz) */
		case SIOCGIWMODE:     /* Get operation mode */
		case SIOCGIWAP:       /* Get access point MAC addresses */
		case SIOCSIWAP:
		case SIOCGIWESSID:    /* Get ESSID */
		case SIOCSIWRATE:     /* Set default bit rate (bps) */
		case SIOCGIWRATE:     /* Get default bit rate (bps) */
		case SIOCSIWTXPOW:    /* Set transmit power (dBm) */
		case SIOCGIWTXPOW:    /* Get transmit power (dBm) */
		default:
		nerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
		ret = -ENOTTY;  /* Special return value for this case */
		break;
	}
	return ret;
}
#endif

static uint8_t xradio_mac_addr[] = { 0xfc,0xd7,0x3a, 0xa4, 0xb3, 0xaf };

static void xradio_mac_random(uint8_t mac_addr[6])
{
	int i;
	for (i = 0; i < 6; ++i) {
		mac_addr[i] = (uint8_t)((uint32_t)((rand() & 0xffffff) | (clock_systimer() << 24)));
	}
	mac_addr[0] = 0xFC;
}

static void xradio_drv_if_up_work(FAR void *arg)
{
	FAR struct xradio_dev_s *priv = (struct xradio_dev_s *)arg;
	xradio_drv_ifup(&priv->xr_dev);
}

int xradio_netdev_register(FAR struct xradio_dev_s *priv)
{
	if(priv == NULL) {
		return -ENOMEM;
	}

	g_xradio_priv = priv;

	xradio_mac_random(xradio_mac_addr);

	memcpy(priv->xr_dev.d_mac.ether.ether_addr_octet,xradio_mac_addr,6);

#if (USE_TX_GET_BUFF == 0)
	priv->xr_dev.d_buf     = g_pktbuf;
#endif
	priv->xr_dev.d_ifup    = xradio_drv_ifup;
	priv->xr_dev.d_ifdown  = xradio_drv_ifdown;
	priv->xr_dev.d_txavail = xradio_drv_txavail;
#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
	priv->xr_dev.d_addmac  = xradio_drv_addmac;
#endif
#ifdef CONFIG_NET_IGMP
	priv->xr_dev.d_rmmac   = xradio_drv_rmmac;
#endif
#ifdef CONFIG_NETDEV_IOCTL
	priv->xr_dev.d_ioctl     = xradio_drv_ioctl;
#endif
	priv->xr_dev.d_private = (FAR void *)priv;

    //priv->xr_dev.d_flags |= IFF_UP;

	priv->xr_txpolldog = wd_create();

	netdev_register(&priv->xr_dev, NET_LL_IEEE80211);

	work_queue(LPWORK, &priv->xr_pollwork, xradio_drv_if_up_work, priv, (CLK_TCK/2));
	return 0;
}
