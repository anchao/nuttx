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

#include <debug.h>
#include <string.h>
#include "net/wlan/wlan.h"
#include <stdio.h>
#include <sys/mbuf_0.h>
#include <stdbool.h>

#define ETH_DBG_ON      0
#define ETH_WRN_ON      1
#define ETH_ERR_ON      1
#define ETH_ABORT_ON    0

#define ETH_SYSLOG      printf
#define ETH_ABORT()     sys_abort()

#define ETH_LOG(flags, fmt, arg...)     \
    do {                                \
        if (flags)                      \
            ETH_SYSLOG("%d" fmt,__LINE__, ##arg);     \
    } while (0)

#define ETH_DBG(fmt, arg...)   ETH_LOG(ETH_DBG_ON, "[eth] "fmt, ##arg)
#define ETH_WRN(fmt, arg...)   ETH_LOG(ETH_WRN_ON, "[eth W] "fmt, ##arg)
#define ETH_ERR(fmt, arg...)                            \
    do {                                                \
        ETH_LOG(ETH_ERR_ON, "[eth E] %s():%d, "fmt,     \
               __func__, __LINE__, ##arg);              \
    } while (0)

struct ethernetif {
	struct netif nif;
	bool netdev_open;
	enum wlan_mode mode;
};

#define ethernetif2netif(eth)	((struct netif *)(eth))
#define netif2ethernetif(nif)	((struct ethernetif *)(nif))

static struct ethernetif g_eth_netif = {
	.netdev_open = false,
};

void net_hex_dump(char *pref, int width, unsigned char *buf, int len)
{
       int i,n;
    for (i = 0, n = 1; i < len; i++, n++){
        if (n == 1)
            printf("%s ", pref);
        printf("%2.2X ", buf[i]);
        if (n == width) {
            printf("\n");
            n = 0;
        }
    }
    if (i && n!=1)
        printf("\n");
}


#if 0
static err_t ethernetif_sta_init(struct netif *nif)
{
	ETH_DBG("-+-+-+\n");
	return ERR_OK;
}

static err_t ethernetif_hostap_init(struct netif *nif)
{
	ETH_DBG("-+-+-+\n");
	return ERR_OK;
}

static err_t ethernetif_monitor_init(struct netif *nif)
{
	ETH_DBG("-+-+-+\n");
	return ERR_OK;

}
static err_t tcpip_null_input(struct pbuf *p, struct netif *inp)
{
	ETH_DBG("-+-+-+\n");
	//pbuf_free(p); TODO pbuf free.
	return ERR_OK;
}

static err_t tcpip_input(struct pbuf *p, struct netif *inp)
{
	ETH_DBG("-+-+-+\n");
	//TODO,driver input callback.
	return ERR_OK;
}
#endif

static void ethernetif_hw_deinit(struct netif *nif)
{
	wlan_if_delete(nif->state);
}

static err_t ethernetif_hw_init(struct netif *nif, enum wlan_mode mode)
{
	char name[4];
	name[0] = nif->name[0];
	name[1] = nif->name[1];
	name[2] = nif->num + '0';
	name[3] = '\0';

	nif->state = wlan_if_create(mode, nif, name);
	if (nif->state == NULL) {
		ETH_ERR("wlan interface create failed\n");
		return ERR_IF;
	}

	if (wlan_get_mac_addr(nif, nif->hwaddr, ETHARP_HWADDR_LEN) != ETHARP_HWADDR_LEN) {
		ETH_DBG("get mac addr failed\n");
		wlan_if_delete(nif->state);
		nif->state = NULL;
		return ERR_IF;
	}
	return ERR_OK;
}

#if (__CONFIG_MBUF_IMPL_MODE == 0)
err_t ethernetif_raw_input(struct netif *nif, uint8_t *data, u16_t len)
{
#if 0
	struct xr_frame_s frame;
	frame.data = (uint8_t*)malloc(sizeof(uint8_t) *len);
	if(frame.data == NULL) {
		ETH_ERR("rx malloc failed.\n");
	}
	frame.len = len;

	memcpy(frame.data,data,len);

	//net_hex_dump(__func__,20,frame.data,frame.len);

	xradio_rx_poll(&frame);

	free(frame.data);
#endif

	//net_hex_dump(__func__,20,frame.data,frame.len);

	//xradio_rx_poll(&frame);
	void xradio_rx_notify_rx(uint8_t *data, uint16_t len);
	xradio_rx_notify_rx(data,len);
	return ERR_OK;
}
#endif /* (__CONFIG_MBUF_IMPL_MODE == 0) */
static struct mbuf *trans_to_mbuf(uint8_t *data, uint16_t len)
{
	struct mbuf *m;
	uint8_t *_data;

	m = mb_get(len,1 | MBUF_GET_FLAG_LIMIT_TX);
	if(NULL == m) {
		ETH_ERR("mbuff get failed.\n");
		return NULL;
	}

	_data = mtod(m,uint8_t *);
	memcpy(_data,data,len);
	return m;
}
#if 0
//TODO: tx to driver
err_t ethernetif_linkoutput(struct netif *nif, struct pbuf *p)
#else
err_t ethernetif_linkoutput(uint8_t *data, uint16_t len)
#endif
{
	struct mbuf *m;
	int ret;
	struct netif *nif;
	nif = ethernetif2netif(&g_eth_netif);

	//net_hex_dump(__func__,20,frame->data,frame->len);

#if (__CONFIG_MBUF_IMPL_MODE == 0)
	m = trans_to_mbuf(data,len);
#elif (__CONFIG_MBUF_IMPL_MODE == 1)
	;
#endif
	ret = wlan_linkoutput(nif, m);
	if(ret < 0) {
		ETH_ERR("linkoutput failed (%d)\n", ret);
	}
	return ERR_OK;
}
#if 0
static uint8_t m_mac_addr[] = { 0x00, 0x80, 0xE1, 0x29, 0xE8, 0xD1 };

static void ethernetif_mac_random(uint8_t mac_addr[6])
{
	int i;
	for (i = 0; i < 6; ++i) {
		mac_addr[i] = (uint8_t)OS_Rand32();
	}
	mac_addr[0] &= 0xFC;
}
#endif

void ethernetif_set_mac_addr(void)
{
	extern struct net_driver_s* xradio_get_net_dev(void);

	struct net_driver_s *xr_dev = xradio_get_net_dev();

	wlan_set_mac_addr(NULL,xr_dev->d_mac.ether.ether_addr_octet,ETHARP_HWADDR_LEN);
}
struct netif *ethernetif_create(enum wlan_mode mode)
{
	struct netif *nif;
	ETH_DBG("-+-+-+\n");

	nif = ethernetif2netif(&g_eth_netif);
	memset(nif, 0, sizeof(*nif));
	g_eth_netif.mode = mode;
	nif->name[0]='w';
	nif->name[1]='l';
	nif->hwaddr_len = 6;

	//extern struct net_driver_s* xradio_get_net_dev(void);
	//nif->net_dev = xradio_get_net_dev();
	nif->net_dev = NULL;

	nif->mtu=1500;

	ethernetif_hw_init(nif, mode);
	return nif;

}

void ethernetif_delete(struct netif *nif)
{
	ethernetif_hw_deinit(nif);
}

static wlan_event_cb_func event_cb = NULL;

void wlan_set_event_callback(wlan_event_cb_func cb)
{
	event_cb = cb;
}
static void wlan_event_callback(uint32_t param0, uint32_t param1)
{
	ETH_DBG("para0:%d,para1:%d\n",param0,param1);
	if(event_cb) {
		event_cb(param0,param1);
	}
}

struct netif* wlan_get_netif(void)
{
	if(g_eth_netif.netdev_open == true)
		return ethernetif2netif(&g_eth_netif);
	return NULL;
}

int xradio_wlan_init(enum wlan_mode mode,struct net_driver_s *dev)
{
	struct netif *nif;

	ETH_DBG("enter mode %d\n",mode);
	if(g_eth_netif.netdev_open == true)
		return 0;
#ifndef __CONFIG_ETF_CLI

	wlan_set_mac_addr(NULL,dev->d_mac.ether.ether_addr_octet,ETHARP_HWADDR_LEN);

	wlan_attach(wlan_event_callback);

	nif = ethernetif_create(mode);

	if(nif == NULL) {
		ETH_ERR("create net failed.");
		return -1;
	}

	nif->net_dev = dev;

	//init ip address.
	g_eth_netif.netdev_open = true;
#endif
}

void xradio_wlan_deinit(void)
{
	struct netif *nif;
	nif = ethernetif2netif(&g_eth_netif);
	g_eth_netif.netdev_open = false;
	wlan_netif_delete(nif);
	wlan_detach();
	return 0;
}

enum wlan_mode ethernetif_get_mode(struct netif *nif)
{
	ETH_DBG("-+-+-+ %p\n",nif);
	if (nif == ethernetif2netif(&g_eth_netif)) {
		return g_eth_netif.mode;
	} else {
		return WLAN_MODE_INVALID;
	}

}

void *ethernetif_get_state(struct netif *nif)
{
	ETH_DBG("-+-+-+\n");
	return (nif ? nif->state : NULL);
}
