/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Author: laumy <liumingyuan@allwinnertech.com>
* Date  : 2020-03-09
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/arp.h>

#include <xradio_netif.h>
#include <xradio_driver.h>
#include <net/wlan/wlan.h>
#include <aw/wifi/wifi_adapter.h>

#include "xradio_netdev.h"
#include "xradio_wlan.h"

#define XR_DRV_DBG printf
#define XR_DRV_ERR printf

#define XRADIO_SCAN_TIMEOUT_TICK    (5 * CLOCKS_PER_SEC)
#define XRADIO_CONNECT_TIMEOUT_TICK (10 * CLOCKS_PER_SEC)
#define XRADIO_DEVICE_COUNT         (1)
#define XRADIO_PASSWORD_LEN         (64)

#define IW_EV_LEN(field) \
	  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))


static struct xradio_dev_s *g_xr_wlan_dev[XRADIO_DEVICE_COUNT + 1];

static char password[XRADIO_PASSWORD_LEN] = {0};

static void xradio_state_timeout(int argc, wdparm_t arg1, ...)
{
  FAR struct xradio_state_s *state = (FAR struct xradio_state_s *)arg1;

  if (state->status < XRADIO_STATUS_RUN)
    {
      return;
    }

  state->status = XRADIO_STATUS_TIMEOUT;
  nxsem_post(&state->mutex);
}

static int xradio_state_run(FAR struct xradio_state_s *state, int32_t delay)
{
  if (state->status == XRADIO_STATUS_DONE)
    {
      return 0;
    }

  state->status = XRADIO_STATUS_RUN;

  return wd_start(state->timeout, delay,
      xradio_state_timeout, 1, (wdparm_t)state);
}

static int xradio_state_wait(FAR struct xradio_state_s *state)
{
  return state->status == XRADIO_STATUS_DONE ? 0 : nxsem_wait(&state->mutex);
}

static void xradio_state_post(FAR struct xradio_state_s *state, int status)
{
  if (state->status == XRADIO_STATUS_RUN)
    {
      wd_cancel(state->timeout);
      nxsem_post(&state->mutex);
    }

  state->status = status;
}

static void xradio_state_deinit(FAR struct xradio_state_s *state)
{
  wd_delete(state->timeout);
}

static int xradio_state_init(FAR struct xradio_state_s *state)
{
  if (nxsem_init(&state->mutex, 0, 0) != OK)
    {
      return -ENOMEM;
    }

  state->timeout = wd_create();
  if (!state->timeout)
    {
      return -ENOMEM;
    }

  state->status = XRADIO_STATUS_DISABLED;

  return 0;
}


static char *xradio_wl_iwe_add_event(char *stream, char *stop,
                                     struct iw_event *iwe, int event_len)
{
  if (stream + event_len > stop)
    {
      return stream;
    }

  iwe->len = event_len;

  return stream + event_len;
}

#define MAX_SCAN_RESULTS_NUM 32

static int xradio_wl_format_scan_results(struct xradio_dev_s *priv, struct iwreq *iwr)
{
	struct iw_event *iwe;
	char *start, *stop;
	int i;
	FAR struct xradio_state_s *state = &priv->scan;
	wlan_sta_scan_results_t *results = (wlan_sta_scan_results_t *)state->data;
	start = iwr->u.data.pointer;
	stop = (char *)iwr->u.data.pointer + iwr->u.data.length;

	for (i = 0; i < results->num; i++) {
      iwe = (struct iw_event *)start;
      iwe->cmd = SIOCGIWAP;
      iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
      memcpy(&iwe->u.ap_addr.sa_data, results->ap[i].bssid, IFHWADDRLEN);
      start = xradio_wl_iwe_add_event(start, stop, iwe, IW_EV_LEN(ap_addr));

      iwe = (struct iw_event *)start;
      iwe->cmd = SIOCGIWESSID;
      iwe->u.essid.flags = 0;
      iwe->u.essid.length = results->ap[i].ssid.ssid_len;
      iwe->u.essid.pointer = (FAR void *)sizeof(iwe->u.essid);
      memcpy(&iwe->u.essid + 1, results->ap[i].ssid.ssid, results->ap[i].ssid.ssid_len);
      start = xradio_wl_iwe_add_event(start, stop, iwe,
                                   IW_EV_LEN(essid) + ((results->ap[i].ssid.ssid_len + 3) & -4));

      iwe = (struct iw_event *)start;
      iwe->cmd = IWEVQUAL;
      iwe->u.qual.qual = 0;
      iwe->u.qual.level = results->ap[i].level;
      iwe->u.qual.noise = 0;
      iwe->u.qual.updated |= IW_QUAL_DBM;
      start = xradio_wl_iwe_add_event(start, stop, iwe, IW_EV_LEN(qual));

      iwe = (struct iw_event *)start;
      iwe->cmd = SIOCGIWFREQ;
      iwe->u.freq.e = 0;
      iwe->u.freq.m = results->ap[i].channel;
      start = xradio_wl_iwe_add_event(start, stop, iwe, IW_EV_LEN(freq));

      iwe = (struct iw_event *)start;
      iwe->cmd = SIOCGIWENCODE;
      iwe->u.data.flags = IW_ENCODE_DISABLED;
      iwe->u.data.length = 0;
      iwe->u.essid.pointer = NULL;
      start = xradio_wl_iwe_add_event(start, stop, iwe, IW_EV_LEN(data));
    }

	return start - (char *)iwr->u.data.pointer;
}

void xradio_wl_scan_handler(int index,uint32_t event,void *arg)
{
	FAR struct xradio_dev_s *priv = g_xr_wlan_dev[index];
	FAR struct xradio_state_s *state = &priv->scan;
	wlan_sta_scan_results_t *scan_results = (wlan_sta_scan_results_t *)state->data;

	//handle manual scan only.
	if(state->scan_req != XR_MANUAL_SCAN_REQ)
		return;

	if(event == XR_WIFI_SCAN_SUCCESS) {
		if(wlan_sta_scan_result(scan_results) != 0) {
			XR_DRV_ERR("get scan results failed.");
		}
		state->size = MAX_SCAN_RESULTS_NUM * sizeof(wlan_sta_ap_t);
	}

	xradio_state_post(state,XRADIO_STATUS_DONE);
}

void xradio_wl_connection_handler(int index, uint32_t event,void *arg)
{
  FAR struct xradio_dev_s *priv = g_xr_wlan_dev[index];
  FAR struct xradio_state_s *state = &priv->conn;

	if (!priv) {
		return;
	}

	if(state->scan_req != XR_NORMAL_SCAN_REQ)
		return;

	state = &priv->conn;

	if(event == XR_WIFI_CONNECTED)
	    xradio_state_post(state, XRADIO_STATUS_DONE);
	//TODO AP MODE.
}


int xradio_wl_get_scan_results(struct xradio_dev_s *priv, struct iwreq *iwr)
{
	FAR struct xradio_state_s *state = &priv->scan;
	wlan_sta_scan_results_t *scan_results = (wlan_sta_scan_results_t *)state->data;
	int ret = OK;

	if (state->status == XRADIO_STATUS_RUN) {
		ret = -EAGAIN;
		goto exit_failed;
    }

	if (state->status != XRADIO_STATUS_DONE) {
	    ret = -EINVAL;
	    goto exit_failed;
	}

	if ((ret = xradio_state_wait(state)) < 0) {
	    goto exit_failed;
	}

	if (!state->data) {
	    ret = OK;
		iwr->u.data.length = 0;
		goto exit_sem_post;
    }

	if (state->size <= 0) {
		ret = OK;
		iwr->u.data.length = 0;
		goto exit_free_buffer;
	}

	if (iwr->u.data.pointer == NULL ||
		  iwr->u.data.length < state->size) {
		ret = -E2BIG;
		iwr->u.data.pointer = NULL;
		iwr->u.data.length = state->size;
		goto exit_sem_post;
    }

	iwr->u.data.length = xradio_wl_format_scan_results(priv, iwr);

exit_free_buffer:

	kmm_free(scan_results->ap);
	kmm_free(scan_results);

	state->data = NULL;
	state->size = 0;

exit_sem_post:
	nxsem_post(&state->mutex);

exit_failed:
	if (ret < 0){
		iwr->u.data.length = 0;
    }

	return ret;
}


int xradio_wl_start_scan(struct xradio_dev_s *priv,struct iwreq *iwr)
{
	FAR struct xradio_state_s *state = &priv->scan;
	wlan_sta_scan_results_t *scan_results = NULL;

	if(wlan_sta_scan_once() != 0) {
		XR_DRV_ERR("xradio start scan failed.\n");
		return -1;
	}
	if(state->data == NULL) {
		scan_results = (wlan_sta_scan_results_t*)kmm_malloc(sizeof(wlan_sta_scan_results_t));
		if(scan_results == NULL) {
			XR_DRV_ERR("scan results malloc failed.\n");
			goto failed;
		}
		scan_results->ap = kmm_malloc(MAX_SCAN_RESULTS_NUM * sizeof(wlan_sta_ap_t));
		if(scan_results->ap == NULL) {
			XR_DRV_ERR("scan results ap malloc failed.\n");
			goto failed;
		}
	}
	state->scan_req = XR_MANUAL_SCAN_REQ;
	state->data = (void*)scan_results;
	state->size = 0;

	return xradio_state_run(state,XRADIO_SCAN_TIMEOUT_TICK);
failed:
	state->status = XRADIO_STATUS_DISABLED;
	return -ENOMEM;
}

int xradio_wl_set_mac_address(struct xradio_dev_s *priv,struct iwreq *iwr)
{

	XR_DRV_DBG("[%s,%d] enter\n",__func__,__LINE__);
	return 0;
}

int xradio_wl_set_encode_ext(struct xradio_dev_s *priv,struct iwreq *iwr)
{
	struct iw_encode_ext *ext;
	FAR struct xradio_state_s *state = &priv->conn;

#if 1
	state->data = NULL;

	ext = (struct iw_encode_ext *)iwr->u.encoding.pointer;

	memcpy(password, &ext->key, ext->key_len);

	password[ext->key_len] = '\0';

	XR_DRV_DBG("set passphrase successful(%s).\n",password);
#else
	char *pwd = NULL;

	pwd = (char *)kmm_malloc(sizeof(XRADIO_PASSWORD_LEN));
	if(NULL == pwd) {
		XR_DRV_ERR("pwd malloc failed.");
		return -1;
	}
	memset(pwd, 0, sizeof(sizeof(XRADIO_PASSWORD_LEN)));

	ext = (struct iw_encode_ext *)iwr->u.encoding.pointer;

	memcpy(pwd, &ext->key, ext->key_len);
	pwd[ext->key_len] = '\0';

	state->data = (void *)pwd;
	state->size = ext->key_len + 1;
	XR_DRV_DBG("set passphrase successful(%p:%s).\n",state->data,state->data);
#endif
	return 0;
}

int xradio_wl_set_ssid(struct xradio_dev_s *priv,struct iwreq *iwr)
{

	FAR struct xradio_state_s *state = &priv->conn;
	int ret;

	if (!iwr->u.essid.flags) {
	    return 0;
	}

	if(priv->mode == XR_WLAN_MODE_STA) {

		state->scan_req = XR_NORMAL_SCAN_REQ;

		//wifi_connect(iwr->u.essid.pointer, state->data);
		wifi_connect(iwr->u.essid.pointer, password);

		ret = xradio_state_run(state, XRADIO_CONNECT_TIMEOUT_TICK);
		if (ret < 0) {
			goto end;
		}

		if ((ret = xradio_state_wait(state)) < 0) {
			goto end;
		}

		if (state->status != XRADIO_STATUS_DONE) {
		    ret = -ETIME;
		}
	} else if(priv->mode == XR_WLAN_MODE_HOSTAP) {
		//wifi_ap_start(iwr->u.essid.pointer, state->data);
		wifi_ap_start(iwr->u.essid.pointer,password);
	}

	state->status = XRADIO_STATUS_DISABLED;
end:
	if(ret < 0) {
		XR_DRV_ERR("set ssid error(%d)\n",ret);
	}
	if(state->data != NULL) {
		kmm_free(state->data);
		state->data = NULL;
	}
	return ret;
}

int xradio_wl_set_mode(struct xradio_dev_s *priv,struct iwreq *iwr)
{

	if(iwr->u.mode == IW_MODE_MASTER) {
		wifi_on(XR_WLAN_MODE_HOSTAP);
		priv->mode = XR_WLAN_MODE_HOSTAP;
	} else if(iwr->u.mode == IW_MODE_INFRA) {
		wifi_on(XR_WLAN_MODE_STA);
		priv->mode = XR_WLAN_MODE_STA;
	}else {
		return -EINVAL;
	}
	return 0;
}

struct xradio_dev_s *xradio_allocate_device(int devnum)
{
	FAR struct xradio_dev_s *priv;
	int ret;

	priv = (FAR struct xradio_dev_s *)kmm_zalloc(sizeof(*priv));
	if (!priv) {
		return NULL;
	}

	memset(priv,0,sizeof(struct xradio_dev_s));

	ret = xradio_state_init(&priv->scan);
	ret |= xradio_state_init(&priv->conn);

	if (ret) {
		kmm_free(priv);
		return NULL;
	}

	priv->devnum = devnum;

	return priv;
}

void xradio_free_device(FAR struct xradio_dev_s *priv)
{
  xradio_state_deinit(&priv->scan);
  xradio_state_deinit(&priv->conn);

  kmm_free(priv);
}

int xradio_drv_init(void)
{
	FAR struct xradio_dev_s *priv = NULL;
	int ret;
	int i;

	for(i=0;i<XRADIO_DEVICE_COUNT;i++) {
		priv = NULL;
		priv = xradio_allocate_device(i);
		if(!priv) {
			ret = -ENOMEM;
			goto free_dev;
		}

		ret = xradio_netdev_register(priv);
		if(ret < 0) {
			xradio_free_device(priv);
			goto free_dev;
		}
		g_xr_wlan_dev[i] = priv;
	}
	return ret;

free_dev:
	for(i=0;g_xr_wlan_dev[i];i++) {
		netdev_unregister(&g_xr_wlan_dev[i]->xr_dev);
		xradio_free_device(g_xr_wlan_dev[i]);
		g_xr_wlan_dev[i] = NULL;
	}
	return ret;
}
