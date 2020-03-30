#include <stdio.h>
#include "xradio_wlan.h"
#include "xradio_driver.h"

#define XR_WLAN_DRV_DBG printf
#define XR_WLAN_DRV_ERR printf
void xradio_msg_indicate(uint32_t event,uint32_t data,void *arg)
{
	//XR_WLAN_DRV_DBG("event:%d\n",event);
	switch(event) {
		case XR_WIFI_SCAN_SUCCESS:
		case XR_WIFI_SCAN_FAILED:
			xradio_wl_scan_handler(0,event,NULL);
			break;
		case XR_WIFI_CONNECTED:
		case XR_WIFI_DISCONNECTED:
		case XR_WIFI_4WAY_HANDSHAKE_FAILED:
		case XR_WIFI_CONNECT_FAILED:
		case XR_WIFI_CONNECTION_LOSS:
			xradio_wl_connection_handler(0,event,NULL);
			break;
		case XR_WIFI_NETWORK_UP:
			break;
		case XR_WIFI_NETWORK_DOWN:
			break;
		default:
			XR_WLAN_DRV_DBG("event not support(%d)\n",event);
			break;
	}
}
