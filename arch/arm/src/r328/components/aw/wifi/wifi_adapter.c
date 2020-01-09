#include "net/wlan/wlan_defs.h"
#include "net/wlan/wlan.h"
#include "net_ctrl.h"
#include "sysinfo.h"
#include "wifi_adapter.h"
#include <string.h>
#include <stdio.h>
#include "wifimanager.h"

#include "net/wlan/wlan_ext_req.h"

#define WIFI_DEBUG(...) do { \
                            printf("[%s,%d]:",__func__,__LINE__); \
                            printf(__VA_ARGS__); \
                        } while (0)

#define WIFI_ERROR(...) do { \
                            printf("[%s,%d]:",__func__,__LINE__); \
                            printf(__VA_ARGS__); \
                        } while (0)

#define MAX_SCAN_RESULTS	32

static uint16_t scan_status;
static uint16_t connect_status;

extern void aw_wifi_indicate_event_handle(aw_wifi_state_t event_cmd);

void wifi_msg_process(uint32_t event, uint32_t data, void *arg)
{
	uint16_t type = EVENT_SUBTYPE(event);
	WIFI_DEBUG("%s msg (%u, %u)\n", __func__, type, data);
	aw_wifi_state_t state = WIFI_MAX;

	switch (type) {
	case NET_CTRL_MSG_WLAN_CONNECTED:
		connect_status = type;
		state = WIFI_CONNECTED;
		break;
	case NET_CTRL_MSG_WLAN_DISCONNECTED:
		connect_status = type;
		state = WIFI_DISCONNECTED;
		break;
	case NET_CTRL_MSG_WLAN_SCAN_SUCCESS:
		scan_status = type;
		break;
	case NET_CTRL_MSG_WLAN_SCAN_FAILED:
		scan_status = type;
		break;
	case NET_CTRL_MSG_WLAN_4WAY_HANDSHAKE_FAILED:
		break;
	case NET_CTRL_MSG_WLAN_CONNECT_FAILED:
		state = WIFI_HANDSHAKE_FAILED;
		break;
	case NET_CTRL_MSG_CONNECTION_LOSS:
		break;
	case NET_CTRL_MSG_NETWORK_UP:
		break;
	case NET_CTRL_MSG_NETWORK_DOWN:
		break;
#if (!defined(__CONFIG_LWIP_V1) && LWIP_IPV6)
	case NET_CTRL_MSG_NETWORK_IPV6_STATE:
		break;
#endif
	default:
		WIFI_DEBUG("unknown msg (%u, %u)\n", type, data);
		break;
	}
	aw_wifi_indicate_event_handle(state);
}

int wifi_event_init(void)
{
	observer_base *ob;

	WIFI_DEBUG("wlan init, register network observer\n");
	ob = sys_callback_observer_create(CTRL_MSG_TYPE_NETWORK,
									  NET_CTRL_MSG_ALL,
									  wifi_msg_process, NULL);
	if (ob == NULL)
		return -1;
	if (sys_ctrl_attach(ob) != 0)
		return -1;

	WIFI_DEBUG("wifi init success!!\n");
	return 0;
}

void wifi_deinit(void)
{
	;
}

#define TRY_FIND_SSID_TIME 1
int wifi_scan(aw_wifi_scan_results_t *scan_result,int max_num)
{
	int ret = -1;
	int j;
	int size;
	int i;
	wlan_sta_scan_results_t results;
	uint32_t scan_timeout_ms = 3000;

	size = max_num; //max_scan_num;

	results.ap = malloc(size * sizeof(wlan_sta_ap_t));
	if (results.ap == NULL) {
		WIFI_DEBUG("do not have the mem\n");
		return -1;
	}
	results.size = size;

	for (j = 0;j < TRY_FIND_SSID_TIME; j++) {
		ret = wlan_sta_scan_once();
		if (ret != 0) {
			free(results.ap);
			WIFI_DEBUG("wlan sta scan cmd failed\n");
			return -1;
		}

		uint32_t end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + scan_timeout_ms;
		scan_status = NET_CTRL_MSG_WLAN_SCAN_FAILED;
		while (!(scan_status == NET_CTRL_MSG_WLAN_SCAN_SUCCESS) &&
			OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {
			OS_MSleep(20);
		}

		if(!OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)){
			WIFI_DEBUG("wlan sta scan timeout %d\n", scan_timeout_ms);
			free(results.ap);
			return -1;
		}

		ret = wlan_sta_scan_result(&results);
		if(ret == 0) {
#if 1
			for (i = 0; i < results.num; i++) {
				memcpy(scan_result[i].ssid, results.ap[i].ssid.ssid, results.ap[i].ssid.ssid_len);
				scan_result[i].ssid[results.ap[i].ssid.ssid_len] = '\0';

				memcpy(scan_result[i].bssid, results.ap[i].bssid, 6);
				scan_result[i].rssi = results.ap[i].level;
				scan_result[i].channel = results.ap[i].channel;
				if(results.ap[i].wpa_flags & WPA_FLAGS_WEP) {
					scan_result[i].key = WIFIMG_WEP;
				} else if(results.ap[i].wpa_flags & WPA_FLAGS_WPA2){
					scan_result[i].key = WIFIMG_WPA2_PSK;
				} else {
					scan_result[i].key = WIFIMG_WPA_PSK;
				}
			}
			ret = results.num;
#endif
#if 0
			char scan_ssid[WLAN_SSID_MAX_LEN+1];
			for (i = 0; i < results.num; i++) {
				#if 1
				WIFI_DEBUG("%2d    BSS %02X:%02X:%02X:%02X:%02X:%02X    SSID: %-32.32s\ 
CHAN: %2d    RSSI: %d    flags: %08x    wpa_key_mgmt: %08x    \
wpa_cipher: %08x    wpa2_key_mgmt: %08x    wpa2_cipher: %08x\n",
						 i + 1, (results.ap[i].bssid)[0], (results.ap[i].bssid)[1],
						 (results.ap[i].bssid)[2], (results.ap[i].bssid)[3],
						 (results.ap[i].bssid)[4], (results.ap[i].bssid)[5],
						 results.ap[i].ssid.ssid, results.ap[i].channel,results.ap[i].level,
						 results.ap[i].wpa_flags, results.ap[i].wpa_key_mgmt,
						 results.ap[i].wpa_cipher,results.ap[i].wpa2_key_mgmt,
						 results.ap[i].wpa2_cipher);
			 	#endif
			}
#endif
		}
	}
	free(results.ap);
	return ret;
}
int get_wep_security(int *is_wep_security, char *ssid, int ssid_len){

	int ret = -1;
	int j;
	int size;
	int find_ssid = 0;
	wlan_sta_scan_results_t results;
	uint32_t scan_timeout_ms = 3000;

	size = MAX_SCAN_RESULTS;
	WIFI_DEBUG("%s,ssid %s len %d\n", __func__, ssid, ssid_len);
	results.ap = malloc(size * sizeof(wlan_sta_ap_t));
	if (results.ap == NULL) {
		WIFI_DEBUG("do not have the mem\n");
		return -1;
	}
	results.size = size;

	for (j = 0;j < TRY_FIND_SSID_TIME; j++) {
		ret = wlan_sta_scan_once();
		if (ret != 0) {
			free(results.ap);
			WIFI_DEBUG("wlan sta scan cmd failed\n");
			return -1;
		}

		uint32_t end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + scan_timeout_ms;
		scan_status = NET_CTRL_MSG_WLAN_SCAN_FAILED;
		while (!(scan_status == NET_CTRL_MSG_WLAN_SCAN_SUCCESS) &&
			OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {
			OS_MSleep(20);
		}

		if(!OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)){
			WIFI_DEBUG("wlan sta scan timeout %d\n", scan_timeout_ms);
			free(results.ap);
			return -1;
		}

		ret = wlan_sta_scan_result(&results);
		if (ret == 0) {
			int i;
#if 0
			char scan_ssid[WLAN_SSID_MAX_LEN+1];
			for (i = 0; i < results.num; i++) {
				memcpy(scan_ssid, results.ap[i].ssid.ssid, results.ap[i].ssid.ssid_len);
				scan_ssid[results.ap[i].ssid.ssid_len] = '\0';
				WIFI_DEBUG("ssid %s, len %d\n", scan_ssid, results.ap[i].ssid.ssid_len);
				#if 0
				WIFI_DEBUG("%2d    BSS %02X:%02X:%02X:%02X:%02X:%02X    SSID: %-32.32s    "
						 "CHAN: %2d    RSSI: %d    flags: %08x    wpa_key_mgmt: %08x    "
						 "wpa_cipher: %08x    wpa2_key_mgmt: %08x    wpa2_cipher: %08x\n",
						 i + 1, (results.ap[i].bssid)[0], (results.ap[i].bssid)[1],
						 (results.ap[i].bssid)[2], (results.ap[i].bssid)[3],
						 (results.ap[i].bssid)[4], (results.ap[i].bssid)[5],
						 scan_ssid, results.ap[i].channel,results.ap[i].level,
						 results.ap[i].wpa_flags, results.ap[i].wpa_key_mgmt,
						 results.ap[i].wpa_cipher,results.ap[i].wpa2_key_mgmt,
						 results.ap[i].wpa2_cipher);
			 	#endif
			}
#endif
			for (i = 0; i < results.num; i++) {
				if((results.ap[i].ssid.ssid_len == ssid_len) && \
	            	(0 == memcmp(ssid, \
	                        	 results.ap[i].ssid.ssid,\
	                        	 results.ap[i].ssid.ssid_len))) {
	                 //WIFI_DEBUG("%s, find ssid, i %d\n", __func__, i);
					find_ssid = 1;
					if(results.ap[i].wpa_flags & WPA_FLAGS_WEP) {
						*is_wep_security = 1;
					} else {
						*is_wep_security = 0;
					}

					break;
				}
			}

			if (find_ssid == 1) {
				WIFI_DEBUG("%s, find ssid, j %d\n", __func__, j);
				break;
			}
		} else {
			free(results.ap);
			ret = -1;
			return ret;
		}
	}

	free(results.ap);
	if(j == TRY_FIND_SSID_TIME) {
		WIFI_DEBUG("3 times scan, can not find the ssid\n");
		return -1;
	} else {
		return 0;
	}
}

static inline size_t strlcpy(char *dest, const char *src, size_t size)
{
	size_t ret = strlen(src);

	if (size) {
		size_t len = (ret >= size) ? size - 1 : ret;
		memcpy(dest, src, len);
		dest[len] = '\0';
	}
	return ret;
}
int wifi_wep_connect(char *ssid, char *passwd)
{
	uint8_t ssid_len;
	wlan_sta_config_t config;
	uint32_t wep_open_connect_timeout_ms = 5000;
	WIFI_DEBUG("%s,ssid %s,passwd, %s\n", __func__, ssid, passwd);
	uint32_t timeout = OS_GetTicks() + OS_TicksToMSecs(wep_open_connect_timeout_ms);

	if (ssid)
		ssid_len = strlen(ssid);
	else
		goto err;


	if (ssid_len > WLAN_SSID_MAX_LEN)
		ssid_len = WLAN_SSID_MAX_LEN;

	net_switch_mode(WLAN_MODE_STA);
	wlan_sta_disable();

	memset(&config, 0, sizeof(config));
	connect_status = NET_CTRL_MSG_WLAN_DISCONNECTED;

	/* ssid */
	config.field = WLAN_STA_FIELD_SSID;
	memcpy(config.u.ssid.ssid, ssid, ssid_len);
	config.u.ssid.ssid_len = ssid_len;
	if (wlan_sta_set_config(&config) != 0)
		goto err;

	/* WEP key0 */
	config.field = WLAN_STA_FIELD_WEP_KEY0;
	strlcpy((char *)config.u.wep_key, passwd, sizeof(config.u.wep_key));
	if (wlan_sta_set_config(&config) != 0)
	return -1;

	/* WEP key index */
	config.field = WLAN_STA_FIELD_WEP_KEY_INDEX;
	config.u.wep_tx_keyidx = 0;
	if (wlan_sta_set_config(&config) != 0)
		goto err;

	/* auth_alg: OPEN */
	config.field = WLAN_STA_FIELD_AUTH_ALG;
	config.u.auth_alg = WPA_AUTH_ALG_OPEN | WPA_AUTH_ALG_SHARED;
	if (wlan_sta_set_config(&config) != 0)
		goto err;

	/* key_mgmt: NONE */
	config.field = WLAN_STA_FIELD_KEY_MGMT;
	config.u.key_mgmt = WPA_KEY_MGMT_NONE;
	if (wlan_sta_set_config(&config) != 0)
		goto err;

	if (wlan_sta_enable()!= 0)
		goto err;

	while ((connect_status == NET_CTRL_MSG_WLAN_DISCONNECTED) &&
	       OS_TimeBeforeEqual(OS_GetTicks(), timeout)) {
		OS_MSleep(20);
	}

	if (connect_status == NET_CTRL_MSG_WLAN_CONNECTED) {
		return 0;
	} else {//WLAN_EVENT_CONNECT_FAILED or timeout
		WIFI_DEBUG("%s, WPA_AUTH_ALG_SHARED\n", __func__);
		config.field = WLAN_STA_FIELD_AUTH_ALG;
		config.u.auth_alg = WPA_AUTH_ALG_SHARED;
		if (wlan_sta_set_config(&config) != 0)
			goto err;
	}

	return 0;
err:
	WIFI_DEBUG("connect ap failed\n");

	//TODO: wait dhcp ready here
    return -1;
}

int wifi_normal_connect(char *ssid, char *passwd)
{
	uint8_t *_psk;
	uint8_t ssid_len;

	if (ssid)
		ssid_len = strlen(ssid);
	else
		goto err;

	if (ssid_len > WLAN_SSID_MAX_LEN)
		ssid_len = WLAN_SSID_MAX_LEN;

	net_switch_mode(WLAN_MODE_STA);
	wlan_sta_disable();
	if (passwd) {
		WIFI_DEBUG("=======================\n");
		WIFI_DEBUG("connect ssid: ""\033[31m""%s" "\033[0m" "\n", ssid);
		WIFI_DEBUG("=======================\n");
	}

	if (passwd) {
		if (strlen(passwd) == 0) {// passwd is '\0'
			//WIFI_DEBUG("passwd is null\n");
			_psk = NULL;
		} else {
			_psk = (uint8_t *)passwd;
		}
	} else {
		_psk = NULL;
	}

	if (wlan_sta_set((uint8_t *)ssid, ssid_len, _psk)) {
		goto err;
	}

	wlan_sta_enable();

	return 0;

err:
	WIFI_DEBUG("connect ap failed\n");

	//TODO: wait dhcp ready here
    return -1;
}

#if 0
int wifi_connect(const char *ssid, const char *passwd, uint32_t timems)
{

	uint8_t ssid_len = 0;
	uint8_t passwd_len = 0;
	int wep_security = 0;
	struct netif *nif = g_wlan_netif;
	uint32_t connection_timeout_ms = timems;

	if (ssid)
		ssid_len = strlen(ssid);
	else
		goto err;

	if (ssid_len > WLAN_SSID_MAX_LEN)
		goto err;

	uint32_t timeout = OS_GetTicks() + OS_TicksToMSecs(connection_timeout_ms);

	WIFI_DEBUG("%s,%d to:%d\n", __func__, __LINE__, connection_timeout_ms);

	if (passwd) {
		passwd_len = strlen(passwd);
	}

	WIFI_DEBUG("%s,ssid %s,passwd, %s\n", __func__, ssid, passwd);

	if(5 == passwd_len) {
		wep_security = 1;
	}

	if((13 == passwd_len) | (10 == passwd_len) |
		(26 == passwd_len)) {
		int ret;
		ret = get_wep_security(&wep_security, (char *)ssid, ssid_len);
		if(ret < 0) {
			printf("get the wep security fail, ret %d",ret);
		}
	}

	if(wep_security == 1) {
		wifi_wep_connect(ssid, passwd);
	} else {
		wifi_normal_connect(ssid, passwd);
	}


#if 1
	while (!(nif && netif_is_up(nif) && netif_is_link_up(nif)) &&
	       OS_TimeBeforeEqual(OS_GetTicks(), timeout)) {
		OS_MSleep(20);
	}

	if (OS_TimeBeforeEqual(OS_GetTicks(), timeout)) {
		return 0;
	}
#endif

err:
	WIFI_DEBUG("connect ap failed\n");

	//TODO: wait dhcp ready here
    return -1;
}
#endif

int wifi_connect(const char *ssid, const char *passwd)
{
	uint8_t ssid_len = 0;
	uint8_t passwd_len = 0;
	int wep_security = 0;
	int ret = -1;
	struct netif *nif = g_wlan_netif;


	if (ssid)
		ssid_len = strlen(ssid);
	else
		goto err;

	if (ssid_len > WLAN_SSID_MAX_LEN)
		goto err;


	if (passwd) {
		passwd_len = strlen(passwd);
	}

	WIFI_DEBUG("%s,ssid %s,passwd, %s\n", __func__, ssid, passwd);

	if(5 == passwd_len) {
		wep_security = 1;
	}

	if((13 == passwd_len) | (10 == passwd_len) |
		(26 == passwd_len)) {
		ret = get_wep_security(&wep_security, (char *)ssid, ssid_len);
		if(ret < 0) {
			printf("get the wep security fail, ret %d",ret);
		}
	}

	if(wep_security == 1) {
		ret = wifi_wep_connect(ssid, passwd);
	} else {
		ret= wifi_normal_connect(ssid, passwd);
	}

	return ret;
err:
	WIFI_DEBUG("connect ap failed\n");

	//TODO: wait dhcp ready here
    return -1;
}
#if 0
#define MATCH_MIN_RSSI (-70)
int wifi_get_select_network(GLWifiConfig_t *netcfg,int netcfg_num)
{
	int ret = -1;
	int j = 0;
	int size;
	int find_ssid = 0;
	wlan_sta_scan_results_t results;
	uint32_t scan_timeout_ms = 3000;

	size = MAX_SCAN_RESULTS;
	//WIFI_DEBUG("%s,ssid %s len %d\n", __func__, ssid, ssid_len);
	results.ap = malloc(size * sizeof(wlan_sta_ap_t));
	if (results.ap == NULL) {
		WIFI_DEBUG("do not have the mem\n");
		return -1;
	}
	results.size = size;

	ret = wlan_sta_scan_once();
	if (ret != 0) {
		free(results.ap);
		WIFI_DEBUG("wlan sta scan cmd failed\n");
		return -1;
	}

	uint32_t end_time = OS_JiffiesToMSecs(OS_GetJiffies()) + scan_timeout_ms;
	scan_status = NET_CTRL_MSG_WLAN_SCAN_FAILED;
	while (!(scan_status == NET_CTRL_MSG_WLAN_SCAN_SUCCESS) &&
		OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)) {
		OS_MSleep(20);
	}

	if(!OS_TimeBefore(OS_JiffiesToMSecs(OS_GetJiffies()), end_time)){
		WIFI_DEBUG("wlan sta scan timeout %d\n", scan_timeout_ms);
		free(results.ap);
		return -1;
	}

	ret = wlan_sta_scan_result(&results);
	if (ret == 0) {
			int i;
#if 0
			char scan_ssid[WLAN_SSID_MAX_LEN+1];
			for (i = 0; i < results.num; i++) {
				memcpy(scan_ssid, results.ap[i].ssid.ssid, results.ap[i].ssid.ssid_len);
				scan_ssid[results.ap[i].ssid.ssid_len] = '\0';
				WIFI_DEBUG("ssid %s, len %d\n", scan_ssid, results.ap[i].ssid.ssid_len);
				#if 0
				WIFI_DEBUG("%2d    BSS %02X:%02X:%02X:%02X:%02X:%02X    SSID: %-32.32s    "
						 "CHAN: %2d    RSSI: %d    flags: %08x    wpa_key_mgmt: %08x    "
						 "wpa_cipher: %08x    wpa2_key_mgmt: %08x    wpa2_cipher: %08x\n",
						 i + 1, (results.ap[i].bssid)[0], (results.ap[i].bssid)[1],
						 (results.ap[i].bssid)[2], (results.ap[i].bssid)[3],
						 (results.ap[i].bssid)[4], (results.ap[i].bssid)[5],
						 scan_ssid, results.ap[i].channel,results.ap[i].level,
						 results.ap[i].wpa_flags, results.ap[i].wpa_key_mgmt,
						 results.ap[i].wpa_cipher,results.ap[i].wpa2_key_mgmt,
						 results.ap[i].wpa2_cipher);
			 	#endif
			}
#endif

			for (i = 0; i < results.num; i++) {
				for (j = netcfg_num-1; j>=0; j--) {
					if (netcfg[j].enable == true) {
						if ((results.ap[i].ssid.ssid_len == strlen(netcfg[j].ssid)) && \
		            	(0 == memcmp(netcfg[j].ssid, results.ap[i].ssid.ssid,\
		                        	 strlen(netcfg[j].ssid))) &&
		                        	 (results.ap[i].rssi >= MATCH_MIN_RSSI))
		                     WIFI_DEBUG(" find ssid, %s, j %d\n", results.ap[i].ssid.ssid, j);
							 free(results.ap);
		                     return j;
					}

				}
			}
	}
	free(results.ap);
	WIFI_DEBUG("can not find the ssid\n");
	return -1;
}
#endif
int wifi_disconnect()
{
	return wlan_sta_disable();
}

int wifi_get_mac_address(char *mac)
{

	struct sysinfo *sysinfo;

	sysinfo = sysinfo_get();
	memcpy(mac, sysinfo->mac_addr, IEEE80211_ADDR_LEN);

	return 0;
}

int wifi_get_rssi(int* rssi)
{
	int ret;
	wlan_ext_signal_t signal;
	ret = wlan_ext_request(g_wlan_netif, WLAN_EXT_CMD_GET_SIGNAL, (int)(&signal));
	//WIFI_DEBUG("current rssi:%d, noise:%d\n", signal.rssi, signal.noise);
	if (ret == -2) {
		WIFI_DEBUG("invalid arg\n");
	} else if (ret == -1) {
		WIFI_DEBUG("exec failed\n");
	}

	*rssi = signal.rssi/2 + signal.noise;
	//WIFI_DEBUG("rssi =%d\n", *rssi);
	return 0;
}

int getChannelByBssid(const char *bssid,
        unsigned int *channel)
{


}

static int monitor_chan;
int wifi_set_channel(int chan)
{
	int ret;
    ret = wlan_monitor_set_channel(g_wlan_netif, (int16_t)chan);
	if(ret != 0) {
		return -1;
	}
	//monitor_chan = chan;
	return 0;
}

//extern SNIFFER_CALLBACK snif_cb;
#if 0
static void monitor_rx_cb(uint8_t *buf, uint32_t len, void *userdata)
{
    if(NULL == snif_cb) {
        return;
    }

    snif_cb((BYTE_T *)buf,(USHORT_T)len);

}
#endif

//static int max_chan = 0;

static void recv_rawframe(uint8_t *data, uint32_t len, void *info)
{
	struct frame_info *p;
	if (len < sizeof(struct ieee80211_frame)) {
		WIFI_DEBUG("%s():%d, len %u\n", __func__, __LINE__, len);
		return;
	}

   // if(NULL == snif_cb) {
   //     return;
   // }
#if 0
    p = info;
	if((p->type == IEEE80211_FC_STYPE_BEACON) ||
		(p->type == IEEE80211_FC_STYPE_PROBE_RESP)) {
		if (p->ap_channel > max_chan || p->ap_channel < 1) {
			//printf("p->ap_channel=%d\n", p->ap_channel);
			return;
		}
	}
#endif
    //snif_cb((BYTE_T *)data,(USHORT_T)len);
}

static void wlan_sw_ch_cb(struct netif *nif, int16_t channel)
{
	monitor_chan = channel;
}
#if 0
#if 1
#define TIME_OUT_MS 120000
int wifi_set_promisc(int enable)
{
	sc_assistant_fun_t sca_fun;
	sc_assistant_time_config_t config;
	int ret = -1;

	/*the sc assistant switch channel time param is no use for tuya*/
	config.time_total = TIME_OUT_MS;
	config.time_sw_ch_long = 0;
	config.time_sw_ch_short = 0;

	if (enable == 1) {
		sc_assistant_get_fun(&sca_fun);
		sc_assistant_init(g_wlan_netif, &sca_fun, &config);

		ret = sc_assistant_monitor_register_rx_cb(g_wlan_netif, recv_rawframe);
		if (ret != 0) {
			WIFI_DEBUG("%s monitor set rx cb fail\n", __func__);
			return ret;
		}
		ret = sc_assistant_monitor_register_sw_ch_cb(g_wlan_netif, wlan_sw_ch_cb);
		if (ret != 0) {
			WIFI_DEBUG("%s monitor sw ch cb fail\n", __func__);
			return ret;
		}

		return 0;
	}else {
		if (sc_assistant_monitor_unregister_rx_cb(g_wlan_netif, recv_rawframe)) {
			WIFI_DEBUG("%s,%d cancel rx cb fail\n", __func__, __LINE__);
		}
		if (sc_assistant_monitor_unregister_sw_ch_cb(g_wlan_netif, wlan_sw_ch_cb)) {
			WIFI_DEBUG("%s,%d cancel sw ch cb fail\n", __func__, __LINE__);
		}
		sc_assistant_deinit(g_wlan_netif);
	}
	return 0;
}
#endif
#endif
int wifi_on(int mode)
{
	enum wlan_mode current_mode = wlan_if_get_mode(g_wlan_netif);
	int ret = 0;

	ret = sys_ctrl_create();
	if(ret) {
		WIFI_ERROR("sys create failed.\n");
		goto end;
	}

	ret = sysinfo_init();
	if(ret) {
		WIFI_ERROR("sys info init failed.\n");
		goto end;
	}
	ret = wifi_event_init();
	if(ret) {
		WIFI_ERROR("wifi event init failed.\n");
		goto end;
	}
	struct sysinfo *sysinfo = sysinfo_get();
	ret = net_sys_start(sysinfo->wlan_mode);
	if(ret) {
		WIFI_ERROR("net start failed.\n");
		goto end;
	}
#if 0
    switch(mode) {
        case WLAN_MODE_STA :
#if 0
			if (current_mode == WLAN_MODE_MONITOR) {
				//wifi_set_promisc(0);
			}
#endif
			net_switch_mode(WLAN_MODE_STA);
			break;
		case WLAN_MODE_HOSTAP :
			WIFI_DEBUG("ap mode is not support\n");
			return -1;
        	break;
		case WLAN_MODE_MONITOR :
			//wlan_monitor_set_rx_cb(g_wlan_netif, monitor_rx_cb);
			net_switch_mode(WLAN_MODE_MONITOR);
#if 0
			if(wifi_set_promisc(1))
				WIFI_DEBUG("set promisc sc_assisant failed\n");
#endif
			break;
		default:
			WIFI_DEBUG("the wlan mode is not support\n");
    }
#endif
end:
	return ret;
}

int wifi_off()
{
	return 0;
}

#define IP_ADDR_LEN 16
int wifi_get_ip_address(char *ipaddr)
{
  struct netif *nif = g_wlan_netif;
#if 0
  if (NET_IS_IP4_VALID(nif) && netif_is_link_up(nif)) {
    inet_ntoa_r(nif->ip_addr, ipaddr, IP_ADDR_LEN);
    printf("ipaddr %s\n", (char*)ipaddr);
  } else {
    return -1;
  }
#endif
  return 0;
}

int wifi_get_ap_bssid(unsigned char *bssid)
{
    wlan_sta_ap_t apinfo;
    int ret = wlan_sta_ap_info(&apinfo);
    if(ret != 0) {
        return -1;
    }

    memcpy(bssid, apinfo.bssid, SYSINFO_MAC_ADDR_LEN);
    return 0;
}

int wifi_get_ap_channel(uint8_t *channel)
{
    wlan_sta_ap_t apinfo;
    int ret = wlan_sta_ap_info(&apinfo);
    if(ret != 0) {
        return -1;
    }

    *channel = apinfo.channel;
    return 0;
}
#if 0
int wifi_get_config(GLWifiConfig_t *config)
{
    wlan_sta_ap_t apinfo;
    int ret = wlan_sta_ap_info(&apinfo);
    if(ret != 0) {
        return -1;
    }


}
#endif

void wifi_reconnect(void)
{
    wifi_disconnect();
    wlan_sta_enable();
}
