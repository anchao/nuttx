#ifndef WIFI_ADAPTER_H
#define WIFI_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

int wifi_init(void);

void wifi_deinit(void);

int wifi_connect(const char *ssid, const char *passwd);

int wifi_disconnect(void);


int wifi_get_mac_address(char *mac);


int wifi_get_rssi(int* rssi);


int wifi_set_channel(int chan);

int wifi_on(int mode);

int wifi_off(void);

int wifi_get_ip_address(char *ipaddr);

int wifi_get_ap_bssid(unsigned char *bssid);

void wifi_reconnect(void);

#ifdef __cplusplus
}
#endif

#endif
