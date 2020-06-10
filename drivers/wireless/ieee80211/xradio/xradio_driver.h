#ifndef __XRADIO_DRVER_H__
#define __XRADIO_DRVER_H__

#include <nuttx/wireless/wireless.h>
#include <nuttx/config.h>
#include <semaphore.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netdev.h>

enum
{
  XRADIO_STATUS_DONE = 0,
  XRADIO_STATUS_DISABLED,
  XRADIO_STATUS_RUN,
  XRADIO_STATUS_TIMEOUT,
};
enum
{
  XR_WLAN_MODE_STA = 0,   /* Infrastructure station */
  XR_WLAN_MODE_HOSTAP,    /* Software Access Point */
  XR_WLAN_MODE_MONITOR,   /* Monitor mode */
  XR_WLAN_MODE_NUM,
};

enum
{
	XR_INITIAL_SCAN_REQ = 0,
	XR_NORMAL_SCAN_REQ,
	XR_MANUAL_SCAN_REQ,
};

struct xradio_state_s
{
	sem_t                   mutex;
	WDOG_ID                 timeout;
	int                     status;
	FAR void*               *data;
	unsigned int            size;
	int						scan_req;
};

struct xradio_dev_s
{
	bool if_up;

	//int   devnum;

	WDOG_ID xr_txpolldog;

	struct work_s xr_pollwork;
	struct net_driver_s xr_dev;
	struct xr_frame_s *cur_tx_frame;
	struct xradio_state_s scan;
	struct xradio_state_s conn;

	int   devnum;
	int   mode;
};

int xradio_wl_start_scan(struct xradio_dev_s *priv,struct iwreq *iwr);

int xradio_wl_get_scan_results(struct xradio_dev_s *priv,struct iwreq *iwr);

int xradio_wl_set_mac_address(struct xradio_dev_s *priv,struct iwreq *iwr);

int xradio_wl_set_encode_ext(struct xradio_dev_s *priv,struct iwreq *iwr);

int xradio_wl_set_ssid(struct xradio_dev_s *priv,struct iwreq *iwr);

int xradio_wl_set_mode(struct xradio_dev_s *priv,struct iwreq *iwr);

void xradio_wl_scan_handler(int index,uint32_t event,void *arg);

void xradio_wl_connection_handler(int index, uint32_t event,void *arg);

#endif
