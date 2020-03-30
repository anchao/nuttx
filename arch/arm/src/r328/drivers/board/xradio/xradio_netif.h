#ifndef __XRADIO_NETIF__H
#define __XRADIO_NETIF__H

#ifdef CONFIG_OS_NUTTX

#include <nuttx/net/netdev.h>
#include <stdint.h>
#include <stdbool.h>
//#include "net/wlan/wlan_common.h"

#ifndef NETIF_MAX_HWADDR_LEN
#define NETIF_MAX_HWADDR_LEN 6U
#endif

#ifndef ETHARP_HWADDR_LEN
#define ETHARP_HWADDR_LEN 6
#endif

typedef int8_t                  s8;
typedef int8_t                  s8_t;
//typedef uint8_t                 u8;
typedef uint8_t                 u8_t;

typedef int16_t                 s16;
typedef int16_t                 s16_t;
//typedef uint16_t                u16;
typedef uint16_t                u16_t;

typedef int32_t                 s32;
typedef int32_t                 s32_t;
//typedef uint32_t                u32;
typedef uint32_t                u32_t;

typedef int64_t                 s64;
typedef int64_t                 s64_t;
typedef uint64_t                u64;
typedef uint64_t                u64_t;


typedef int8_t err_t;

struct netif;

enum wlan_mode {
  WLAN_MODE_STA = 0,   /* Infrastructure station */
  WLAN_MODE_HOSTAP,    /* Software Access Point */
  WLAN_MODE_MONITOR,   /* Monitor mode */
  WLAN_MODE_NUM,
  WLAN_MODE_INVALID = WLAN_MODE_NUM
};

/** Definitions for error constants. */
typedef enum {
    /** No error, everything OK. */
    ERR_OK         = 0,
    /** Out of memory error.     */
    ERR_MEM        = -1,
    /** Buffer error.            */
    ERR_BUF        = -2,
    /** Timeout.                 */
    ERR_TIMEOUT    = -3,
    /** Routing problem.         */
    ERR_RTE        = -4,
    /** Operation in progress    */
    ERR_INPROGRESS = -5,
    /** Illegal value.           */
    ERR_VAL        = -6,
    /** Operation would block.   */
    ERR_WOULDBLOCK = -7,
    /** Address in use.          */
    ERR_USE        = -8,
    /** Already connecting.      */
    ERR_ALREADY    = -9,
    /** Conn already established.*/
    ERR_ISCONN     = -10,
    /** Not connected.           */
    ERR_CONN       = -11,
    /** Low-level netif error    */
    ERR_IF         = -12,
    /** Connection aborted.      */
    ERR_ABRT       = -13,
    /** Connection reset.        */
    ERR_RST        = -14,
    /** Connection closed.       */
    ERR_CLSD       = -15,
    /** Illegal argument.        */
    ERR_ARG        = -16
} err_enum_t;

struct netif {
  /** pointer to next in linked list */
  struct netif *next;

  /** This function is called by the network device driver
   *  to pass a packet up the TCP/IP stack. */
  //netif_input_fn input;
  /** This function is called by ethernet_output() when it wants
   *  to send a packet on the interface. This function outputs
   *  the pbuf as-is on the link medium. */
  //netif_linkoutput_fn linkoutput;
  /** This field can be set by the device driver and could point
   *  to state information for the device. */
  void *state;
  /** maximum transfer unit (in bytes) */
  u16_t mtu;
  /** number of bytes used in hwaddr */
  u8_t hwaddr_len;
  /** link level hardware address of this interface */
  u8_t hwaddr[NETIF_MAX_HWADDR_LEN];
  /** flags (@see @ref netif_flags) */
  u8_t flags;
  /** descriptive abbreviation */
  char name[2];
  /** number of this interface */
  u8_t num;
};

#define XR_FRAME_USE_MBUFF 1
#define USE_TX_GET_BUFF 1

struct xr_frame_s {
	uint16_t len;
#if XR_FRAME_USE_MBUFF
	uint8_t *data;
	void *priv;
#else
	uint8_t data[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];
#endif
};

struct xr_ring_buff {
	struct xr_frame_s *frame;
	uint8_t data_len;
	uint8_t read_idx;
	uint8_t write_idx;
};

struct xr_frame_s* xradio_tx_buff_get(void);
int xradio_tx_buff_free(void);


err_t ethernetif_linkoutput(uint8_t *data, uint16_t len);
void net_hex_dump(char *pref, int width, unsigned char *buf, int len);

int xradio_wlan_init(enum wlan_mode mode, struct net_driver_s *dev);
void xradio_wlan_deinit(void);
#endif/*CONFIG_OS_NUTTX*/

#endif
