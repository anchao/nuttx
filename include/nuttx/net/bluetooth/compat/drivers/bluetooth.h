/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_BLUETOOTH_H_
#define ZEPHYR_INCLUDE_DRIVERS_BLUETOOTH_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BT HCI driver interface - NuttX native */

#ifndef _BT_HCI_EVT_HDR_DEFINED
#define _BT_HCI_EVT_HDR_DEFINED
struct bt_hci_evt_hdr {
    uint8_t evt;
    uint8_t len;
} __attribute__((packed));
#endif

#ifndef _BT_HCI_ACL_HDR_DEFINED
#define _BT_HCI_ACL_HDR_DEFINED
struct bt_hci_acl_hdr {
    uint16_t handle;
    uint16_t len;
} __attribute__((packed));
#endif

#ifndef _BT_HCI_ISO_HDR_DEFINED
#define _BT_HCI_ISO_HDR_DEFINED
struct bt_hci_iso_hdr {
    uint16_t handle;
    uint16_t len;
} __attribute__((packed));
#endif

/* HCI driver vtable */
struct bt_hci_driver_api {
    int (*open)(void);
    int (*send)(const uint8_t *data, size_t len);
    int (*close)(void);
};

/* HCI driver quirks (bitmask) */
#define BT_HCI_QUIRK_NO_RESET       BIT(0)
#define BT_HCI_QUIRK_NO_AUTO_DLE    BIT(1)
#define BT_HCI_QUIRK_NO_FLOW_CONTROL BIT(2)

/* Forward declarations */
struct device;
struct net_buf;
#include <nuttx/net/bluetooth/addr.h>

/* HCI driver data */
struct bt_hci_driver_data {
    bt_addr_t public_addr;
};

/* HCI driver API functions */
typedef int (*bt_hci_recv_t)(const struct device *dev, struct net_buf *buf);

int bt_hci_send(const struct device *dev, struct net_buf *buf);
int bt_hci_open(const struct device *dev, bt_hci_recv_t recv);
int bt_hci_close(const struct device *dev);
bool bt_hci_can_close(const struct device *dev);
void bt_hci_set_public_addr(const struct device *dev, const bt_addr_t *addr);
const bt_addr_t *bt_hci_get_public_addr(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_BLUETOOTH_H_ */
