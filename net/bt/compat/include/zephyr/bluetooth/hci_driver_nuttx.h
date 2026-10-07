/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_BT_HCI_DRIVER_NUTTX_H_
#define ZEPHYR_BT_HCI_DRIVER_NUTTX_H_

/* NuttX HCI driver integration for Zephyr Bluetooth Host Stack.
 *
 * This provides the bridge between the Zephyr BT host (which expects
 * the Zephyr driver model) and NuttX's hardware drivers.
 *
 * To use with a specific HCI transport:
 *   1. Register an HCI driver via bt_hci_driver_register()
 *   2. Driver calls bt_recv() when it receives data from the controller
 *   3. BT host calls the driver's send() when it has data for the controller
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

struct net_buf;

/* HCI driver operations */
struct bt_hci_driver_ops {
    int (*open)(void);
    int (*send)(struct net_buf *buf);
    int (*close)(void);
};

/* Register an HCI driver. Returns 0 on success. */
int bt_hci_driver_register(const struct bt_hci_driver_ops *ops);

/* Called by the HCI driver when it receives data from the controller */
int bt_recv(struct net_buf *buf);

/* Called by the HCI driver when it receives an ISO packet */
int bt_recv_iso(struct net_buf *buf);

/* Get a buffer for HCI command/event/ACL data */
struct net_buf *bt_buf_get_cmd_complete(k_timeout_t timeout);
struct net_buf *bt_buf_get_evt(uint8_t evt, bool discardable, k_timeout_t timeout);
struct net_buf *bt_buf_get_acl(k_timeout_t timeout);
struct net_buf *bt_buf_get_iso(k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_BT_HCI_DRIVER_NUTTX_H_ */