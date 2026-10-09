/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX HCI driver integration.
 * This provides the bridge between Zephyr BT host and NuttX hardware.
 * For now this is a skeleton - actual HCI transport drivers (UART H4, etc.)
 * need to be implemented separately.
 */

#include <string.h>
#include <errno.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/buf.h>
#include <nuttx/net/bluetooth/hci_driver_nuttx.h>
#include <nuttx/net/bluetooth/compat/drivers/bluetooth.h>
#include <syslog.h>

static const struct bt_hci_driver_ops *g_hci_ops;

/* Buffer pools for HCI data */
NET_BUF_POOL_FIXED_DEFINE(hci_cmd_pool, CONFIG_BT_BUF_CMD_TX_COUNT,
                          BT_BUF_CMD_TX_SIZE, 0, NULL);
NET_BUF_POOL_FIXED_DEFINE(hci_evt_pool, CONFIG_BT_BUF_EVT_RX_COUNT,
                          BT_BUF_EVT_RX_SIZE, 0, NULL);
NET_BUF_POOL_FIXED_DEFINE(hci_acl_tx_pool, CONFIG_BT_BUF_ACL_TX_COUNT,
                          BT_BUF_ACL_SIZE(CONFIG_BT_BUF_ACL_TX_SIZE), 0, NULL);
NET_BUF_POOL_FIXED_DEFINE(hci_acl_rx_pool, BT_BUF_ACL_RX_COUNT,
                          BT_BUF_ACL_RX_SIZE, 0, NULL);

int bt_hci_driver_register(const struct bt_hci_driver_ops *ops)
{
    g_hci_ops = ops;
    if (ops && ops->open)
        return ops->open();
    return 0;
}

struct net_buf *bt_buf_get_cmd_complete(clock_t timeout)
{
    return net_buf_alloc(&hci_cmd_pool, timeout);
}

struct net_buf *bt_buf_get_evt(uint8_t evt, bool discardable, clock_t timeout)
{
    (void)evt;
    (void)discardable;
    return net_buf_alloc(&hci_evt_pool, timeout);
}

struct net_buf *bt_buf_get_acl(clock_t timeout)
{
    return net_buf_alloc(&hci_acl_rx_pool, timeout);
}

struct net_buf *bt_buf_get_iso(clock_t timeout)
{
    (void)timeout;
    return NULL;
}

int bt_recv(struct net_buf *buf)
{
    /* Forward to BT host stack - will be called by hci_core.c */
    extern int bt_recv_unsafe(struct net_buf *buf);
    return bt_recv_unsafe(buf);
}

int bt_recv_iso(struct net_buf *buf)
{
    (void)buf;
    return 0;
}

/* ===== Zephyr HCI driver API (bt_hci_*) ===== */

static bt_hci_recv_t g_hci_recv_cb;

int bt_hci_open(const struct device *dev, bt_hci_recv_t recv)
{
    g_hci_recv_cb = recv;
    if (g_hci_ops && g_hci_ops->open)
        return g_hci_ops->open();
    return 0;
}

int bt_hci_send(const struct device *dev, struct net_buf *buf)
{
    if (g_hci_ops && g_hci_ops->send)
        return g_hci_ops->send(buf);
    return -ENODEV;
}

int bt_hci_close(const struct device *dev)
{
    g_hci_recv_cb = NULL;
    if (g_hci_ops && g_hci_ops->close)
        return g_hci_ops->close();
    return 0;
}

bool bt_hci_can_close(const struct device *dev)
{
    return true;
}
