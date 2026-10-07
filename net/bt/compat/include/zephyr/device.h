/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_DEVICE_H_
#define ZEPHYR_INCLUDE_DEVICE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <nuttx/compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Minimal device model for BT HCI driver binding */

struct device_state {
    bool initialized;
    int init_res;
};

struct device {
    const char *name;
    const void *config;
    const void *api;
    void *data;
    struct device_state *state;
};

static inline bool device_is_ready(const struct device *dev)
{
    return dev && dev->state && dev->state->initialized && dev->state->init_res == 0;
}

#define DEVICE_DECLARE(name) extern const struct device name
#define DEVICE_NAME_GET(dev) ((dev)->name)

/* NuttX-compatible HCI driver interface */
struct bt_hci_driver {
    const struct device dev;
    int (*open)(void);
    int (*send)(const uint8_t *data, size_t len);
    int (*close)(void);
    void *priv;
};

/* DT helpers - hardcoded for BT */
#define DT_NODELABEL(bt_hci) bt_hci
#define DT_PROP(node_id, prop) 0  /* Hardcoded for NuttX */
#define DT_IRQN(node_id) 0
#define DT_IRQ(node_id, prop) 0
#define DT_REG_ADDR(node_id) 0
#define DT_REG_SIZE(node_id) 0

/* Hardcode the BT HCI UART device reference */
#define DEVICE_DT_GET(node_id) NULL

/* Additional DT macros */
#define DT_CHOSEN(prop) NULL
#define DT_COMPAT_GET_ANY_STATUS_OKAY(compat) NULL
#define DT_HAS_CHOSEN(prop) 0

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DEVICE_H_ */