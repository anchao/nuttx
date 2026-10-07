/* SPDX-License-Identifier: Apache-2.0 */
#include <stdint.h>
#include <zephyr/device.h>

/* Minimal device structure for BT */
static struct device_state bt_dev_state = { .initialized = false, .init_res = 0 };
const struct device bt_hci_dev = {
    .name = "bt-hci",
    .state = &bt_dev_state,
};