/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal devicetree stub for NuttX BT port */

#ifndef ZEPHYR_INCLUDE_DEVICETREE_H_
#define ZEPHYR_INCLUDE_DEVICETREE_H_

/* Include generated devicetree stubs if available */
#include <zephyr/devicetree_generated.h>

/* All DT macros return hardcoded values since NuttX doesn't use DT */

#define DT_INVALID_NODE 0
#define DT_ROOT 1

#define DT_NODE_EXISTS(node_id) ((node_id) != DT_INVALID_NODE)

/* GPIO / pinmux / other peripherals not needed for BT */
#define DT_GPIO_CTLR(node_id, prop) 0
#define DT_GPIO_FLAGS(node_id, prop) 0
#define DT_GPIO_PIN(node_id, prop) 0
#define DT_PROP_OR(node_id, prop, default_val) (default_val)
#define DT_PROP_LEN(node_id, prop) 0

/* Clock frequency */
#define DT_PROP(node_id, prop) 0
#define DT_CLOCKS_CTLR(node_id) 0
#define DT_CLOCKS_CELL(node_id, cell) 0

/* Alias */
#define DT_ALIAS(label) DT_INVALID_NODE

/* Instances */
#define DT_INST(inst, compat) DT_INVALID_NODE
#define DT_DRV_COMPAT "none"

/* For HCI driver: hardcode to configure via Kconfig */
#define DT_PROP_BY_IDX(node, prop, idx) 0

/* CONFIG-dependent helpers */
#define DT_NUM_INST_STATUS_OKAY(compat) 0
#define DT_HAS_COMPAT_STATUS_OKAY(compat) 0

/* DT_HAS_CHOSEN - returns 1 if chosen node exists, 0 otherwise */
#ifndef DT_HAS_CHOSEN
#define DT_HAS_CHOSEN(x) 1
#endif

/* DT_CHOSEN - get chosen node (stub returns NULL) */
#ifndef DT_CHOSEN
#define DT_CHOSEN(x) NULL
#endif

/* Device reference */
#ifndef DEVICE_DT_GET
#define DEVICE_DT_GET(x) NULL
#endif

#endif /* ZEPHYR_INCLUDE_DEVICETREE_H_ */