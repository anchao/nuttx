/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_DEVICETREE_GENERATED_H_
#define ZEPHYR_INCLUDE_DEVICETREE_GENERATED_H_

/* NuttX stubs for Zephyr devicetree macros used by BT host code */

/* DT_HAS_CHOSEN returns 1 if the chosen node exists. For BT host,
 * we always claim zephyr,bt-hci is chosen so the first code path is taken. */
#define DT_HAS_CHOSEN(x) 1

/* The devicetree node label is not meaningful on NuttX */
#define DT_CHOSEN(x)       NULL

/* Device reference */
#define DEVICE_DT_GET(x)   NULL

/* HCI bus/name/quirks from devicetree - simple stubs */
#define BT_DT_HCI_BUS_GET(x)    0
#define BT_DT_HCI_NAME_GET(x)   "nuttx_bt_hci"
#define BT_DT_HCI_QUIRKS_GET(x) 0

#endif /* ZEPHYR_INCLUDE_DEVICETREE_GENERATED_H_ */