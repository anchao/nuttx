/******************************************************************************
 *
 * Copyright (C) 2024 Xiaomi Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *****************************************************************************/

#include <stddef.h>

/* bt_l2cap_fixed_chan START */
extern struct bt_l2cap_fixed_chan bt_l2cap_fixed_chanz_att_fixed_chan;
extern struct bt_l2cap_fixed_chan bt_l2cap_fixed_chanle_fixed_chan;
extern struct bt_l2cap_fixed_chan bt_l2cap_fixed_chansmp_fixed_chan;

struct bt_l2cap_fixed_chan *_bt_l2cap_fixed_chan_list[] = {
#if defined(CONFIG_BT_CONN)
	&bt_l2cap_fixed_chanz_att_fixed_chan,
	&bt_l2cap_fixed_chanle_fixed_chan,
#endif /* CONFIG_BT_CONN */
#if defined(CONFIG_BT_SMP)
	&bt_l2cap_fixed_chansmp_fixed_chan,
#endif /* CONFIG_BT_SMP */
	NULL,
};
/* bt_l2cap_fixed_chan END */

/* bt_l2cap_br_fixed_chan START */
#if defined(CONFIG_BT_CLASSIC)
extern struct bt_l2cap_br_fixed_chan br_fixed_chan;
#if defined(CONFIG_BT_SMP_BR)
extern struct bt_l2cap_br_fixed_chan smp_br_fixed_chan;
#endif /* CONFIG_BT_SMP_BR */

struct bt_l2cap_br_fixed_chan *_bt_l2cap_br_fixed_chan_list[] = {
	&br_fixed_chan,
#if defined(CONFIG_BT_SMP_BR)
	&smp_br_fixed_chan,
#endif /* CONFIG_BT_SMP_BR */
	NULL,
};
#endif /* CONFIG_BT_CLASSIC */
/* bt_l2cap_br_fixed_chan END */

/* bt_gatt_service_static START */
extern const struct bt_gatt_service_static cas_svc;
extern const struct bt_gatt_service_static _2_gap_svc;
extern const struct bt_gatt_service_static _1_gatt_svc;
extern const struct bt_gatt_service_static cts_svc;
extern const struct bt_gatt_service_static dis_svc;
extern const struct bt_gatt_service_static hrs_svc;
extern const struct bt_gatt_service_static tps_svc;
extern const struct bt_gatt_service_static bas;
extern const struct bt_gatt_service_static ias_svc;

const struct bt_gatt_service_static *_bt_gatt_service_static_list[] = {
#if defined(CONFIG_BT_CAP_ACCEPTOR) && !defined(CONFIG_BT_CAP_ACCEPTOR_SET_MEMBER)
	&cas_svc,
#endif /* CONFIG_BT_CAP_ACCEPTOR && !CONFIG_BT_CAP_ACCEPTOR_SET_MEMBER */
#if defined(CONFIG_BT_CONN)
	&_2_gap_svc,    &_1_gatt_svc,
#endif /* CONFIG_BT_CONN */
#if defined(CONFIG_BT_CTS)
	&cts_svc,
#endif /* CONFIG_BT_CTS */
#if defined(CONFIG_BT_DIS)
	&dis_svc,
#endif /* CONFIG_BT_DIS */
#if defined(CONFIG_BT_HRS)
	&hrs_svc,
#endif /* CONFIG_BT_HRS */
#if defined(CONFIG_BT_TPS)
	&tps_svc,
#endif /* CONFIG_BT_TPS */
#if defined(CONFIG_BT_BAS)
	&bas,
#endif /* CONFIG_BT_BAS */
#if defined(CONFIG_BT_IAS)
	&ias_svc,
#endif /* CONFIG_BT_IAS */
	NULL,
};
/* bt_gatt_service_static END */

/* bt_sco_conn_cb START */
const struct bt_sco_conn_cb *_bt_sco_conn_cb_list[] = {
	NULL,
};
/* bt_sco_conn_cb END */

/* bt_sco_hci_cb START */
struct bt_sco_hci_cb *_bt_sco_hci_cb_list[] = {
	NULL,
};
/* bt_sco_hci_cb END */

/* bt_conn_cb START */
const struct bt_conn_cb *_bt_conn_cb_list[] = {
	NULL,
};
/* bt_conn_cb END */

/* bt_mesh_friend_cb START */
extern const struct bt_mesh_friend_cb bt_mesh_friend_cb_friend_cb;
const struct bt_mesh_friend_cb *_bt_mesh_friend_cb_list[] = {
#if defined(CONFIG_BT_TESTER)
#if defined(CONFIG_BT_MESH)
	&bt_mesh_friend_cb_friend_cb,
#endif /* CONFIG_BT_MESH */
#endif /* CONFIG_BT_TESTER */
	NULL,
};
/* bt_mesh_friend_cb END */

/* bt_mesh_subnet_cb START */
extern const struct bt_mesh_subnet_cb bt_mesh_subnet_cb_app_keys;
extern const struct bt_mesh_subnet_cb bt_mesh_subnet_cb_beacon;
extern const struct bt_mesh_subnet_cb bt_mesh_subnet_cb_sbr;
extern const struct bt_mesh_subnet_cb bt_mesh_subnet_cb_friend;
extern const struct bt_mesh_subnet_cb bt_mesh_subnet_cb_lpn;
extern const struct bt_mesh_subnet_cb bt_mesh_subnet_cb_proxy_cli;
extern const struct bt_mesh_subnet_cb bt_mesh_subnet_cb_gatt_services;
const struct bt_mesh_subnet_cb *_bt_mesh_subnet_cb_list[] = {
#if defined(CONFIG_BT_MESH)
	&bt_mesh_subnet_cb_app_keys,
	&bt_mesh_subnet_cb_beacon,
#if defined(CONFIG_BT_MESH_BRG_CFG_SRV)
	&bt_mesh_subnet_cb_sbr,
#endif /* CONFIG_BT_MESH_BRG_CFG_SRV */
#if defined(CONFIG_BT_MESH_FRIEND)
	&bt_mesh_subnet_cb_friend,
#endif /* CONFIG_BT_MESH_FRIEND */
#if defined(CONFIG_BT_MESH_LPN)
	&bt_mesh_subnet_cb_lpn,
#endif /* CONFIG_BT_MESH_LPN */
#if defined(CONFIG_BT_MESH_PROXY_CLIENT)
	&bt_mesh_subnet_cb_proxy_cli,
#endif /* CONFIG_BT_MESH_PROXY_CLIENT */
#if defined(CONFIG_BT_MESH_GATT_PROXY)
	&bt_mesh_subnet_cb_gatt_services,
#endif /* CONFIG_BT_MESH_GATT_PROXY */
#endif /* CONFIG_BT_MESH */
	NULL,
};
/* bt_mesh_subnet_cb END */

/* bt_mesh_proxy_cb START */
const struct bt_mesh_proxy_cb *_bt_mesh_proxy_cb_list[] = {
	NULL,
};
/* bt_mesh_proxy_cb END */

/* bt_mesh_hb_cb START */
extern const struct bt_mesh_hb_cb hb_cb;
const struct bt_mesh_hb_cb *_bt_mesh_hb_cb_list[] = {
#if defined(CONFIG_BT_MESH)
#if defined(CONFIG_BT_MESH_DEMO)
	&hb_cb,
#endif /* CONFIG_BT_MESH_DEMO */
#endif /* CONFIG_BT_MESH */
	NULL,
};
/* bt_mesh_hb_cb END */

/* bt_mesh_app_key_cb START */
extern const struct bt_mesh_app_key_cb bt_mesh_app_key_cb_app_key_evt;
const struct bt_mesh_app_key_cb *_bt_mesh_app_key_cb_list[] = {
#if defined(CONFIG_BT_MESH)
	&bt_mesh_app_key_cb_app_key_evt,
#endif /* CONFIG_BT_MESH */
	NULL,
};
/* bt_mesh_app_key_cb END */

/* bt_ias_cb START */
#if defined(CONFIG_BT_IAS)
struct bt_ias_cb *_bt_ias_cb_list[] = {
	NULL,
};
#endif /* CONFIG_BT_IAS */
/* bt_ias_cb END */
