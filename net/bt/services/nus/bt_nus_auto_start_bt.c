/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/classic/classic.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>
#include <stdio.h>
#if defined(CONFIG_BT_A2DP_SINK)
#include <nuttx/net/bluetooth/classic/a2dp_codec_sbc.h>
#include <nuttx/net/bluetooth/classic/a2dp.h>
#include <nuttx/net/bluetooth/classic/sdp.h>
#endif
#include <nuttx/net/bluetooth/services/nus.h>

LOG_MODULE_REGISTER(bt_nus_auto_start);

#define DEVICE_NAME_SEPARATOR	"-BT-"
#define DEVICE_NAME_SUFFIX_LEN	4
#define DEVICE_NAME_DYNAMIC_LEN	(sizeof(CONFIG_BT_DEVICE_NAME) - 1 + \
					 sizeof(DEVICE_NAME_SEPARATOR) - 1 + \
					 DEVICE_NAME_SUFFIX_LEN)

#if defined(CONFIG_BT_DEVICE_NAME_DYNAMIC)
#define DEVICE_NAME_BUF_LEN	MAX(CONFIG_BT_DEVICE_NAME_MAX, DEVICE_NAME_DYNAMIC_LEN)
#else
#define DEVICE_NAME_BUF_LEN	MAX(sizeof(CONFIG_BT_DEVICE_NAME) - 1, DEVICE_NAME_DYNAMIC_LEN)
#endif

static char device_name[DEVICE_NAME_BUF_LEN + 1];

#if defined(CONFIG_BT_BROADCASTER)
#if defined(CONFIG_BT_CLASSIC)
#define NUS_AD_FLAGS		BT_LE_AD_GENERAL
#else
#define NUS_AD_FLAGS		(BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)
#endif

static struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, NUS_AD_FLAGS),
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name, 0),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};
#endif

#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
#if defined(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
static enum bt_security_err bt_nus_pairing_accept(struct bt_conn *conn,
						  const struct bt_conn_pairing_feat *const feat)
{
	(void)feat;

	LOG_INF("Pairing accepted: %s", bt_conn_dst_str(conn));
	return BT_SECURITY_ERR_SUCCESS;
}
#endif

static void bt_nus_auth_cancel(struct bt_conn *conn)
{
	LOG_INF("Pairing cancelled: %s", bt_conn_dst_str(conn));
}

static void bt_nus_pairing_confirm(struct bt_conn *conn)
{
	int err;

	LOG_INF("Pairing confirmed: %s", bt_conn_dst_str(conn));

	err = bt_conn_auth_pairing_confirm(conn);
	if (err) {
		LOG_WRN("Failed to confirm pairing for %s (err %d)",
			bt_conn_dst_str(conn), err);
	}
}

#if defined(CONFIG_BT_CLASSIC)
static void bt_nus_pincode_entry(struct bt_conn *conn, bool highsec)
{
	const char *pin = highsec ? "0000000000000000" : "0000";
	int err;

	LOG_INF("BR/EDR PIN requested: %s highsec %u", bt_conn_dst_str(conn),
		(unsigned int)highsec);

	err = bt_conn_auth_pincode_entry(conn, pin);
	if (err) {
		LOG_WRN("Failed to enter BR/EDR PIN for %s (err %d)",
			bt_conn_dst_str(conn), err);
	}
}
#endif

static const struct bt_conn_auth_cb bt_nus_auth_cb = {
#if defined(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
	.pairing_accept = bt_nus_pairing_accept,
#endif
#if defined(CONFIG_BT_CLASSIC)
	.pincode_entry = bt_nus_pincode_entry,
#endif
	.cancel = bt_nus_auth_cancel,
	.pairing_confirm = bt_nus_pairing_confirm,
};

static int bt_nus_auth_register(void)
{
	int err;

	err = bt_conn_auth_cb_register(&bt_nus_auth_cb);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("Failed to register auth callbacks (err %d)", err);
		return err;
	}

	LOG_INF("Bluetooth auth callbacks registered");
	return 0;
}
#endif

#if defined(CONFIG_BT_CLASSIC)
static struct bt_data eir[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, device_name, 0),
};

#endif

static int bt_nus_update_device_name(void)
{
	bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
	size_t count = ARRAY_SIZE(addrs);
#if defined(CONFIG_BT_DEVICE_NAME_DYNAMIC)
	int err;
#endif
	int len;

	bt_id_get(addrs, &count);
	if (count > 0 && !bt_addr_le_eq(&addrs[BT_ID_DEFAULT], BT_ADDR_LE_ANY)) {
		len = snprintf(device_name, sizeof(device_name), "%s%s%02X%02X",
			       CONFIG_BT_DEVICE_NAME, DEVICE_NAME_SEPARATOR,
			       addrs[BT_ID_DEFAULT].a.val[1],
			       addrs[BT_ID_DEFAULT].a.val[0]);
	} else {
		len = snprintf(device_name, sizeof(device_name), "%s", CONFIG_BT_DEVICE_NAME);
	}

	if (len < 0 || (size_t)len >= sizeof(device_name)) {
		return -ENOMEM;
	}

#if defined(CONFIG_BT_DEVICE_NAME_DYNAMIC)
	err = bt_set_name(device_name);

	if (err < 0) {
		LOG_ERR("Failed to set Bluetooth device name (err %d)", err);
		return err;
	}
#endif

#if defined(CONFIG_BT_BROADCASTER)
	ad[1].data_len = strlen(device_name);
#endif
#if defined(CONFIG_BT_CLASSIC)
	eir[0].data_len = strlen(device_name);
#endif

	LOG_INF("Bluetooth device name: %s", device_name);
	return 0;
}

#if defined(CONFIG_BT_CLASSIC)
#if defined(CONFIG_BT_A2DP_SINK)
#define A2DP_VERSION 0x0104U

#define A2DP_AAC_OBJECT_TYPE_MPEG2_AAC_LC 0x80U
#define A2DP_AAC_OBJECT_TYPE_MPEG4_AAC_LC 0x40U
#define A2DP_AAC_SAMP_FREQ_48000_HI       0x10U
#define A2DP_AAC_SAMP_FREQ_44100_HI       0x08U
#define A2DP_AAC_CHANNELS_2               0x04U
#define A2DP_AAC_VBR                      0x80U
#define A2DP_AAC_BIT_RATE_320000          0x04e200U

BT_A2DP_SBC_SINK_EP_DEFAULT(bt_nus_a2dp_sink_sbc_endpoint);

static struct bt_a2dp_codec_ie bt_nus_a2dp_sink_aac_cap = {
	.len = BT_A2DP_MPEG_2_4_IE_LENGTH,
	.codec_ie = {
		A2DP_AAC_OBJECT_TYPE_MPEG2_AAC_LC | A2DP_AAC_OBJECT_TYPE_MPEG4_AAC_LC,
		A2DP_AAC_SAMP_FREQ_48000_HI | A2DP_AAC_SAMP_FREQ_44100_HI,
		A2DP_AAC_CHANNELS_2,
		A2DP_AAC_VBR | ((A2DP_AAC_BIT_RATE_320000 >> 16) & 0x7fU),
		(A2DP_AAC_BIT_RATE_320000 >> 8) & 0xffU,
		A2DP_AAC_BIT_RATE_320000 & 0xffU,
	},
};

static struct bt_a2dp_ep bt_nus_a2dp_sink_aac_endpoint =
	BT_A2DP_SINK_EP_INIT(BT_A2DP_MPEG2, &bt_nus_a2dp_sink_aac_cap, true);

static struct bt_a2dp_stream bt_nus_a2dp_sink_stream;

static struct bt_sdp_attribute bt_nus_a2dp_sink_attrs[] = {
	BT_SDP_NEW_SERVICE,
	BT_SDP_LIST(
		BT_SDP_ATTR_SVCLASS_ID_LIST,
		BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 3),
		BT_SDP_DATA_ELEM_LIST(
		{
			BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
			BT_SDP_ARRAY_16(BT_SDP_AUDIO_SINK_SVCLASS)
		},
		)
	),
	BT_SDP_LIST(
		BT_SDP_ATTR_PROTO_DESC_LIST,
		BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 16),
		BT_SDP_DATA_ELEM_LIST(
		{
			BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 6),
			BT_SDP_DATA_ELEM_LIST(
			{
				BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
				BT_SDP_ARRAY_16(BT_SDP_PROTO_L2CAP)
			},
			{
				BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
				BT_SDP_ARRAY_16(BT_UUID_AVDTP_VAL)
			},
			)
		},
		{
			BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 6),
			BT_SDP_DATA_ELEM_LIST(
			{
				BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
				BT_SDP_ARRAY_16(BT_UUID_AVDTP_VAL)
			},
			{
				BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
				BT_SDP_ARRAY_16(AVDTP_VERSION)
			},
			)
		},
		)
	),
	BT_SDP_LIST(
		BT_SDP_ATTR_PROFILE_DESC_LIST,
		BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 8),
		BT_SDP_DATA_ELEM_LIST(
		{
			BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 6),
			BT_SDP_DATA_ELEM_LIST(
			{
				BT_SDP_TYPE_SIZE(BT_SDP_UUID16),
				BT_SDP_ARRAY_16(BT_SDP_ADVANCED_AUDIO_SVCLASS)
			},
			{
				BT_SDP_TYPE_SIZE(BT_SDP_UINT16),
				BT_SDP_ARRAY_16(A2DP_VERSION)
			},
			)
		},
		)
	),
	BT_SDP_SERVICE_NAME("A2DPSink"),
	BT_SDP_SUPPORTED_FEATURES(0x0001U),
};

static struct bt_sdp_record bt_nus_a2dp_sink_rec =
	BT_SDP_RECORD(bt_nus_a2dp_sink_attrs);

static void bt_nus_a2dp_connected(struct bt_a2dp *a2dp, int err)
{
	if (err) {
		LOG_WRN("A2DP sink connect failed (err %d)", err);
		return;
	}

	LOG_INF("A2DP sink connected");
}

static void bt_nus_a2dp_disconnected(struct bt_a2dp *a2dp)
{
	LOG_INF("A2DP sink disconnected");
}

static void bt_nus_a2dp_stream_recv(struct bt_a2dp_stream *stream,
					    struct net_buf *buf, uint16_t seq_num,
					    uint32_t ts)
{
	LOG_DBG("A2DP sink media: seq %u ts %u len %u", seq_num,
		(unsigned int)ts, buf->len);
}

static struct bt_a2dp_stream_ops bt_nus_a2dp_stream_ops = {
	.recv = bt_nus_a2dp_stream_recv,
};

static int bt_nus_a2dp_config_req(struct bt_a2dp *a2dp,
					  struct bt_a2dp_ep *ep,
					  struct bt_a2dp_codec_cfg *codec_cfg,
					  struct bt_a2dp_stream **stream,
					  uint8_t *rsp_err_code)
{
	bt_a2dp_stream_cb_register(&bt_nus_a2dp_sink_stream,
					 &bt_nus_a2dp_stream_ops);
	*stream = &bt_nus_a2dp_sink_stream;
	*rsp_err_code = 0;

	LOG_INF("A2DP sink config request accepted");
	return 0;
}

static int bt_nus_a2dp_accept_req(struct bt_a2dp_stream *stream,
					  uint8_t *rsp_err_code)
{
	*rsp_err_code = 0;
	return 0;
}

static int bt_nus_a2dp_reconfig_req(struct bt_a2dp_stream *stream,
					    struct bt_a2dp_codec_cfg *codec_cfg,
					    uint8_t *rsp_err_code)
{
	*rsp_err_code = 0;
	return 0;
}

static struct bt_a2dp_cb bt_nus_a2dp_cb = {
	.connected = bt_nus_a2dp_connected,
	.disconnected = bt_nus_a2dp_disconnected,
	.config_req = bt_nus_a2dp_config_req,
	.reconfig_req = bt_nus_a2dp_reconfig_req,
	.establish_req = bt_nus_a2dp_accept_req,
	.release_req = bt_nus_a2dp_accept_req,
	.start_req = bt_nus_a2dp_accept_req,
	.suspend_req = bt_nus_a2dp_accept_req,
	.abort_req = bt_nus_a2dp_accept_req,
	.get_config_req = bt_nus_a2dp_accept_req,
};

static int bt_nus_a2dp_sink_register(void)
{
	int err;

	err = bt_sdp_register_service(&bt_nus_a2dp_sink_rec);
	if (err < 0 && err != -EEXIST) {
		LOG_ERR("Failed to register A2DP sink SDP record (err %d)", err);
		return err;
	}

	err = bt_a2dp_register_cb(&bt_nus_a2dp_cb);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("Failed to register A2DP sink callbacks (err %d)", err);
		return err;
	}

	err = bt_a2dp_register_ep(&bt_nus_a2dp_sink_sbc_endpoint,
				  BT_AVDTP_AUDIO, BT_AVDTP_SINK);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("Failed to register A2DP SBC sink endpoint (err %d)", err);
		return err;
	}

	LOG_INF("A2DP SBC sink endpoint registered");

	err = bt_a2dp_register_ep(&bt_nus_a2dp_sink_aac_endpoint,
				  BT_AVDTP_AUDIO, BT_AVDTP_SINK);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("Failed to register A2DP AAC sink endpoint (err %d)", err);
		return err;
	}

	LOG_INF("A2DP AAC sink endpoint registered");
	return 0;
}
#endif

static enum bt_br_conn_req_rsp bt_nus_br_conn_req(const bt_addr_t *addr, uint32_t cod)
{
	char addr_str[BT_ADDR_STR_LEN];

	bt_addr_to_str(addr, addr_str, sizeof(addr_str));
	LOG_INF("BR/EDR conn req: addr %s cod 0x%06x", addr_str, (unsigned int)cod);

	return BT_BR_CONN_REQ_ACCEPT_PERIPHERAL;
}

static void bt_nus_connected(struct bt_conn *conn, uint8_t err)
{
	if (!bt_conn_is_type(conn, BT_CONN_TYPE_BR)) {
		return;
	}

	if (err) {
		LOG_WRN("BR/EDR connection failed: %s err 0x%02x", bt_conn_dst_str(conn), err);
		return;
	}

	LOG_INF("BR/EDR connected: %s", bt_conn_dst_str(conn));
}

static void bt_nus_disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (!bt_conn_is_type(conn, BT_CONN_TYPE_BR)) {
		return;
	}

	LOG_INF("BR/EDR disconnected: %s reason 0x%02x", bt_conn_dst_str(conn), reason);
}

static struct bt_conn_cb bt_nus_conn_callbacks = {
	.connected = bt_nus_connected,
	.disconnected = bt_nus_disconnected,
};
#endif

static int bt_nus_auto_start(void)
{
	int err;

	err = bt_enable(NULL);
	DEBUGASSERT(!err);

	err = bt_nus_update_device_name();
	DEBUGASSERT(!err);

#if defined(CONFIG_BT_SMP) || defined(CONFIG_BT_CLASSIC)
	err = bt_nus_auth_register();
	DEBUGASSERT(!err);
#endif

#if defined(CONFIG_BT_BROADCASTER)
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	DEBUGASSERT(!err);
#endif

#ifdef CONFIG_BT_CLASSIC
	err = bt_conn_cb_register(&bt_nus_conn_callbacks);
	DEBUGASSERT(!err);

#if defined(CONFIG_BT_A2DP_SINK)
	err = bt_nus_a2dp_sink_register();
	DEBUGASSERT(!err);
#endif

	err = bt_br_set_connectable(true, bt_nus_br_conn_req);
	DEBUGASSERT(!err);

	err = bt_br_set_class_of_device(CONFIG_BT_COD);
	DEBUGASSERT(!err);

	err = bt_br_write_eir(eir, ARRAY_SIZE(eir), true);
	DEBUGASSERT(!err);

	err = bt_br_set_discoverable(true, false);
	DEBUGASSERT(!err);
#endif

	return 0;
}

SYS_INIT(bt_nus_auto_start, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
