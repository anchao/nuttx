/** @file
 *  @brief Custom logging over UART
 */

/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2026 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <nuttx/spinlock.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <nuttx/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <nuttx/clock.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>

#include "monitor.h"
#include "monitor_buffer.h"

/* This is the same default priority as for other console handlers,
 * except that we're not exporting it as a Kconfig variable until a
 * clear need arises.
 */
#define MONITOR_INIT_PRIORITY 60

/* TS resolution is 1/10th of a millisecond */
#define MONITOR_TS_FREQ 10000

enum {
	BT_LOG_BUSY,
};

static atomic_t flags;

static struct {
	atomic_t cmd;
	atomic_t evt;
	atomic_t acl_tx;
	atomic_t acl_rx;
#if defined(CONFIG_BT_CLASSIC)
	atomic_t sco_tx;
	atomic_t sco_rx;
#endif
	atomic_t other;
} drops;

static void drop_add(uint16_t opcode)
{
	switch (opcode) {
	case BT_MONITOR_COMMAND_PKT:
		atomic_add(&drops.cmd, 1);
		break;
	case BT_MONITOR_EVENT_PKT:
		atomic_add(&drops.evt, 1);
		break;
	case BT_MONITOR_ACL_TX_PKT:
		atomic_add(&drops.acl_tx, 1);
		break;
	case BT_MONITOR_ACL_RX_PKT:
		atomic_add(&drops.acl_rx, 1);
		break;
#if defined(CONFIG_BT_CLASSIC)
	case BT_MONITOR_SCO_TX_PKT:
		atomic_add(&drops.sco_tx, 1);
		break;
	case BT_MONITOR_SCO_RX_PKT:
		atomic_add(&drops.sco_rx, 1);
		break;
#endif
	default:
		atomic_add(&drops.other, 1);
		break;
	}
}

#if defined(CONFIG_BT_DEBUG_MONITOR_UART)
static const struct device *const monitor_dev = NULL;

static void monitor_poll_send(const struct bt_monitor_data *frags, size_t count)
{
	for (size_t i = 0; i < count; i++) {
		const uint8_t *buf = frags[i].data;

		for (size_t j = 0; j < frags[i].len; j++) {
			uart_poll_out(monitor_dev, buf[j]);
		}
	}
}

#if defined(CONFIG_BT_DEBUG_MONITOR_UART_INTERRUPT_DRIVEN)
/* Single producer (serialized by BT_LOG_BUSY) and single consumer (the UART
 * ISR), matching the circular buffer's lock-free SPSC contract. The memory
 * ordering that contract additionally requires on SMP systems is provided by
 * the fences in the bt_monitor_ring_buf_* helpers.
 */
static uint8_t monitor_tx_data[CONFIG_BT_DEBUG_MONITOR_UART_BUFFER_SIZE];
static struct circbuf_s monitor_tx_buf = {
	.base = monitor_tx_data,
	.size = sizeof(monitor_tx_data),
	.external = true,
};
static atomic_t monitor_tx_busy;
static spinlock_t monitor_tx_lock;
/* Set when the UART driver lacks interrupt support */
static bool poll_mode;

static void monitor_uart_tx(void)
{
	uint8_t *data;
	uint32_t len;
	int sent;

	len = bt_monitor_ring_buf_get_ptr(&monitor_tx_buf, &data);
	if (len > 0) {
		sent = uart_fifo_fill(monitor_dev, data, len);
		if (sent > 0) {
			bt_monitor_ring_buf_consume(&monitor_tx_buf, sent);
			return;
		}

		/* TX ready but nothing accepted: disable TX so a persistent
		 * driver error cannot cause an interrupt storm. The next
		 * committed record re-enables TX and retries. Unlike the
		 * empty-buffer path below, deliberately no re-check here:
		 * the buffer is non-empty at this point, so re-enabling
		 * would defeat the backoff.
		 */
		uart_irq_tx_disable(monitor_dev);
		atomic_set(&monitor_tx_busy, 0);
		return;
	}

	uart_irq_tx_disable(monitor_dev);
	atomic_set(&monitor_tx_busy, 0);

	/* Close the race with a producer that committed while TX was disabled. */
	if (!circbuf_is_empty(&monitor_tx_buf) &&
	    atomic_cmpxchg_value(&monitor_tx_busy, 0, 1)) {
		uart_irq_tx_enable(monitor_dev);
	}
}

static void monitor_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	irqstate_t key = spin_lock_irqsave(&monitor_tx_lock);

	uart_irq_update(dev);
	if (uart_irq_tx_ready(dev) > 0) {
		monitor_uart_tx();
	}

	spin_unlock_irqrestore(&monitor_tx_lock, key);
}

static bool monitor_send(const struct bt_monitor_data *frags, size_t count)
{
	if (poll_mode) {
		monitor_poll_send(frags, count);
		return true;
	}

	if (!bt_monitor_ring_buf_put(&monitor_tx_buf, frags, count)) {
		return false;
	}

	if (atomic_cmpxchg_value(&monitor_tx_busy, 0, 1)) {
		uart_irq_tx_enable(monitor_dev);
	}

	return true;
}
#else
static bool monitor_send(const struct bt_monitor_data *frags, size_t count)
{
	monitor_poll_send(frags, count);

	return true;
}
#endif /* CONFIG_BT_DEBUG_MONITOR_UART_INTERRUPT_DRIVEN */
#endif /* CONFIG_BT_DEBUG_MONITOR_UART */

static void encode_drops(struct bt_monitor_hdr *hdr, uint8_t type,
			 atomic_t *val)
{
	int32_t count;

	count = atomic_xchg(val, 0);
	if (count) {
		hdr->ext[hdr->hdr_len++] = type;
		hdr->ext[hdr->hdr_len++] = MIN(count, 255);
		if (count > 255) {
			/* Keep the surplus for the next record */
			atomic_add(val, count - 255);
		}
	}
}

static atomic_t *drop_counter(uint8_t type)
{
	switch (type) {
	case BT_MONITOR_COMMAND_DROPS:
		return &drops.cmd;
	case BT_MONITOR_EVENT_DROPS:
		return &drops.evt;
	case BT_MONITOR_ACL_TX_DROPS:
		return &drops.acl_tx;
	case BT_MONITOR_ACL_RX_DROPS:
		return &drops.acl_rx;
#if defined(CONFIG_BT_CLASSIC)
	case BT_MONITOR_SCO_TX_DROPS:
		return &drops.sco_tx;
	case BT_MONITOR_SCO_RX_DROPS:
		return &drops.sco_rx;
#endif
	case BT_MONITOR_OTHER_DROPS:
		return &drops.other;
	default:
		return NULL;
	}
}

/* Restore drop counts that encode_hdr() consumed into a record which then
 * failed to be sent: without this, every record dropped back-to-back would
 * destroy the counts accumulated by its predecessors, understating drops
 * under sustained overload.
 */
static void restore_drops(const struct bt_monitor_hdr *hdr)
{
	/* The timestamp is always encoded first, drop pairs follow */
	for (uint8_t i = sizeof(struct bt_monitor_ts32); i + 1U < hdr->hdr_len; i += 2U) {
		atomic_t *counter = drop_counter(hdr->ext[i]);

		if (counter != NULL) {
			atomic_add(counter, hdr->ext[i + 1]);
		}
	}
}

/* Convert microseconds to the monitor timestamp resolution */
static uint64_t monitor_ts_from_us(uint64_t us)
{
	return us / (USEC_PER_SEC / MONITOR_TS_FREQ);
}

/* Timestamp for HCI packets: the real-time clock when configured,
 * otherwise the system timer. Conversions go via microseconds rather than
 * dividing by hw_cycles / MONITOR_TS_FREQ: the latter truncates badly for
 * cycle rates that are not a multiple of MONITOR_TS_FREQ (e.g. 32768 Hz
 * yields a divisor of 3 instead of 3.2768, making the timestamps run 9.2%
 * fast).
 */
static uint64_t monitor_ts_get(void)
{
	struct timespec ts;
	uint64_t us;

	if (IS_ENABLED(CONFIG_LOG_TIMESTAMP_USE_REALTIME) &&
	    clock_gettime(CLOCK_REALTIME, &ts) == 0) {
		us = ((uint64_t)ts.tv_sec * USEC_PER_SEC) + (ts.tv_nsec / NSEC_PER_USEC);
	} else {
		us = clock_systime_nsec() / NSEC_PER_USEC;
	}

	return monitor_ts_from_us(us);
}

static inline void encode_hdr(struct bt_monitor_hdr *hdr, uint64_t timestamp,
			      uint16_t opcode, uint16_t len)
{
	struct bt_monitor_ts32 *ts;

	hdr->opcode   = sys_cpu_to_le16(opcode);
	hdr->flags    = 0U;

	ts = (void *)hdr->ext;
	ts->type = BT_MONITOR_TS32;
	/* The btsnoop protocol only supports 32-bit timestamps (in 1/10th ms).
	 * This overflows after 4.97 days, which is acceptable as it only affects
	 * rendering in btmon/wireshark. Recordings are usually not that long
	 * anyway, and packet ordering is still preserved.
	 */
	ts->ts32 = (uint32_t)timestamp;
	hdr->hdr_len = sizeof(*ts);

	encode_drops(hdr, BT_MONITOR_COMMAND_DROPS, &drops.cmd);
	encode_drops(hdr, BT_MONITOR_EVENT_DROPS, &drops.evt);
	encode_drops(hdr, BT_MONITOR_ACL_TX_DROPS, &drops.acl_tx);
	encode_drops(hdr, BT_MONITOR_ACL_RX_DROPS, &drops.acl_rx);
#if defined(CONFIG_BT_CLASSIC)
	encode_drops(hdr, BT_MONITOR_SCO_TX_DROPS, &drops.sco_tx);
	encode_drops(hdr, BT_MONITOR_SCO_RX_DROPS, &drops.sco_rx);
#endif
	encode_drops(hdr, BT_MONITOR_OTHER_DROPS, &drops.other);

	hdr->data_len = sys_cpu_to_le16(4 + hdr->hdr_len + len);
}

void bt_monitor_send(uint16_t opcode, const void *data, size_t len)
{
	struct bt_monitor_hdr hdr;
	struct bt_monitor_data frags[] = {
		{ &hdr, 0 }, /* Length known only after encode_hdr() */
		{ data, len },
	};

	if (atomic_test_and_set_bit(&flags, BT_LOG_BUSY)) {
		drop_add(opcode);
		return;
	}

	encode_hdr(&hdr, monitor_ts_get(), opcode, len);
	frags[0].len = BT_MONITOR_BASE_HDR_LEN + hdr.hdr_len;

	if (!monitor_send(frags, ARRAY_SIZE(frags))) {
		restore_drops(&hdr);
		drop_add(opcode);
	}

	atomic_clear_bit(&flags, BT_LOG_BUSY);
}

void bt_monitor_new_index(uint8_t type, uint8_t bus, const bt_addr_t *addr,
			  const char *name)
{
	struct bt_monitor_new_index pkt;

	pkt.type = type;
	pkt.bus = bus;
	memcpy(pkt.bdaddr, addr, 6);
	strncpy(pkt.name, name, sizeof(pkt.name) - 1);
	pkt.name[sizeof(pkt.name) - 1] = '\0';

	bt_monitor_send(BT_MONITOR_NEW_INDEX, &pkt, sizeof(pkt));
}

static int bt_monitor_init(void)
{

#if defined(CONFIG_BT_DEBUG_MONITOR_UART)
	DEBUGASSERT(device_is_ready(monitor_dev));

#if defined(CONFIG_BT_DEBUG_MONITOR_UART_INTERRUPT_DRIVEN)
	/* SERIAL_SUPPORT_INTERRUPT only guarantees that some UART driver
	 * supports the interrupt API, not necessarily the monitor UART's.
	 * Fall back to polling instead of failing, so that a misconfigured
	 * board still produces monitor output.
	 */
	if (uart_irq_callback_user_data_set(monitor_dev, monitor_uart_isr, NULL) != 0) {
		poll_mode = true;
	}
#endif /* CONFIG_BT_DEBUG_MONITOR_UART_INTERRUPT_DRIVEN */

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_rx_disable(monitor_dev);
	uart_irq_tx_disable(monitor_dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
#endif /* CONFIG_BT_DEBUG_MONITOR_UART */
	return 0;
}

SYS_INIT(bt_monitor_init, PRE_KERNEL_1, MONITOR_INIT_PRIORITY);
