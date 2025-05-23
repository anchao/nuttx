/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * $FreeBSD$
 */

#ifndef _RPMSG_INTERNAL_H_
#define _RPMSG_INTERNAL_H_

#include <stdint.h>
#include <openamp/rpmsg.h>

#if defined __cplusplus
extern "C" {
#endif

#ifdef RPMSG_DEBUG
#include <metal/log.h>

#define RPMSG_ASSERT(_exp, _msg) do { \
		if (!(_exp)) { \
			metal_log(METAL_LOG_EMERGENCY, \
				  "FATAL: %s - "_msg, __func__); \
			metal_assert(_exp); \
		} \
	} while (0)
#else
#define RPMSG_ASSERT(_exp, _msg) metal_assert(_exp)
#endif

/* Mask to get the rpmsg buffer held counter from rpmsg_hdr reserved field */
#define RPMSG_BUF_HELD_SHIFT 16
#define RPMSG_BUF_HELD_MASK  (0xFFFFU << RPMSG_BUF_HELD_SHIFT)

#define RPMSG_LOCATE_HDR(p) \
	((struct rpmsg_hdr *)((unsigned char *)(p) - sizeof(struct rpmsg_hdr)))
#define RPMSG_LOCATE_DATA(p) ((unsigned char *)(p) + sizeof(struct rpmsg_hdr))

/**
 * @brief dynamic name service announcement flags
 */
enum rpmsg_ns_flags {
	/** A new remote service was just created */
	RPMSG_NS_CREATE = 0,
	/** A known remote service was just destroyed */
	RPMSG_NS_DESTROY = 1,
	/** Aknowledge the previous creation message*/
	RPMSG_NS_CREATE_ACK = 2,
};

/**
 * @brief Common header for all RPMsg messages
 *
 * Every message sent(/received) on the RPMsg bus begins with this header.
 */
METAL_PACKED_BEGIN
struct rpmsg_hdr {
	/** Source address */
	uint32_t src;

	/** Destination address */
	uint32_t dst;

	/** Reserved for future use */
	uint32_t reserved;

	/** Length of payload (in bytes) */
	uint16_t len;

	/** Message flags */
	uint16_t flags;
} METAL_PACKED_END;

/**
 * @brief Dynamic name service announcement message
 *
 * This message is sent across to publish a new service, or announce
 * about its removal. When we receive these messages, an appropriate
 * RPMsg channel (i.e device) is created/destroyed. In turn, the ->probe()
 * or ->remove() handler of the appropriate RPMsg driver will be invoked
 * (if/as-soon-as one is registered).
 */
METAL_PACKED_BEGIN
struct rpmsg_ns_msg {
	/** Name of the remote service that is being published */
	char name[RPMSG_NAME_SIZE];

	/** Endpoint address of the remote service that is being published */
	uint32_t addr;

	/** Indicates whether service is created or destroyed */
	uint32_t flags;
} METAL_PACKED_END;

int rpmsg_send_ns_message(struct rpmsg_endpoint *ept, unsigned long flags);

struct rpmsg_endpoint *rpmsg_get_endpoint(struct rpmsg_device *rvdev,
					  const char *name, uint32_t addr,
					  uint32_t dest_addr);
void rpmsg_register_endpoint(struct rpmsg_device *rdev,
			     struct rpmsg_endpoint *ept,
			     const char *name,
			     uint32_t src, uint32_t dest,
			     rpmsg_ept_cb cb,
			     rpmsg_ns_unbind_cb ns_unbind_cb, void *priv);

static inline struct rpmsg_endpoint *
rpmsg_get_ept_from_addr(struct rpmsg_device *rdev, uint32_t addr)
{
	return rpmsg_get_endpoint(rdev, NULL, addr, RPMSG_ADDR_ANY);
}

/**
 * @internal
 *
 * @brief Increase the endpoint reference count
 *
 * This function is used to avoid calling ept_cb after release lock causes race condition
 * it should be called under lock protection.
 *
 * @param ept	pointer to rpmsg endpoint
 *
 */
void rpmsg_ept_incref(struct rpmsg_endpoint *ept);

/**
 * @internal
 *
 * @brief Decrease the end point reference count
 *
 * This function is used to avoid calling ept_cb after release lock causes race condition
 * it should be called under lock protection.
 *
 * @param ept	pointer to rpmsg endpoint
 */
void rpmsg_ept_decref(struct rpmsg_endpoint *ept);

#if defined __cplusplus
}
#endif

#endif /* _RPMSG_INTERNAL_H_ */
