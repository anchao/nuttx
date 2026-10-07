/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_SYS_CRC_H_
#define ZEPHYR_INCLUDE_SYS_CRC_H_

#include <stddef.h>
#include <stdint.h>
#include <nuttx/crc32.h>

static inline uint16_t crc16_reflect(uint16_t poly, uint16_t seed,
				     const uint8_t *src, size_t len)
{
	uint16_t crc = seed;

	while (len-- > 0) {
		crc ^= *src++;

		for (int i = 0; i < 8; i++) {
			if ((crc & 0x0001) != 0) {
				crc = (crc >> 1) ^ poly;
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

#endif /* ZEPHYR_INCLUDE_SYS_CRC_H_ */
