/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal uuid.h for NuttX - provides struct uuid and string conversion
 * used by Zephyr Bluetooth host. Uses only standard C library.
 */
#ifndef ZEPHYR_INCLUDE_SYS_UUID_H_
#define ZEPHYR_INCLUDE_SYS_UUID_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UUID_SIZE 16U
#define UUID_STR_LEN 37U

struct uuid {
    uint8_t val[UUID_SIZE];
};

int uuid_to_string(const struct uuid *data, char out[UUID_STR_LEN]);
int uuid_from_string(const char data[UUID_STR_LEN], struct uuid *out);
int uuid_cmp(const struct uuid *a, const struct uuid *b);
bool uuid_is_nil(const struct uuid *data);
int uuid_copy(const struct uuid *data, struct uuid *out);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_UUID_H_ */