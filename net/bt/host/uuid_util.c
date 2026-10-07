/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal uuid string conversion implementation for NuttX BT host.
 * Uses standard C library only (snprintf/sscanf).
 */
#include <stdio.h>
#include <string.h>
#include "zephyr/sys/uuid.h"

int uuid_to_string(const struct uuid *data, char out[UUID_STR_LEN])
{
    return snprintf(out, UUID_STR_LEN,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        data->val[0], data->val[1], data->val[2], data->val[3],
        data->val[4], data->val[5], data->val[6], data->val[7],
        data->val[8], data->val[9], data->val[10], data->val[11],
        data->val[12], data->val[13], data->val[14], data->val[15]);
}

int uuid_from_string(const char data[UUID_STR_LEN], struct uuid *out)
{
    unsigned int v[16];
    int n = sscanf(data,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
        &v[8], &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15]);
    if (n != 16) return -1;
    for (int i = 0; i < 16; i++) out->val[i] = (uint8_t)v[i];
    return 0;
}

int uuid_cmp(const struct uuid *a, const struct uuid *b)
{
    return memcmp(a->val, b->val, UUID_SIZE);
}

bool uuid_is_nil(const struct uuid *data)
{
    for (int i = 0; i < UUID_SIZE; i++)
        if (data->val[i] != 0) return false;
    return true;
}

int uuid_copy(const struct uuid *data, struct uuid *out)
{
    memcpy(out->val, data->val, UUID_SIZE);
    return 0;
}