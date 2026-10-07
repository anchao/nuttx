/* SPDX-License-Identifier: Apache-2.0 */
/* Flash operations stub - backed by the filesystem layer */
#include <stdint.h>
#include <errno.h>

struct flash_area {
    int id;
    uint32_t offset;
    uint32_t size;
    int fd;
};

int flash_area_open(int id, struct flash_area **fa)
{
    /* Stub - flash access not needed for BT host */
    (void)id;
    (void)fa;
    return -ENODEV;
}

void flash_area_close(struct flash_area *fa)
{
    (void)fa;
}

int flash_area_read(struct flash_area *fa, uint32_t off, void *dst, uint32_t len)
{
    (void)fa;
    (void)off;
    (void)dst;
    (void)len;
    return -ENODEV;
}

int flash_area_write(struct flash_area *fa, uint32_t off, const void *src, uint32_t len)
{
    (void)fa;
    (void)off;
    (void)src;
    (void)len;
    return -ENODEV;
}

int flash_area_erase(struct flash_area *fa, uint32_t off, uint32_t len)
{
    (void)fa;
    (void)off;
    (void)len;
    return -ENODEV;
}