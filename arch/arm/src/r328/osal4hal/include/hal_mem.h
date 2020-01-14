#ifndef SUNXI_HAL_MEM_H
#define SUNXI_HAL_MEM_H

#include <stddef.h>
#include <stdint.h>

void *hal_malloc(uint32_t size);
void *hal_zalloc(uint32_t size);
void hal_free(void *p);
void *hal_align_malloc(uint32_t size, uint32_t align);
void hal_align_free(void *p);

#endif /* SUNXI_HAL_MEM_H */
