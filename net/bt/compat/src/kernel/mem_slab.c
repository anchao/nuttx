/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>

void k_mem_slab_init(struct k_mem_slab *slab, void *buffer, size_t block_size,
                     uint32_t num_blocks)
{
    slab->buffer = (char *)buffer;
    slab->block_size = block_size;
    slab->num_blocks = num_blocks;
    slab->num_used = 0;
    slab->max_used = 0;
    slab->lock = SP_UNLOCKED;

    /* Build free list */
    char *p = (char *)buffer;
    slab->free_list = NULL;
    for (uint32_t i = 0; i < num_blocks; i++)
        {
            *(char **)p = slab->free_list;
            slab->free_list = p;
            p += block_size;
        }
}

int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem, k_timeout_t timeout)
{
    (void)timeout;

    irqstate_t flags = spin_lock_irqsave(&slab->lock);

    if (slab->free_list == NULL)
        {
            spin_unlock_irqrestore(&slab->lock, flags);
            return -ENOMEM;
        }

    *mem = slab->free_list;
    slab->free_list = *(char **)slab->free_list;
    slab->num_used++;

    if (slab->num_used > slab->max_used)
        slab->max_used = slab->num_used;

    spin_unlock_irqrestore(&slab->lock, flags);
    return 0;
}

void k_mem_slab_free(struct k_mem_slab *slab, void **mem)
{
    irqstate_t flags = spin_lock_irqsave(&slab->lock);

    *(char **)(*mem) = slab->free_list;
    slab->free_list = (char *)(*mem);

    if (slab->num_used > 0)
        slab->num_used--;

    spin_unlock_irqrestore(&slab->lock, flags);
    *mem = NULL;
}

uint32_t k_mem_slab_num_free_get(struct k_mem_slab *slab)
{
    return slab->num_blocks - slab->num_used;
}

uint32_t k_mem_slab_max_used_get(struct k_mem_slab *slab)
{
    return slab->max_used;
}