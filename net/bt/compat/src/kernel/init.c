/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <syslog.h>

/* Init entry iteration.
 * Uses SYS_INIT sections placed by the compiler/linker.
 * The __start_init_entries and __stop_init_entries symbols are
 * provided by the linker when the init_entries section is populated.
 */

/* Weak default empty symbols if linker doesn't provide them */
__attribute__((weak)) const struct init_entry __start_init_entries[0];
__attribute__((weak)) const struct init_entry __stop_init_entries[0];

int z_sys_init_run_level(int level)
{
    uintptr_t start = (uintptr_t)__start_init_entries;
    uintptr_t end   = (uintptr_t)__stop_init_entries;

    if (start == 0 || end == 0 || start >= end)
        return 0;

    const struct init_entry *entry;
    for (entry = __start_init_entries; entry < __stop_init_entries; entry++)
        {
            if (entry->level == level && entry->init)
                {
                    int ret = entry->init();
                    if (ret < 0)
                        {
                            syslog(LOG_ERR, "Init %s failed: %d\n",
                                   entry->name ? entry->name : "?", ret);
                            return ret;
                        }
                }
        }
    return 0;
}

int do_k_mem_slab_init(void)
{
    return z_sys_init_run_level(PRE_KERNEL_1);
}