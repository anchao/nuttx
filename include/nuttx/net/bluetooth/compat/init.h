/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_INIT_H_
#define ZEPHYR_INCLUDE_INIT_H_

#include <nuttx/config.h>

#include "sys/iterable_sections.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Init levels */
#define PRE_KERNEL_1  0
#define PRE_KERNEL_2  1
#define POST_KERNEL   2
#define APPLICATION   3
#define SMP_BOOT      4

struct init_entry {
    int (*init)(void);
    int level;
    int prio;
    const char *name;
};

/* Default priorities used by NuttX Bluetooth code when NuttX config does not
 * provide an equivalent symbol.
 */
#ifndef CONFIG_KERNEL_INIT_PRIORITY_DEFAULT
#define CONFIG_KERNEL_INIT_PRIORITY_DEFAULT 40
#endif

#ifndef CONFIG_APPLICATION_INIT_PRIORITY
#define CONFIG_APPLICATION_INIT_PRIORITY 90
#endif

/* SYS_INIT macro - creates an init entry collected by defines.c */
#define SYS_INIT(init_fn, init_level, init_prio) \
    const struct init_entry _bt_init_##init_fn \
        __attribute__((__used__)) = { \
        .init = init_fn, .level = init_level, .prio = init_prio, \
        .name = #init_fn, \
    }

/* SYS_INIT_NAMED - same with explicit name */
#define SYS_INIT_NAMED(name, init_fn, init_level, init_prio) \
    const struct init_entry _bt_init_##name \
        __attribute__((__used__)) = { \
        .init = init_fn, .level = init_level, .prio = init_prio, \
        .name = #name, \
    }

int bt_sys_init_run(void);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_INIT_H_ */
