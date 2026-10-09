/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_INIT_H_
#define ZEPHYR_INCLUDE_INIT_H_

#include "sys/iterable_sections.h"

/* Init levels */
#define PRE_KERNEL_1  0
#define PRE_KERNEL_2  1
#define POST_KERNEL   2
#define APPLICATION   3
#define SMP_BOOT      4

struct init_entry {
    int (*init)(void);
    int level;
    const char *name;
};

/* SYS_INIT macro - creates an init entry in a section */
#define SYS_INIT(init_fn, init_level, prio) \
    static const struct init_entry _init_##init_fn \
        __attribute__((__used__)) \
        __attribute__((__section__(".init_entries"))) = { \
        .init = init_fn, .level = init_level, \
    }

/* SYS_INIT_NAMED - same with explicit name */
#define SYS_INIT_NAMED(name, init_fn, init_level, prio) \
    static const struct init_entry _init_##name \
        __attribute__((__used__)) \
        __attribute__((__section__(".init_entries"))) = { \
        .init = init_fn, .level = init_level, \
    }

#endif /* ZEPHYR_INCLUDE_INIT_H_ */
