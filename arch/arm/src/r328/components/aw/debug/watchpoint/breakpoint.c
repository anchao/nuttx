#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gdb_stub.h"

/* static int gdb_hw_break_max; */
static int gdb_hw_watch_max;

static struct gdb_bkpt *gdb_hw_break = NULL;
static struct gdb_bkpt *gdb_hw_watch = NULL;

static int init_flag = 0;

extern int kgdb_arch_init(void);

struct bp_type_state_match
{
    int type;
    char *str;
};

static struct bp_type_state_match bp_type_match [] =
{
    {BP_BREAKPOINT, "breakpoint"},
    {BP_HARDWARE_BREAKPOINT, "hardware breakpoint"},
    {BP_WRITE_WATCHPOINT, "write watchpoint"},
    {BP_READ_WATCHPOINT, "read watchpoint"},
    {BP_ACCESS_WATCHPOINT, "access watchpoint"},
    {BP_POKE_BREAKPOINT, "poke breakpoint"},
};

static struct bp_type_state_match bp_state_match[] =
{
    {BP_UNDEFINED, "undefined"},
    {BP_REMOVED, "removed"},
    {BP_SET, "set"},
    {BP_ACTIVE, "active"},
};

int create_hw_break_watch(unsigned int hw_break, unsigned int hw_watch)
{
    int i;

    (void)hw_break;

    if (hw_watch)
    {
        gdb_hw_watch = calloc(hw_break, sizeof(struct gdb_bkpt));
        if (gdb_hw_watch)
        {
            gdb_hw_watch_max = hw_watch;
            struct gdb_bkpt *bkpt = gdb_hw_watch;
            for (i = 0; i < hw_break; i++)
            {
                bkpt->state = BP_UNDEFINED;
                bkpt++;
            }
        }
        else
        {
            free(gdb_hw_break);
            gdb_hw_break = NULL;
            return -1;
        }
    }
    return 0;
}

int destory_hw_break_watch(void)
{
    if (gdb_hw_watch)
    {
        free(gdb_hw_watch);
        gdb_hw_watch = NULL;
    }
    return 0;
}

static int gdb_arch_set_hw_watchpoint(enum gdb_bptype type, unsigned int no, unsigned long addr)
{
    if (arch_gdb_ops.set_hw_watchpoint)
    {
        return arch_gdb_ops.set_hw_watchpoint(type, no, addr);
    }
    return -1;
}

static int gdb_arch_remove_hw_watchpoint(enum gdb_bptype type, unsigned int no, unsigned long addr)
{
    if (arch_gdb_ops.set_hw_watchpoint)
    {
        return arch_gdb_ops.set_hw_watchpoint(type, no, addr);
    }
    return -1;
}

/*
 * HW watchpoint management:
 */
int gdb_set_hw_watch(unsigned long addr, enum gdb_bptype type)
{
    int i;
    int ret = -1;
    int breakno = -1;

    if (init_flag == 0)
    {
        if (debug_watchpoint_init())
        {
            return -1;
        }
        init_flag = 1;
    }

    for (i = 0; i < gdb_hw_watch_max; i++)
    {
        if ((gdb_hw_watch[i].state == BP_ACTIVE) &&
            (gdb_hw_watch[i].bpt_addr == addr))
        {
            return 0;
        }
    }
    for (i = 0; i < gdb_hw_watch_max; i++)
    {
        if (gdb_hw_watch[i].state == BP_REMOVED)
        {
            breakno = i;
            break;
        }
    }

    if (breakno == -1)
    {
        for (i = 0; i < gdb_hw_watch_max; i++)
        {
            if (gdb_hw_watch[i].state == BP_UNDEFINED)
            {
                breakno = i;
                break;
            }
        }
    }

    if (breakno == -1)
    {
        return -1;
    }

    gdb_hw_watch[breakno].state = BP_ACTIVE;
    gdb_hw_watch[breakno].type = type;
    gdb_hw_watch[breakno].bpt_addr = addr;

    ret = gdb_arch_set_hw_watchpoint(gdb_hw_watch[breakno].type, breakno, gdb_hw_watch[breakno].bpt_addr);
    if (ret)
    {
        printf("set hw watchpoint 0x%08x failed!\n", gdb_hw_watch[breakno].bpt_addr);
    }
    return ret;
}

int gdb_remove_hw_watch(unsigned long addr)
{
    int i;

    for (i = 0; i < gdb_hw_watch_max; i++)
    {
        if ((gdb_hw_watch[i].state == BP_ACTIVE) &&
            (gdb_hw_break[i].bpt_addr == addr))
        {
            if (gdb_arch_remove_hw_watchpoint(gdb_hw_watch[i].type, i, addr))
            {
                printf("remove hw watchpoint 0x%08x failed!\n", addr);
                return -1;
            }
            gdb_hw_watch[i].state = BP_REMOVED;
            return 0;
        }
    }
    return -1;
}

int gdb_isremoved_hw_watch(unsigned long addr)
{
    int i;

    for (i = 0; i < gdb_hw_watch_max; i++)
    {
        if ((gdb_hw_watch[i].state == BP_REMOVED) &&
            (gdb_hw_watch[i].bpt_addr == addr))
        {
            return 1;
        }
    }
    return 0;
}

int remove_all_break_watch_points(void)
{
    unsigned long addr;
    int error = 0;
    int i;

    for (i = 0; i < gdb_hw_watch_max; i++)
    {
        if (gdb_hw_watch[i].state != BP_ACTIVE)
        {
            goto hw_watch_setundefined;
        }
        addr = gdb_hw_watch[i].bpt_addr;
        error = gdb_arch_remove_hw_watchpoint(gdb_hw_watch[i].type, i, addr);
        if (error)
        {
            printf("GDB: breakpoint remove failed: %lx\n", addr);
        }
hw_watch_setundefined:
        gdb_hw_watch[i].state = BP_UNDEFINED;
    }

    return 0;
}


static char *get_bp_type_str(enum gdb_bptype type)
{
    if (type < sizeof(bp_type_match) / sizeof(bp_type_match[0]))
    {
        return bp_type_match[type].str;
    }
    return NULL;
}

static char *get_bp_state_str(enum gdb_bpstate state)
{
    if (state < sizeof(bp_state_match) / sizeof(bp_state_match[0]))
    {
        return bp_state_match[state].str;
    }
    return NULL;
}

void debug_dump_all_breaks_info(void)
{
    int i;

    if (init_flag == 0)
    {
        printf("watchpoint debug not init!\n");
        return;
    }

    printf("watchpoint num = %d:\n", gdb_hw_watch_max);
    printf("Id    Addr    State      Type\n");

    for (i = 0; i < gdb_hw_watch_max; i++)
    {
        printf("%d  0x%08x  %s      %s\n", i,
               gdb_hw_watch[i].bpt_addr,
               get_bp_state_str(gdb_hw_watch[i].state),
               get_bp_type_str(gdb_hw_watch[i].type));
    }
}

int debug_watchpoint_init(void)
{
    if (!kgdb_arch_init())
    {
        init_flag = 1;
        return 0;
    }
    return -1;
}
