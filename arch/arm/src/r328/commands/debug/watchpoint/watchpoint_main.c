#include <nuttx/config.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <aw_debug.h>

static void show_help(void)
{
    debug_dump_all_breaks_info();
    printf("Usage: watchpoint [write | read | access | remove] addr\n");
}

int main(int argc, FAR char *argv[])
{
    unsigned long addr = 0;
    char *err = NULL;

    if (argc < 3)
    {
        show_help();
        return -1;
    }

    addr = strtoul(argv[2], &err, 0);
    if (*err != NULL)
    {
        printf("addr error\n");
        return -1;
    }

    if (!strcmp(argv[1], "write"))
    {
        return gdb_set_hw_watch(addr, BP_WRITE_WATCHPOINT);
    }
    else if (!strcmp(argv[1], "read"))
    {
        return gdb_set_hw_watch(addr, BP_READ_WATCHPOINT);
    }
    else if (!strcmp(argv[1], "access"))
    {
        return gdb_set_hw_watch(addr, BP_ACCESS_WATCHPOINT);
    }
    else if (!strcmp(argv[1], "remove"))
    {
        return gdb_remove_hw_watch(addr);
    }
    else
    {
        show_help();
    }
    return -1;
}
