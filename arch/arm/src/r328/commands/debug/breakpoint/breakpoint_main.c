#include <nuttx/config.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <aw_breakpoint.h>

static void show_help(void)
{
    debug_dump_all_breaks_info();
    printf("Usage: breakpoint [set | remove] addr\n");
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
    if (*err != 0)
    {
        printf("addr error\n");
        return -1;
    }

    if (!strcmp(argv[1], "set"))
    {
        return gdb_set_hw_break(addr);
    }
    else if (!strcmp(argv[1], "remove"))
    {
        return gdb_remove_hw_break(addr);
    }
    else
    {
        show_help();
    }
    return -1;
}
