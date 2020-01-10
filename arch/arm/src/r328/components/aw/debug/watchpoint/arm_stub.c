#include "hardware_wbp.h"
#include "gdb_stub.h"
#include <stdio.h>

int arm_arch_set_hw_watchpoint(enum gdb_bptype type, int i, unsigned long addr)
{
    return arm_install_hw_watchpoint(type, i, addr);
}

int arm_arch_remove_hw_watchpoint(enum gdb_bptype type, int i, unsigned long addr)
{
    arm_uninstall_hw_watchpoint(i);
    return 0;
}

struct gdb_arch arch_gdb_ops =
{
    .set_hw_watchpoint = arm_arch_set_hw_watchpoint,
    .remove_hw_watchpoint = arm_arch_remove_hw_watchpoint,
};

int kgdb_arch_init(void)
{
    int ret = -1;
    static int kgdb_arch_init_flag = 0;
    unsigned int brp = 0;
    unsigned int wrp = 0;

    if (kgdb_arch_init_flag > 0)
    {
        return 0;
    }

    if (!monitor_mode_enabled())
    {
        ret = enable_monitor_mode();
        if (ret)
        {
            printf("enter monitor mode failed!\n");
            return -1;
        }
    }

    brp = get_num_brp_resources();
    wrp = get_num_wrp_resources();

    if (create_hw_break_watch(brp, wrp))
    {
        return -1;
    }

    kgdb_arch_init_flag = 1;
    return ret;
}
