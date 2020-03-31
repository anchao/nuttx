#ifndef AW_DEBUG_H
#define AW_DEBUG_H

#define BREAK_INSTR_SIZE 4

enum gdb_bptype
{
    BP_BREAKPOINT = 0,
    BP_HARDWARE_BREAKPOINT,
    BP_WRITE_WATCHPOINT,
    BP_READ_WATCHPOINT,
    BP_ACCESS_WATCHPOINT,
    BP_POKE_BREAKPOINT,
};

enum gdb_bpstate
{
    BP_UNDEFINED = 0,
    BP_REMOVED,
    BP_SET,
    BP_ACTIVE
};

struct gdb_bkpt
{
    unsigned long       bpt_addr;
    unsigned char       saved_instr[BREAK_INSTR_SIZE];
    enum gdb_bptype type;
    enum gdb_bpstate    state;
};

struct gdb_arch
{
    unsigned char gdb_bpt_instr[BREAK_INSTR_SIZE];
    unsigned long flags;

    int (*set_sw_breakpoint)(unsigned long addr, char *saved_instr);
    int (*remove_sw_breakpoint)(unsigned long addr, char *bundle);
    int (*set_hw_breakpoint)(int, unsigned long);
    int (*remove_hw_breakpoint)(int, unsigned long);
    int (*set_hw_watchpoint)(enum gdb_bptype, int, unsigned long);
    int (*remove_hw_watchpoint)(enum gdb_bptype, int, unsigned long);
};

extern struct gdb_arch arch_gdb_ops;

int gdb_set_hw_break(unsigned long addr);
int gdb_remove_hw_break(unsigned long addr);

int gdb_set_hw_watch(unsigned long addr, enum gdb_bptype type);
int gdb_remove_hw_watch(unsigned long addr);
int gdb_isremoved_hw_watch(unsigned long addr);
int remove_all_break_watch_points(void);
int debug_watchpoint_init(void);
void debug_dump_all_breaks_info(void);

#endif  /*AW_DEBUG_H*/
