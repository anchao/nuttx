#include <nuttx/config.h>
#include <stdio.h>
//#include "../../components/aw/bluetooth/bluedroid/osi/include/osi/xr829_bt.h"
extern void cmd_bt_init(void);
extern int bt_example(void);

int main(int argc, FAR char *argv[])
{
	cmd_bt_init();
	bt_example();
	return 0;
}

