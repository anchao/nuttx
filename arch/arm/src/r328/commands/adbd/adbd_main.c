#include <nuttx/config.h>
#include <stdio.h>


extern int cmd_adbd(int argc, char *argv[]);
int main(int argc, FAR char *argv[])
{
	cmd_adbd(argc, argv);
	return 0;
}
