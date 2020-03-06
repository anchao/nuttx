#include <nuttx/config.h>
#include <stdio.h>


extern int cmd_adbforward(int argc, char *argv[]);
void af_test(void);
int main(int argc, FAR char *argv[])
{
	if (argc == 2 && !strcmp(argv[1], "-t"))
		af_test();
	else
		cmd_adbforward(argc, argv);
	return 0;
}
