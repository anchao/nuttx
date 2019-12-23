#include <nuttx/config.h>
#include <stdio.h>

extern int cmd_soundcard(int argc, char *argv[]);
int main(int argc, FAR char *argv[])
{
	return cmd_soundcard(argc, argv);
}
