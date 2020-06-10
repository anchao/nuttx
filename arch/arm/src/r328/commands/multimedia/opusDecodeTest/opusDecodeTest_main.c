#include <nuttx/config.h>
#include <stdio.h>

extern int cmd_opus_decode_test(int argc, char ** argv);

int main(int argc, FAR char *argv[])
{
	return cmd_opus_decode_test(argc, argv);
}
