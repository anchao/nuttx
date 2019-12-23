#include <nuttx/config.h>
#include <stdio.h>

#if 0
/* components/aw/sound/aw-alsa-utils/amixer.c */
__attribute__((weak)) int cmd_mixer(int argc, char *argv[])
{
	printf("[%s] line:%d \n", __func__, __LINE__);
	return 0;
}
#else
extern int cmd_amixer(int argc, char ** argv);
#endif

int main(int argc, FAR char *argv[])
{
	return cmd_amixer(argc, argv);
}
