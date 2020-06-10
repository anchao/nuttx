#include <nuttx/config.h>
#include <stdio.h>
#if 0
/* components/aw/sound/aw-alsa-utils/aplay.c */
__attribute__((weak)) int cmd_aplay(int argc, char *argv[])
{
	printf("[%s] line:%d \n", __func__, __LINE__);
	return 0;
}
#else
extern int cmd_rtplayer_test(int argc, char ** argv);
#endif

int main(int argc, FAR char *argv[])
{
	return cmd_rtplayer_test(argc, argv);
}
