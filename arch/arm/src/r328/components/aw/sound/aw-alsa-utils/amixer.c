/*
* Copyright (c) 2019-2025 Allwinner Technology Co., Ltd. ALL rights reserved.
*
* Allwinner is a trademark of Allwinner Technology Co.,Ltd., registered in
* the the people's Republic of China and other countries.
* All Allwinner Technology Co.,Ltd. trademarks are used with permission.
*
* DISCLAIMER
* THIRD PARTY LICENCES MAY BE REQUIRED TO IMPLEMENT THE SOLUTION/PRODUCT.
* IF YOU NEED TO INTEGRATE THIRD PARTY’S TECHNOLOGY (SONY, DTS, DOLBY, AVS OR MPEGLA, ETC.)
* IN ALLWINNERS’SDK OR PRODUCTS, YOU SHALL BE SOLELY RESPONSIBLE TO OBTAIN
* ALL APPROPRIATELY REQUIRED THIRD PARTY LICENCES.
* ALLWINNER SHALL HAVE NO WARRANTY, INDEMNITY OR OTHER OBLIGATIONS WITH RESPECT TO MATTERS
* COVERED UNDER ANY REQUIRED THIRD PARTY LICENSE.
* YOU ARE SOLELY RESPONSIBLE FOR YOUR USAGE OF THIRD PARTY’S TECHNOLOGY.
*
*
* THIS SOFTWARE IS PROVIDED BY ALLWINNER"AS IS" AND TO THE MAXIMUM EXTENT
* PERMITTED BY LAW, ALLWINNER EXPRESSLY DISCLAIMS ALL WARRANTIES OF ANY KIND,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING WITHOUT LIMITATION REGARDING
* THE TITLE, NON-INFRINGEMENT, ACCURACY, CONDITION, COMPLETENESS, PERFORMANCE
* OR MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* IN NO EVENT SHALL ALLWINNER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS, OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <aw-alsa-lib/pcm.h>
#include <aw-alsa-lib/control.h>

#define DEFAULT_SOUNDCARD 	"audiocodec"
static char *g_card_name = DEFAULT_SOUNDCARD;

static int amixer_controls(void)
{
	int num, ret, i;
	snd_ctl_info_t info;

	num = snd_ctl_num(g_card_name);
	if (num <= 0)
		return -1;
	for (i = 0; i < num; i++) {
		memset(&info, 0, sizeof(snd_ctl_info_t));
		ret = snd_ctl_get_bynum(g_card_name, i, &info);
		if (ret < 0)
			continue;
		printf("numid=%u, name=\'%s\'\n", info.id, info.name);
		if (info.count > 1) {
			int j;
			for (j = 0; j < info.count; j++)
				printf("value[%d]=%lu ", j, info.private_data[i]);
			printf("min=%u, max=%u\n", info.min, info.max);
		} else {
			printf("value=%u, min=%u, max=%u\n",
					info.value, info.min, info.max);
		}
	}
	return 0;
}

static int parser_ctl_numid(const char *cmd, int *id)
{
	char *id_str = "numid=";

	if (strncmp(cmd, id_str, strlen(id_str)) != 0)
		return -1;
	*id = atoi(cmd+strlen(id_str));
	return 0;
}

static void amixer_ctl_info_print(snd_ctl_info_t *info)
{
	printf("numid=%u, name=\'%s\'\n", info->id, info->name);
	printf("value=%u, min=%u, max=%u\n",
			info->value, info->min, info->max);
}
static void amixer_usage(void)
{
	printf("Usage:amixer <options> command\n");
	printf("Available commands:\n");
	printf("-c,          select the card\n");
	printf("controls     show all controls\n");
	printf("cget id      get control content for one control\n");
	printf("cset id      set control contetn for one control\n");
}

static int amixer(int argc, char *argv[])
{
	int ret = 0, c;
	int numid = 0;
	snd_ctl_info_t info;
	g_card_name = DEFAULT_SOUNDCARD;

	while ((c = getopt(argc, argv, "hc:")) != -1) {
		switch (c) {
		case 'c':
			g_card_name = (char *)snd_card_name(atoi(optarg));
			if (!g_card_name) {
				printf("unknown card number\n");
				return -1;
			}
			break;
		case 'h':
		default:
			amixer_usage();
			return -1;
		}
	}

	if (optind >= argc)
		goto amixer_help;
	if (!strcmp(argv[optind], "controls")) {
		return amixer_controls();
	}

	if (argc - optind <= 1)
		goto amixer_help;

	if (parser_ctl_numid(argv[optind + 1], &numid) != 0)
		goto amixer_help;

	if (!strcmp(argv[optind], "cget")) {
		ret = snd_ctl_get_bynum(g_card_name, numid, &info);
		if (!ret) {
			amixer_ctl_info_print(&info);
			return 0;
		}
	} else if (!strcmp(argv[optind], "cset")) {
		int value = -1;
		if (argc - optind >= 3)
			value = atoi(argv[optind + 2]);
		if (value >= 0) {
			if (snd_ctl_get_bynum(g_card_name, numid, &info) != 0) {
				printf("snd_ctl_get failed\n");
				return -1;
			}
			snd_ctl_set_bynum(g_card_name, numid, value);
		}
		if (!snd_ctl_get_bynum(g_card_name, numid, &info)) {
			amixer_ctl_info_print(&info);
			return 0;
		}
	} else {
		goto amixer_help;
	}

	return ret;
amixer_help:
	amixer_usage();
	return -1;
}

int cmd_amixer(int argc, char ** argv)
{
	amixer(argc, argv);
	return 0;
}
//FINSH_FUNCTION_EXPORT_CMD(cmd_amixer, amixer, amixer utils);
