#include <stdio.h>
#include <string.h>
#include <adb_forward.h>

void af_test(void)
{
	char *text = "String for test...\n";
	int ret, count = 10;
	int port = 20195;
#if 0
	ret = adb_forward_create(port);
#else
	ret = adb_forward_create_with_rawdata(port);
#endif
	if (ret != 0)
		return;

	while (count--) {
		ret = adb_forward_send(port, (void *)text, strlen(text));
		if (ret != 0) {
			printf("adb_forward_send failed\n");
			break;
		}
	}
	adb_forward_destroy(20195);
	return;
}
