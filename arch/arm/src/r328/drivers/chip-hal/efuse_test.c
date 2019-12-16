#include <stdio.h>
#include <stdlib.h>
#include <efuse_i.h>
#include <hal_efuse.h>
#include <ctype.h>

int cmd_get_sbit(int argc, char ** argv)
{
	int ret = hal_get_security_mode();

	printf("secure enable bit: %d\n", ret);

	return 0;
}

int cmd_set_sbit(int argc, char ** argv)
{
	int ret = hal_set_security_mode();

	if (ret)
		printf("set secure enable bit failed: %d\n", ret);
	else
		printf("set secure enable bit success!\n");

	return 0;
}

int cmd_efuse_read(int argc, char ** argv)
{
	unsigned char *key_data = NULL;
	efuse_key_map_new_t *key_map = NULL;
	int read_size = 0;
	int ret = 0;

	if (argc != 2) {
		printf("Argument Error!\n");
		printf("Usage: efuse_read <key_name>\n");
		return -1;
	}

	key_map = efuse_search_key_by_name(argv[1]);
	if (key_map->size == 0) {
		printf("unknow key name: %s\n", argv[1]);
		return -1;
	}

	key_data = malloc(key_map->size >> 3);
	if (!key_data) {
		printf("malloc %d bytes error!\n", key_map->size >> 3);
		return -1;
	}

	read_size = hal_efuse_read(key_map->name, key_data, key_map->size);

	if (key_map->size != (read_size << 3)) {
		printf("read len %d error, should be %d\n!", read_size, key_map->size);
		ret = -1;
	}

	free(key_data);
	return ret;
}

static int hexstr_to_byte(const char* source, uint8_t* dest, int sourceLen)
{
	uint32_t i;
	uint8_t highByte, lowByte;

	for (i = 0; i < sourceLen; i += 2) {
		highByte = toupper(source[i]);
		lowByte  = toupper(source[i + 1]);

		if (highByte < '0' || (highByte > '9' && highByte < 'A' ) || highByte > 'F') {
			printf("input buf[%d] is %c, not in 0123456789ABCDEF\n", i, source[i]);
			return -1;
		}

		if (lowByte < '0' || (lowByte > '9' && lowByte < 'A' ) || lowByte > 'F') {
			printf("input buf[%d] is %c, not in 0123456789ABCDEF\n", i+1, source[i+1]);
			return -1;
		}

		if (highByte > 0x39)
			highByte -= 0x37;
		else
			highByte -= 0x30;


		if (lowByte > 0x39)
			lowByte -= 0x37;
		else
			lowByte -= 0x30;

		dest[i / 2] = (highByte << 4) | lowByte;
	}
	return 0;
}

int cmd_efuse_write(int argc, char ** argv)
{
	char *key_name = NULL;
	char *key_hex_string = NULL;
	uint32_t buf_len = 0;
	char *key_data = NULL;
	int ret = 0;

	if (argc != 3) {
		printf("Argument Error!\n");
		printf("Usage: efuse_write <key_name> <key_hex_string>\n");
		return -1;
	}

	key_name = argv[1];
	key_hex_string = argv[2];
	buf_len = strlen(argv[2]);
	printf("key_hex_string: %s, buf_len: %d\n", key_hex_string, buf_len);
	if ( buf_len % 2 != 0) {
		printf("key_hex_string len: %d is error!\n", buf_len);
	}

	key_data = malloc(buf_len / 2);
	if (!key_data) {
		printf("malloc %d bytes error!", buf_len / 2);
		return -1;
	}

	hexstr_to_byte(key_hex_string, key_data, buf_len);

	ret = hal_efuse_write(key_name, key_data, (buf_len / 2) << 3);
	if (ret)
		printf("efuse write error: %d\n", ret);

	free(key_data);
	return ret;
}
