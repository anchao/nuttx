/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_SYS_UTIL_UTF8_H_
#define ZEPHYR_INCLUDE_SYS_UTIL_UTF8_H_

#include <stddef.h>
#include <string.h>

static inline char *utf8_trunc(char *utf8_str)
{
	unsigned char *str = (unsigned char *)utf8_str;
	size_t len;
	size_t i;

	if (utf8_str == NULL) {
		return NULL;
	}

	len = strlen(utf8_str);
	if (len == 0) {
		return utf8_str;
	}

	i = len;
	while (i > 0 && (str[i - 1] & 0xc0) == 0x80) {
		i--;
	}

	if (i == len) {
		return utf8_str;
	}

	if (i == 0) {
		str[0] = '\0';
		return utf8_str;
	}

	if ((str[i - 1] & 0x80) == 0x00) {
		str[i] = '\0';
	} else {
		str[i - 1] = '\0';
	}

	return utf8_str;
}

static inline char *utf8_lcpy(char *dst, const char *src, size_t n)
{
	if (n == 0) {
		return dst;
	}

	strncpy(dst, src, n - 1);
	dst[n - 1] = '\0';
	return utf8_trunc(dst);
}

static inline int utf8_count_chars(const char *s)
{
	int count = 0;

	if (s == NULL) {
		return 0;
	}

	while (*s != '\0') {
		unsigned char c = (unsigned char)*s;

		if ((c & 0x80) == 0x00) {
			s += 1;
		} else if ((c & 0xe0) == 0xc0) {
			s += 2;
		} else if ((c & 0xf0) == 0xe0) {
			s += 3;
		} else if ((c & 0xf8) == 0xf0) {
			s += 4;
		} else {
			return -1;
		}

		count++;
	}

	return count;
}

#endif /* ZEPHYR_INCLUDE_SYS_UTIL_UTF8_H_ */
