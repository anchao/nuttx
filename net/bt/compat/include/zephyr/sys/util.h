/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_UTIL_H_
#define ZEPHYR_INCLUDE_SYS_UTIL_H_

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IS_ENABLED trick */
#define Z_IS_ENABLED1(config_macro) Z_IS_ENABLED2(config_macro)
#define Z_IS_ENABLED2(config_macro) Z_IS_ENABLED3(config_macro 1, 0)
#define Z_IS_ENABLED3(ignore, val, ...) val
#define IS_ENABLED(config_macro) Z_IS_ENABLED1(config_macro)

/* Conditional code selection */
#define COND_CODE_1(_flag, _if_1_code, _else_code) \
    __COND_CODE(_flag, _if_1_code, _else_code)
#define __COND_CODE(_flag, _if_1_code, _else_code) \
    ___COND_CODE(Z_IS_ENABLED1(_flag), _if_1_code, _else_code)
#define ___COND_CODE(_flag, _if_1_code, _else_code) \
    ____COND_CODE(UTIL_CAT(_CC1_, _flag), _if_1_code, _else_code)
#define ____COND_CODE(selector, _if_1_code, _else_code) \
    selector(_if_1_code, _else_code)
#define Z_DEBRACKET(...) __VA_ARGS__
#define _CC1_1(_if_1_code, _else_code) Z_DEBRACKET _if_1_code
#define _CC1_0(_if_1_code, _else_code) Z_DEBRACKET _else_code

/* IF_ENABLED - compile code only when config is enabled */
#define IF_ENABLED(_cfg, _code) COND_CODE_1(_cfg, _code, ())

/* Arithmetic - using simple ternary for constant-foldability */
/* Force override toolchain/gcc.h statement-expression version */
#ifdef Z_MAX
#undef Z_MAX
#endif
#ifndef Z_MAX
#define Z_MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifdef Z_MIN
#undef Z_MIN
#endif
#ifndef Z_MIN
#define Z_MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifdef Z_CLAMP
#undef Z_CLAMP
#endif
#ifndef Z_CLAMP
#define Z_CLAMP(val, low, high) Z_MIN(Z_MAX(val, low), high)
#endif

#ifndef CLAMP
#define CLAMP(val, low, high) Z_CLAMP(val, low, high)
#endif

/* MIN/MAX convenience */
#ifndef MAX
#define MAX(a, b) Z_MAX(a, b)
#endif

#ifndef MIN
#define MIN(a, b) Z_MIN(a, b)
#endif

#define Z_POW2_CEIL(x) \
    ((x) <= 2UL ? (x) : (1UL << (8 * sizeof(long) - __builtin_clzl((x) - 1))))

#define Z_IS_POW2(x) (((x) != 0) && (((x) & ((x)-1)) == 0))

/* Container of */
#ifndef container_of
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member) container_of(ptr, type, member)
#endif

/* Array size */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

#define ARRAY_INDEX(array, ptr) ((ptr) - (array))

#define POINTER_TO_UINT(x) ((uintptr_t)(x))
#define UINT_TO_POINTER(x) ((void *)(uintptr_t)(x))

/* Round up/down */
#define ROUND_UP(x, align)   (((x) + ((align) - 1)) & ~((align) - 1))
#define ROUND_DOWN(x, align) ((x) & ~((align) - 1))
#define DIV_ROUND_UP(n, d)   (((n) + (d) - 1) / (d))

/* Inclusive range check */
#ifndef IN_RANGE
#define IN_RANGE(val, min, max) (((val) >= (min)) && ((val) <= (max)))
#endif

/* Part of array */
#define PART_OF_ARRAY(array, ptr) \
    ((uintptr_t)(ptr) >= (uintptr_t)(array) && \
     (uintptr_t)(ptr) < (uintptr_t)(&(array)[ARRAY_SIZE(array)]))

#define IS_ARRAY_ELEMENT(array, ptr) PART_OF_ARRAY(array, ptr)

/* Field size */
#define MEMBER_SIZE(type, member) sizeof(((type *)0)->member)

/* Number of VA args */
#define NUM_VA_ARGS_LESS_1(...) \
    NUM_VA_ARGS_LESS_1_IMPL(__VA_ARGS__, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, ~)

#define NUM_VA_ARGS_LESS_1_IMPL(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, _62, _63, N, ...) N

/* Listify - expand macro F(idx, ...) LEN times. */
#define LISTIFY(LEN, F, SEP, ...) UTIL_CAT(LISTIFY_, LEN)(F, SEP, __VA_ARGS__)
#define LISTIFY_0(F, SEP, ...)
#define LISTIFY_1(F, SEP, ...)  F(0, __VA_ARGS__)
#define LISTIFY_2(F, SEP, ...)  LISTIFY_1(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(1, __VA_ARGS__)
#define LISTIFY_3(F, SEP, ...)  LISTIFY_2(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(2, __VA_ARGS__)
#define LISTIFY_4(F, SEP, ...)  LISTIFY_3(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(3, __VA_ARGS__)
#define LISTIFY_5(F, SEP, ...)  LISTIFY_4(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(4, __VA_ARGS__)
#define LISTIFY_6(F, SEP, ...)  LISTIFY_5(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(5, __VA_ARGS__)
#define LISTIFY_7(F, SEP, ...)  LISTIFY_6(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(6, __VA_ARGS__)
#define LISTIFY_8(F, SEP, ...)  LISTIFY_7(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(7, __VA_ARGS__)
#define LISTIFY_9(F, SEP, ...)  LISTIFY_8(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(8, __VA_ARGS__)
#define LISTIFY_10(F, SEP, ...) LISTIFY_9(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(9, __VA_ARGS__)
#define LISTIFY_11(F, SEP, ...) LISTIFY_10(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(10, __VA_ARGS__)
#define LISTIFY_12(F, SEP, ...) LISTIFY_11(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(11, __VA_ARGS__)
#define LISTIFY_13(F, SEP, ...) LISTIFY_12(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(12, __VA_ARGS__)
#define LISTIFY_14(F, SEP, ...) LISTIFY_13(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(13, __VA_ARGS__)
#define LISTIFY_15(F, SEP, ...) LISTIFY_14(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(14, __VA_ARGS__)
#define LISTIFY_16(F, SEP, ...) LISTIFY_15(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(15, __VA_ARGS__)
#define LISTIFY_17(F, SEP, ...) LISTIFY_16(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(16, __VA_ARGS__)
#define LISTIFY_18(F, SEP, ...) LISTIFY_17(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(17, __VA_ARGS__)
#define LISTIFY_19(F, SEP, ...) LISTIFY_18(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(18, __VA_ARGS__)
#define LISTIFY_20(F, SEP, ...) LISTIFY_19(F, SEP, __VA_ARGS__) Z_DEBRACKET SEP F(19, __VA_ARGS__)

#define REVERSE_ARGS(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p) \
    p, o, n, m, l, k, j, i, h, g, f, e, d, c, b, a

#define UTIL_CAT(a, b) UTIL_PRIMITIVE_CAT(a, b)
#define UTIL_PRIMITIVE_CAT(a, b) a##b

/* POINTER_BYTE - get byte from pointer storage */
#define POINTER_BYTE(buf, byte) ((uint8_t *)(buf))[(byte)]

/* For each macro */
#define Z_FOR_EACH_IDX(F, ...) \
    Z_FOR_EACH_IDX_EXEC(0, F, __VA_ARGS__)
#define Z_FOR_EACH_IDX_EXEC(idx, F, ...) \
    COND_CODE_1(NUM_VA_ARGS_LESS_1(__VA_ARGS__), \
        (F(idx, _GET_ARG_1(__VA_ARGS__)) \
         Z_FOR_EACH_IDX_EXEC(idx + 1, F, _GET_REST_ARGS(__VA_ARGS__))), ())
#define _GET_ARG_1(a1, ...) a1
#define _GET_REST_ARGS(a1, ...) __VA_ARGS__

/* Array iteration macros */
#define ARRAY_FOR_EACH(array, idx) \
    for (size_t idx = 0; (idx) < ARRAY_SIZE(array); (idx)++)

#define ARRAY_FOR_EACH_PTR(array, ptr) \
    for (__typeof__(*(array)) *ptr = (array); \
         ptr < &(array)[ARRAY_SIZE(array)]; ptr++)

/* KB/MB/GB helpers */
#define KB(x) ((x) << 10)
#define MB(x) ((x) << 20)
#define GB(x) ((x) << 30)

/* BIT macro */
#ifndef BIT
#define BIT(n) (1UL << (n))
#endif

/* BIT_MASK */
#define BIT_MASK(n) (BIT(n) - 1)

#ifndef GENMASK
#define GENMASK(h, l) \
    (((~0UL) >> ((sizeof(unsigned long) * 8) - 1 - MAX(h, l))) & \
     (~0UL << MIN(h, l)))
#endif

#ifndef FIELD_GET
#define FIELD_GET(mask, value) (((value) & (mask)) >> __builtin_ctzl(mask))
#endif

#ifndef FIELD_PREP
#define FIELD_PREP(mask, value) (((value) << __builtin_ctzl(mask)) & (mask))
#endif

#ifndef WRITE_BIT
#define WRITE_BIT(var, bit, set) \
    do { \
        if (set) { \
            (var) |= BIT(bit); \
        } else { \
            (var) &= ~BIT(bit); \
        } \
    } while (0)
#endif

/* WAIT_FOR */
#define WAIT_FOR(expr, timeout, delay_stmt) \
    ({ \
        int _ret = 0; \
        unsigned int _cnt = 0; \
        for (_cnt = 0; _cnt < (timeout); _cnt++) { \
            if (expr) break; \
            delay_stmt; \
        } \
        if (_cnt >= (timeout)) _ret = -1; \
        _ret; \
    })

/**
 * @brief Convert a hex string to binary data.
 *
 * @param hex    Hex string to convert
 * @param hexlen Length of hex string
 * @param buf    Output buffer
 * @param buflen Size of output buffer
 * @return Number of bytes written to buf, or 0 on error
 */
static inline size_t hex2bin(const char *hex, size_t hexlen,
			     uint8_t *buf, size_t buflen)
{
	size_t i;

	if ((hexlen % 2) != 0 || (hexlen / 2) > buflen) {
		return 0;
	}

	for (i = 0; i < hexlen / 2; i++) {
		char byte[3] = { hex[i * 2], hex[i * 2 + 1], '\0' };
		unsigned int val;

		if (sscanf(byte, "%02x", &val) != 1) {
			return 0;
		}

		buf[i] = (uint8_t)val;
	}

	return hexlen / 2;
}

static inline bool util_memeq(const void *a, const void *b, size_t len)
{
	return memcmp(a, b, len) == 0;
}

static inline bool util_eq(const void *a, size_t a_len,
                           const void *b, size_t b_len)
{
    return a_len == b_len && memcmp(a, b, a_len) == 0;
}

static inline void mem_xor_n(uint8_t *dst, const uint8_t *src1,
                             const uint8_t *src2, size_t len)
{
    size_t i;

    for (i = 0; i < len; i++)
        {
            dst[i] = src1[i] ^ src2[i];
        }
}

static inline void mem_xor_128(uint8_t dst[16], const uint8_t src1[16],
                               const uint8_t src2[16])
{
    mem_xor_n(dst, src1, src2, 16);
}

static inline size_t sys_count_bits(const void *mem, size_t size)
{
    const uint8_t *bytes = mem;
    size_t count = 0;
    size_t i;

    for (i = 0; i < size; i++)
        {
            count += __builtin_popcount(bytes[i]);
        }

    return count;
}

/* u8_to_dec - convert uint8_t to decimal string */
static inline uint8_t u8_to_dec(char *buf, uint8_t buflen, uint8_t val)
{
    uint8_t pos = 0;
    if (val >= 100 && pos < buflen) buf[pos++] = '0' + val / 100;
    if (val >= 10 && pos < buflen) buf[pos++] = '0' + (val / 10) % 10;
    if (pos < buflen) buf[pos++] = '0' + val % 10;
    if (pos < buflen) buf[pos] = '\0';
    return pos;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_SYS_UTIL_H_ */
