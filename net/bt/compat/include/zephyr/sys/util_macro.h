/* SPDX-License-Identifier: Apache-2.0 */
#ifndef ZEPHYR_INCLUDE_SYS_UTIL_MACRO_H_
#define ZEPHYR_INCLUDE_SYS_UTIL_MACRO_H_

/* Parts of Zephyr's util_macro that aren't in sys/util.h */

#define UTIL_INC(v) UTIL_PRIMITIVE_CAT(UTIL_INC_, v)
#define UTIL_INC_0 1
#define UTIL_INC_1 2
#define UTIL_INC_2 3
#define UTIL_INC_3 4
#define UTIL_INC_4 5
#define UTIL_INC_5 6
#define UTIL_INC_6 7
#define UTIL_INC_7 8
#define UTIL_INC_8 9

#define UTIL_DEC(v) UTIL_PRIMITIVE_CAT(UTIL_DEC_, v)
#define UTIL_DEC_1 0
#define UTIL_DEC_2 1
#define UTIL_DEC_3 2
#define UTIL_DEC_4 3
#define UTIL_DEC_5 4
#define UTIL_DEC_6 5
#define UTIL_DEC_7 6
#define UTIL_DEC_8 7
#define UTIL_DEC_9 8

#define UTIL_X2(v) UTIL_PRIMITIVE_CAT(UTIL_X2_, v)
#define UTIL_X2_0 0
#define UTIL_X2_1 2
#define UTIL_X2_2 4
#define UTIL_X2_3 6
#define UTIL_X2_4 8
#define UTIL_X2_5 10
#define UTIL_X2_6 12
#define UTIL_X2_7 14
#define UTIL_X2_8 16
#define UTIL_X2_9 18

#endif /* ZEPHYR_INCLUDE_SYS_UTIL_MACRO_H_ */