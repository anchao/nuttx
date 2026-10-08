/* SPDX-License-Identifier: Apache-2.0 */

#ifndef ZEPHYR_INCLUDE_TOOLCHAIN_H_
#define ZEPHYR_INCLUDE_TOOLCHAIN_H_

#include <nuttx/compiler.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef __has_builtin
#define HAS_BUILTIN(x) __has_builtin(x)
#else
#define HAS_BUILTIN(x) HAS_BUILTIN_##x
#endif

#if __GNUC__ >= 4
#define HAS_BUILTIN___builtin_bswap16 1
#define HAS_BUILTIN___builtin_bswap32 1
#define HAS_BUILTIN___builtin_bswap64 1
#endif

#define TOOLCHAIN_WARNING_UNALIGNED_ACCESS
#define TOOLCHAIN_DISABLE_CLANG_WARNING(warning)
#define TOOLCHAIN_ENABLE_CLANG_WARNING(warning)

#define Z_STRINGIFY(x) #x
#define STRINGIFY(s) Z_STRINGIFY(s)

#define _DO_CONCAT(x, y) x ## y
#define _CONCAT(x, y) _DO_CONCAT(x, y)

#if defined(__cplusplus) && (__cplusplus >= 201103L)
#define BUILD_ASSERT(EXPR, MSG...) static_assert(EXPR, "" MSG)
#else
#define BUILD_ASSERT(EXPR, MSG...) _Static_assert((EXPR), "" MSG)
#endif

#ifndef FLEXIBLE_ARRAY_DECLARE
#define FLEXIBLE_ARRAY_DECLARE(type, name, ...) type name[0]
#endif

#define Z_DECL_ALIGN(type) __aligned(__alignof(type)) type

#define CODE_UNREACHABLE compiler_assume(false)

#define UNALIGNED_GET(g) \
__extension__ ({ \
	struct __packed { \
		__typeof__(*(g)) __v; \
	} *__g = (__typeof__(__g))(g); \
	__g->__v; \
})
#define ___in_section(a, b, c)
#define __in_section(a, b, c) ___in_section(a, b, c)

#ifndef __noinit
#define __noinit
#endif

#ifndef __fallthrough
#if __GNUC__ >= 7
#define __fallthrough __attribute__((fallthrough))
#else
#define __fallthrough
#endif
#endif

#ifndef __packed
#define __packed end_packed_struct
#endif

#ifndef __aligned
#define __aligned(x) aligned_data(x)
#endif

#ifndef __printf_like
#define __printf_like(f, a) printf_like(f, a)
#endif

#define __used used_code
#define __unused unused_code
#define __maybe_unused unused_code

#ifndef __deprecated
#define __deprecated deprecated_function
#endif

#ifndef __must_check
#define __must_check __attribute__((warn_unused_result))
#endif

#define ARG_UNUSED(x) (void)(x)
#define unlikely(x) (predict_false((bool)!!(x)) != 0L)
#define POPCOUNT(x) __builtin_popcount(x)

#ifndef __weak
#define __weak weak_function
#endif

#define __WARN(msg) __WARN1(GCC warning msg)
#define __WARN1(s) _Pragma(#s)

#ifndef __DEPRECATED_MACRO
#define __DEPRECATED_MACRO __WARN("Macro is deprecated")
#endif

#if defined(CONFIG_ASAN)
#define __noasan nosanitize_address
#else
#define __noasan
#endif

#endif /* ZEPHYR_INCLUDE_TOOLCHAIN_H_ */
