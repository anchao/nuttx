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

#if __GNUC__ >= 5
#define HAS_BUILTIN___builtin_add_overflow 1
#define HAS_BUILTIN___builtin_sub_overflow 1
#define HAS_BUILTIN___builtin_mul_overflow 1
#define HAS_BUILTIN___builtin_div_overflow 1
#endif

#if __GNUC__ >= 4
#define HAS_BUILTIN___builtin_clz 1
#define HAS_BUILTIN___builtin_clzl 1
#define HAS_BUILTIN___builtin_clzll 1
#define HAS_BUILTIN___builtin_ctz 1
#define HAS_BUILTIN___builtin_ctzl 1
#define HAS_BUILTIN___builtin_ctzll 1
#define HAS_BUILTIN___builtin_bswap16 1
#define HAS_BUILTIN___builtin_bswap32 1
#define HAS_BUILTIN___builtin_bswap64 1
#endif

#define TOOLCHAIN_WARNING_UNALIGNED_ACCESS
#define TOOLCHAIN_DISABLE_CLANG_WARNING(warning)
#define TOOLCHAIN_ENABLE_CLANG_WARNING(warning)

#ifdef __cplusplus
#define EXTERN_C extern "C"
#define TASK_ENTRY_CPP extern "C"
#define ZRESTRICT __restrict
#else
#define EXTERN_C extern
#define ZRESTRICT restrict
#endif

#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE always_inline_function
#endif

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

#define IS_PTR_ALIGNED_BYTES(ptr, bytes) ((((uintptr_t)(ptr)) % (bytes)) == 0)
#define IS_PTR_ALIGNED(ptr, type) IS_PTR_ALIGNED_BYTES(ptr, __alignof(type))

#define LINKER_KEEP(symbol) \
	static const void * const symbol##_ptr __used \
	__attribute__((__section__(".symbol_to_keep"))) = (void *)&symbol

#define ALIAS_OF(of) __attribute__((alias(#of)))
#define FUNC_ALIAS(real_func, new_alias, return_type) \
	return_type new_alias() ALIAS_OF(real_func)

#define CODE_UNREACHABLE compiler_assume(false)
#define FUNC_NORETURN noreturn_function

#define UNALIGNED_GET(g) \
__extension__ ({ \
	struct __attribute__((__packed__)) { \
		__typeof__(*(g)) __v; \
	} *__g = (__typeof__(__g))(g); \
	__g->__v; \
})

#define UNALIGNED_PUT(v, p) \
do { \
	struct __attribute__((__packed__)) { \
		__typeof__(*(p)) __v; \
	} *__p = (__typeof__(__p))(p); \
	__p->__v = (v); \
} while (false)

#define __GENERIC_SECTION(segment) __attribute__((section(STRINGIFY(segment))))
#define Z_GENERIC_SECTION(segment) __GENERIC_SECTION(segment)

#define __GENERIC_DOT_SECTION(segment) \
	__attribute__((section("." STRINGIFY(segment))))
#define Z_GENERIC_DOT_SECTION(segment) __GENERIC_DOT_SECTION(segment)

#define ___in_section(a, b, c)
#define __in_section(a, b, c) ___in_section(a, b, c)
#define __in_section_unique(seg) ___in_section(seg, __FILE__, __COUNTER__)
#define __in_section_unique_named(seg, name) ___in_section(seg, __FILE__, name)

#ifndef __ramfunc
#define __ramfunc
#endif

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

#define __may_alias __attribute__((__may_alias__))

#ifndef __printf_like
#define __printf_like(f, a) printf_like(f, a)
#endif

#define __used used_code
#define __unused unused_code
#define __maybe_unused unused_code

#ifndef __deprecated
#define __deprecated __attribute__((deprecated))
#endif

#ifndef __attribute_const__
#define __attribute_const__ __attribute__((__const__))
#endif

#ifndef __must_check
#define __must_check __attribute__((warn_unused_result))
#endif

#define ARG_UNUSED(x) (void)(x)
#define likely(x) (predict_true((bool)!!(x)) != 0L)
#define unlikely(x) (predict_false((bool)!!(x)) != 0L)
#define POPCOUNT(x) __builtin_popcount(x)

#ifndef __no_optimization
#define __no_optimization nooptimiziation_function
#endif

#ifndef __weak
#define __weak weak_function
#endif

#ifndef __attribute_nonnull
#define __attribute_nonnull(...) __attribute__((nonnull(__VA_ARGS__)))
#endif

#define __WARN(msg) __WARN1(GCC warning msg)
#define __WARN1(s) _Pragma(#s)

#ifndef __DEPRECATED_MACRO
#define __DEPRECATED_MACRO __WARN("Macro is deprecated")
#endif

#define GEN_OFFSET_EXTERN(name) extern const char name[]

#define GEN_ABS_SYM_BEGIN(name) \
	EXTERN_C void name(void); \
	void name(void) \
	{

#define GEN_ABS_SYM_END }

#define compiler_barrier() do { \
	__asm__ __volatile__("" ::: "memory"); \
} while (false)

#if defined(CONFIG_ASAN)
#define __noasan nosanitize_address
#else
#define __noasan
#endif

#if defined(CONFIG_UBSAN)
#define __noubsan nosanitize_undefined
#else
#define __noubsan
#endif

#define FUNC_NO_STACK_PROTECTOR nostackprotect_function

#define TOOLCHAIN_IGNORE_WSHADOW_BEGIN \
	_Pragma("GCC diagnostic push") \
	_Pragma("GCC diagnostic ignored \"-Wshadow\"")

#define TOOLCHAIN_IGNORE_WSHADOW_END \
	_Pragma("GCC diagnostic pop")

#endif /* ZEPHYR_INCLUDE_TOOLCHAIN_H_ */
