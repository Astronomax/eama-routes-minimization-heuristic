#ifndef TARANTOOL_UTIL_H_INCLUDED
#define TARANTOOL_UTIL_H_INCLUDED
/*
 * Copyright 2010-2016, Tarantool AUTHORS, please see AUTHORS file.
 *
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 *
 * 1. Redistributions of source code must retain the above
 *    copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * <COPYRIGHT HOLDER> OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <assert.h>
#include <stdbool.h>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#define restrict __restrict__

#ifndef NDEBUG
#define TRASH(ptr) memset(ptr, '#', sizeof(*ptr))
#else
#define TRASH(ptr) (void) (ptr)
#endif

#define EPS1 1e-1
#define EPS2 1e-2
#define EPS3 1e-3
#define EPS4 1e-4
#define EPS5 1e-5
#define EPS6 1e-6
#define EPS7 1e-7
#define EPS8 1e-8
#define EPS9 1e-9

#ifndef MAX
# define MAX(a, b) ((a) > (b) ? (a) : (b))
# define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define SWAP(a, b) do {							\
	__typeof__(a) tmp = (a);					\
	(a) = (b);							\
	(b) = tmp;							\
} while (0)

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

#define nelem(x)     (sizeof((x))/sizeof((x)[0]))
#define field_sizeof(compound_type, field) sizeof(((compound_type *)NULL)->field)
#ifndef lengthof
#define lengthof(array) (sizeof (array) / sizeof ((array)[0]))
#endif

static inline void
alloc_failure(const char *filename, int line, size_t size)
{
	fprintf(stderr, "Can't allocate %zu bytes at %s:%d",
		size, filename, line);
	exit(EXIT_FAILURE);
}

/**
 * An x* variant of a memory allocation function calls the original function
 * and panics if it fails (i.e. it should never return NULL).
 */
#define xalloc_impl(size, func, args...)					\
	({									\
		void *ret = func(args);						\
		if (unlikely(ret == NULL))					\
			alloc_failure(__FILE__, __LINE__, (size));		\
		ret;								\
	})

#define xmalloc(size)		xalloc_impl((size), malloc, (size))
#define xcalloc(n, size)	xalloc_impl((n) * (size), calloc, (n), (size))
#define xrealloc(ptr, size)	xalloc_impl((size), realloc, (ptr), (size))
#define xstrdup(s)		xalloc_impl(strlen((s)) + 1, strdup, (s))
#define xstrndup(s, n)		xalloc_impl((n) + 1, strndup, (s), (n))
#define xaligned_alloc(size, align) \
		xalloc_impl((size), aligned_alloc, (align), (size))
#define xmempool_alloc(p)	xalloc_impl((p)->objsize, mempool_alloc, (p))
#define xregion_alloc(p, size)	xalloc_impl((size), region_alloc, (p), (size))
#define xregion_aligned_alloc(p, size, align) \
		xalloc_impl((size), region_aligned_alloc, (p), (size), (align))
#define xregion_join(p, size)	xalloc_impl((size), region_join, (p), (size))
#define xibuf_alloc(p, size)	xalloc_impl((size), ibuf_alloc, (p), (size))
#define xibuf_reserve(p, size)	xalloc_impl((size), ibuf_reserve, (p), (size))
#define xruntime_memory_alloc(size) \
	xalloc_impl((size), runtime_memory_alloc, (size))
#define xlsregion_alloc(p, size, id) \
	xalloc_impl((size), lsregion_alloc, (p), (size), (id))
#define xlsregion_aligned_alloc(p, size, align, id) \
	xalloc_impl((size), lsregion_aligned_alloc, (p), (size), (align), (id))
#define xlsregion_alloc_object(lsregion, id, T) ({				\
	(T *)xlsregion_aligned_alloc((lsregion), sizeof(T), alignof(T), (id));	\
})
#define xlsregion_reserve(p, size) \
	xalloc_impl((size), lsregion_reserve, (p), (size))
#define xregion_alloc_object(region, T) ({					\
	(T *)xregion_aligned_alloc((region), sizeof(T), alignof(T));		\
})
#define xregion_alloc_array(region, T, count) ({				\
	(T *)xregion_aligned_alloc((region), sizeof(T) * (count), alignof(T));\
})

#define xobuf_alloc(p, size)	xalloc_impl((size), obuf_alloc, (p), (size))
#define xobuf_reserve(p, size)	xalloc_impl((size), obuf_reserve, (p), (size))

#define xobuf_dup(p, src, size)							\
	({									\
		size_t ret = obuf_dup((p), (src), (size));			\
		if (unlikely(ret != (size_t)(size)))				\
			alloc_failure(__FILE__, __LINE__, (size));		\
	})

/** \cond public */

/**
 * Feature test macros for -std=c11 / -std=c++11
 *
 * Sic: clang aims to be gcc-compatible and thus defines __GNUC__
 */
#ifndef __has_feature
#  define __has_feature(x) 0
#endif
/**
 * The special operator __has_builtin (operand) may be used in constant integer
 * contexts and in preprocessor "#if" and "#elif" expressions to test whether
 * the symbol named by its operand is recognized as a built-in function by GCC
 * in the current language and conformance mode.
 */
#ifndef __has_builtin
#  define __has_builtin(x) 0
#endif
/**
 * The special operator __has_attribute (operand) may be used in "#if"
 * and "#elif" expressions to test whether the attribute referenced by its
 * operand is recognized by GCC.
 */
#ifndef __has_attribute
#  define __has_attribute(x) 0
#endif
/**
 * The special operator __has_cpp_attribute (operand) may be used in "#if" and
 * "#elif" expressions in C++ code to test whether the attribute referenced by
 * it operand is recognized by GCC.
 */
#ifndef __has_cpp_attribute
#  define __has_cpp_attribute(x) 0
#endif

/**
 * Compiler-independent built-ins.
 *
 * \see https://gcc.gnu.org/onlinedocs/gcc/Other-Builtins.html
 *
 * {{{ Built-ins
 */

/**
 * You may use likely()/unlikely() to provide the compiler with branch
 * prediction information.
 */
#if __has_builtin(__builtin_expect) || defined(__GNUC__)
#  define likely(x)    __builtin_expect(!! (x),1)
#  define unlikely(x)  __builtin_expect(!! (x),0)
#else
#  define likely(x)    (x)
/**
 * You may use likely()/unlikely() to provide the compiler with branch
 * prediction information.
 *
 * @sa https://en.cppreference.com/w/cpp/language/attributes/likely
 */
#  define unlikely(x)  (x)
#endif

/**
 * This macro is used to minimize cache-miss latency by moving data into
 * a cache before it is accessed. You can insert calls to prefetch() into
 * code for which you know addresses of data in memory that is likely to be
 * accessed soon. If the target supports them, data prefetch instructions
 * will be generated. If the prefetch is done early enough before the access
 * then the data will be in the cache by the time it is accessed.
 *
 * The value of addr is the address of the memory to prefetch. There are two
 * optional arguments, rw and locality. The value of rw is a compile-time
 * constant one or zero; one means that the prefetch is preparing for a write
 * to the memory address and zero, the default, means that the prefetch is
 * preparing for a read. The value locality must be a compile-time constant
 * integer between zero and three. A value of zero means that the data has
 * no temporal locality, so it need not be left in the cache after the access.
 * A value of three means that the data has a high degree of temporal locality
 * and should be left in all levels of cache possible. Values of one and two
 * mean, respectively, a low or moderate degree of temporal locality.
 * The default is three.
 */
#if __has_builtin(__builtin_prefetch) || defined(__GNUC__)
#  define prefetch(addr, ...) (__builtin_prefetch(addr, __VA_ARGS__))
#else
#  define prefetch(addr, ...) ((void) addr)
#endif

/**
 * If control flow reaches the point of the unreachable(), the program is
 * undefined. It is useful in situations where the compiler cannot deduce
 * the unreachability of the code.
 */
#if __has_builtin(__builtin_unreachable) || defined(__GNUC__)
#  define unreachable() (assert(0), __builtin_unreachable())
#else
#  define unreachable() (assert(0))
#endif

/**
 * The macro offsetof expands to an integral constant expression of
 * type size_t, the value of which is the offset, in bytes, from
 * the beginning of an object of specified type to its specified member,
 * including padding if any.
 */
#ifndef offsetof
#  if __has_builtin(__builtin_offsetof)
#    define offsetof(type, member) __builtin_offsetof(type, member)
#  else
#    define offsetof(type, member) ((size_t)&((type *)0)->member)
#  endif
#endif /* offsetof */

/**
 * This macro is used to retrieve an enclosing structure from a pointer to
 * a nested element.
 */
#ifndef container_of
#define container_of(ptr, type, member) ({ \
	const typeof( ((type *)0)->member  ) *__mptr = (ptr); \
	(type *)( (char *)__mptr - offsetof(type,member)  );})
#endif

/**
 * C11/C++11 keyword. Appears in the declaration syntax as one of the type
 * specifiers to modify the alignment requirement of the object being
 * declared.
 *
 * Sic: alignas() doesn't work on anonymous structs on gcc < 4.9
 *
 * Example:
 *
 * \code
 * struct obuf { int a; int b; alignas(16) int c; };
 * \endcode
 */
#if defined(__cplusplus)
#  include <stdalign.h>
#endif
#if !defined(alignas) && !defined(__alignas_is_defined)
#  if __has_feature(c_alignas) || (defined(__GNUC__) && __GNUC__ >= 5)
#    include <stdalign.h>
#  elif __has_attribute(aligned) || defined(__GNUC__)
#    define alignas(_n) __attribute__((aligned(_n)))
#    define __alignas_is_defined 1
#  else
#    define alignas(_n)
#  endif
#endif

#if !defined(alignof) && !defined(__alignof_is_defined)
#  if __has_feature(c_alignof) || (defined(__GNUC__) && __GNUC__ >= 5)
#    include <stdalign.h>
#  elif defined(__GNUC__)
#    define alignof(_T) __alignof(_T)
/*! @cond Doxygen_Suppress */
#    define __alignof_is_defined 1
/*! @endcond */
#  else
/**
 * C11/C++11 operator. Returns the alignment, in bytes, required for any
 * instance of the type indicated by type-id, which is either complete type,
 * an array type, or a reference type.
 */
#    define alignof(_T) offsetof(struct { char c; _T member; }, member)
/*! @cond Doxygen_Suppress */
#    define __alignof_is_defined 1
/*! @endcond */
#  endif
#endif

/** Built-ins }}} */

/**
 * Compiler-independent function attributes.
 *
 * \see https://gcc.gnu.org/onlinedocs/gcc/Type-Attributes.html
 * \see http://clang.llvm.org/docs/AttributeReference.html#function-attributes
 * \see http://en.cppreference.com/w/cpp/language/attributes
 *
 * {{{ Function Attributes
 */

/**
 * The MAYBE_UNUSED function attribute can be used to silence -Wunused
 * diagnostics when the entity cannot be removed. For instance, a local
 * variable may exist solely for use in an assert() statement, which makes
 * the local variable unused when NDEBUG is defined.
 *
 * Example:
 *
 * \code
 * int fun(MAYBE_UNUSED int unused_arg);
 * \endcode
 */
#if defined(__cplusplus) && __has_cpp_attribute(maybe_unused)
#  define MAYBE_UNUSED [[maybe_unused]]
#elif __has_attribute(unused) || defined(__GNUC__)
#  define MAYBE_UNUSED __attribute__((unused))
#else
#  define MAYBE_UNUSED
#endif

/**
 * A diagnostic is generated when a function is marked with NODISCARD and
 * the function call appears as a potentially-evaluated discarded-value
 * expression that is not explicitly cast to void.
 *
 * Example:
 *
 * \code
 * NODISCARD int function() { return -1 };
 * \endcode
 */
#if defined(__cplusplus) && __has_cpp_attribute(nodiscard)
#  define NODISCARD [[nodiscard]]
#elif __has_attribute(warn_unused_result) || defined(__GNUC__)
#  define NODISCARD __attribute__((warn_unused_result))
#else
#  define NODISCARD
#endif

/**
 * This function attribute prevents a function from being considered for
 * inlining.
 *
 * Example:
 *
 * \code
 * NOINLINE int function() { return 0; };
 * \endcode
 */
#if __has_attribute(noinline) || defined(__GNUC__)
#  define NOINLINE __attribute__((noinline))
#else
#  define NOINLINE
#endif

/**
 * Adds the 'always_inline' attribute to the function if it's supported. This
 * attribute forces the function to be inlined if it's possible. If it's not,
 * this results in a diagnostic.
 *
 * Example:
 *
 * \code
 * ALWAYS_INLINE int function() { return 0; }
 * \endcode
 */
#if __has_attribute(always_inline) || defined(__GNUC__)
#  define ALWAYS_INLINE inline __attribute__((always_inline))
#else
#  define ALWAYS_INLINE inline
#endif

/**
 * A function declared as NORETURN shall not return to its caller.
 * The compiler will generate a diagnostic for a function declared as
 * NORETURN that appears to be capable of returning to its caller.
 *
 * Example:
 *
 * \code
 * NORETURN void abort();
 * \endcode
 */
#if defined(__cplusplus) && __has_cpp_attribute(noreturn)
#  define NORETURN [[noreturn]]
#elif __has_attribute(noreturn) || defined(__GNUC__)
#  define NORETURN  __attribute__((noreturn))
#else
#  define NORETURN
#endif

/**
 * The DEPRECATED attribute can be applied to a function, a variable, or
 * a type. This is useful when identifying functions, variables, or types
 * that are expected to be removed in a future version of a program.
 */
#if defined(__cplusplus) && __has_cpp_attribute(deprecated)
#  define DEPRECATED(_msg) [[deprecated(_msg)]]
#elif __has_attribute(deprecated) || defined(__GNUC__)
#  define DEPREACTED  __attribute__((deprecated(_msg)))
#else
#  define DEPRECATED(_msg)
#endif

/**
 * The API_EXPORT attribute declares public C API function.
 */
#if defined(__cplusplus) && defined(__GNUC__)
#  define API_EXPORT extern "C" __attribute__ ((nothrow, visibility ("default")))
#elif defined(__cplusplus)
#  define API_EXPORT extern "C"
#elif defined(__GNUC__)
#  define API_EXPORT extern __attribute__ ((nothrow, visibility ("default")))
#else
#  define API_EXPORT extern
#endif

/**
 * The CFORMAT attribute specifies that a function takes printf, scanf,
 * strftime or strfmon style arguments that should be type-checked against
 * a format string.
 *
 * \see https://gcc.gnu.org/onlinedocs/gcc/Common-Function-Attributes.html
 */
#if __has_attribute(format) || defined(__GNUC__)
#  define CFORMAT(_archetype, _stringindex, _firsttocheck) \
	__attribute__((format(_archetype, _stringindex, _firsttocheck)))
#else
#  define CFORMAT(archetype, stringindex, firsttocheck)
#endif

/**
 * The PACKED qualifier is useful to map a structure to an external data
 * structure, or for accessing unaligned data, but it is generally not
 * useful to save data size because of the relatively high cost of
 * unaligned access on some architectures.
 *
 * Example:
 *
 * \code
 * struct PACKED name { char a; int b; };
 * \endcode
 */
#if __has_attribute(packed) || defined(__GNUC__)
#  define PACKED  __attribute__((packed))
#elif defined(__CC_ARM)
#  define PACKED __packed
#else
#  define PACKED
#endif

/**
 * Helper macro to handle easily snprintf() result
 */
#define SNPRINT(_total, _fun, _buf, _size, ...) do {				\
	int written =_fun(_buf, _size, ##__VA_ARGS__);				\
	if (written < 0)							\
		return -1;							\
	_total += written;							\
	if (written < _size) {							\
		_buf += written, _size -= written;				\
	} else {								\
		_buf = NULL, _size = 0;						\
	}									\
} while(0)

/** Like assert() but evaluates the given expression even if NDEBUG is set. */
#ifndef NDEBUG
# define VERIFY(expr) assert(expr)
#else
# define VERIFY(expr) ((void)(expr))
#endif

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#endif /* TARANTOOL_UTIL_H_INCLUDED */
