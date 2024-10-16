#ifndef TARANTOOL_LIB_CORE_FIBER_H_INCLUDED
#define TARANTOOL_LIB_CORE_FIBER_H_INCLUDED
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
#include <stdbool.h>
#include <stdint.h>
#include "cord_on_demand.h"
#include "diag.h"
#include "small/mempool.h"
#include "small/region.h"
#include "small/rlist.h"

#include <coro/coro.h>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

enum {
	/**
	 * This flag is set when fiber function ends and before
	 * the fiber is recycled.
	 */
	FIBER_IS_DEAD		= 1 << 3,
	/**
	 * This flag is set when fiber uses custom stack size.
	 */
	FIBER_CUSTOM_STACK	= 1 << 4,
	/**
	 * The flag is set for the fiber currently being executed by the cord.
	 */
	FIBER_IS_RUNNING	= 1 << 5,
	FIBER_DEFAULT_FLAGS	= 0
};

/**
 * Fiber attributes container
 */
struct fiber_attr;

/**
 * Create a new fiber attribute container and initialize it
 * with default parameters.
 * Can be used for many fibers creation, corresponding fibers
 * will not take ownership.
 */
struct fiber_attr *
fiber_attr_new(void);

/**
 * Delete the fiber_attr and free all allocated resources.
 * This is safe when fibers created with this attribute still exist.
 *
 *\param fiber_attr fiber attribute
 */
void
fiber_attr_delete(struct fiber_attr *fiber_attr);

/**
 * Set stack size for the fiber attribute.
 *
 * \param fiber_attr fiber attribute container
 * \param stack_size stack size for new fibers
 */
int
fiber_attr_setstacksize(struct fiber_attr *fiber_attr, size_t stack_size);

/**
 * Get stack size from the fiber attribute.
 *
 * \param fiber_attr fiber attribute container or NULL for default
 * \retval stack size
 */
size_t
fiber_attr_getstacksize(struct fiber_attr *fiber_attr);

typedef int (*fiber_func)(va_list);

/**
 * Create a new fiber.
 *
 * Takes a fiber from fiber cache, if it's not empty.
 * Can fail only if there is not enough memory for
 * the fiber structure or fiber stack.
 *
 * The created fiber automatically returns itself
 * to the fiber cache when its "main" function
 * completes.
 *
 * \param f          func for run inside fiber
 *
 * \sa fiber_start
 */
struct fiber *
fiber_new(fiber_func f);

/** Free all fiber's resources and the fiber itself. */
void
fiber_delete(struct cord *cord, struct fiber *f);

/**
 * Create a new fiber with defined attributes.
 *
 * Can fail only if there is not enough memory for
 * the fiber structure or fiber stack.
 *
 * The created fiber automatically returns itself
 * to the fiber cache if has default stack size
 * when its "main" function completes.
 *
 * \param fiber_attr fiber attributes
 * \param f          func for run inside fiber
 *
 * \sa fiber_start
 */
struct fiber *
fiber_new_ex(const struct fiber_attr *fiber_attr, fiber_func f);

/**
 * Return control to another fiber and wait until it'll be woken.
 *
 * \note this is not a cancellation point (\sa fiber_testcancel()), but it is
 * considered a good practice to call fiber_testcancel() after each yield.
 *
 * \sa fiber_wakeup
 */
void
fiber_yield(void);

/**
 * Start execution of created fiber.
 *
 * \param callee fiber to start
 * \param ...    arguments to start the fiber with
 *
 * \sa fiber_new
 */
void
fiber_start(struct fiber *callee, ...);

/**
 * Set a pointer to context for the fiber. Can be used to avoid calling
 * fiber_start which means no yields.
 *
 * \param f     fiber to set the context for
 *              if NULL, the current fiber is used
 * \param f_arg context for the fiber function
 */
void
fiber_set_ctx(struct fiber *f, void *f_arg);

/**
 * Get the context for the fiber which was set via the fiber_set_ctx
 * function. Can be used to avoid calling fiber_start which means no yields.
 * If \a f is NULL, the current fiber is used.
 *
 * \retval      context for the fiber function set by fiber_set_ctx function
 *
 * \sa fiber_set_ctx
 */
void *
fiber_get_ctx(struct fiber *f);

/**
 * Fiber attribute container
 */
struct fiber_attr {
	/** Fiber stack size. */
	size_t stack_size;
	/** Fiber flags. */
	uint32_t flags;
};

/**
 * Init fiber attr with default values
 */
void
fiber_attr_create(struct fiber_attr *fiber_attr);

struct fiber {
	coro_context ctx;
	/** Coro stack slab. */
	struct slab *stack_slab;
	/** Coro stack addr. */
	void *stack;
	/** Coro stack size. */
	size_t stack_size;
	/* A garbage-collected memory pool. */
	struct region gc;
	/**
	 * The fiber which should be scheduled when
	 * this fiber yields.
	 */
	struct fiber *caller;
	/** Fiber flags */
	uint32_t flags;
	/** Link in cord->alive or cord->dead list. */
	struct rlist link;
	/**
	 * This struct is considered as non-POD when compiling by g++.
	 * You can safely ignore all offset_of-related warnings.
	 * See http://gcc.gnu.org/bugzilla/show_bug.cgi?id=31488
	 */
	fiber_func f;
	union {
		/**
		 * Argument list passed when the fiber is invoked in a blocking
		 * way, so the caller may pass arguments from its own stack.
		 */
		va_list f_data;
		/**
		 * Fiber function argument which passed asynchronously. Can be
		 * used not to call fiber_start to avoid yields, but still pass
		 * something into the fiber.
		 */
		void *f_arg;
	};
	int f_ret;
	/** Exception which caused this fiber's death. */
	struct diag diag;
};

/**
 * @brief An independent execution unit that can be managed by a separate OS
 * thread. Each cord consists of fibers to implement cooperative multitasking
 * model.
 */
struct cord {
	/** The fiber that is currently being executed. */
	struct fiber *fiber;
	/** All fibers */
	struct rlist alive;
	/** A cache of dead fibers for reuse */
	struct rlist dead;
	/**
	 * Latest dead fiber which couldn't be reused and waits for its
	 * deletion. A fiber can't be reused if it is somehow non-standard. For
	 * instance, has a special stack.
	 * A fiber can't be deleted if it is the current fiber - can't delete
	 * own stack safely. Then it schedules own deletion for later. The
	 * approach is very similar to pthread stacks deletion - pthread can't
	 * delete own stack, so they are saved and deleted later by a newer
	 * pthread or by some other dying pthread. Same here with fibers.
	 */
	struct fiber *garbage;
	/** A memory cache for (struct fiber) */
	struct mempool fiber_mempool;
	/** A runtime slab cache for general use in this cord. */
	struct slab_cache slabc;
	/** The "main" fiber of this thread, the scheduler. */
	struct fiber sched;
};

extern __thread struct cord *cord_ptr;

/**
 * Returns a thread-local cord object.
 *
 * If the cord object wasn't initialized at thread start (cord_create()
 * wasn't called), a cord object is created automatically and destroyed
 * at thread exit.
 */
#define cord() ({							\
	if (unlikely(cord_ptr == NULL))					\
		cord_ptr = cord_on_demand();				\
	cord_ptr;							\
})

#define fiber() cord()->fiber

void
cord_create(struct cord *cord);

void
cord_destroy(struct cord *cord);

/**
 * Delete the latest garbage fiber which couldn't be deleted somewhy before. Can
 * safely rely on the fiber being not the current one. Because if it was added
 * here before, it means some previous fiber put itself here, then died
 * immediately afterwards for good, and gave control to another fiber. It
 * couldn't be scheduled again.
 */
void
cord_collect_garbage(struct cord *cord);

void
fiber_init(int (*fiber_invoke)(fiber_func f, va_list ap));

void
fiber_gc(void);

void
fiber_call(struct fiber *callee);

static inline bool
fiber_is_dead(struct fiber *f)
{
	return f->flags & FIBER_IS_DEAD;
}

/** Useful for C unit tests */
static inline int
fiber_c_invoke(fiber_func f, va_list ap)
{
	return f(ap);
}

#if defined(__cplusplus)
} /* extern "C" */

static inline int
fiber_cxx_invoke(fiber_func f, va_list ap)
{
	try {
		return f(ap);
	} catch (...) {
		return -1;
	}
}

#endif /* defined(__cplusplus) */

#endif /* TARANTOOL_LIB_CORE_FIBER_H_INCLUDED */