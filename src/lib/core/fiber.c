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
#include "fiber.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pmatomic.h>
#include "memory.h"
#include "tt_static.h"

static int (*fiber_invoke)(fiber_func f, va_list ap);

static inline int
fiber_mprotect(void *addr, size_t len, int prot)
{
	if (mprotect(addr, len, prot) != 0) {
		diag_set(SystemError, "fiber mprotect failed");
		return -1;
	}
	return 0;
}

__thread struct cord *cord_ptr = NULL;

static size_t page_size;
static int stack_direction;

#ifndef FIBER_STACK_SIZE_DEFAULT
#error "Default fiber stack size is not set"
#endif

enum {
    /* The minimum allowable fiber stack size in bytes */
    FIBER_STACK_SIZE_MINIMAL = 16384,
};

/** Default fiber attributes */
static const struct fiber_attr fiber_attr_default = {
	.stack_size = FIBER_STACK_SIZE_DEFAULT
};

void
fiber_attr_create(struct fiber_attr *fiber_attr)
{
	*fiber_attr = fiber_attr_default;
}

struct fiber_attr *
fiber_attr_new(void)
{
	struct fiber_attr *fiber_attr = malloc(sizeof(*fiber_attr));
	if (fiber_attr == NULL)  {
		diag_set(OutOfMemory, sizeof(*fiber_attr),
			 "runtime", "fiber attr");
		return NULL;
	}
	fiber_attr_create(fiber_attr);
	return fiber_attr;
}

void
fiber_attr_delete(struct fiber_attr *fiber_attr)
{
	free(fiber_attr);
}

int
fiber_attr_setstacksize(struct fiber_attr *fiber_attr, size_t stack_size)
{
	if (stack_size < FIBER_STACK_SIZE_MINIMAL) {
		errno = EINVAL;
		diag_set(SystemError, "stack size is too small");
		return -1;
	}
	fiber_attr->stack_size = stack_size;
	if (stack_size != FIBER_STACK_SIZE_DEFAULT) {
		fiber_attr->flags |= FIBER_CUSTOM_STACK;
	} else {
		fiber_attr->flags &= ~FIBER_CUSTOM_STACK;
	}
	return 0;
}

size_t
fiber_attr_getstacksize(struct fiber_attr *fiber_attr)
{
	return fiber_attr != NULL ? fiber_attr->stack_size :
	       fiber_attr_default.stack_size;
}

static void
fiber_recycle(struct fiber *fiber);

/**
 * Try to delete a fiber right now or later if can't do now. The latter happens
 * for self fiber - can't delete own stack.
 */
static void
cord_add_garbage(struct cord *cord, struct fiber *f);

/**
 * True if a fiber with `fiber_flags` can be reused.
 * A fiber can not be reused if it is somehow non-standard.
 */
static bool
fiber_is_reusable(uint32_t fiber_flags)
{
	/* For now we can not reuse fibers with custom stack size. */
	return (fiber_flags & FIBER_CUSTOM_STACK) == 0;
}

/**
 * Transfer control to callee fiber.
 */
static void
fiber_call_impl(struct fiber *callee)
{
	struct fiber *caller = fiber();
	struct cord *cord = cord();

	assert(callee->f != NULL);
	assert(! (callee->flags & FIBER_IS_DEAD));

	assert(caller);
	assert(caller != callee);
	assert((caller->flags & FIBER_IS_RUNNING) != 0);
	assert((callee->flags & FIBER_IS_RUNNING) == 0);

	caller->flags &= ~FIBER_IS_RUNNING;
	cord->fiber = callee;
	callee->flags = callee->flags | FIBER_IS_RUNNING;

	coro_transfer(&caller->ctx, &callee->ctx);
}

void
fiber_call(struct fiber *callee)
{
	struct fiber *caller = fiber();

	callee->caller = caller;
	fiber_call_impl(callee);
}

void
fiber_start(struct fiber *callee, ...)
{
	va_start(callee->f_data, callee);
	fiber_call(callee);
	va_end(callee->f_data);
}

void
fiber_set_ctx(struct fiber *f, void *f_arg)
{
	if (f == NULL)
		f = fiber();
	f->f_arg = f_arg;
}

void *
fiber_get_ctx(struct fiber *f)
{
	if (f == NULL)
		f = fiber();
	return f->f_arg;
}

/**
 * Implementation of `fiber_yield()` and `fiber_yield_final()`.
 * `will_switch_back` argument is used only by ASAN.
 */
static void
fiber_yield_impl()
{
	struct cord *cord = cord();
	struct fiber *caller = cord->fiber;
	struct fiber *callee = caller->caller;
	caller->caller = &cord->sched;

	assert(! (callee->flags & FIBER_IS_DEAD));
	assert((caller->flags & FIBER_IS_RUNNING) != 0);
	assert((callee->flags & FIBER_IS_RUNNING) == 0);

	caller->flags &= ~FIBER_IS_RUNNING;
	cord->fiber = callee;
	callee->flags = callee->flags | FIBER_IS_RUNNING;

	coro_transfer(&caller->ctx, &callee->ctx);
}

void
fiber_yield(void)
{
	fiber_yield_impl();
}

void
fiber_gc(void)
{
	if (region_used(&fiber()->gc) < 128 * 1024) {
		region_reset(&fiber()->gc);
		return;
	}

	region_free(&fiber()->gc);
}

/** Destroy an active fiber and prepare it for reuse or delete it. */
static void
fiber_recycle(struct fiber *fiber)
{
	assert((fiber->flags & FIBER_IS_DEAD) != 0);
	/* no exceptions are leaking */
	assert(diag_is_empty(&fiber->diag));
	fiber->f = NULL;
	region_free(&fiber->gc);
	if (fiber_is_reusable(fiber->flags)) {
		rlist_move_entry(&cord()->dead, fiber, link);
	} else {
		cord_add_garbage(cord(), fiber);
	}
}

static void
fiber_loop(MAYBE_UNUSED void *data)
{
	for (;;) {
		struct fiber *fiber = fiber();

		assert(fiber != NULL && fiber->f != NULL);
		fiber->f_ret = fiber_invoke(fiber->f, fiber->f_data);
		if (fiber->f_ret != 0) {
			struct error *e = diag_last_error(&fiber->diag);
			/* diag must not be empty on error */
			assert(e != NULL);
			error_log(e);
			diag_clear(&fiber()->diag);
		} else {
			/*
			 * Make sure a leftover exception does not
			 * propagate up to the joiner.
			 */
			diag_clear(&fiber()->diag);
		}
		fiber->flags |= FIBER_IS_DEAD;
		fiber_recycle(fiber);
		/*
		 * Crash if spurious wakeup happens, don't call the old
		 * function again, ap is garbage by now.
		 */
		fiber->f = NULL;
		/*
		 * Give control back to the scheduler.
		 */
		fiber_yield();
	}
}

static inline void *
page_align_down(void *ptr)
{
	return (void *)((intptr_t)ptr & ~(page_size - 1));
}

static inline void *
page_align_up(void *ptr)
{
	return page_align_down(ptr + page_size - 1);
}

static void
fiber_stack_destroy(struct fiber *fiber, struct slab_cache *slabc)
{
	static const int mprotect_flags = PROT_READ | PROT_WRITE;

	if (fiber->stack != NULL) {
		void *guard;
		if (stack_direction < 0)
			guard = page_align_down(fiber->stack - page_size);
		else
			guard = page_align_up(fiber->stack + fiber->stack_size);

		if (fiber_mprotect(guard, page_size, mprotect_flags) != 0) {
			/*
			 * FIXME: We need some intelligent handling:
			 * say put this slab into a queue and retry
			 * to setup the original protection back in
			 * background.
			 *
			 * For now lets keep such slab referenced and
			 * leaked: if mprotect failed we must not allow
			 * to reuse such slab with PROT_NONE'ed page
			 * inside.
			 *
			 * Note that in case if we're called from
			 * fiber_stack_create() the @a mprotect_flags is
			 * the same as the slab been created with, so
			 * calling mprotect for VMA with same flags
			 * won't fail.
			 */
			say_syserror("fiber: Can't put guard page to slab. "
				     "Leak %zu bytes", (size_t)fiber->stack_size);
		} else {
			slab_put(slabc, fiber->stack_slab);
		}
	}
}

static int
fiber_stack_create(struct fiber *fiber, const struct fiber_attr *fiber_attr,
		   struct slab_cache *slabc)
{
	size_t stack_size = fiber_attr->stack_size - slab_sizeof();
	fiber->stack_slab = slab_get(slabc, stack_size);

	if (fiber->stack_slab == NULL) {
		diag_set(OutOfMemory, stack_size,
			 "runtime arena", "fiber stack");
		return -1;
	}
	void *guard;
	/* Adjust begin and size for stack memory chunk. */
	if (stack_direction < 0) {
		/*
		 * A stack grows down. First page after begin of a
		 * stack memory chunk should be protected and memory
		 * after protected page until end of memory chunk can be
		 * used for coro stack usage.
		 */
		guard = page_align_up(slab_data(fiber->stack_slab));
		fiber->stack = guard + page_size;
		fiber->stack_size = slab_data(fiber->stack_slab) + stack_size -
				    fiber->stack;
	} else {
		/*
		 * A stack grows up. Last page should be protected and
		 * memory from begin of chunk until protected page can
		 * be used for coro stack usage
		 */
		guard = page_align_down(fiber->stack_slab + stack_size) -
			page_size;
		fiber->stack = fiber->stack_slab + slab_sizeof();
		fiber->stack_size = guard - fiber->stack;
	}

	if (fiber_mprotect(guard, page_size, PROT_NONE)) {
		/*
		 * Write an error into the log since a guard
		 * page is critical for functionality.
		 */
		diag_log();
		fiber_stack_destroy(fiber, slabc);
		return -1;
	}
	return 0;
}

struct fiber *
fiber_new_ex(const struct fiber_attr *fiber_attr, fiber_func f)
{
	struct cord *cord = cord();
	struct fiber *fiber = NULL;
	assert(fiber_attr != NULL);

	if (fiber_is_reusable(fiber_attr->flags) && !rlist_empty(&cord->dead)) {
		fiber = rlist_first_entry(&cord->dead, struct fiber, link);
		rlist_move_entry(&cord->alive, fiber, link);
		assert(fiber_is_dead(fiber));
	} else {
		fiber = (struct fiber *)
			mempool_alloc(&cord->fiber_mempool);
		if (fiber == NULL) {
			diag_set(OutOfMemory, sizeof(struct fiber),
				 "fiber pool", "fiber");
			return NULL;
		}
		memset(fiber, 0, sizeof(struct fiber));

		if (fiber_stack_create(fiber, fiber_attr, &cord()->slabc)) {
			mempool_free(&cord->fiber_mempool, fiber);
			return NULL;
		}
		coro_create(&fiber->ctx, fiber_loop, NULL,
			    fiber->stack, fiber->stack_size);

		region_create(&fiber->gc, &cord->slabc);

		diag_create(&fiber->diag);

		rlist_add_entry(&cord->alive, fiber, link);
	}
	fiber->flags = fiber_attr->flags;
	fiber->f = f;

	return fiber;

}

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
 */
struct fiber *
fiber_new(fiber_func f)
{
	return fiber_new_ex(&fiber_attr_default, f);
}

/** Free all fiber's resources. */
static void
fiber_destroy(struct cord *cord, struct fiber *f)
{
	assert(f != cord->fiber);
	rlist_del(&f->link);
	region_destroy(&f->gc);
	fiber_stack_destroy(f, &cord->slabc);
	diag_destroy(&f->diag);
	TRASH(f);
}

void
fiber_delete(struct cord *cord, struct fiber *f)
{
	assert(f != &cord->sched);
	fiber_destroy(cord, f);
	mempool_free(&cord->fiber_mempool, f);
}

/** Delete all fibers in the given list so it becomes empty. */
static void
cord_delete_fibers_in_list(struct cord *cord, struct rlist *list)
{
	while (!rlist_empty(list)) {
		struct fiber *f = rlist_first_entry(list, struct fiber, link);
		fiber_delete(cord, f);
	}
}

void
fiber_delete_all(struct cord *cord)
{
	cord_collect_garbage(cord);
	cord_delete_fibers_in_list(cord, &cord->alive);
	cord_delete_fibers_in_list(cord, &cord->dead);
}

void
cord_create(struct cord *cord)
{
	cord_ptr = cord;
	slab_cache_set_thread(&cord()->slabc);

	slab_cache_create(&cord->slabc, &runtime);
	mempool_create(&cord->fiber_mempool, &cord->slabc,
		       sizeof(struct fiber));
	rlist_create(&cord->alive);
	rlist_create(&cord->dead);

	/* sched fiber is not present in alive/dead list. */
	rlist_create(&cord->sched.link);
	diag_create(&cord->sched.diag);
	region_create(&cord->sched.gc, &cord->slabc);
	cord->fiber = &cord->sched;
	cord->sched.flags = FIBER_IS_RUNNING;

	cord->sched.stack = NULL;
	cord->sched.stack_size = 0;
}

void
cord_collect_garbage(struct cord *cord)
{
	struct fiber *garbage = cord->garbage;
	if (garbage == NULL)
		return;
	fiber_delete(cord, garbage);
	cord->garbage = NULL;
}

static void
cord_add_garbage(struct cord *cord, struct fiber *f)
{
	cord_collect_garbage(cord);
	assert(cord->garbage == NULL);
	if (f != cord->fiber)
		fiber_delete(cord, f);
	else
		cord->garbage = f;
}

void
cord_destroy(struct cord *cord)
{
	slab_cache_set_thread(&cord->slabc);
	fiber_delete_all(cord);
	cord->fiber = NULL;
	region_destroy(&cord->sched.gc);
	diag_destroy(&cord->sched.diag);
	slab_cache_destroy(&cord->slabc);
}

static NOINLINE int
check_stack_direction(void *prev_stack_frame)
{
	return __builtin_frame_address(0) < prev_stack_frame ? -1: 1;
}

void
fiber_init(int (*invoke)(fiber_func f, va_list ap))
{
	page_size = small_getpagesize();
	stack_direction = check_stack_direction(__builtin_frame_address(0));
	fiber_invoke = invoke;
}