#include "eama_solver.h"

#include "core/fiber.h"
#include "core/random.h"

#include "tt_static.h"

void
eliminate_random_route(struct solution *s)
{
	int route_idx = randint(0, s->n_routes - 1);
	struct route *r = s->routes[route_idx];
	SWAP(s->routes[route_idx], s->routes[s->n_routes - 1]);
	--s->n_routes;

	struct customer *c, *tmp;
	rlist_foreach_entry_safe(c, &r->list, in_route, tmp) {
		rlist_del_entry(c, in_route);
		if (c->id == 0) {
			customer_delete(c);
		} else {
			c->route = NULL;
			rlist_add_tail_entry(&s->ejection_pool, c, in_eject);
		}
	}
	route_delete(r);
}

const char	*RESET 	= "\033[0m",
		*RED	= "\033[91m",
		*GREEN	= "\033[92m",
		*YELLOW	= "\033[93m",
		*BLUE	= "\033[94m",
		*PURPLE	= "\033[95m",
		*CYAN	= "\033[96m",
		*WHITE	= "\033[97m";

#define debug_print(msg, color) do {					\
	printf("%s%s: %s%s%s\n", BLUE, __func__, color, msg, RESET);	\
	fflush(stdout);							\
} while (0)

int
insert_feasible(struct solution *s, bool debug)
{
	assert(solution_find_customer_by_id(s, s->w->id) == s->w);
	if (debug)
		debug_print("started", RESET);

	assert(s->w != NULL);
	assert(is_ejected(s->w));
	assert(solution_feasible(s));
	struct modification m = solution_find_feasible_insertion(s, s->w);
	if (m.v != NULL) {
		_solution_check(s);
		modification_apply(m);
		s->w = NULL;
		_solution_check(s);
		assert(solution_feasible(s));
		if (debug)
			debug_print("completed successfully", GREEN);
		return 0;
	}
	if (debug)
		debug_print("failed", RED);
	return -1;
}

int
squeeze(struct eama *eama, struct solution *s, bool debug)
{
	if (debug)
		debug_print("started", RESET);

	assert(s->w != NULL);
	assert(is_ejected(s->w));
	assert(solution_feasible(s));
	struct solution *s_dup = solution_dup(s);
	assert(solution_find_customer_by_id(s, s->w->id) == s->w);
	assert(solution_find_customer_by_id(s_dup, s_dup->w->id) == s_dup->w);

	modification_apply(solution_find_optimal_insertion(s, s->w, eama->alpha, eama->beta));
	s->w = NULL;
	_solution_check(s);
	assert(!solution_feasible(s));

	int infeasible = split_by_feasibility(s);
	while(infeasible > 0) {
		if (debug) {
			debug_print(tt_sprintf("penalty_sum: %f",
				solution_penalty(s, eama->alpha, eama->beta)), RESET);
			//double p_tw = 0.;
			//for(int i = 0; i < s->n_routes; i++)
			//	p_tw += route_penalty(s->routes[i], 1., 0.);
			//debug_print(tt_sprintf("tw_penalty_sum: %f", p_tw), RESET);
			//double c_tw = 0.;
			//for(int i = 0; i < s->n_routes; i++)
			//	c_tw += route_penalty(s->routes[i], 0., 1.);
			//debug_print(tt_sprintf("c_penalty_sum: %f", c_tw), RESET);
		}
		int route_idx = randint(0, infeasible - 1);
		struct route *v_route = s->routes[route_idx];
		assert(!route_feasible(v_route));

		double v_route_penalty = route_penalty(v_route, eama->alpha, eama->beta);
		struct modification opt_modification = modification_new(INSERT, NULL, NULL);
		double opt_delta = INFINITY;
		struct modification m;
		struct fiber *f = fiber_new(solution_modification_neighbourhood_f);
		fiber_start(f, s, v_route, eama->settings.n_near, &m);
		while (!fiber_is_dead(f)) {
			double delta = modification_delta(m, eama->alpha, eama->beta);
			if (delta < opt_delta) {
				opt_modification = m;
				opt_delta = delta;
				//TODO: break early
				//if (opt_delta <= -v_route_penalty + EPS5) {
					/**
					 * We won't be able to reuse (recycle) the fiber because it hasn't
					 * died. However, the current API does not allow you to cancel it.
					 * Therefore, we are forced to simply remove it manually.
					 */
				//	fiber_delete(cord(), f);
				//	break;
				//}
			}
			fiber_call(f);
		}

		if (debug)
			debug_print(tt_sprintf("opt modification delta: %f", opt_delta), RESET);
		if (opt_delta > -EPS5) {
			//if (solution_penalty(s, 1., -1.) < 0.)
			//	eama->beta /= 0.99;
			//else
			//	eama->beta *= 0.99;
			if (debug) {
				debug_print("failed", RED);
				debug_print(tt_sprintf("beta after correction: %0.12f", eama->beta), RESET);
			}
			solution_move(s, s_dup);
			return -1;
		}
		_solution_check(s);
		modification_apply(opt_modification);
		_solution_check(s);
		infeasible = split_by_feasibility(s);
	}
	if (debug)
		debug_print("completed successfully", GREEN);
	solution_delete(s_dup);
	assert(solution_feasible(s));
	return 0;
}

int
insert_eject(struct eama *eama, struct solution *s, bool debug)
{
	if (debug)
		debug_print("started", RESET);

	assert(s->w != NULL);
	assert(is_ejected(s->w));
	assert(solution_feasible(s));
	struct solution *s_dup = solution_dup(s);

	int64_t p_best = INT64_MAX;
	struct modification opt_insertion = modification_new(INSERT, NULL, s->w);
	struct rlist ejection, opt_ejection;
	rlist_create(&ejection);
	rlist_create(&opt_ejection);
	for (int i = 0; i < s->n_routes; i++) {
		struct route *v_route = s->routes[i];
		struct modification m = route_find_optimal_insertion(
			v_route, s->w, eama->alpha, eama->beta);
		if (!modification_applicable(m))
			continue;
		modification_apply(m);
		assert(!is_ejected(s->w));
		assert(!route_feasible(v_route));

		struct fiber *f = fiber_new(feasible_ejections_f);
		fiber_start(f, v_route, eama->settings.k_max, eama->p, &ejection, &p_best);
		while(!fiber_is_dead(f)) {
			opt_insertion = m;
			rlist_del(&opt_ejection);
			struct customer *c;
			rlist_foreach_entry(c, &ejection, in_eject)
				rlist_add_tail_entry(&opt_ejection, c, in_opt_eject);
			fiber_call(f);
		}
		assert(rlist_empty(&ejection));
		m = modification_new(EJECT, s->w, NULL);
		assert(modification_applicable(m));
		modification_apply(m);
	}
	if (debug)
		debug_print(tt_sprintf("opt insertion-ejection p_sum: %ld", p_best), RESET);

	if (unlikely(opt_insertion.v == NULL && rlist_empty(&opt_ejection))) {
		solution_move(s, s_dup);
		return -1;
	}

	{
		if (debug) {
			printf("opt insertion-ejection: ");
			struct customer *c;
			rlist_foreach_entry(c, &opt_ejection, in_opt_eject)
				printf("%d ", c->id);
			printf("\n");
			fflush(stdout);
		}
	}

	modification_apply(opt_insertion);
	s->w = NULL;
	_solution_check(s);

	struct customer *c;
	rlist_foreach_entry(c, &opt_ejection, in_opt_eject) {
		struct modification m = modification_new(EJECT, c, NULL);
		assert(modification_applicable(m));
		modification_apply(m);
		rlist_add_tail_entry(&s->ejection_pool, c, in_eject);
	}

	if (debug)
		debug_print("completed successfully", GREEN);
	solution_delete(s_dup);
	assert(solution_feasible(s));
	_solution_check(s);
	return 0;
}

int
delete_route(struct eama *eama, struct solution *s, bool debug)
{
	if (debug)
		debug_print("started", RESET);

	assert(rlist_empty(&s->ejection_pool));
	assert(solution_feasible(s));
	struct solution *s_dup = solution_dup(s);
	eliminate_random_route(s);
	assert(solution_feasible(s));
	_solution_check(s);
	while (!rlist_empty(&s->ejection_pool)) {
		if (debug) {
			struct customer *c;
			int n_ejected = 0;
			rlist_foreach_entry(c, &s->ejection_pool, in_eject) ++n_ejected;
			debug_print(tt_sprintf("ejection_pool: %d", n_ejected), RESET);
		}
		/** remove v from EP with the LIFO strategy */
		s->w = rlist_last_entry(
			&s->ejection_pool, struct customer, in_eject);
		rlist_del_entry(s->w, in_eject);

		assert(solution_find_customer_by_id(s, s->w->id) == s->w);

		_solution_check(s);
		if (insert_feasible(s, debug) == 0) {
			_solution_check(s);
			continue;
		} else if (squeeze(eama, s, debug) == 0) {
			_solution_check(s);
			continue;
		}
		assert(solution_find_customer_by_id(s, s->w->id) == s->w);
		++eama->p[s->w->id];
		if (debug)
			debug_print(tt_sprintf("p[%d] = %ld", s->w->id, eama->p[s->w->id]), RESET);

		if (insert_eject(eama, s, debug) == 0) {
			//TODO: perturb
			continue;
		}
		/** fail */
		if (debug)
			debug_print("failed", RED);
		solution_move(s, s_dup);
		//TODO: remember new p-values
		//TODO: make this smarter
		//for (int i = 0; i <= p.n_customers; i++)
		//	solution_find_customer_by_id(s_dup, i)->p =
		//		solution_find_customer_by_id(s, i)->p;
		return -1;
	}
	if (debug)
		debug_print("completed successfully", GREEN);
	solution_delete(s_dup);
	return 0;
}

struct eama *
eama_new(struct eama_settings settings)
{
	struct eama *eama = xmalloc(sizeof(*eama));
	eama->settings = settings;
	eama->alpha = eama->beta = 1.;
	memset(&eama->p[0], 0, sizeof(eama->p));
	return eama;
}

void
eama_delete(struct eama *eama)
{
	free(eama);
}

struct solution *
eama_solver_solve(struct eama *eama, bool debug)
{
	if (debug)
		debug_print("started", RESET);
	struct solution *s = solution_default();
	_solution_check(s);
	int lower_bound = MAX(problem_routes_straight_lower_bound(),
			      eama->settings.lower_bound);
	while (s->n_routes > lower_bound) {
		if (debug)
			debug_print(tt_sprintf("routes number: %d", s->n_routes), PURPLE);
		if (delete_route(eama, s, debug) != 0)
			break;
		assert(rlist_empty(&s->ejection_pool));
		assert(s->w == NULL);
		_solution_check(s);
		assert(rlist_empty(&s->ejection_pool));
	}
	assert(s->w == NULL);
	assert(rlist_empty(&s->ejection_pool));
	if (debug)
		debug_print("completed successfully", GREEN);
	return s;
}
