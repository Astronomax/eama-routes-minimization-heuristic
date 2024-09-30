#include "eama_solver.h"

#include "cli.h"

#include "core/fiber.h"
#include "core/random.h"
#include "tt_static.h"

struct eama_solver eama_solver;

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
insert_feasible(struct solution *s)
{
	assert(solution_find_customer_by_id(s, s->w->id) == s->w);
	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print("started", RESET);

	assert(s->w != NULL);
	assert(is_ejected(s->w));
	assert(solution_feasible(s));
	struct modification m = solution_find_feasible_insertion(s, s->w);
	if (m.v != NULL) {
		solution_check_missed_customers(s);
		modification_apply(m);
		s->w = NULL;
		solution_check_missed_customers(s);
		assert(solution_feasible(s));
		if (options.log_level == LOGLEVEL_VERBOSE)
			debug_print("completed successfully", GREEN);
		return 0;
	}
	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print("failed", RED);
	return -1;
}

int
squeeze(struct solution *s)
{
	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print("started", RESET);

	assert(s->w != NULL);
	assert(is_ejected(s->w));
	assert(solution_feasible(s));
	struct solution *s_dup = solution_dup(s);
	assert(solution_find_customer_by_id(s, s->w->id) == s->w);
	assert(solution_find_customer_by_id(s_dup, s_dup->w->id) == s_dup->w);

	modification_apply(solution_find_optimal_insertion(s, s->w, eama_solver.alpha, eama_solver.beta));
	s->w = NULL;
	solution_check_missed_customers(s);
	assert(!solution_feasible(s));

	int infeasible = split_by_feasibility(s);
	while(infeasible > 0) {
		if (options.log_level == LOGLEVEL_VERBOSE) {
			debug_print(tt_sprintf("penalty_sum: %f",
				solution_penalty(s, eama_solver.alpha, eama_solver.beta)), RESET);
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

		double v_route_penalty = route_penalty(v_route, eama_solver.alpha, eama_solver.beta);
		struct modification opt_modification = modification_new(INSERT, NULL, NULL);
		double opt_delta = INFINITY;
		struct modification m;
		struct fiber *f = fiber_new(solution_modification_neighbourhood_f);
		fiber_start(f, s, v_route, options.n_near, &m);
		while (!fiber_is_dead(f)) {
			double delta = modification_delta(m, eama_solver.alpha, eama_solver.beta);
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

		if (options.log_level == LOGLEVEL_VERBOSE)
			debug_print(tt_sprintf("opt modification delta: %f", opt_delta), RESET);
		if (opt_delta > -EPS5) {
			//if (solution_penalty(s, 1., -1.) < 0.)
			//	eama.beta /= 0.99;
			//else
			//	eama.beta *= 0.99;
			if (options.log_level == LOGLEVEL_VERBOSE) {
				debug_print("failed", RED);
				debug_print(tt_sprintf("beta after correction: %0.12f", eama_solver.beta), RESET);
			}
			solution_move(s, s_dup);
			return -1;
		}
		solution_check_missed_customers(s);
		modification_apply(opt_modification);
		solution_check_missed_customers(s);
		infeasible = split_by_feasibility(s);
	}
	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print("completed successfully", GREEN);
	solution_delete(s_dup);
	assert(solution_feasible(s));
	return 0;
}

void
perturb(struct solution *s)
{

	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print("started", RESET);
	int n_modifications = 0;
	for (int i = 0; i < options.i_rand; i++) {
		struct customer *v = solution_find_customer_by_id(s,
			randint(1, p.n_customers - 1));
		if (is_ejected(v))
			continue;
		struct customer *w = solution_find_customer_by_id(s,
			randint(1, p.n_customers - 1));
		if (is_ejected(w))
			continue;
		if (v->route != w->route) {
			enum modification_type t = randint(0, EXCHANGE);
			struct modification m = modification_new(t, v, w);
			if (modification_applicable(m) &&
				modification_delta(m, 1., 1.) < EPS5) {
				modification_apply(m);
				++n_modifications;
			}
		}
	}
	if (options.log_level == LOGLEVEL_VERBOSE) {
		debug_print(tt_sprintf("applied %d modifications", n_modifications), RESET);
		debug_print("completed successfully", GREEN);
	}
}

int
insert_eject(struct solution *s)
{
	if (options.log_level == LOGLEVEL_VERBOSE)
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
			v_route, s->w, eama_solver.alpha, eama_solver.beta);
		if (!modification_applicable(m))
			continue;
		modification_apply(m);
		assert(!is_ejected(s->w));
		assert(!route_feasible(v_route));

		struct fiber *f = fiber_new(feasible_ejections_f);
		fiber_start(f, v_route, options.k_max, eama_solver.p, &ejection, &p_best);
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
	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print(tt_sprintf("opt insertion-ejection p_sum: %ld", p_best), RESET);

	if (unlikely(opt_insertion.v == NULL && rlist_empty(&opt_ejection))) {
		solution_move(s, s_dup);
		return -1;
	}

	{
		if (options.log_level == LOGLEVEL_VERBOSE) {
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
	solution_check_missed_customers(s);

	struct customer *c;
	rlist_foreach_entry(c, &opt_ejection, in_opt_eject) {
		struct modification m = modification_new(EJECT, c, NULL);
		assert(modification_applicable(m));
		modification_apply(m);
		rlist_add_tail_entry(&s->ejection_pool, c, in_eject);
	}

	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print("completed successfully", GREEN);
	solution_delete(s_dup);
	assert(solution_feasible(s));
	solution_check_missed_customers(s);
	return 0;
}

int
delete_route(struct solution *s)
{
	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print("started", RESET);

	assert(rlist_empty(&s->ejection_pool));
	assert(solution_feasible(s));
	struct solution *s_dup = solution_dup(s);
	solution_eliminate_random_route(s);
	assert(solution_feasible(s));
	solution_check_missed_customers(s);
	while (!rlist_empty(&s->ejection_pool)) {
		if (options.log_level == LOGLEVEL_VERBOSE) {
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

		solution_check_missed_customers(s);
		if (insert_feasible(s) == 0) {
			solution_check_missed_customers(s);
			continue;
		} else if (squeeze(s) == 0) {
			solution_check_missed_customers(s);
			continue;
		}
		assert(solution_find_customer_by_id(s, s->w->id) == s->w);
		++eama_solver.p[s->w->id];
		if (options.log_level == LOGLEVEL_VERBOSE)
			debug_print(tt_sprintf("p[%d] = %ld", s->w->id, eama_solver.p[s->w->id]), RESET);

		if (insert_eject(s) == 0) {
			solution_check_missed_customers(s);
			perturb(s);
			continue;
		}
		/** fail */
		if (options.log_level == LOGLEVEL_VERBOSE)
			debug_print("failed", RED);
		solution_move(s, s_dup);
		return -1;
	}
	if (options.log_level == LOGLEVEL_VERBOSE)
		debug_print("completed successfully", GREEN);
	solution_delete(s_dup);
	return 0;
}

struct solution *
eama_solver_solve(void)
{
	if (options.log_level >= LOGLEVEL_NORMAL)
		debug_print("started", RESET);

	eama_solver.alpha = eama_solver.beta = 1.;
	memset(&eama_solver.p[0], 0, sizeof(eama_solver.p));

	struct solution *s = solution_default();
	solution_check_missed_customers(s);
	int lower_bound = MAX(problem_routes_straight_lower_bound(),
			      options.lower_bound);
	while (s->n_routes > lower_bound) {
		if (options.log_level >= LOGLEVEL_NORMAL)
			debug_print(tt_sprintf("routes number: %d", s->n_routes), PURPLE);
		if (delete_route(s) != 0)
			break;
		assert(rlist_empty(&s->ejection_pool));
		assert(s->w == NULL);
		solution_check_missed_customers(s);
		assert(rlist_empty(&s->ejection_pool));
	}
	assert(s->w == NULL);
	assert(rlist_empty(&s->ejection_pool));
	if (options.log_level >= LOGLEVEL_NORMAL)
		debug_print("completed successfully", GREEN);
	return s;
}
