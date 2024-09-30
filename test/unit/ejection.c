#include "generators.h"

#include "core/fiber.h"
#include "core/memory.h"
#include "core/random.h"

#include "small_extra/rlist_persistent.h"

#define MAX_N_CUSTOMERS_TEST 10

struct customer *cs[MAX_N_CUSTOMERS_TEST + 2];

#define randint (int)pseudo_random_in_range

struct subset_elem {
    int idx;
    struct rlist in_list;
};

/**
 * @brief Iterate over subsets of size no more than \a k_max in lexicographic
 * order.
 *
 * @param n 	set size
 * @param k_max maximum subset size
 * @param e	subset (must be initially passed empty)
 */
int
iterate_over_subsets_f(va_list ap)
{
	int n = va_arg(ap, int);
	int k_max = va_arg(ap, int);
	struct rlist *e = va_arg(ap, struct rlist *);

	struct rlist ne, s;
	rlist_create(&ne);
	rlist_create(&s);
	/** Will be initialized after incr_last */
	struct subset_elem *ne_last = NULL;
	struct subset_elem *tmp = NULL;
	for (int i = 0; i < n; i++) {
		tmp = malloc(sizeof(*tmp));
		tmp->idx = i;
		rlist_add_tail_entry(&s, tmp, in_list);
	}
	int k = 0;

	/** Will be initialized after incr_k */
	struct subset_elem *e_last = NULL;
	struct subset_elem *s_first = rlist_first_entry(&s, struct subset_elem, in_list);

	goto incr_k;
	for(;;) {
		fiber_yield();

		if (!rlist_empty(&s) && k < k_max)
			goto incr_k;
		if (!rlist_empty(&s)) goto incr_last;

	/** backtrack */
		if (unlikely(k == 1)) return 0;

		assert(e_last == rlist_last_entry(e, struct subset_elem, in_list));
		s_first = e_last;
		e_last = rlist_prev_entry(e_last, in_list);
		rlist_move_entry(&s, s_first, in_list);
		rlist_foreach_entry_safe_reverse(ne_last, &ne, in_list, tmp) {
			if (ne_last->idx <= e_last->idx) break;
			/**
			 * Let's ignore that s_first becomes irrelevant.
			 * We will update it once later.
			 */
			rlist_move_entry(&s, ne_last, in_list);
		}
		s_first = rlist_first_entry(&s, struct subset_elem, in_list);
		--k;
	incr_last:
		assert(e_last == rlist_last_entry(e, struct subset_elem, in_list));
		ne_last = e_last;
		//e_last = rlist_prev_entry(e_last, in_list);
		rlist_move_tail_entry(&ne, ne_last, in_list);
		--k;
	incr_k:
		assert(s_first == rlist_first_entry(&s, struct subset_elem, in_list));
		e_last = s_first;
		s_first = rlist_next_entry(s_first, in_list);
		rlist_move_tail_entry(e, e_last, in_list);
		++k;
	}
}

void
ejections_random_route(int n_tests)
{
	memory_init();
	fiber_init(fiber_c_invoke);

	int64_t ps[MAX_N_CUSTOMERS_TEST];

	for (int i = 0; i < n_tests; i++) {
		generate_random_problem(MAX_N_CUSTOMERS_TEST);

		for (int j = 0; j < MAX_N_CUSTOMERS_TEST; j++)
			ps[j] = randint(0, 5);

		struct route *route = route_new();
		struct customer *w;
		int j = 0;
		rlist_foreach_entry(w, &p.customers, in_route) {
			if (w->id == 0) continue;
			cs[j++] = w;
		}
		route_init(route, &cs[0], p.n_customers);

		int64_t p_best_act = INT64_MAX,
			p_best_exp = INT64_MAX;
		struct rlist ejection_act, ejection_idx_exp;
		rlist_create(&ejection_act);
		rlist_create(&ejection_idx_exp);

		struct fiber *f1 = fiber_new(iterate_over_subsets_f),
			*f2 = fiber_new(feasible_ejections_f);

		fiber_start(f1, p.n_customers, 5, &ejection_idx_exp);
		fiber_start(f2, route, 5, &ps[0], &ejection_act, &p_best_act);

		rlist_persistent_history history;
		rlist_create(&history);
		rlist_persistent_svp svp = rlist_persistent_create_svp(&history);
		
		while(!fiber_is_dead(f1)) {
			struct subset_elem *idx;
			rlist_foreach_entry(idx, &ejection_idx_exp, in_list)
				rlist_persistent_del_entry(&history, cs[idx->idx], in_route);
			route_init_penalty(route);
			double p_c = c_penalty_get_penalty(route),
				p_tw = tw_penalty_get_penalty(route);
			rlist_persistent_rollback_to_svp(&history, svp);
			route_init_penalty(route);

			if (p_c < EPS5 && p_tw < EPS5) {
				int64_t p_sum = 0;
				rlist_foreach_entry(idx, &ejection_idx_exp, in_list)
					p_sum += ps[cs[idx->idx]->id];
				if (p_sum < p_best_exp) {
					p_best_exp = p_sum;
					assert(p_best_act == p_best_exp);

					w = rlist_first_entry(&ejection_act, struct customer, in_eject);
					rlist_foreach_entry(idx, &ejection_idx_exp, in_list) {
						assert(w->idx == idx->idx + 1);
						w = rlist_next_entry(w, in_eject);
					}
					assert(rlist_entry_is_head(w, &ejection_act, in_eject));

					assert(!fiber_is_dead(f2));
					fiber_call(f2);
				}
			}
			fiber_call(f1);
		}
	}
	memory_free();
}

int
main(void)
{
	random_init();
	ejections_random_route(100);
	return 0;
}
