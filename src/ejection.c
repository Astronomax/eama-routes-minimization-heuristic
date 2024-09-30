#include "ejection.h"

#include "core/fiber.h"

#include "dist.h"
#include "route.h"

int
feasible_ejections_f(va_list ap)
{
	struct route *r = va_arg(ap, struct route *);
	int k_max = va_arg(ap, int);
	int64_t *ps = va_arg(ap, int64_t *);
	struct rlist *e = va_arg(ap, struct rlist *);
	int64_t *p_best = va_arg(ap, int64_t *);

	struct rlist ne, s;
	rlist_create(&ne);
	rlist_create(&s);
	struct customer *ne_last = depot_head(r);
	rlist_add_tail_entry(&ne, ne_last, in_eject);
	ne_last->a_temp = ne_last->a;
	struct customer *tmp;
	for(tmp = rlist_next_entry(ne_last, in_route);
	    !rlist_entry_is_head(tmp, &r->list, in_route);
	    tmp = rlist_next_entry(tmp, in_route))
		rlist_add_tail_entry(&s, tmp, in_eject);

	double total_demand;
	total_demand = depot_tail(r)->demand_pf;
	int64_t p_sum = 0;
	int k = 0;

	/** Will be initialized after incr_k */
	struct customer *e_last;
	struct customer *s_first = rlist_first_entry(&s, struct customer, in_eject);

	//printf("route len: %d\n", route_len(r));
	//fflush(stdout);

	goto incr_k;
	for(;;) {
		if (/** Is better than current optimum */
		    p_sum < *p_best &&
		    /** Doesn't violate time-window constraint */
		    s_first->a_quote <= s_first->l && s_first->a_temp <= s_first->z && s_first->tw_sf == 0. &&
		    /** Doesn't violate capacity constraint */
		    total_demand <= p.vc) {
			*p_best = p_sum;
			fiber_yield();
		}

		if (p_sum < *p_best && s_first->id != 0 && k < k_max)
			goto incr_k;
		if (s_first->id != 0) goto incr_last;

		do {
		/** backtrack */
			//if (unlikely(k == 1)) return 0;

			assert(e_last == rlist_last_entry(e, struct customer, in_eject));
			s_first = e_last;
			e_last = rlist_prev_entry(e_last, in_eject);
			rlist_move_entry(&s, s_first, in_eject);

			if (unlikely(k == 1)) return 0;

			p_sum -= ps[s_first->id];//p_sum -= s_first->p;
			total_demand += s_first->demand;
			rlist_foreach_entry_safe_reverse(ne_last, &ne, in_eject, tmp) {
				if (ne_last->idx <= e_last->idx) break;
				/**
				 * Let's ignore that s_first becomes irrelevant.
				 * We will update it once later.
				 */
				rlist_move_entry(&s, ne_last, in_eject);
			}
			s_first = rlist_first_entry(&s, struct customer, in_eject);
			--k;
		incr_last:
			assert(e_last == rlist_last_entry(e, struct customer, in_eject));
			ne_last = e_last;
			//e_last = rlist_prev_entry(e_last, in_eject);
			rlist_move_tail_entry(&ne, ne_last, in_eject);
			p_sum -= ps[ne_last->id];//p_sum -= ne_last->p;
			total_demand += ne_last->demand;
			--k;
		incr_k:
			assert(s_first == rlist_first_entry(&s, struct customer, in_eject));
			e_last = s_first;
			s_first = rlist_next_entry(s_first, in_eject);
			rlist_move_tail_entry(e, e_last, in_eject);
			p_sum += ps[e_last->id];//p_sum += e_last->p;
			total_demand -= e_last->demand;
			++k;
		/** update */
			assert(e_last == rlist_last_entry(e, struct customer, in_eject));
			assert(ne_last == rlist_last_entry(&ne, struct customer, in_eject));
			assert(s_first == rlist_first_entry(&s, struct customer, in_eject));
			e_last->a_quote = ne_last->a_temp + ne_last->s + dist(ne_last, e_last);
			e_last->a_temp = MIN(MAX(e_last->a_quote, e_last->e), e_last->l);
			s_first->a_quote = ne_last->a_temp + ne_last->s + dist(ne_last, s_first);
			s_first->a_temp = MIN(MAX(s_first->a_quote, s_first->e), s_first->l);
		} while (ne_last->l < ne_last->a_quote);
	}
}
