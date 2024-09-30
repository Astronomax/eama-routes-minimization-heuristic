#include "modification.h"

#include "route.h"

#include "small_extra/rlist_extra.h"

struct modification
modification_new(enum modification_type type, struct customer *v, struct customer *w)
{
	struct modification m = {
		type, v, w,
		{false, 0., 0},
	};
	return m;
}

bool
modification_applicable(struct modification m)
{
	struct route *v_route, *w_route;
	v_route = m.v->route;
	/** Possible only in the case of EJECT */
	w_route = (m.w == NULL) ? NULL : m.w->route;
	switch (m.type) {
	case TWO_OPT:
		if (v_route == w_route)
			return false;
		if (depot_tail(m.v->route) == m.v)
			return false;
		if (depot_tail(m.w->route) == m.w)
			return false;
		return true;
	case OUT_RELOCATE:
		if (w_route == NULL)
			return false;
	case INSERT:
		if (m.w->id == 0)
			return false;
		if (v_route == NULL)
			return false;
		/**
		 * A customer can be inserted before a depot, but only
		 * before the one at the end of the path
		 */
		if (depot_head(v_route) == m.v)
			return false;
		return true;
	case EXCHANGE:
		if (m.v->id == 0 || m.w->id == 0)
			return false;
		if (v_route == NULL || w_route == NULL)
			return false;
		return true;
	case EJECT:
		if (m.v->id == 0)
			return false;
		if (v_route == NULL)
			return false;
		return true;
	default:
		unreachable();
	}
}

void
modification_apply(struct modification m)
{
	assert(modification_applicable(m));
	struct route *v_route, *w_route;
	v_route = m.v->route;
	/** Possible only in the case of EJECT */
	w_route = (m.w == NULL) ? NULL : m.w->route;
	switch (m.type) {
	case TWO_OPT: {
		rlist_swap_tails(&m.v->route->list,
				 &m.w->route->list,
				 &m.v->in_route,
				 &m.w->in_route);
		struct customer *c;
		for (c = rlist_next_entry(m.w, in_route);
		     !rlist_entry_is_head(c, &w_route->list, in_route);
		     c = rlist_next_entry(c, in_route))
			c->route = w_route;
		for (c = rlist_next_entry(m.v, in_route);
		     !rlist_entry_is_head(c, &v_route->list, in_route);
		     c = rlist_next_entry(c, in_route))
			c->route = v_route;
		break;
	}
	case OUT_RELOCATE: {
		if (m.v == m.w)
			return;
		rlist_move_entry(
			rlist_prev(&m.v->in_route), m.w, in_route);
		m.w->route = v_route;
		break;
	}
	case EXCHANGE: {
		if (m.v == m.w)
			return;
		rlist_swap_items(&m.v->in_route, &m.w->in_route);
		SWAP(m.v->route, m.w->route);
		break;
	}
	case INSERT: {
		rlist_move_entry(
			rlist_prev(&m.v->in_route), m.w, in_route);
		m.w->route = v_route;
		route_init_penalty(v_route);
		route_check(v_route);
		/**
		 * This will probably never be true.
		 * Most likely INSERT will always be used only after EJECT.
		 */
		if (unlikely(w_route != NULL)) {
			route_init_penalty(w_route);
			route_check(w_route);
		}
		return;
	}
	case EJECT: {
		rlist_del_entry(m.v, in_route);
		m.v->route = NULL;
		route_init_penalty(v_route);
		route_check(v_route);
		return;
	}
	default:
		unreachable();
	}
	route_init_penalty(v_route);
	route_init_penalty(w_route);
	route_check(v_route);
	route_check(w_route);
}

double
modification_delta(struct modification m, double alpha, double beta)
{
	assert(modification_applicable(m));
	if (!m.delta_initialized) {
		switch (m.type) {
			case TWO_OPT:
				m.tw_penalty_delta = tw_penalty_two_opt_penalty_delta(m.v, m.w);
				m.c_penalty_delta = c_penalty_two_opt_penalty_delta(m.v, m.w);
				break;
			case OUT_RELOCATE:
				m.tw_penalty_delta = tw_penalty_out_relocate_penalty_delta(m.v, m.w);
				m.c_penalty_delta = c_penalty_out_relocate_penalty_delta(m.v, m.w);
				break;
			case EXCHANGE:
				m.tw_penalty_delta = tw_penalty_exchange_penalty_delta(m.v, m.w);
				m.c_penalty_delta = c_penalty_exchange_penalty_delta(m.v, m.w);
				break;
			case INSERT:
				m.tw_penalty_delta = tw_penalty_get_insert_delta(m.v, m.w);
				m.c_penalty_delta = c_penalty_get_insert_delta(m.v, m.w);
				break;
			case EJECT:
				m.tw_penalty_delta = tw_penalty_get_eject_delta(m.v);
				m.c_penalty_delta = c_penalty_get_eject_delta(m.v);
				break;
			default:
				unreachable();
		}
		m.delta_initialized = true;
	}
	return alpha * m.c_penalty_delta + beta * m.tw_penalty_delta;
}
