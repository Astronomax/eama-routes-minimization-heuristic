#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_PENALTY_COMMON_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_PENALTY_COMMON_H

#define penalty_common_modification_apply_straight_delta(prefix)		\
double										\
prefix##_penalty_modification_apply_straight_delta(struct modification m)	\
{	assert(false);									\
	if (m.v->route == m.w->route) {						\
		struct route *r = route_dup(m.v->route);			\
		m.v = route_find_customer_by_id(r, m.v->id);			\
		m.w = route_find_customer_by_id(r, m.w->id);			\
		assert(modification_applicable(m) == 0);			\
		double p_before = prefix##_penalty_get_penalty(r);		\
		modification_apply(m);						\
		prefix##_penalty_init(r);/** TODO: update only what is needed */\
		double p_delta = prefix##_penalty_get_penalty(r) - p_before;	\
		route_delete(r);							\
		return p_delta;							\
	} else {								\
		struct route *v_route = route_dup(m.v->route);			\
		struct route *w_route = route_dup(m.w->route);			\
		m.v = route_find_customer_by_id(v_route, m.v->id);		\
		m.w = route_find_customer_by_id(w_route, m.w->id);		\
		assert(modification_applicable(m) == 0);			\
		double p_before = prefix##_penalty_get_penalty(v_route)		\
				     + prefix##_penalty_get_penalty(w_route);	\
		modification_apply(m);						\
		prefix##_penalty_init(v_route);/** TODO: update only what is needed */\
		prefix##_penalty_init(w_route);/** TODO: update only what is needed */\
		double p_delta = prefix##_penalty_get_penalty(v_route)		\
				    + prefix##_penalty_get_penalty(w_route)	\
				    - p_before;					\
		route_delete(v_route);						\
		route_delete(w_route);						\
		return p_delta;							\
	}									\
}

//TODO: move all copy-pasted commons from tw_penalty and c_penalty here

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_PENALTY_COMMON_H
