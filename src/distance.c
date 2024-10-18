#include "distance.h"

#include "assert.h"

#include "dist.h"
#include "route.h"

void
distance_init(struct route *r)
{
	struct customer *next;
	struct customer *prev = depot_head(r);
	prev->dist_pf = 0.;
	for (next = rlist_next_entry(prev, in_route);
	     !rlist_entry_is_head(next, &r->list, in_route);
	     next = rlist_next_entry(next, in_route)) {
		next->dist_pf = prev->dist_pf + dist(prev, next);
		prev = next;
	}
	next = depot_tail(r);
	next->dist_sf = 0.;
	for (prev = rlist_prev_entry(next, in_route);
	     !rlist_entry_is_head(prev, &r->list, in_route);
	     prev = rlist_prev_entry(prev, in_route)) {
		prev->dist_sf = next->dist_sf + dist(prev, next);
		next = prev;
	}
}

double
distance_get_distance(struct route *r)
{
	return depot_tail(r)->dist_pf;
}

double
distance_get_insert_distance(struct customer *v, struct customer *w)
{
	struct customer *v_minus = rlist_prev_entry(v, in_route);
	return v_minus->dist_pf + dist(v_minus, w) + dist(w, v) + v->dist_sf;
}

double
distance_get_insert_delta(struct customer *v, struct customer *w)
{
	struct customer *v_minus = rlist_prev_entry(v, in_route);
	return dist(w, v) + dist(v_minus, w)  - dist(v_minus, v);
}

double
distance_get_eject_distance(struct customer *v)
{
	struct customer *v_minus = rlist_prev_entry(v, in_route);
	struct customer *v_plus = rlist_next_entry(v, in_route);
	return v_minus->dist_pf + dist(v_minus, v_plus) + v_plus->dist_sf;
}

double
distance_get_eject_delta(struct customer *v)
{
	struct customer *v_minus = rlist_prev_entry(v, in_route);
	struct customer *v_plus = rlist_next_entry(v, in_route);
	return dist(v_minus, v_plus) - (dist(v_minus, v) + dist(v, v_plus));
}

double
distance_get_replace_distance(struct customer *v, struct customer *w)
{
	struct customer *v_minus = rlist_prev_entry(v, in_route);
	struct customer *v_plus = rlist_next_entry(v, in_route);
	return v_minus->dist_pf + dist(v_minus, w) + dist(w, v_plus) + v_plus->dist_sf;
}

double
distance_get_replace_delta(struct customer *v, struct customer *w)
{
	struct customer *v_minus = rlist_prev_entry(v, in_route);
	struct customer *v_plus = rlist_next_entry(v, in_route);
	return (dist(v_minus, w) + dist(w, v_plus))
		- (dist(v_minus, v) + dist(v, v_plus));
}

double
distance_one_opt_distance(struct customer *v, struct customer *w)
{
	struct customer *w_plus = rlist_next_entry(w, in_route);
	return v->dist_pf + dist(v, w_plus) + w_plus->dist_sf;
}

double
distance_two_opt_distance_delta(struct customer *v, struct customer *w)
{
	assert(v->route != w->route);
	return (distance_one_opt_distance(v, w)
		- distance_get_distance(v->route))
		+ (distance_one_opt_distance(w, v)
		- distance_get_distance(w->route));
}

double
distance_out_relocate_distance_delta(struct customer *v, struct customer *w)
{
	struct customer *w_plus = rlist_next_entry(w, in_route);
	if (w_plus == v)
		return 0.;
	return distance_get_eject_delta(w) + distance_get_insert_delta(v, w);
}

double
distance_exchange_distance_delta(struct customer *v, struct customer *w)
{
	struct customer *v_minus = rlist_prev_entry(v, in_route);
	struct customer *v_plus = rlist_next_entry(v, in_route);
	struct customer *w_minus = rlist_prev_entry(w, in_route);
	struct customer *w_plus = rlist_next_entry(w, in_route);
	if (v_plus == w)
		return dist(v_minus, w) + dist(v, w_plus)
			- (dist(v_minus, v) + dist(w, w_plus));
	else if (w_plus == v)
		return dist(w_minus, v) + dist(w, v_plus)
			- (dist(w_minus, w) + dist(v, v_plus));
	return distance_get_replace_delta(v, w)
		+ distance_get_replace_delta(w, v);
}
