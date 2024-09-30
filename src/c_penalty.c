#include "c_penalty.h"

#include "assert.h"

#include "modification.h"
#include "penalty_common.h"
#include "problem.h"
#include "route.h"
#include "utils.h"

void
c_penalty_init(struct route *r)
{
	struct customer *next;
	struct customer *prev = depot_head(r);
	prev->demand_pf = prev->demand;
	for (next = rlist_next_entry(prev, in_route);
	     !rlist_entry_is_head(next, &r->list, in_route);
	     next = rlist_next_entry(next, in_route)) {
		next->demand_pf =
			prev->demand_pf + next->demand;
		prev = next;
	}
	next = depot_tail(r);
	next->demand_sf = next->demand;
	for (prev = rlist_prev_entry(next, in_route);
	     !rlist_entry_is_head(prev, &r->list, in_route);
	     prev = rlist_prev_entry(prev, in_route)) {
		prev->demand_sf =
			next->demand_sf + prev->demand;
		next = prev;
	}
}

double
c_penalty_get_penalty(struct route *r)
{
	return MAX(0., depot_tail(r)->demand_pf - p.vc);
}

double
c_penalty_get_insert_penalty(struct customer *v, struct customer *w)
{
	struct route *r = v->route;
	assert(r != NULL);
	return MAX(0., depot_tail(r)->demand_pf + w->demand - p.vc);
}

double
c_penalty_get_insert_delta(struct customer *v, struct customer *w)
{
	return c_penalty_get_insert_penalty(v, w)
	       - c_penalty_get_penalty(v->route);
}

double
c_penalty_get_replace_penalty(struct customer *v, struct customer *w)
{
	struct route *r = v->route;
	assert(r != NULL);
	return MAX(0., depot_tail(r)->demand_pf - v->demand + w->demand
		- p.vc);
}

double
c_penalty_get_replace_delta(struct customer *v, struct customer *w)
{
	return c_penalty_get_replace_penalty(v, w)
	       - c_penalty_get_penalty(v->route);
}

double
c_penalty_get_eject_penalty(struct customer *v)
{
	struct route *r = v->route;
	assert(r != NULL);
	return MAX(0., depot_tail(r)->demand_pf - v->demand - p.vc);
}

double
c_penalty_get_eject_delta(struct customer *v)
{
	return c_penalty_get_eject_penalty(v)
	       - c_penalty_get_penalty(v->route);
}

/** This probably doesn't make sense for c_penalty and will never be used */
penalty_common_modification_apply_straight_delta(c)

double
c_penalty_one_opt_penalty(struct customer *v, struct customer *w)
{
	assert(v->route != w->route);
	struct customer *w_plus = rlist_next_entry(w, in_route);
	return MAX(0., v->demand_pf + w_plus->demand_sf - p.vc);
}

double
c_penalty_two_opt_penalty_delta(struct customer *v, struct customer *w)
{
	assert(v->route != w->route);
	return (c_penalty_one_opt_penalty(v, w)
		- c_penalty_get_penalty(v->route))
		+ (c_penalty_one_opt_penalty(w, v)
		- c_penalty_get_penalty(w->route));
}

double
c_penalty_out_relocate_penalty_delta(struct customer *v, struct customer *w)
{
	/** This will probably never be called */
	if (unlikely(v->route == w->route))
		return 0.f;
	else {
		return c_penalty_get_eject_delta(w)
		       + c_penalty_get_insert_delta(v, w);
	}
}

double
c_penalty_exchange_penalty_delta(struct customer *v, struct customer *w)
{
	/** This will probably never be called */
	if (unlikely(v->route == w->route))
		return 0.f;
	else {
		return c_penalty_get_replace_delta(v, w)
		       + c_penalty_get_replace_delta(w, v);
	}
}
