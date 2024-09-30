#include "route.h"

struct route *
route_new(void)
{
	struct route *r = xmalloc(sizeof(struct route)); //TODO: use mempool
	rlist_create(&r->list);
	rlist_create(&r->in_routes);
	return r;
}

void
route_init(struct route *r, struct customer **arr, int n)
{
	struct customer *c;
	c = customer_dup(p.depot);
	rlist_add_tail_entry(&r->list, c, in_route);
	c->route = r;
	for(int j = 0; j < n; j++) {
		c = arr[j];
		rlist_move_tail_entry(&r->list, c, in_route);
		c->route = r;
	}
	c = customer_dup(p.depot);
	rlist_add_tail_entry(&r->list, c, in_route);
	c->route = r;
	route_init_penalty(r);
	route_check(r);
}

void
route_init_penalty(struct route *r)
{
	c_penalty_init(r);
        tw_penalty_init(r);
        distance_init(r);
}

struct route *
route_dup(struct route *r)
{
	struct route *dup = xmalloc(sizeof(*dup));
	rlist_create(&dup->list);
	rlist_create(&dup->in_routes);
	struct customer *c;
	/** One mempool allocation for each customer in list */
	rlist_foreach_entry(c, &r->list, in_route) {
		struct customer *c_dup = customer_dup(c);
		rlist_add_tail_entry(&dup->list, c_dup, in_route);
		c_dup->route = dup;
	}
	assert(rlist_first_entry(&dup->list, struct customer, in_route)->id == 0);
	assert(rlist_last_entry(&dup->list, struct customer, in_route)->id == 0);
	route_check(r);
	return dup;
}

void
route_delete(struct route *r)
{
	struct customer *c, *tmp;
	rlist_foreach_entry_safe(c, &r->list, in_route, tmp) {
		rlist_del_entry(c, in_route);
		customer_delete(c);
	}
}

struct customer *
route_find_customer_by_id(struct route *r, int id)
{
	/** depot id */
	if (id == 0)
		return NULL;
	struct customer *c;
	rlist_foreach_entry(c, &r->list, in_route)
		if (id == c->id)
			return c;
	return NULL;
}

bool
route_feasible(struct route *r)
{
	return (tw_penalty_get_penalty(r) < EPS5 && c_penalty_get_penalty(r) < EPS5);
}

double
route_penalty(struct route *r, double alpha, double beta)
{
	return alpha * c_penalty_get_penalty(r) +
	       beta * tw_penalty_get_penalty(r);
}

int
route_len(struct route *r)
{
	int len = 0;
	struct customer *c;
	rlist_foreach_entry(c, &r->list, in_route)
		++len;
	return len - 2;
}

struct modification
route_find_optimal_insertion(struct route *r, struct customer *w,
				double alpha, double beta)
{
	assert(is_ejected(w));
	struct customer *v;
	struct modification opt_modification =
		modification_new(INSERT, NULL, NULL);
	double opt_penalty = INFINITY;
	rlist_foreach_entry(v, &r->list, in_route) {
		if (v == depot_head(r))
			continue;
		struct modification m = modification_new(INSERT, v, w);
		double penalty = modification_delta(m, alpha, beta);
		if (penalty < opt_penalty) {
			if (penalty < EPS5)
				return m;
			opt_modification = m;
			opt_penalty = penalty;
		}
	}
	return opt_modification;
}
