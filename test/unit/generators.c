#include "core/random.h"
#include "generators.h"

struct customer *
generate_random_customer(void)
{
	/** It doesn’t matter how to allocate customers in tests */
	struct customer *c = malloc(sizeof(*c));
	/** Uninitialized */
	c->id = -1;
	c->x = (double)real_random_in_range(0, 100);
	c->y = (double)real_random_in_range(0, 100);
	c->demand = (double)real_random_in_range(0, 10);
	c->e = (double)real_random_in_range(0, 50);
	c->l = c->e + (double)real_random_in_range(0, 50);
	c->s = (double)real_random_in_range(0, 20);
	//c->p = (int)real_random_in_range(0, 5);
	rlist_create(&c->in_route);
	return c;
}

/*
struct route *
generate_random_route(struct customer *depot, int max_customer_count)
{
	if (depot == NULL) {
		depot = generate_random_customer();
		depot->id = 0;
		depot->demand = depot->s = 0.;
		depot->route = NULL;
	}
	assert(depot->id == 0);
	assert(depot->demand == 0.);
	assert(depot->s == 0.);
	assert(depot->route == NULL);
	int len = (int)pseudo_random_in_range(10, 100);
	struct route *r = route_new();
	rlist_add_tail_entry(&r->list, depot, in_route);
	for (int i = 0; i < len; i++) {
		struct customer *c = generate_random_customer();
		c->route = r;
		rlist_add_tail_entry(&r->list, c, in_route);
	}
	rlist_add_tail_entry(&r->list, depot, in_route);
	return r;
}*/

void
generate_random_problem(int max_n_customers)
{
	rlist_create(&p.customers);
	p.vc = (double)pseudo_random_in_range(10, 10);
	p.depot = generate_random_customer();
	p.depot->id = 0;
	p.depot->demand = p.depot->s = 0.;
	assert(max_n_customers >= 2);
	p.n_customers =
		(int)pseudo_random_in_range(2, max_n_customers);
	for (int i = 0; i < p.n_customers; i++) {
		struct customer *c = generate_random_customer();
		c->id = i + 1;
		rlist_add_tail_entry(&p.customers, c, in_route);
	}
}
