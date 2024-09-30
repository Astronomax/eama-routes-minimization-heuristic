#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_ROUTE_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_ROUTE_H

#include "small/rlist.h"

#include "customer.h"
#include "modification.h"
#include "utils.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

struct route {
    /**
     * List of customers. Contract:
     * rlist_first_entry(&list, in_route) ==
     * rlist_last_entry(&list, in_route) == p->depot
     */
    struct rlist list;
    struct rlist in_routes;
};

#define route_foreach(c, r) rlist_foreach_entry(c, &r->list, in_route)

static ALWAYS_INLINE void
route_check(struct route *r)
{
	(void)r;
#ifndef NDEBUG
	assert(rlist_first_entry(&r->list, struct customer, in_route)->id == 0);
	assert(rlist_last_entry(&r->list, struct customer, in_route)->id == 0);
	struct customer *c;
	route_foreach(c, r) assert(c->route == r);
#endif
}

#define route_foreach_from(c, from)  \
        for (c = from; !rlist_entry_is_head(c, &c->route->list, in_route); \
        c = rlist_next_entry(c, in_route))

#define depot_head(r) rlist_first_entry(&r->list, struct customer, in_route)
#define depot_tail(r) rlist_last_entry(&r->list, struct customer, in_route)

#define route_prev(c) rlist_prev_entry(c, in_route)
#define route_next(c) rlist_next_entry(c, in_route)

struct route *
route_new(void);

void
route_init(struct route *r, struct customer **arr, int n);

void
route_init_penalty(struct route *r);

struct route *
route_dup(struct route *r);

void
route_delete(struct route *r);

struct customer *
route_find_customer_by_id(struct route *r, int id);

bool
route_feasible(struct route *r);

double
route_penalty(struct route *r, double alpha, double beta);

int
route_len(struct route *r);

struct modification
route_find_optimal_insertion(struct route *r, struct customer *w,
			     double alpha, double beta);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_ROUTE_H
