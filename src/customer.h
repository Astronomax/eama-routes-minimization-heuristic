#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_CUSTOMER_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_CUSTOMER_H

#include "math.h"

#include "small/rlist.h"
#include "tw_penalty.h"
#include "c_penalty.h"
#include "ejection.h"
#include "distance.h"

struct route;

struct customer {
	int id;
	double x;
	double y;
	double demand;
	double e;
	double l;
	double s;
	feasible_ejections_attr;
	tw_penalty_attr;
	c_penalty_attr;
	distance_attr;
	struct route *route;
	struct rlist in_route;
    	struct rlist in_eject;
	struct rlist in_opt_eject;
};

#define is_ejected(c) rlist_empty(&c->in_route)

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

struct customer *
customer_dup(struct customer *c);

void
customer_delete(struct customer *c);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_CUSTOMER_H
