#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_PROBLEM_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_PROBLEM_H

#include "customer.h"
#include "small/rlist.h"

#define MAX_N_CUSTOMERS 2000

struct problem {
	double vc;
	struct customer *depot;
	struct rlist customers;
	int n_customers;
};

extern struct problem p;

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

void
problem_customers_dup(struct rlist *list);

int
problem_routes_straight_lower_bound(void);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_PROBLEM_H
