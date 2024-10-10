#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_SOLUTION_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_SOLUTION_H

#include "small/rlist.h"

#include "customer.h"
#include "modification.h"
#include "random_utils.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

struct solution_meta;

struct solution {
    	struct customer *w;
    	struct solution_meta *meta;
	struct rlist ejection_pool;
	int n_routes;
	struct route *routes[0];
};

void
solution_global_init();

int
solution_modification_neighbourhood_f(va_list ap);

struct solution *
solution_default(void);

struct solution *
solution_dup(struct solution *s);

void
solution_move(struct solution *dst, struct solution *src);

void
solution_delete(struct solution *s);

double
solution_penalty(struct solution *s, double alpha, double beta);

bool
solution_feasible(struct solution *s);

int
split_by_feasibility(struct solution *s);

void
solution_print(struct solution *s);

void
solution_check(struct solution *s);

void
_solution_check(struct solution *s);

struct modification
solution_find_feasible_insertion(struct solution *s, struct customer *w);

struct modification
solution_find_optimal_insertion(struct solution *s, struct customer *w,
				double alpha, double beta);

struct customer *
solution_find_customer_by_id(struct solution *s, int id);

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_SOLUTION_H
