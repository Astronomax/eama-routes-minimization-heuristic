#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_EAMA_SOLVER_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_EAMA_SOLVER_H

#include "customer.h"
#include "problem.h"
#include "solution.h"
#include "modification.h"

struct eama_settings {
	int n_near;
	int k_max;
	int t_max;
	int i_rand;
	int lower_bound;
};

struct eama {
    struct eama_settings settings;
    double alpha;
    double beta;
    int64_t p[MAX_N_CUSTOMERS];
};

struct eama *
eama_new(struct eama_settings settings);

void
eama_delete(struct eama *eama);

/** determine the minimum possible number of routes */
struct solution *
eama_solver_solve(struct eama *eama, bool debug);

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_EAMA_SOLVER_H
