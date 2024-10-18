#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_EAMA_SOLVER_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_EAMA_SOLVER_H

#include "customer.h"
#include "problem.h"
#include "solution.h"
#include "modification.h"

struct eama_solver {
    double alpha;
    double beta;
    int64_t p[MAX_N_CUSTOMERS];
};

extern struct eama_solver eama_solver;

/** determine the minimum possible number of routes */
struct solution *
eama_solver_solve(void);

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_EAMA_SOLVER_H
