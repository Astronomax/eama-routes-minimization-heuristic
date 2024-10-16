#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_RANDOM_UTILS_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_RANDOM_UTILS_H

#include "customer.h"

#include "core/random.h"

#define randint (int)pseudo_random_in_range

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

void
random_subset(struct customer **arr, int n, int k);

void
random_shuffle(struct customer **arr, int n);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_RANDOM_UTILS_H
