#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_INSERT_EJECT_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_INSERT_EJECT_H

#include <stdarg.h>

#define feasible_ejections_attr	\
	/**
	 * We don't want the customer::a calculated in tw_init_penalty to break
	 * while iterating through feasible ejections.
	 */				\
	double a_temp;			\
	double a_quote

/**
 * @brief Iterate over feasible ejections (a subsets of route customers such
 * that if they were ejected, this route would not violate the constraints, i.e.
 * tw_penalty and c_penalty equals to zero). Iteration occurs in lexicographic
 * order and only subsets of size no more than \a k_max are considered. In this
 * case, the next ejection is returned only if the sum of p in it is less than
 * the current minimum (p_best).
 *
 * @param r 		route
 * @param k_max		maximum subset size
 * @param e		ejection (must be initially passed empty)
 * @param p_best	current minimum sum of p
 */
int
feasible_ejections_f(va_list ap);

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_INSERT_EJECT_H
