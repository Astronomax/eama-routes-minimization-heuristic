#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_MODIFICATION_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_MODIFICATION_H

#include "customer.h"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

enum modification_type {
    TWO_OPT,
    OUT_RELOCATE,
    EXCHANGE,
    INSERT,
    EJECT,
    modification_max
};

struct modification {
    enum modification_type type;
    struct customer *v;
    /** Used only in TWO_OPT, OUT_RELOCATE, EXCHANGE, INSERT */
    struct customer *w;
    struct {
	bool delta_initialized;
	double tw_penalty_delta;
	double c_penalty_delta;
    };
};

struct modification
modification_new(enum modification_type type, struct customer *v, struct customer *w);

bool
modification_applicable(struct modification m);

void
modification_apply(struct modification m);

double
modification_delta(struct modification m, double alpha, double beta);

#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_MODIFICATION_H
