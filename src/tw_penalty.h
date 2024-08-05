#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_TW_PENALTY_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_TW_PENALTY_H

#include "small/rlist.h"

#define tw_penalty_attr \
    double a;		\
    double tw_pf;	\
    double z;		\
    double tw_sf

#include "customer.h"
#include "route.h"

void
tw_penalty_init(struct route *r);

double
tw_penalty_get_penalty(struct route *r);

double
tw_penalty_get_insert_penalty(struct customer *v, struct customer *w);

double
tw_penalty_get_insert_delta(struct customer *v, struct customer *w);

double
tw_penalty_get_replace_penalty(struct customer *v, struct customer *w);

double
tw_penalty_get_replace_delta(struct customer *v, struct customer *w);

double
tw_penalty_get_eject_penalty(struct customer *v);

double
tw_penalty_get_eject_delta(struct customer *v);

#ifdef TW_PENALTY_TEST
#include "modification.h"

double
tw_penalty_modification_apply_straight_delta(struct modification m);
#endif

#ifdef TW_PENALTY_TEST
double
tw_penalty_one_opt_penalty(struct customer *v, struct customer *w);
#endif

double
tw_penalty_two_opt_penalty_delta(struct customer *v, struct customer *w);

double
tw_penalty_out_relocate_penalty_delta_slow(struct customer *v, struct customer *w);

double
tw_penalty_out_relocate_penalty_delta_fast(struct customer *v, struct customer *w);

double
tw_penalty_out_relocate_penalty_delta(struct customer *v, struct customer *w);

double
tw_penalty_exchange_penalty_delta_slow(struct customer *v, struct customer *w);

double
tw_penalty_exchange_penalty_delta_fast(struct customer *v, struct customer *w);

double
tw_penalty_exchange_penalty_delta(struct customer *v, struct customer *w);


#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_TW_PENALTY_H
