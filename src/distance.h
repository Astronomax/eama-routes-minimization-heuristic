#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_DISTANCE_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_DISTANCE_H

#define distance_attr	\
	double dist_pf;\
	double dist_sf

#include "customer.h"
//#include "route.h"
#include "utils.h"

struct route;

void
distance_init(struct route *r);

double
distance_get_distance(struct route *r);

double
distance_get_insert_distance(struct customer *v, struct customer *w);

double
distance_get_insert_delta(struct customer *v, struct customer *w);

double
distance_get_eject_distance(struct customer *v);

double
distance_get_eject_delta(struct customer *v);

double
distance_get_replace_distance(struct customer *v, struct customer *w);

double
distance_get_replace_delta(struct customer *v, struct customer *w);

double
distance_two_opt_distance_delta(struct customer *v, struct customer *w);

double
distance_out_relocate_distance_delta(struct customer *v, struct customer *w);

double
distance_exchange_distance_delta(struct customer *v, struct customer *w);

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_DISTANCE_H
