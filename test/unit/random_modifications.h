#ifndef EAMA_ROUTES_MINIMIZATION_HEURISTIC_RANDOM_MODIFICATIONS_H
#define EAMA_ROUTES_MINIMIZATION_HEURISTIC_RANDOM_MODIFICATIONS_H

#include "unit.h"

#include "core/random.h"

#include "generators.h"
#include "modification.h"
#include "problem.h"

#define MAX_N_CUSTOMERS_TEST 100

#define modify(modification_type, v, w) do {			\
        struct modification m = {modification_type, (v), (w)};	\
        assert(modification_applicable(m));			\
        modification_apply(m);					\
} while(0)

struct customer *cs[MAX_N_CUSTOMERS_TEST + 2];

#define randint (int)pseudo_random_in_range

void
random_insertions(int n_tests)
{
	for (int i = 0; i < n_tests; i++) {
		generate_random_problem(MAX_N_CUSTOMERS_TEST);
		struct route *route = route_new();
		int route_len = randint(1, p.n_customers / 2);
		struct customer *w;

		int j = 0;
		rlist_foreach_entry(w, &p.customers, in_route) {
			/** Skip route_len to put the depot there */
			if (j == route_len) ++j;
			cs[j++] = w;
		}
		route_init(route, &cs[0], route_len);
		cs[route_len] = depot_tail(route);

		/** Insert the remaining customers into random places */
		while (route_len < p.n_customers) {
			w = cs[route_len + 1];
			j = randint(0, route_len);
			struct customer *v = cs[j];
			RANDOM_MODIFICATIONS_BEFORE_INSERT;
			modify(INSERT, v, w);
			++route_len;
			RANDOM_MODIFICATIONS_AFTER_INSERT;
		}
	}
}

void
random_ejections(int n_tests)
{
	for (int i = 0; i < n_tests; i++) {
		generate_random_problem(MAX_N_CUSTOMERS_TEST);
		struct route *route = route_new();
		struct customer *v;
		int j = 0;
		rlist_foreach_entry(v, &p.customers, in_route)
			cs[j++] = v;
		route_init(route, &cs[0], p.n_customers);

		for (j = 0; j < p.n_customers; j++) {
			int k = randint(j, p.n_customers - 1);
			SWAP(cs[j], cs[k]);
			v = cs[j];
			RANDOM_MODIFICATIONS_BEFORE_EJECT;
			modify(EJECT, v, NULL);
			RANDOM_MODIFICATIONS_AFTER_EJECT;
		}
	}
}

void
random_replacements(int n_tests)
{
	for (int i = 0; i < n_tests; i++) {
		generate_random_problem(MAX_N_CUSTOMERS_TEST);
		struct route *route = route_new();
		int route_len = randint(1, p.n_customers / 2);
		struct customer *w;
		int j = 0;
		rlist_foreach_entry(w, &p.customers, in_route)
			cs[j++] = w;
		route_init(route, &cs[0], route_len);

		for (j = route_len; j < p.n_customers; j++) {
			w = cs[j];
			int k = randint(0, route_len - 1);
			struct customer *v = cs[k];
			RANDOM_MODIFICATIONS_BEFORE_REPLACE;
			modify(INSERT, v, w);
			modify(EJECT, v, w);
			SWAP(cs[j], cs[k]);
			RANDOM_MODIFICATIONS_AFTER_REPLACE;
		}
	}
}

void
random_two_opts(int n_tests)
{
	for (int i = 0; i < n_tests; i++) {
		generate_random_problem(MAX_N_CUSTOMERS_TEST);
		struct route *v_route = route_new();
		struct route *w_route = route_new();
		int v_route_len = randint(1, p.n_customers / 2);
		int w_route_len = p.n_customers - v_route_len;
		struct customer *w;
		/** Skip the first element to put the depot there */
		int j = 1;
		rlist_foreach_entry(w, &p.customers, in_route)
			cs[j++] = w;
		route_init(v_route, &cs[1], v_route_len);
		cs[0] = depot_head(v_route);
		route_init(w_route, &cs[v_route_len + 1], w_route_len);
		/**
		 * depot, v1, v2, ..., v_{v_route_len},
		 * w_{w_route_len}, ..., w_2, w_1, depot
		 * This is convenient for managing sets of customers in paths
		 */
		cs[p.n_customers + 1] = depot_head(w_route);
		for (j = 0; j < w_route_len / 2; j++)
			SWAP(cs[v_route_len + 1 + j], cs[p.n_customers - j]);

		for (j = 0; j < p.n_customers; j++) {
			int v_idx = randint(0, v_route_len);
			int w_idx = randint(v_route_len + 1, p.n_customers + 1);
			struct customer *v = cs[v_idx];
			w = cs[w_idx];
			RANDOM_MODIFICATIONS_BEFORE_TWO_OPT;
			modify(TWO_OPT, v, w);
			for (int k = 0; k < (w_idx - v_idx - 1) / 2; k++)
				SWAP(cs[v_idx + k + 1], cs[w_idx - k - 1]);
			int v_route_tail_len = v_route_len - v_idx;
			int w_route_tail_len = w_idx - (v_route_len + 1);
			v_route_len += w_route_tail_len - v_route_tail_len;
			w_route_len += v_route_tail_len - w_route_tail_len;
			RANDOM_MODIFICATIONS_AFTER_TWO_OPT;
		}
	}
}

void
random_out_relocations(int n_tests)
{
	for (int i = 0; i < n_tests; i++) {
		generate_random_problem(MAX_N_CUSTOMERS_TEST);
		struct route *v_route = route_new();
		struct route *w_route = route_new();
		/**
		 * v_route is initially empty to test for a non-zero delta at
		 * the beginning. When both paths violate constraints, delta
		 * will most likely be 0
		 */
		int v_route_len = 0, w_route_len = p.n_customers;
		struct customer *w;
		int j = 0;
		rlist_foreach_entry(w, &p.customers, in_route) {
			/** Skip route_len to put the depot there */
			if (j == v_route_len) ++j;
			cs[j++] = w;
		}
		route_init(v_route, &cs[0], v_route_len);
		cs[v_route_len] = depot_tail(v_route);
		route_init(w_route, &cs[v_route_len + 1], w_route_len);

		/** Relocate all customers from one list to another */
		while (v_route_len < p.n_customers) {
			int k = randint(v_route_len + 1, p.n_customers);
			w = cs[k];
			j = randint(0, v_route_len);
			struct customer *v = cs[j];
			RANDOM_MODIFICATIONS_BEFORE_OUT_RELOCATE;
			modify(OUT_RELOCATE, v, w);
			SWAP(cs[v_route_len + 1], cs[k]);
			++v_route_len;
			/** Do it purely for the sake of readability */
			--w_route_len;
			RANDOM_MODIFICATIONS_AFTER_OUT_RELOCATE;
		}
	}
}

void
random_inter_route_exchanges(int n_tests)
{
	for (int i = 0; i < n_tests; i++) {
		generate_random_problem(MAX_N_CUSTOMERS_TEST);
		struct route *v_route = route_new();
		struct route *w_route = route_new();
		int v_route_len = randint(1, p.n_customers / 2);
		int w_route_len = p.n_customers - v_route_len;
		struct customer *w;
		int j = 0;
		rlist_foreach_entry(w, &p.customers, in_route)
		cs[j++] = w;
		route_init(v_route, &cs[0], v_route_len);
		route_init(w_route, &cs[v_route_len], w_route_len);

		for (j = 0; j < p.n_customers; j++) {
			int v_idx = randint(0, v_route_len - 1);
			int w_idx = randint(v_route_len, p.n_customers - 1);
			struct customer *v = cs[v_idx];
			w = cs[w_idx];
			RANDOM_MODIFICATIONS_BEFORE_INTER_ROUTE_EXCHANGE;
			modify(EXCHANGE, v, w);
			SWAP(cs[v_idx], cs[w_idx]);
			RANDOM_MODIFICATIONS_AFTER_INTER_ROUTE_EXCHANGE;
		}
	}
}

void
random_intra_route_exchanges(int n_tests)
{
	for (int i = 0; i < n_tests; i++) {
		generate_random_problem(MAX_N_CUSTOMERS_TEST);
		struct route *route = route_new();
		struct customer *w;
		int j = 0;
		rlist_foreach_entry(w, &p.customers, in_route)
			cs[j++] = w;
		route_init(route, &cs[0], p.n_customers);

		for (j = 0; j < p.n_customers; j++) {
			int v_idx = randint(0, p.n_customers - 1);
			int w_idx = randint(0, p.n_customers - 1);
			struct customer *v = cs[v_idx];
			w = cs[w_idx];
			RANDOM_MODIFICATIONS_BEFORE_INTRA_ROUTE_EXCHANGE;
			modify(EXCHANGE, v, w);
			SWAP(cs[v_idx], cs[w_idx]);
			RANDOM_MODIFICATIONS_AFTER_INTRA_ROUTE_EXCHANGE;
		}
	}
}

#endif //EAMA_ROUTES_MINIMIZATION_HEURISTIC_RANDOM_MODIFICATIONS_H
