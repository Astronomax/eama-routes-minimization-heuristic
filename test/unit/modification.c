#include "unit.h"

#include "core/random.h"

#include "problem.h"

#define assert_eq(lhs, rhs) do{if(lhs!=rhs)exit(1);}while(0)
#define assert_neq(lhs, rhs) do{if(lhs==rhs)exit(1);}while(0)

#define RANDOM_MODIFICATIONS_BEFORE_INSERT
#define RANDOM_MODIFICATIONS_AFTER_INSERT			\
	int k = route_len;					\
	while (k > j) {                         		\
		SWAP(cs[k - 1], cs[k]);				\
        	--k;						\
	}                                          		\
	k = 0;							\
	route_foreach(v, route) {				\
		if (v->id == 0)					\
			continue;				\
		assert_eq(cs[k++], v);				\
		assert_eq(v->route, route);			\
	}							\
	assert_eq(k, route_len)

#define RANDOM_MODIFICATIONS_BEFORE_EJECT
#define RANDOM_MODIFICATIONS_AFTER_EJECT 			\
	while (k > j + 1) {                         		\
		SWAP(cs[k - 1], cs[k]);				\
        	--k;						\
	}                                          		\
	k = j + 1;						\
	route_foreach(v, route) {				\
		if (v->id == 0)					\
			continue;				\
		assert_eq(cs[k++], v);				\
		assert_eq(v->route, route);			\
	}							\
	assert_eq(k, p.n_customers)

#define RANDOM_MODIFICATIONS_BEFORE_REPLACE
#define RANDOM_MODIFICATIONS_AFTER_REPLACE			\
	k = 0;							\
	route_foreach(v, route) {				\
		if (v->id == 0)					\
			continue;				\
		assert_eq(cs[k++], v);				\
		assert_eq(v->route, route);			\
	}							\
	assert_eq(k, route_len)

#define RANDOM_MODIFICATIONS_BEFORE_TWO_OPT
#define RANDOM_MODIFICATIONS_AFTER_TWO_OPT			\
	int k = 0;						\
	route_foreach(v, v_route) {				\
		if (k == v_route_len)				\
			break;					\
		assert_eq(cs[k++], v);				\
		assert_eq(v->route, v_route);			\
	}							\
	assert_eq(k, v_route_len);				\
	k = 0;							\
	route_foreach(w, w_route) {				\
		if (k == w_route_len)				\
			break;					\
		assert_eq(cs[p.n_customers + 1 - k++], w);	\
		assert_eq(w->route, w_route);			\
	}							\
	assert_eq(k, w_route_len)

#define RANDOM_MODIFICATIONS_BEFORE_OUT_RELOCATE
#define RANDOM_MODIFICATIONS_AFTER_OUT_RELOCATE			\
	while(k > j) {						\
		if (k != v_route_len + 1)			\
			SWAP(cs[k - 1], cs[k]);			\
		--k;						\
	}							\
	k = 0;							\
	route_foreach(v, v_route) {				\
		if (v->id == 0)					\
			continue;				\
		assert_eq(cs[k++], v);				\
		assert_eq(v->route, v_route);			\
	}							\
	assert_eq(k, v_route_len);				\
	++k;							\
	route_foreach(w, w_route) {				\
		if (w->id == 0)					\
			continue;				\
		assert_eq(cs[k++], w);				\
		assert_eq(w->route, w_route);			\
	}							\
	assert_eq(k, p.n_customers + 1)

#define RANDOM_MODIFICATIONS_BEFORE_INTER_ROUTE_EXCHANGE
#define RANDOM_MODIFICATIONS_AFTER_INTER_ROUTE_EXCHANGE		\
	int k = 0;						\
	route_foreach(v, v_route) {				\
		if (v->id == 0)					\
			continue;				\
		assert_eq(cs[k++], v);				\
		assert_eq(v->route, v_route);			\
	}							\
	assert_eq(k, v_route_len);				\
	route_foreach(w, w_route) {				\
		if (w->id == 0)					\
			continue;				\
		assert_eq(cs[k++], w);				\
		assert_eq(w->route, w_route);			\
	}							\
	assert_eq(k, p.n_customers)

#define RANDOM_MODIFICATIONS_BEFORE_INTRA_ROUTE_EXCHANGE
#define RANDOM_MODIFICATIONS_AFTER_INTRA_ROUTE_EXCHANGE

#include "random_modifications.h"

#undef RANDOM_MODIFICATIONS_BEFORE_INSERT
#undef RANDOM_MODIFICATIONS_AFTER_INSERT
#undef RANDOM_MODIFICATIONS_BEFORE_EJECT
#undef RANDOM_MODIFICATIONS_AFTER_EJECT
#undef RANDOM_MODIFICATIONS_BEFORE_REPLACE
#undef RANDOM_MODIFICATIONS_AFTER_REPLACE
#undef RANDOM_MODIFICATIONS_BEFORE_TWO_OPT
#undef RANDOM_MODIFICATIONS_AFTER_TWO_OPT
#undef RANDOM_MODIFICATIONS_BEFORE_OUT_RELOCATE
#undef RANDOM_MODIFICATIONS_AFTER_OUT_RELOCATE
#undef RANDOM_MODIFICATIONS_BEFORE_INTER_ROUTE_EXCHANGE
#undef RANDOM_MODIFICATIONS_AFTER_INTER_ROUTE_EXCHANGE
#undef RANDOM_MODIFICATIONS_BEFORE_INTRA_ROUTE_EXCHANGE
#undef RANDOM_MODIFICATIONS_AFTER_INTRA_ROUTE_EXCHANGE

int
applicable(void)
{
	generate_random_problem(MAX_N_CUSTOMERS_TEST);
	struct route *v_route = route_new();
	struct route *w_route = route_new();
	int v_route_len = randint(1, p.n_customers / 3);
	int w_route_len = randint(1, (p.n_customers - v_route_len) / 2);
	int ep_len = p.n_customers - v_route_len - w_route_len;
	int v_start = 0, w_start = v_route_len,
		ep_start = v_route_len + w_route_len;

	struct customer *v;
	int j = 0;
	rlist_foreach_entry(v, &p.customers, in_route)
		cs[j++] = v;
	route_init(v_route, &cs[0], v_route_len);
	route_init(w_route, &cs[v_route_len], w_route_len);
	struct modification m = {-1, NULL, NULL};

	/** two-opt */
	m.type = TWO_OPT;
	m.v = depot_tail(v_route);
	m.w = cs[w_start + randint(0, w_route_len - 1)];
	assert(!modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	m.w = depot_tail(w_route);
	assert(!modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	m.w = cs[v_start + randint(0, v_route_len - 1)];
	assert(!modification_applicable(m));

	m.v = depot_head(v_route);
	m.w = depot_head(w_route);
	assert(modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	m.w = cs[w_start + randint(0, w_route_len - 1)];
	assert(modification_applicable(m));

	/** insert and out-relocate */
	m.v = depot_head(v_route);
	m.w = cs[w_start + randint(0, w_route_len - 1)];
	m.type = INSERT;
	assert(!modification_applicable(m));
	m.type = OUT_RELOCATE;
	assert(!modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	m.w = depot_head(w_route);
	m.type = INSERT;
	assert(!modification_applicable(m));
	m.type = OUT_RELOCATE;
	assert(!modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	m.w = cs[ep_start + randint(0, ep_len - 1)];
	m.type = INSERT;
	assert(modification_applicable(m));
	m.type = OUT_RELOCATE;
	assert(!modification_applicable(m));

	m.v = depot_tail(v_route);
	m.w = cs[w_start + randint(0, w_route_len - 1)];
	m.type = INSERT;
	assert(modification_applicable(m));
	m.type = OUT_RELOCATE;
	assert(modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	m.w = cs[w_start + randint(0, w_route_len - 1)];
	m.type = INSERT;
	assert(modification_applicable(m));
	m.type = OUT_RELOCATE;
	assert(modification_applicable(m));

	/** eject */
	m.type = EJECT;
	m.v = depot_head(w_route);
	assert(!modification_applicable(m));

	m.v = cs[ep_start + randint(0, ep_len - 1)];
	assert(!modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	assert(modification_applicable(m));

	/** exchange */
	m.type = EXCHANGE;
	m.v = depot_head(v_route);
	m.w = cs[w_start + randint(0, w_route_len - 1)];
	assert(!modification_applicable(m));
	m.v = depot_tail(v_route);
	assert(!modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	m.w = depot_head(w_route);
	assert(!modification_applicable(m));
	m.w = depot_tail(w_route);
	assert(!modification_applicable(m));

	m.v = cs[v_start + randint(0, v_route_len - 1)];
	m.w = cs[w_start + randint(0, w_route_len - 1)];
	assert(modification_applicable(m));
	return 0;
}

int
main(void)
{
	random_init();
	random_insertions(100);
	random_ejections(100);
	random_replacements(100);
	random_two_opts(100);
	random_out_relocations(100);
	random_inter_route_exchanges(100);

	applicable();
	return 0;
}
