#include "solution.h"

#include "dist.h"
#include "modification.h"

#include <cassert>

#include "core/fiber.h"
#include "core/random.h"
#include "core/exception.h"

#include <algorithm>

struct solution_meta {
    customer *idx[0];
};

bool solution_global_initialized = false;

/**
 * Arrays of neighbours sorted by dist from particular customer.
 * There are no depot in this arrays. Depots are processed separately.
 */
int neighbours_sorted[MAX_N_CUSTOMERS][MAX_N_CUSTOMERS];

void
init_neighbours_sorted()
{
	std::vector<customer *> cs(p.n_customers + 1);
	cs[0] = p.depot;
	{
		customer *c;
		rlist_foreach_entry(c, &p.customers, in_route) cs[c->id] = c;
	}
	assert(cs.begin() != cs.end());

#define init_row() {				\
std::sort(cs.begin() + 1, cs.end(),		\
        [c](customer *a, customer *b) {		\
    	return dist(c, a) < dist(c, b); });	\
std::transform(cs.begin() + 1, cs.end(),	\
       std::begin(neighbours_sorted[c->id]),	\
       [](customer *c) { return c->id; });	\
} while(0)
	struct customer *c = p.depot;
	init_row();
	rlist_foreach_entry(c, &p.customers, in_route)
		init_row();
#undef init_row
}

struct modification_neighbourhood_args {
	solution *s;
	route *r;
	int n_near;
	modification *m;
};

/**
 * pre-calculated data in order to quickly process intra-route
 * out-relocate modifications
 */
struct intra_route_out_relocate_data {
	route *r;
	customer *w;
	customer *id_to_customer[MAX_N_CUSTOMERS];
	double tw_penalty_delta;
};

struct modification_neighbourhood_data {
	modification_neighbourhood_args args;
	intra_route_out_relocate_data out_relocate_current;
	/** number of customers in route (excluding depots) */
	int n;
	/** random permutation of route customers (excluding depots) */
	customer *permutation[MAX_N_CUSTOMERS];
};

void
intra_route_out_relocate_init_delta(
	modification_neighbourhood_data *data,
	customer *v)
{
	intra_route_out_relocate_data *d =
		&data->out_relocate_current;
	v = d->id_to_customer[v->id];
	assert(modification_applicable(
		modification_new(INSERT, v, d->w)));
	data->args.m->delta_initialized = true;
	data->args.m->tw_penalty_delta = d->tw_penalty_delta +
					 tw_penalty_get_insert_delta(v, d->w);
	data->args.m->c_penalty_delta = 0.;
}

/** out-relocate to the tail of some route */
/*void
out_relocation_to_depot_tails(
	modification_neighbourhood_data *data,
	customer *w)
{
	assert(w->id != 0);
	args *a = &data->_args;
	solution *s = a->s;
	route *v_route = s->routes[0];
	for (int i = 0; i < s->n_routes; v_route = s->routes[++i]) {
		customer *v = depot_tail(v_route);
		*a->m = modification_new(OUT_RELOCATE, v, w);
		assert(modification_applicable(*a->m));
		if (w->route == v->route)
			intra_route_out_relocate_init_delta(data, v);
		fiber_yield();
	}
}*/

void
inter_route_modifications(
	modification_neighbourhood_data *data,
	customer *v, customer *w)
{
	assert(v->route != w->route);
	modification_neighbourhood_args *a = &data->args;
	for (int t = 0; t <= EXCHANGE; t++) {
		*a->m = modification_new((modification_type) t, v, w);
		if (modification_applicable(*a->m))
			fiber_yield();
	}
}

void
intra_route_modifications(
	modification_neighbourhood_data *data,
	customer *v, customer *w)
{
	assert(v->route == w->route);
	assert(w->id == data->out_relocate_current.w->id);
	if (v->id == 0 || w->id == 0)
		return;
	modification_neighbourhood_args *a = &data->args;
	/** out-relocate */
	*a->m = modification_new(OUT_RELOCATE, v, w);
	if (modification_applicable(*a->m)) {
		intra_route_out_relocate_init_delta(data, v);
		fiber_yield();
	}
	/** exchange */
	bool exact;
	*a->m = modification_new(EXCHANGE, v, w);
	if (modification_applicable(*a->m)) {
		a->m->tw_penalty_delta =
			tw_penalty_exchange_penalty_delta_lower_bound(v, w, &exact);
		if (exact) {
			a->m->delta_initialized = true;
			a->m->c_penalty_delta = 0.;
			fiber_yield();
		}
	}
}

void modification_neighbourhood_data_init(
	modification_neighbourhood_data *data,
	va_list ap)
{
	data->args.s = va_arg(ap, solution *);
	data->args.r = va_arg(ap, route *);
	data->args.n_near = va_arg(ap, int);
	data->args.m = va_arg(ap, modification *);

	/**
	 * To diversify the search a little, we consider the vertices
	 * of the route in random order.
	 */
	data->n = 0;
	customer *c;
	route_foreach(c, data->args.r)
		data->permutation[data->n++] = c;
	random_shuffle(data->permutation, data->n);
}

void
intra_route_out_relocate_data_init(
	modification_neighbourhood_data *data,
	customer *w)
{
	assert(w->id != 0);
	intra_route_out_relocate_data *d =
		&data->out_relocate_current;
	/** WARNING: ALLOCATION! */
	d->r = route_dup(w->route);
	customer *v;
	route_foreach(v, d->r)
		d->id_to_customer[v->id] = v;
	d->w = d->id_to_customer[w->id];
	modification m = modification_new(EJECT, d->w, nullptr);
	assert(modification_applicable(m));
	d->tw_penalty_delta = tw_penalty_get_eject_delta(d->w);
	modification_apply(m);
}

void
solution_global_init()
{
	if (!solution_global_initialized) {
		init_neighbours_sorted();
		solution_global_initialized = true;
	}
}

int
solution_modification_neighbourhood_f(va_list ap)
{
	if(!solution_global_initialized)
		solution_global_init();

	modification_neighbourhood_data *data =
		xregion_alloc_object(&fiber()->gc, typeof(*data));
	data->out_relocate_current.r = nullptr;

	modification_neighbourhood_data_init(data, ap);
	struct solution *s = data->args.s;
	solution_check_routes(s);
	struct customer **idx = s->meta->idx;

#define check_modifications() do { \
        /**
	 * a few simple checks to filter out obviously
	 * inapplicable modifications
	 */							\
        if (v != w && !is_ejected(v)) {				\
        	if (v->route != w->route)			\
        	        inter_route_modifications(data, v, w);	\
        	else if (w->id != 0)				\
        	        intra_route_modifications(data, v, w);	\
	}							\
} while(0)
	customer *v;
	for (int j = 0; j < data->n; j++) {
		customer *w = data->permutation[j];
		if (w->id != 0) {
			/** TODO: make this smarter */
			if (data->out_relocate_current.r != nullptr) {
				route_delete(data->out_relocate_current.r);
				data->out_relocate_current.r = nullptr;
			}
			/** prepare for intra-route out-relocate */
			intra_route_out_relocate_data_init(data, w);
			/**
			 * here we could only consider out-relocate, but to
			 * simplify the code we check all types of
			 * modifications, it does not cost too much
			 */
			for (int i = 0; i < s->n_routes; i++) {
				v = depot_tail(s->routes[i]);
				check_modifications();
			}
		}
		for (int i = 0; i < MIN(data->args.n_near, p.n_customers); i++) {
			assert(neighbours_sorted[w->id][i] != 0);
			v = idx[neighbours_sorted[w->id][i]];
			check_modifications();
		}
	}

	/** TODO: make this smarter */
	if (data->out_relocate_current.r != nullptr) {
		route_delete(data->out_relocate_current.r);
		data->out_relocate_current.r = nullptr;
	}
	return 0;
}

solution_meta *
solution_meta_new(rlist *problem_customers)
{
	auto *meta = (solution_meta*)
		xmalloc(sizeof(customer *) * (p.n_customers + 1));
	customer *c, *tmp;
	rlist_foreach_entry_safe(c, problem_customers, in_route, tmp) {
		rlist_del_entry(c, in_route);
		meta->idx[c->id] = c;
	}
	return meta;
}

void
solution_meta_delete(solution_meta *meta)
{
	free(meta);
}

void
solution_print(solution *s)
{
	for (int i = 0; i < s->n_routes; i++) {
		customer *c;
		rlist_foreach_entry(c, &s->routes[i]->list, in_route)
			printf("%d ", c->id);
		printf("\n");
	}
	fflush(stdout);
}

solution *
solution_default(void)
{
	auto *s = (solution *)xmalloc(sizeof(solution) +
		sizeof(struct route *) * p.n_customers);

	s->w = nullptr;

	rlist problem_customers{};
	rlist_create(&problem_customers);
	problem_customers_dup(&problem_customers);
	s->meta = solution_meta_new(&problem_customers);
	assert(rlist_empty(&problem_customers));

	rlist_create(&s->ejection_pool);
	s->n_routes = p.n_customers;
	int i = 0;
	customer *c;
	rlist_foreach_entry(c, &p.customers, in_route) {
		route *r = route_new();
		rlist_create(&r->list);
		route_init(r, &s->meta->idx[c->id], 1);
		s->routes[i] = r;
		++i;
	}
	assert(i == p.n_customers);
	return s;
}

solution *
solution_dup(solution *s)
{
	auto *dup = (solution *)xmalloc(sizeof(solution) +
				       sizeof(struct route *) * p.n_customers);

	rlist problem_customers{};
	rlist_create(&problem_customers);
	problem_customers_dup(&problem_customers);

	dup->meta = solution_meta_new(&problem_customers);
	assert(rlist_empty(&problem_customers));

	dup->w = ((s->w != nullptr) ? dup->meta->idx[s->w->id] : nullptr);

	rlist_create(&dup->ejection_pool);
	customer *c;
	rlist_foreach_entry(c, &s->ejection_pool, in_eject) {
		assert(c->id != 0);
		rlist_add_tail_entry(&dup->ejection_pool, dup->meta->idx[c->id], in_eject);
	}
	dup->n_routes = s->n_routes;
	for (int i = 0; i < dup->n_routes; i++) {
		route *r = route_new();
		rlist_foreach_entry(c, &s->routes[i]->list, in_route) {
			auto c_dup = (c->id == 0) ? customer_dup(c) :
				dup->meta->idx[c->id];
			c_dup->route = r;
			rlist_add_tail_entry(&r->list, c_dup, in_route);
		}
		route_init_penalty(r);
		route_check(r);
		dup->routes[i] = r;
	}
	return dup;
}

void
solution_move(solution *dst, solution *src)
{
	SWAP(dst->w, src->w);
	SWAP(dst->meta, src->meta);
	rlist_create(&dst->ejection_pool);
	rlist_splice(&dst->ejection_pool, &src->ejection_pool);
	for (int i = 0; i < MAX(dst->n_routes, src->n_routes); i++)
		SWAP(dst->routes[i], src->routes[i]);
	SWAP(dst->n_routes, src->n_routes);
	solution_delete(src);
}

void
solution_delete(solution *s)
{
	solution_meta_delete(s->meta);
	free(s->w);
	struct customer *c, *tmp;
	rlist_foreach_entry_safe(c, &s->ejection_pool, in_eject, tmp) customer_delete(c);
	for (int i = 0; i < s->n_routes; i++)
		route_delete(s->routes[i]);
	free(s);
}

double
solution_penalty(struct solution *s, double alpha, double beta)
{
	double penalty = 0.;
	for(int i = 0; i < s->n_routes; i++)
		penalty += route_penalty(s->routes[i], alpha, beta);
	return penalty;
}

bool
solution_feasible(solution *s)
{
	for(int i = 0; i < s->n_routes; i++)
		if (!route_feasible(s->routes[i]))
			return false;
	return true;
}

int
split_by_feasibility(solution *s)
{
	int infeasible = 0;
	for (int i = 0; i < s->n_routes; i++) {
		if (!route_feasible(s->routes[i])) {
			SWAP(s->routes[infeasible], s->routes[i]);
			++infeasible;
		}
	}
	return infeasible;
}

struct modification
solution_find_feasible_insertion(struct solution *s, struct customer *w)
{
	assert(is_ejected(w));
	if (!solution_global_initialized)
		solution_global_init();
#define check_insertion() do {					\
	struct modification m = modification_new(INSERT, v, w);	\
	if (modification_applicable(m)) {			\
		double penalty = modification_delta(m, 1., 1.);	\
		if (penalty < EPS5)				\
			return m;				\
	}							\
} while (0)
	customer *v;
	for (int i = 0; i < p.n_customers; i++) {
		int id = neighbours_sorted[w->id][i];
		assert(id != 0);
		v = s->meta->idx[id];
		check_insertion();
	}
	for (int i = 0; i < s->n_routes; i++) {
		v = depot_tail(s->routes[i]);
		check_insertion();
	}
#undef check_insertion
	return modification_new(INSERT, nullptr, w);
}

struct modification
solution_find_optimal_insertion(struct solution *s, struct customer *w,
				double alpha, double beta)
{
	assert(is_ejected(w));
	struct customer *v;
	struct modification opt_modification =
		modification_new(INSERT, nullptr, nullptr);
	double opt_penalty = INFINITY;
	for(int i = 0; i < s->n_routes; i++) {
		rlist_foreach_entry(v, &s->routes[i]->list, in_route) {
			if (v == depot_head(s->routes[i]))
				continue;
			struct modification m = modification_new(INSERT, v, w);
			double penalty = modification_delta(m, alpha, beta);
			if (penalty < opt_penalty) {
				if (penalty < EPS5)
					return m;
				opt_modification = m;
				opt_penalty = penalty;
			}
		}
	}
	return opt_modification;
}

struct customer *
solution_find_customer_by_id(struct solution *s, int id)
{
	return s->meta->idx[id];
}

void
solution_eliminate_random_route(struct solution *s)
{
	int route_idx = randint(0, s->n_routes - 1);
	struct route *r = s->routes[route_idx];
	SWAP(s->routes[route_idx], s->routes[s->n_routes - 1]);
	--s->n_routes;

	struct customer *c, *tmp;
	rlist_foreach_entry_safe(c, &r->list, in_route, tmp) {
		rlist_del_entry(c, in_route);
		if (c->id == 0) {
			customer_delete(c);
		} else {
			c->route = nullptr;
			rlist_add_tail_entry(&s->ejection_pool, c, in_eject);
		}
	}
	route_delete(r);
}
