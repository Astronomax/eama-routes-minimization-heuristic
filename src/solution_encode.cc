#include "solution_encode.h"

#include <fstream>

#include "dist.h"
#include "utils.h"

void
solution_encode(solution *s, const char *file)
{
	std::string file_string = std::string(file);
	std::ofstream f(file_string);
	for (int i = 0; i < s->n_routes; i++) {
		route *r = s->routes[i];
		customer *next;
		customer *prev = depot_head(r);
		double t = -(double)INFINITY;
		for (next = rlist_next_entry(prev, in_route);
		     !rlist_entry_is_head(prev, &r->list, in_route);
		     next = rlist_next_entry(next, in_route)) {
			t = MAX(prev->e, t);
			f << prev->id << " " << t;
			if (prev != depot_tail(s->routes[i])) {
				f << " ";
				t += prev->s + dist(prev, next);
			}
			prev = next;
		}
		f << "\n";
	}
}
