#include "eama_solver.h"
#include "problem_parser.h"

#include "core/fiber.h"
#include "core/memory.h"

int
main(int argc, char **argv)
{
	memory_init();
	fiber_init(fiber_c_invoke);
	random_init();

	problem_parse(argv[1]);
	struct eama_settings settings;
	settings.n_near = 100;
	settings.k_max = 5;
	settings.lower_bound = 30;
	struct eama *eama = eama_new(settings);
	struct solution *s = eama_solver_solve(eama, true);
	printf("n_routes: %d\n", s->n_routes);
	fflush(stdout);
	_solution_check(s);

	eama_delete(eama);
	memory_free();
	return 0;
}
