#include "cli.h"
#include "eama_solver.h"
#include "problem_decode.h"
#include "solution_encode.h"

#include "core/fiber.h"
#include "core/memory.h"

int
main(int argc, const char *argv[])
{
	parse_arguments(argc, argv);

	memory_init();
	fiber_init(fiber_c_invoke);
	random_init();

	problem_decode(options.problem_file);

	struct solution *s = eama_solver_solve();
	printf("n_routes: %d\n", s->n_routes);
	fflush(stdout);
	solution_check_missed_customers(s);
	solution_encode(s, options.solution_file);
	solution_delete(s);

	memory_free();
	return 0;
}
