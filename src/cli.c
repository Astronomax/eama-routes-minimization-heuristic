#include "cli.h"

#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "core/diag.h"

static int arg_index;
static int arg_count;
static const char **args;
static const char *current_arg;

struct cli_options options;

static const char *log_levels[3] = {
	[LOGLEVEL_NONE] = "none",
	[LOGLEVEL_NORMAL] = "normal",
	[LOGLEVEL_VERBOSE] = "verbose",
};

static void
usage(void)
{
	printf("Usage: %s <file1> <file2> [<options>]\nfile1 - problem statement, file2 - where to output the solution\n", args[0]);
	printf("\n");
	printf("Options:\n");
	printf("  --log_level=<option>    - Log level: none, normal, verbose.\n");
	printf("  --n_near=<value>        - Sets the preferred n_near size.\n");
	printf("  --k_max=<value>         - Sets the preferred k_max size.\n");
	//printf("  --t_max=<value>         - Sets the preferred t_max size.\n");
	printf("  --i_rand=<value>        - Sets the preferred i_rand size.\n");
	printf("  --lower_bound=<value>   - Sets the preferred lower_bound size.\n");
}

bool
str_eq(const char *str1, const char *str2)
{
	return str1 == str2 || (str1 && str2 && strcmp(str1, str2) == 0);
}

int
str_findlist(const char *value, unsigned count, const char** elements)
{
	for (unsigned i = 0; i < count; i++)
	{
		if (strcmp(value, elements[i]) == 0) return (int)i;
	}
	return -1;
}

static inline bool
at_end(void)
{
	return arg_index == arg_count - 1;
}

static inline const char *
next_arg(void)
{
	assert(!at_end());
	current_arg = args[++arg_index];
	return current_arg;
}

bool
match_longopt(const char *name)
{
	return str_eq(&current_arg[2], name);
}

static inline bool
match_shortopt(const char *name)
{
	return str_eq(&current_arg[1], name);
}

static inline const char *
match_argopt(const char *name)
{
	size_t len = strlen(name);
	if (memcmp(&current_arg[2], name, len) != 0) return false;
	if (current_arg[2 + len] != '=') return false;
	return &current_arg[2 + len + 1];
}

static int
parse_multi_option(const char *start, unsigned count, const char **elements)
{
	const char *arg = current_arg;
	int select = str_findlist(start, count, elements);
	if (select < 0) panic("error: %.*s invalid option '%s' given.", (int)(start - arg), start, arg);
	return select;
}

static void
parse_option(void)
{
	const char *argopt;
	switch (current_arg[1]) {
		//case '?':
		//	if (match_shortopt("?")) {
		//		usage();
		//		exit(0);
		//	}
		//	break;
		case '-':
			if ((argopt = match_argopt("log_level"))) {
				options.log_level = (log_level) parse_multi_option(argopt, 3, log_levels);
				return;
			}
			if (match_longopt("n_near"))
			{
				if (at_end())
					panic("error: --n_near needs a valid integer.");
				const char *n_near_string = next_arg();
				char *p_end;
				options.n_near = (int)strtol(n_near_string, &p_end, 10);
				if (p_end != n_near_string + strlen(n_near_string))
					panic("error: --n_near needs a valid integer.");
				return;
			}
			if (match_longopt("k_max"))
			{
				if (at_end())
					panic("error: --k_max needs a valid integer.");
				const char *k_max_string = next_arg();
				char *p_end;
				options.k_max = (int)strtol(k_max_string, &p_end, 10);
				if (p_end != k_max_string + strlen(k_max_string))
					panic("error: --k_max needs a valid integer.");
				return;
			}
			//if (match_longopt("t_max"))
			//{
			//	if (at_end() || next_is_opt()) panic("error: --t_max needs a valid integer.");
			//	options.t_max = atoi(next_arg());
			//	return;
			//}
			if (match_longopt("i_rand"))
			{
				if (at_end())
					panic("error: --i_rand needs a valid integer.");
				const char *i_rand_string = next_arg();
				char *p_end;
				options.i_rand = (int)strtol(i_rand_string, &p_end, 10);
				if (p_end != i_rand_string + strlen(i_rand_string))
					panic("error: --k_max needs a valid integer.");
				return;
			}
			if (match_longopt("lower_bound"))
			{
				if (at_end())
					panic("error: --lower_bound needs a valid integer.");
				const char *lower_bound_string = next_arg();
				char *p_end;
				options.lower_bound = (int)strtol(lower_bound_string, &p_end, 10);
				if (p_end != lower_bound_string + strlen(lower_bound_string))
					panic("error: --k_max needs a valid integer.");
				return;
			}
		default:
			break;
	}
	panic("Cannot process the unknown option \"%s\".", current_arg);
}

void
parse_arguments(int argc, const char *argv[])
{
	arg_count = argc;
	args = argv;

	if (argc < 3)
	{
		usage();
		exit(0);
	}

	options.problem_file = args[1];
	options.solution_file = args[2];
	options.log_level = LOGLEVEL_VERBOSE;
	options.n_near = 100;
	options.k_max = 5;
	//options.t_max = 600;
	options.i_rand = 100000;
	options.lower_bound = 30;

	for (arg_index = 3; arg_index < arg_count; arg_index++)
	{
		current_arg = args[arg_index];
		if (current_arg[0] == '-')
		{
			parse_option();
			continue;
		}
		panic("Found the unexpected argument \"%s\".", current_arg);
	}
}
