#include "problem_decode.h"

#include <fstream>

void
problem_decode(const char *file)
{
	std::string file_string = std::string(file);
	std::ifstream f(file_string);
	//c1_2_1
	//
	std::string test_name;
	f >> test_name;
	//VEHICLE
	//NUMBER     CAPACITY
	std::string vehicle_header[] = {
		"VEHICLE", "NUMBER", "CAPACITY"
	};
	std::string word;
	for (const auto& expected : vehicle_header) {
		f >> word;
		assert(expected == word);
	}
	//  50          200
	int n_vehicles;
	f >> n_vehicles;
	f >> p.vc;
	//CUSTOMER
	//CUST NO.  XCOORD.    YCOORD.    DEMAND   READY TIME  DUE DATE   SERVICE TIME
	std::string customer_header[] = {
		"CUSTOMER", "CUST", "NO.", "XCOORD.",
		"YCOORD.", "DEMAND", "READY", "TIME",
		"DUE", "DATE", "SERVICE", "TIME",
	};
	for (const auto& expected : customer_header) {
		f >> word;
		assert(expected == word);
	}
#define read_customer() do {					\
	f >> c.x >> c.y >> c.demand >> c.e >> c.l >> c.s;	\
} while(0)
	customer c{};
	f >> c.id;
	assert(c.id == 0);
	read_customer();
	p.depot = customer_dup(&c);
	rlist_create(&p.customers);
	while (f >> c.id) {
		read_customer();
		rlist_add_tail_entry(&p.customers, customer_dup(&c), in_route);
		++p.n_customers;
	}
#undef read_customer
}
