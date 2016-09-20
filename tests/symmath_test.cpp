#include <ginac/ginac.h>


#include "munit.hpp"


// TESTS
int testSymbolicDifferentiation(void);


int testSymbolicDifferentiation(void)
{
    GiNaC::symbol x("x");
    GiNaC::symbol y("y");
    GiNaC::ex P;

    P = pow(x, 5) + pow(x, 2) + y;

    std::cout << P.diff(x) << std::endl;  // double diff P wrt x

    return 0;
}

void testSuite(void)
{
    mu_add_test(testSymbolicDifferentiation);
}

mu_run_tests(testSuite)
