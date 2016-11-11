#include "slam/utils/munit.hpp"
#include "slam/utils/math.hpp"
#include "slam/optimization/lls.hpp"


// TEST FUNCTIONS
int testLLSSolver(void);
void testSuite(void);


int testLLSSolver(void)
{
    slam::LLSSolver solver;

    mu_check(solver.configured == false);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testLLSSolver);
}

mu_run_tests(testSuite);
