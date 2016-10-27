#include "munit.hpp"
#include "model.hpp"
#include "symmath.hpp"


// TESTS
int testSymbolicDifferentiation(void);


int testSymbolicDifferentiation(void)
{
    GiNaC::symbol x("x");
    GiNaC::symbol y("y");
    GiNaC::ex P;

    P = pow(x, 5) + pow(x, 2) + y;

    // std::cout << P.diff(x) << std::endl;  // diff P wrt x

    slam::QuadrotorModel quad;

    quad.generateMotionModelJacobian();

    return 0;
}

void testSuite(void)
{
    mu_add_test(testSymbolicDifferentiation);
}

mu_run_tests(testSuite)
