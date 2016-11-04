#include "slam/utils/munit.hpp"
#include "slam/utils/utils.hpp"


// TEST FUNCTIONS
int test_deg2rad(void);
int test_rad2deg(void);
int test_fltcmp(void);
int test_tic(void);
int test_toc(void);
int test_mtoc(void);
int test_kronecker_product(void);
void testSuite(void);


int test_deg2rad(void)
{

    return 0;
}

int test_rad2deg(void)
{

    return 0;
}

int test_fltcmp(void)
{

    return 0;
}

int test_tic(void)
{

    return 0;
}

int test_toc(void)
{

    return 0;
}

int test_mtoc(void)
{

    return 0;
}

int test_kronecker_product(void)
{
    slam::MatX A(2, 2);
    slam::MatX B(2, 2);
    slam::MatX expected(4, 4);
    slam::MatX product;

    // setup
    A << 1, 2,
         3, 4;
    B << 0, 5,
         6, 7;
    expected << 0, 5, 0, 10,
              6, 7, 12, 14,
              0, 15, 0, 20,
              18, 21, 24, 28;

    // test and assert
    product = slam::kronecker_product(A, B);
    std::cout << product << std::endl;
    mu_check(product == expected);
    mu_check(product.rows() == 4);
    mu_check(product.cols() == 4);

    return 0;
}

void testSuite(void)
{
    mu_add_test(test_deg2rad);
    mu_add_test(test_rad2deg);
    mu_add_test(test_fltcmp);
    mu_add_test(test_tic);
    mu_add_test(test_toc);
    mu_add_test(test_mtoc);
    mu_add_test(test_kronecker_product);
}

mu_run_tests(testSuite);
