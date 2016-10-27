#include "slam/utils/munit.hpp"


// TEST FUNCTIONS
int test_deg2rad(void);
int test_rad2deg(void);
int test_fltcmp(void);
int test_tic(void);
int test_toc(void);
int test_mtoc(void);
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

void testSuite(void)
{
    mu_add_test(test_deg2rad);
    mu_add_test(test_rad2deg);
    mu_add_test(test_fltcmp);
    mu_add_test(test_tic);
    mu_add_test(test_toc);
    mu_add_test(test_mtoc);
}

mu_run_tests(testSuite);
