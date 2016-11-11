#include "slam/utils/munit.hpp"
#include "slam/utils/math.hpp"
#include "slam/utils/data.hpp"


#define TEST_DATA "tests/data/matrix.dat"


// TEST FUNCTIONS
int test_csvrows(void);
int test_csvcols(void);
int test_csv2mat(void);
void testSuite(void);


int test_csvrows(void)
{
    int rows;

    rows = slam::csvrows(TEST_DATA);
    mu_print("rows: %d\n", rows);
    mu_check(rows == 281);

    return 0;
}

int test_csvcols(void)
{
    int cols;

    cols = slam::csvcols(TEST_DATA);
    mu_print("cols: %d\n", cols);
    mu_check(cols == 2);

    return 0;
}

int test_csv2mat(void)
{
    slam::MatX data;

    slam::csv2mat(TEST_DATA, true, data);

    mu_print("rows: %d\n", (int) data.rows());
    mu_print("cols: %d\n", (int) data.cols());

    mu_check(data.rows() == 280);
    mu_check(data.cols() == 2);
    mu_check(fltcmp(data(0, 0), -2.22482078596) == 0);
    mu_check(fltcmp(data(0, 1), 9.9625789766) == 0);
    mu_check(fltcmp(data(279, 0), 47.0485650525) == 0);
    mu_check(fltcmp(data(279, 1), 613.503760567) == 0);

    return 0;
}

void testSuite(void)
{
    mu_add_test(test_csvrows);
    mu_add_test(test_csvcols);
    mu_add_test(test_csv2mat);
}

mu_run_tests(testSuite);
