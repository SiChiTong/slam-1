#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include "slam/utils/munit.hpp"
#include "slam/utils/utils.hpp"
#include "slam/optimization/eight_point.hpp"

#define TEST_1_DATA "tests/data/eight_point/img_1.dat"
#define TEST_2_DATA "tests/data/eight_point/img_2.dat"


// TEST FUNCTIONS
int testEightPoint(void);
int testEightPointNormalizePoints(void);
int testEightPointFormMatrixA(void);
int testEightPointApproximateFundamentalMatrix(void);
int testEightPointRefineFundamentalMatrix(void);
int testEightPointDenormalizeFundamentalMatrix(void);
int testEightPointConfigure(void);
void testSuite(void);


slam::MatX load_data(std::string file_path)
{
    int line_no;
    int nb_lines;
    std::string line;
    std::ifstream infile(file_path);
    std::vector<double> vdata;
    std::string element;
    double value;
    slam::MatX data;

    // load file
    if (infile.good() != true) {
        printf("ERROR: FAILED TO LOAD TEST DATA [%s]!!\n", file_path.c_str());
        exit(-1);
    }

    // obtain number of lines
    nb_lines = 0;
    while (std::getline(infile, line)) {
        nb_lines++;
    }

    // rewind file
    infile.clear();
    infile.seekg(0);

    // parse file line by line
    line_no = 0;
    data.resize(nb_lines - 1, 3);
    while (std::getline(infile, line)) {
        std::istringstream ss(line);

        if (line_no != 0) {  // skip header line
            // assuming data is 2 columns
            for (int i = 0; i < 2; i++) {
                std::getline(ss, element, ',');
                value = atof(element.c_str());
                data(line_no - 1, i) = value;
            }
            data(line_no - 1, 2) = 1;
        }

        line_no++;
    }

    return data;
}

int testEightPoint(void)
{
    slam::optimization::EightPoint estimator;

    mu_check(estimator.configured == false);

    mu_check(estimator.image_width == 0);
    mu_check(estimator.image_height == 0);

    return 0;
}

int testEightPointConfigure(void)
{
    slam::optimization::EightPoint estimator;

    // setup
    estimator.configure(800, 600);

    // test and assert
    mu_check(estimator.configured == true);

    mu_check(estimator.image_width == 800);
    mu_check(estimator.image_height == 600);

    return 0;
}

int testEightPointNormalizePoints(void)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);

    // test and assert
    estimator.normalizePoints(pts1, pts2);
    for (int i = 0; i < pts1.rows(); i++) {
        mu_check(pts1(i, 0) < 1.0);
        mu_check(pts1(i, 0) > -1.0);
        mu_check(pts1(i, 1) < 1.0);
        mu_check(pts1(i, 1) > -1.0);

        mu_check(pts2(i, 0) < 1.0);
        mu_check(pts2(i, 0) > -1.0);
        mu_check(pts2(i, 1) < 1.0);
        mu_check(pts2(i, 1) > -1.0);
    }

    return 0;
}

int testEightPointFormMatrixA(void)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX A;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    estimator.normalizePoints(pts1, pts2);

    // test and assert
    estimator.formMatrixA(pts1, pts2, A);

    mu_check(A.rows() == pts1.rows());
    mu_check(A.cols() == 9);

    for (int i = 0; i < pts1.rows(); i++) {
        mu_check(fltcmp(A(i, 0), pts1(i, 0) * pts2(i, 0)) == 0);
        mu_check(fltcmp(A(i, 1), pts1(i, 1) * pts2(i, 0)) == 0);
        mu_check(fltcmp(A(i, 2), pts2(i, 0)) == 0);
        mu_check(fltcmp(A(i, 3), pts1(i, 0) * pts2(i, 1)) == 0);
        mu_check(fltcmp(A(i, 4), pts1(i, 1) * pts2(i, 1)) == 0);
        mu_check(fltcmp(A(i, 5), pts2(i, 1)) == 0);
        mu_check(fltcmp(A(i, 6), pts1(i, 0)) == 0);
        mu_check(fltcmp(A(i, 7), pts1(i, 1)) == 0);
        mu_check(fltcmp(A(i, 8), 1.0) == 0);
    }

    return 0;
}

int testEightPointApproximateFundamentalMatrix(void)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX A, F;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    estimator.normalizePoints(pts1, pts2);
    estimator.formMatrixA(pts1, pts2, A);

    // test and assert
    estimator.approximateFundamentalMatrix(A, F);

    mu_check(F.rows() == 3);
    mu_check(F.cols() == 3);

    return 0;
}

int testEightPointRefineFundamentalMatrix(void)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX A, F;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    estimator.normalizePoints(pts1, pts2);
    estimator.formMatrixA(pts1, pts2, A);
    estimator.approximateFundamentalMatrix(A, F);

    // test and assert
    estimator.refineFundamentalMatrix(F);

    mu_check(F.rows() == 3);
    mu_check(F.cols() == 3);

    return 0;
}

int testEightPointDenormalizeFundamentalMatrix(void)
{
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX A, F;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);
    estimator.normalizePoints(pts1, pts2);
    estimator.formMatrixA(pts1, pts2, A);
    estimator.approximateFundamentalMatrix(A, F);
    estimator.refineFundamentalMatrix(F);

    // test and assert
    estimator.denormalizeFundamentalMatrix(F);

    return 0;
}

int testEightPointEstimate(void)
{
    double result;
    slam::MatX pts1;
    slam::MatX pts2;
    slam::MatX F;
    slam::Vec3 x;
    slam::optimization::EightPoint estimator;

    // setup
    pts1 = load_data(TEST_1_DATA);
    pts2 = load_data(TEST_2_DATA);
    estimator.configure(800, 600);

    // estimate
    estimator.estimate(pts1, pts2, F);

    for (int i = 0; i < pts1.rows(); i++) {
        x = pts1.block(i, 0, 1, 3).transpose();
        result = x.transpose() * F * x;
        std::cout << result << std::endl;
        mu_check(result > 0.0);
        mu_check(result < 1.0);
    }

    for (int i = 0; i < pts2.rows(); i++) {
        x = pts2.block(i, 0, 1, 3).transpose();
        result = x.transpose() * F * x;
        std::cout << result << std::endl;
        mu_check(result > 0.0);
        mu_check(result < 1.0);
    }

    return 0;
}

void testSuite(void)
{
    mu_add_test(testEightPoint);
    mu_add_test(testEightPointConfigure);
    mu_add_test(testEightPointNormalizePoints);
    mu_add_test(testEightPointFormMatrixA);
    mu_add_test(testEightPointApproximateFundamentalMatrix);
    mu_add_test(testEightPointRefineFundamentalMatrix);
    mu_add_test(testEightPointDenormalizeFundamentalMatrix);
    mu_add_test(testEightPointEstimate);
}

mu_run_tests(testSuite);
