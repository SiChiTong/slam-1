#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include "slam/utils/munit.hpp"
#include "slam/utils/utils.hpp"
#include "slam/optimization/ransac.hpp"

#define TEST_DATA "tests/data/ransac/ransac_sample.dat"


// TEST FUNCTIONS
int testRANSAC(void);
int testRANSACConfigure(void);
int testRANSACRandomSample(void);
int testRANSACComputeDistances(void);
int testRANSACComputeInliers(void);
int testRANSACUpdate(void);
void testSuite(void);


slam::MatX load_data(void)
{
    int line_no;
    int nb_lines;
    std::string line;
    std::ifstream infile(TEST_DATA);
    std::vector<double> vdata;
    std::string element;
    double value;
    slam::MatX data;

    // load file
    if (infile.good() != true) {
        printf("ERROR: FAILED TO LOAD TEST DATA [%s]!!\n", TEST_DATA);
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
    data.resize(2, nb_lines - 1);
    while (std::getline(infile, line)) {
        std::istringstream ss(line);

        if (line_no != 0) {  // skip header line
            // assuming data is 2 columns
            for (int i = 0; i < 2; i++) {
                std::getline(ss, element, ',');
                value = atof(element.c_str());
                data(i, line_no - 1) = value;
            }
        }

        line_no++;
    }

    return data;
}

int testRANSAC(void)
{
    slam::optimization::RANSAC ransac;

    mu_check(ransac.configured == false);

    mu_check(ransac.max_iter == 0);
    mu_check(fltcmp(ransac.thresh_ratio, 1.0) == 0);
    mu_check(fltcmp(ransac.thresh_dist, 0.0) == 0);

    mu_check(ransac.iter == 0);
    mu_check(ransac.max_inliers == 0);
    mu_check(fltcmp(ransac.model_params[0], 0.0) == 0);
    mu_check(fltcmp(ransac.model_params[1], 0.0) == 0);

    return 0;
}

int testRANSACConfigure(void)
{
    slam::optimization::RANSAC ransac;

    ransac.configure(10, 0.8, 0.1);

    mu_check(ransac.configured == true);

    mu_check(ransac.max_iter == 10);
    mu_check(fltcmp(ransac.thresh_ratio, 0.8) == 0);
    mu_check(fltcmp(ransac.thresh_dist, 0.1) == 0);

    mu_check(ransac.iter == 0);
    mu_check(ransac.max_inliers == 0);
    mu_check(fltcmp(ransac.model_params[0], 0.0) == 0);
    mu_check(fltcmp(ransac.model_params[1], 0.0) == 0);

    return 0;
}

int testRANSACRandomSample(void)
{
    int retval;
    slam::Vec2 sample;
    slam::MatX data(2, 100);
    slam::optimization::RANSAC ransac;

    // setup
    for (int i = 0; i < 100; i++) {
        data(0, i) = rand() % 100;
        data(1, i) = rand() % 100;
    }
    ransac.configure(10, 0.8, 0.1);

    // test and assert
    sample << -1, -1;
    retval = ransac.randomSample(data, sample);

    mu_check(retval == 0);
    mu_check(sample(0) != -1);
    mu_check(sample(1) != -1);

    return 0;
}

int testRANSACComputeDistances(void)
{
    int retval;
    slam::MatX data(2, 100);
    slam::Vec2 p1;
    slam::Vec2 p2;
    slam::VecX dists;
    slam::optimization::RANSAC ransac;

    // setup
    for (int i = 0; i < 100; i++) {
        data(0, i) = i;
        data(1, i) = i;
    }
    ransac.configure(10, 0.8, 0.1);

    // test and assert
    ransac.randomSample(data, p1);
    ransac.randomSample(data, p2);
    retval = ransac.computeDistances(data, p1, p2, dists);

    mu_check(retval == 0);

    return 0;
}

int testRANSACComputeInliers(void)
{
    int retval;
    slam::optimization::RANSAC ransac;
    slam::VecX dists(10);

    // setup
    dists << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0;
    ransac.configure(10, 0.8, 0.5);

    // test and assert
    retval = ransac.computeInliers(dists);
    mu_check(retval == 0);
    mu_check(ransac.inliers.size() == 5);
    mu_check(fltcmp(ransac.inliers[0], 0) == 0);
    mu_check(fltcmp(ransac.inliers[1], 1) == 0);
    mu_check(fltcmp(ransac.inliers[2], 2) == 0);
    mu_check(fltcmp(ransac.inliers[3], 3) == 0);
    mu_check(fltcmp(ransac.inliers[4], 4) == 0);

    return 0;
}

int testRANSACUpdate(void)
{
    int retval;
    slam::optimization::RANSAC ransac;
    slam::Vec2 p1;
    slam::Vec2 p2;

    // setup
    ransac.configure(10, 0.8, 0.5);
    ransac.threshold = 2;
    ransac.max_inliers = 3;
    ransac.inliers.push_back(0);
    ransac.inliers.push_back(1);
    ransac.inliers.push_back(2);
    ransac.inliers.push_back(3);
    p1 << 1.0, 2.0;
    p2 << 3.0, 4.0;

    // test and assert
    retval = ransac.update(p1, p2);
    mu_check(ransac.max_inliers = 4);
    mu_check(fltcmp(ransac.model_params[0], 1) == 0);
    mu_check(fltcmp(ransac.model_params[1], 1) == 0);

    return 0;
}

int testRANSACOptimize(void)
{
    slam::MatX data;
    slam::optimization::RANSAC ransac;

    // setup
    data = load_data();
    ransac.configure(40, 0.5, 5);

    // test and assert
    clock_t begin = clock();
    ransac.optimize(data);
    clock_t end = clock();

    double secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "elasped: " << secs << " seconds" << std::endl;

    mu_check(ransac.model_params[0] < 23);
    mu_check(ransac.model_params[0] > 18);
    mu_check(ransac.model_params[1] < 13);
    mu_check(ransac.model_params[1] > 8);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testRANSAC);
    mu_add_test(testRANSACConfigure);
    mu_add_test(testRANSACRandomSample);
    mu_add_test(testRANSACComputeDistances);
    mu_add_test(testRANSACComputeInliers);
    mu_add_test(testRANSACUpdate);
    mu_add_test(testRANSACOptimize);
}

mu_run_tests(testSuite);
