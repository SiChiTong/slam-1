#include "munit.hpp"
#include "odometry.hpp"

#define TEST_IMAGE_1 "tests/test_data/sample_0.jpg"
#define TEST_IMAGE_2 "tests/test_data/sample_1.jpg"


// TESTS
int testVisualOdometry(void);
int testVisualOdometryConfigure(void);


int testVisualOdometry(void)
{
    VisualOdometry vo;

    mu_check(vo.configured == false);

    return 0;
}

int testVisualOdometryConfigure(void)
{
    Camera camera;
    VisualOdometry vo;

    vo.configure();
    mu_check(vo.configured == true);

    return 0;
}

int testVisualOdometryFeatureTracking(void)
{
    VisualOdometry vo;
    cv::Mat img_1;
    cv::Mat img_2;

    img_1 = cv::imread(TEST_IMAGE_1);
    img_2 = cv::imread(TEST_IMAGE_2);

    cv::imshow("test", img_1);
    cv::waitKey(10000);


    return 0;
}

void testSuite(void)
{
    mu_add_test(testVisualOdometry);
    mu_add_test(testVisualOdometryConfigure);
    mu_add_test(testVisualOdometryFeatureTracking);
}

mu_run_tests(testSuite)
