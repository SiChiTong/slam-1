#include "munit.hpp"
#include "odometry.hpp"
#include "feature.hpp"

#define TEST_IMAGE_1 "tests/test_data/sample_0.jpg"
#define TEST_IMAGE_2 "tests/test_data/sample_1.jpg"


// TESTS
int testVisualOdometry(void);
int testVisualOdometryConfigure(void);


int testVisualOdometry(void)
{
    slam::VisualOdometry vo;

    mu_check(vo.configured == false);

    return 0;
}

int testVisualOdometryConfigure(void)
{
    slam::Camera camera;
    slam::VisualOdometry vo;

    vo.configure();
    mu_check(vo.configured == true);

    return 0;
}

int testVisualOdometryFeatureTracking(void)
{
    cv::Mat mask;
    cv::Mat frame;
    cv::Mat img_1;
    cv::Mat img_2;
    slam::FastDetector fast;
    slam::VisualOdometry vo;
    std::vector<cv::Point2f> pts_1;
    std::vector<cv::Point2f> pts_2;
    std::vector<uchar> status;

    // setup
    fast.configure(20, true);
    vo.configure();

    img_1 = cv::imread(TEST_IMAGE_1);
    img_2 = cv::imread(TEST_IMAGE_2);

    fast.detect(img_1, pts_1);
    fast.detect(img_2, pts_2);

    vo.featureTracking(img_1, img_2, pts_1, pts_2, status);
    vo.displayOpticalFlow(img_2, pts_1, pts_2);
    cv::waitKey(2000);

    mu_check(pts_1.size() > 0);
    mu_check(pts_2.size() > 0);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testVisualOdometry);
    mu_add_test(testVisualOdometryConfigure);
    mu_add_test(testVisualOdometryFeatureTracking);
}

mu_run_tests(testSuite)
