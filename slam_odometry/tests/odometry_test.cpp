#include "slam/utils/munit.hpp"
#include "slam/feature/fast.hpp"
#include "slam/odometry/odometry.hpp"

#define TEST_IMAGE_1 "tests/data/sample_0.jpg"
#define TEST_IMAGE_2 "tests/data/sample_1.jpg"


// TESTS
int testVisualOdometry(void);
int testVisualOdometryConfigure(void);
int testVisualOdometryFeatureTracking(void);
int testVisualOdometryMeasure(void);
void testSuite(void);


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
    std::vector<float> errors;
    std::vector<uchar> status;

    // setup
    fast.configure(10, true);
    vo.configure();

    img_1 = cv::imread(TEST_IMAGE_1);
    img_2 = cv::imread(TEST_IMAGE_2);

    fast.detect(img_1, pts_1);
    fast.detect(img_2, pts_2);

    std::cout << pts_1.size() << std::endl;
    std::cout << pts_1 << std::endl;

    std::cout << pts_2.size() << std::endl;
    std::cout << pts_2 << std::endl;

    // test and assert
    vo.featureTracking(img_1, img_2, pts_1, pts_2, errors, status);
    for (int i = 0; i < pts_2.size(); i++) {
        std::cout << pts_2[i] << "\t";
        std::cout << (int) status[i] << "\t";
        std::cout << errors[i] << std::endl;
    }
    vo.displayOpticalFlow(img_2, pts_1, pts_2);
    cv::waitKey(0);

    return 0;
}

int testVisualOdometryMeasure(void)
{
    cv::Mat mask;
    cv::Mat frame;
    cv::Mat img_1;
    cv::Mat img_2;
    slam::FastDetector fast;
    slam::VisualOdometry vo;
    std::vector<cv::Point2f> pts_1;
    std::vector<cv::Point2f> pts_2;
    std::vector<float> errors;
    std::vector<uchar> status;

    // setup
    fast.configure(10, true);
    vo.configure();

    img_1 = cv::imread(TEST_IMAGE_1);
    img_2 = cv::imread(TEST_IMAGE_2);

    fast.detect(img_1, pts_1);
    fast.detect(img_2, pts_2);

    // test and assert
    vo.featureTracking(img_1, img_2, pts_1, pts_2, errors, status);
    vo.displayOpticalFlow(img_2, pts_1, pts_2);
    vo.measure(pts_1, pts_2);
    // cv::waitKey(2000);

    mu_check(pts_1.size() > 0);
    mu_check(pts_2.size() > 0);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testVisualOdometry);
    mu_add_test(testVisualOdometryConfigure);
    mu_add_test(testVisualOdometryFeatureTracking);
    // mu_add_test(testVisualOdometryMeasure);
}

mu_run_tests(testSuite)
