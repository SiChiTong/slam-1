#include <Eigen/Dense>

#include "slam/utils/munit.hpp"
#include "slam/vision/fast.hpp"
#include "slam/vision/vo.hpp"

#define TEST_IMAGE_1 "tests/data/sample_0.jpg"
#define TEST_IMAGE_2 "tests/data/sample_1.jpg"
#define TEST_FRAME_1 "tests/data/frame_1.jpg"
#define TEST_FRAME_2 "tests/data/frame_2.jpg"


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
    fast.configure(20, true);
    vo.configure();

    img_1 = cv::imread(TEST_IMAGE_1);
    img_2 = cv::imread(TEST_IMAGE_2);
    fast.detect(img_1, pts_1);

    // test and assert
    vo.featureTracking(img_1, img_2, pts_1, pts_2, errors, status);
    mu_check(pts_1.size() == pts_2.size());
    mu_check(errors.size() == pts_2.size());
    mu_check(status.size() == pts_2.size());

    // vo.displayOpticalFlow(img_2, pts_1, pts_2);
    // cv::imshow("Test Feature Tracking", img_2);
    // cv::waitKey(0);

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

    // // setup
    // fast.configure(60, true);
    vo.configure();
    vo.focal_length = 1.0;
    vo.principle_point = cv::Point2f(0.0, 0.0);
    // img_1 = cv::imread(TEST_FRAME_1);
    // img_2 = cv::imread(TEST_FRAME_1);
    // fast.detect(img_1, pts_1);
    //
    //
    // // test and assert
    // std::cout << pts_1.size() << std::endl;
    // vo.featureTracking(img_1, img_2, pts_1, pts_2, errors, status);
    // mu_check(pts_1.size() == pts_2.size());
    // mu_check(errors.size() == pts_2.size());
    // mu_check(status.size() == pts_2.size());
    //
    // vo.displayOpticalFlow(img_2, pts_1, pts_2);
    // cv::imshow("Frame 1", img_1);
    // cv::imshow("Frame 2", img_2);
    // cv::waitKey(0);

    // std::cout << pts_1.size() << std::endl;
    // std::cout << pts_2.size() << std::endl;
    // std::cout << errors.size() << std::endl;
    // std::cout << status.size() << std::endl;

    // for (int i = 0; i < pts_1.size(); i++) {
    //     std::cout << pts_1[i] << "\t";
    //     std::cout << pts_2[i] << "\n";
    // }

    // for (int i = 0; i < 40; i++) {
    //     pts_1.push_back(cv::Point2f(5.0, (float) i));
    //     pts_2.push_back(cv::Point2f(5.0, (float) i + 10));
    // }

    pts_1.push_back(cv::Point2f(245.77, 169.57));
    pts_1.push_back(cv::Point2f(248.66, 105.82));
    pts_1.push_back(cv::Point2f(263.39, 182.45));
    pts_1.push_back(cv::Point2f(251.11, 146.54));
    pts_1.push_back(cv::Point2f(249.88, 197.99));
    pts_1.push_back(cv::Point2f(287.57, 137.11));
    pts_1.push_back(cv::Point2f(240.93, 113.15));
    pts_1.push_back(cv::Point2f(206.84, 171.57));
    pts_1.push_back(cv::Point2f(224.95, 170.34));
    pts_1.push_back(cv::Point2f(231.47, 170.21));
    pts_1.push_back(cv::Point2f(257.55, 124.43));
    pts_1.push_back(cv::Point2f(264.71, 176.16));
    pts_1.push_back(cv::Point2f(227.67, 127.05));
    pts_1.push_back(cv::Point2f(246.01, 113.26));
    pts_1.push_back(cv::Point2f(244.15, 154.41));

    pts_2.push_back(cv::Point2f(267.07, 172.34));
    pts_2.push_back(cv::Point2f(252.92, 105.02));
    pts_2.push_back(cv::Point2f(254.22, 190.25));
    pts_2.push_back(cv::Point2f(284.33, 145.10));
    pts_2.push_back(cv::Point2f(236.82, 190.63));
    pts_2.push_back(cv::Point2f(220.04, 135.11));
    pts_2.push_back(cv::Point2f(255.90, 114.37));
    pts_2.push_back(cv::Point2f(259.09, 165.90));
    pts_2.push_back(cv::Point2f(241.73, 164.27));
    pts_2.push_back(cv::Point2f(258.62, 168.30));
    pts_2.push_back(cv::Point2f(277.83, 112.98));
    pts_2.push_back(cv::Point2f(219.62, 170.46));
    pts_2.push_back(cv::Point2f(262.91, 129.03));
    pts_2.push_back(cv::Point2f(250.93, 114.50));
    pts_2.push_back(cv::Point2f(287.09, 155.89));

    cv::Mat K(3, 3, CV_64F, double(0));
    K.at<double>(0, 0) = 1.0;  // fx
    K.at<double>(1, 1) = 1.0;  // fy
    K.at<double>(0, 2) = 0.0;  // cx
    K.at<double>(1, 0) = 0.0;  // cy
    K.at<double>(2, 2) = 1.0;  // 1
    std::cout << K << std::endl;

    cv::Mat dis_coef;

    std::cout << pts_1 << std::endl;

    cv::undistortPoints(pts_1, pts_1, K, dis_coef);
    cv::undistortPoints(pts_2, pts_2, K, dis_coef);

    std::cout << pts_1 << std::endl;

    // cv::Mat E = cv::findEssentialMat(
    //     pts_1,
    //     pts_2,
    //     1.0,
    //     cv::Point2f(0.0, 0.0),
    //     cv::RANSAC,  // outlier rejection method
    //     0.999,       // threshold
    //     1.0          // confidence level
    // );
    // std::cout << E << std::endl;

    // cv::Mat Fun = cv::findFundamentalMat(
    //     pts_1,
    //     pts_2,
    //     cv::FM_8POINT
    // );
    //
    // Eigen::Matrix3d K;
    // K << 1.0, 0.0, 0.0,
    //      0.0, 1.0, 0.0,
    //      0.0, 0.0, 1.0;
    //
    // Eigen::Matrix3d F;
    // F << Fun.at<double>(0, 0), Fun.at<double>(0, 1), Fun.at<double>(0, 2),
    //      Fun.at<double>(1, 0), Fun.at<double>(1, 1), Fun.at<double>(1, 2),
    //      Fun.at<double>(2, 0), Fun.at<double>(2, 1), Fun.at<double>(2, 2);
    // Eigen::Matrix3d result = (K.transpose() * F * K);
    //
    // std::cout << std::endl;
    // std::cout << result << std::endl;

    // // draw flow lines
    // cv::Mat image = cv::Mat(300, 300, CV_8UC3, double(0));
    // cv::Point2f p;
    // cv::Point2f q;
    //
    // for (int i = 0; i < std::min(pts_1.size(), pts_2.size()); i++) {
    //     std::cout << i << std::endl;
    //     p.x = pts_1[i].x;
    //     p.y = pts_1[i].y;
    //
    //     q.x = pts_2[i].x;
    //     q.y = pts_2[i].y;
    //
    //     cv::arrowedLine(image, p, q, cv::Scalar(0, 0, 255), 1);
    // }
    //
    // cv::imshow("image", image);
    // cv::waitKey(0);

    // cv::Mat R;
    // cv::Mat t;
    //
    // int retval = cv::recoverPose(
    //     E,
    //     pts_1,
    //     pts_2,
    //     R,
    //     t,
    //     1.0,
    //     cv::Point2f(0.0, 0.0)
    // );
    //
    // std::cout << retval << std::endl;
    // std::cout << t << std::endl;

    // int retval;
    // retval = vo.measure(pts_1, pts_2);
    // std::cout << retval << std::endl;
    // std::cout << vo.t << std::endl;
    // Eigen::Vector3d vec;
    // vec << vo.t.at<double>(0), vo.t.at<double>(1), vo.t.at<double>(2);
    // std::cout << vec.norm() << std::endl;

    // vo.displayOpticalFlow(img_2, pts_1, pts_2);
    // cv::imshow("Test Feature Tracking", img_2);
    // cv::waitKey(0);

    return 0;
}

void testSuite(void)
{
    mu_add_test(testVisualOdometry);
    mu_add_test(testVisualOdometryConfigure);
    mu_add_test(testVisualOdometryFeatureTracking);
    mu_add_test(testVisualOdometryMeasure);
}

mu_run_tests(testSuite)
