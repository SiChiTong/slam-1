#include "munit.hpp"
#include "vision.hpp"


// TESTS
int testVision(void);
int testVisionInitCamera(void);
int testVisionInitFeatureDetector(void);
int testVisionInitFeatureExtractor(void);
int testVisionInitFeatureMatcher(void);
void testSuite(void);


int testVision(void)
{
    Vision vision;

    mu_check(vision.capture_index == 0);
    mu_check(vision.image_width == 0);
    mu_check(vision.image_height == 0);
    mu_check(vision.capture == NULL);

    return 0;
}

int testVisionInitCamera(void)
{
    Vision vision;
    cv::Mat image;

    // init
    vision.capture_index = 0;
    vision.image_width = 320;
    vision.image_height = 280;
    vision.initCamera();

    // loop
    for (int i = 0; i < 10; i++) {
        vision.capture->read(image);
        cv::imshow("test", image);
        cv::waitKey(1);
    }

    return 0;
}

int testVisionInitFeatureDetector(void)
{
    Vision vision;

    // // feature detectors
    // vision.initFeatureDetector("FAST");
    // vision.initFeatureDetector("STAR");
    // vision.initFeatureDetector("ORB");
    // vision.initFeatureDetector("BRISK");
    // vision.initFeatureDetector("MSER");
    // vision.initFeatureDetector("GFTT");
    // vision.initFeatureDetector("HARRIS");
    // vision.initFeatureDetector("DENSE");
    // vision.initFeatureDetector("SimpleBlob");
    //
    // // nonfree feature detectors
    // vision.initFeatureDetector("SURF");
    // vision.initFeatureDetector("SIFT");
    // vision.initFeatureDetector("SURF");

    return 0;
}

int testVisionInitFeatureExtractor(void)
{

    return 0;
}

int testVisionInitFeatureMatcher(void)
{

    return 0;
}

void featureTracking(
    cv::Mat img_1,
    cv::Mat img_2,
    std::vector<cv::Point2f> &points1,
    std::vector<cv::Point2f> &points2,
    std::vector<uchar> &status
)
{
    // this function automatically gets rid of points for which tracking fails
    std::vector<float> err;
    cv::Size win_size = cv::Size(21, 21);
    cv::TermCriteria term_crit = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
        30,
        0.01
    );

    cv::calcOpticalFlowPyrLK(
        img_1,
        img_2,
        points1,
        points2,
        status,
        err,
        win_size,
        3,
        term_crit,
        0,
        0.001
    );

    // get rid of points for which the KLT tracking failed or those who
    // have gone outside the frame
    int correlation_index = 0;
    for (int i=0; i < (int) status.size(); i++) {
        cv::Point2f pt = points2.at(i - correlation_index);

        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0)) {
            if((pt.x<0)||(pt.y<0))    {
                status.at(i) = 0;
            }
            points1.erase(points1.begin() + i - correlation_index);
            points2.erase(points2.begin() + i - correlation_index);
            correlation_index++;
        }
    }
}

// int testSandbox(void)
// {
//     Vision vision;
//     cv::Mat image;
//     std::vector<cv::KeyPoint> key_pts;
//     std::vector<cv::Point2f> points;
//
//     // init
//     vision.capture_index = 0;
//     vision.image_width = 640;
//     vision.image_height = 480;
//     vision.initCamera();
//
//     vision.initFeatureDetector("FAST");
//     vision.initFeatureExtractor("ORB");
//     // vision.fdetector->set("threshold", 100);
//     // vision.fdetector->set("nonmaxSuppression", false);
//
//     cv::Mat image1;
//     cv::Mat image2;
//     std::vector<cv::Point2f> points1;
//     std::vector<cv::Point2f> points2;
//     std::vector<uchar> status;
//     double focal = 1.0;
//     cv::Point2f pp(0, 0);
//     cv::Mat mask;
//     cv::Mat E;
//     cv::Mat R;
//     cv::Mat t;
//
//     // get frame
//     vision.capture->read(image);
//     image1 = image;
//
//     // feature detection
//     vision.detectFeatures(image, key_pts);
//     cv::KeyPoint::convert(key_pts, points1, std::vector<int>());
//
//     // loop
//     while (true) {
//         // get frame
//         vision.capture->read(image);
//         image2 = image;
//
//         // feature detection
//         vision.detectFeatures(image, key_pts);
//         cv::KeyPoint::convert(key_pts, points2, std::vector<int>());
//
//         // feature tracking
//         featureTracking(
//             image1,
//             image2,
//             points1,
//             points2,
//             status
//         );
//         cv::findEssentialMat(
//             points2,
//             points1,
//             focal,  // focal length
//             pp,  // principle point of camera
//             cv::RANSAC,  // method
//             0.999,  // threshold
//             1.0,  // confidence level
//             mask  // output array of N elements
//         );
//         cv::recoverPose(
//             E,
//             points2,
//             points1,
//             R,
//             t,
//             focal,
//             pp,
//             mask
//         );
//
//
//         // update
//         image1 = image;
//         points1 = points2;
//
//         cv::imshow("test", image);
//         cv::waitKey(1);
//     }
//
//     return 0;
// }

void testSuite(void)
{
    mu_add_test(testVision);
    // mu_add_test(testVisionInitCamera);
    // mu_add_test(testVisionInitFeatureDetector);
    // mu_add_test(testVisionInitFeatureExtractor);
    // mu_add_test(testVisionInitFeatureMatcher);
    // mu_add_test(testSandbox);
}

mu_run_tests(testSuite)
