#include "munit.hpp"
#include "camera.hpp"


// TESTS
int testCamera(void);
int testCameraInitCamera(void);
int testCameraInitFeatureDetector(void);
int testCameraInitFeatureExtractor(void);
int testCameraInitFeatureMatcher(void);
void testSuite(void);


int testCamera(void)
{
    Camera camera;

    mu_check(camera.configured == false);
    mu_check(camera.capture == NULL);
    mu_check(camera.capture_index == 0);
    mu_check(camera.image_width == 0);
    mu_check(camera.image_height == 0);

    return 0;
}

int testCameraInitCamera(void)
{
    Camera camera;
    cv::Mat image;

    // init
    camera.configure(0, 320, 240);

    // loop
    for (int i = 0; i < 10; i++) {
        camera.capture->read(image);
        cv::imshow("test", image);
        cv::waitKey(1);
    }

    return 0;
}

int testCameraInitFeatureDetector(void)
{
    Camera camera;

    // // feature detectors
    // camera.initFeatureDetector("FAST");
    // camera.initFeatureDetector("STAR");
    // camera.initFeatureDetector("ORB");
    // camera.initFeatureDetector("BRISK");
    // camera.initFeatureDetector("MSER");
    // camera.initFeatureDetector("GFTT");
    // camera.initFeatureDetector("HARRIS");
    // camera.initFeatureDetector("DENSE");
    // camera.initFeatureDetector("SimpleBlob");
    //
    // // nonfree feature detectors
    // camera.initFeatureDetector("SURF");
    // camera.initFeatureDetector("SIFT");
    // camera.initFeatureDetector("SURF");

    return 0;
}

int testCameraInitFeatureExtractor(void)
{

    return 0;
}

int testCameraInitFeatureMatcher(void)
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
//     Camera camera;
//     cv::Mat image;
//     std::vector<cv::KeyPoint> key_pts;
//     std::vector<cv::Point2f> points;
//
//     // init
//     camera.capture_index = 0;
//     camera.image_width = 640;
//     camera.image_height = 480;
//     camera.initCamera();
//
//     camera.initFeatureDetector("FAST");
//     camera.initFeatureExtractor("ORB");
//     // camera.fdetector->set("threshold", 100);
//     // camera.fdetector->set("nonmaxSuppression", false);
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
//     camera.capture->read(image);
//     image1 = image;
//
//     // feature detection
//     camera.detectFeatures(image, key_pts);
//     cv::KeyPoint::convert(key_pts, points1, std::vector<int>());
//
//     // loop
//     while (true) {
//         // get frame
//         camera.capture->read(image);
//         image2 = image;
//
//         // feature detection
//         camera.detectFeatures(image, key_pts);
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
    mu_add_test(testCamera);
    // mu_add_test(testCameraInitCamera);
    // mu_add_test(testCameraInitFeatureDetector);
    // mu_add_test(testCameraInitFeatureExtractor);
    // mu_add_test(testCameraInitFeatureMatcher);
    // mu_add_test(testSandbox);
}

mu_run_tests(testSuite)
