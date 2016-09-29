#include "odometry.hpp"


VisualOdometry::VisualOdometry(void)
{
    this->configured = false;
}

int VisualOdometry::configure(void)
{
    this->configured = true;

    this->focal_length = 1.0;
    this->principle_point = cv::Point2f(0, 0);

    return 0;
}

void VisualOdometry::featureTracking(
    cv::Mat img_1,
    cv::Mat img_2,
    std::vector<cv::Point2f> &pts_1,
    std::vector<cv::Point2f> &pts_2,
    std::vector<uchar> &status
)
{
    std::vector<float> err;
    cv::Size win_size;
    cv::TermCriteria term_crit;

    // pre-check
    if (this->configured == false) {
        LOG_ERROR("VisualOdometry is not configured!\n");
    }

    // setup
    win_size = cv::Size(21, 21);
    term_crit = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
        30,
        0.01
    );

    // optical flow
    cv::calcOpticalFlowPyrLK(
        img_1,
        img_2,
        pts_1,
        pts_2,
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
        cv::Point2f pt = pts_2.at(i - correlation_index);

        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0)) {
            if((pt.x<0)||(pt.y<0))    {
                status.at(i) = 0;
            }
            pts_1.erase(pts_1.begin() + i - correlation_index);
            pts_2.erase(pts_2.begin() + i - correlation_index);
            correlation_index++;
        }
    }
}

// void VisualOdometry::measure(void)
// {
//     cv::findEssentialMat(
//         pts_2,
//         pts_1,
//         this->focal_length,
//         this->principle_point,
//         cv::RANSAC,  // outlier rejection method
//         0.999,  // threshold
//         1.0,  // confidence level
//         mask  // output array of N elements
//     );
//     cv::recoverPose(
//         this->E,
//         pts_2,
//         pts_1,
//         this->R,
//         this->t,
//         this->focal_length,
//         this->principle_point,
//         mask
//     );
//
//
//     // cv::Mat image;
//     // std::vector<cv::KeyPoint> key_pts;
//     // std::vector<cv::Point2f> points;
//
//     // setup camera
//     // camera.initFeatureDetector("FAST");
//     // camera.initFeatureExtractor("ORB");
//     // camera.fdetector->set("threshold", 100);
//     // camera.fdetector->set("nonmaxSuppression", false);
//
//     // cv::Mat image1;
//     // cv::Mat image2;
//     // std::vector<cv::Point2f> pts_1;
//     // std::vector<cv::Point2f> pts_2;
//     // std::vector<uchar> status;
//     //
//     // // get frame
//     // camera.getFrame(image);
//     // image1 = image;
//     //
//     // // feature detection
//     // camera.detectFeatures(image, key_pts);
//     // cv::KeyPoint::convert(key_pts, pts_1, std::vector<int>());
//     //
//     // // loop
//     // while (true) {
//     //     // get frame
//     //     camera.capture->read(image);
//     //     image2 = image;
//     //
//     //     // feature detection
//     //     camera.detectFeatures(image, key_pts);
//     //     cv::KeyPoint::convert(key_pts, pts_2, std::vector<int>());
//     //
//     //     // feature tracking
//     //     featureTracking(
//     //         image1,
//     //         image2,
//     //         pts_1,
//     //         pts_2,
//     //         status
//     //     );
//     //     cv::findEssentialMat(
//     //         pts_2,
//     //         pts_1,
//     //         focal,  // focal length
//     //         pp,  // principle point of camera
//     //         cv::RANSAC,  // method
//     //         0.999,  // threshold
//     //         1.0,  // confidence level
//     //         mask  // output array of N elements
//     //     );
//     //     cv::recoverPose(
//     //         E,
//     //         pts_2,
//     //         pts_1,
//     //         R,
//     //         t,
//     //         focal,
//     //         pp,
//     //         mask
//     //     );
//     //
//     //
//     //     // update
//     //     image1 = image;
//     //     pts_1 = pts_2;
//     //
//     //     cv::imshow("test", image);
//     //     cv::waitKey(1);
//     // }
// }
