#ifndef __SLAM_ODOMETRY_HPP__
#define __SLAM_ODOMETRY_HPP__

#include "camera.hpp"


class VisualOdometry
{
public:
    bool configured;

    double focal_length;
    cv::Point2f principle_point;
    cv::Mat mask;
    cv::Mat E;
    cv::Mat R;
    cv::Mat t;

    VisualOdometry(void);
    int configure(void);
    void featureTracking(
        cv::Mat img_1,
        cv::Mat img_2,
        std::vector<cv::Point2f> &points1,
        std::vector<cv::Point2f> &points2,
        std::vector<uchar> &status
    );
    void measure(void);
};


#endif
