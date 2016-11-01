#ifndef __SLAM_FAST_HPP__
#define __SLAM_FAST_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace slam {

class FastDetector
{
public:
    bool configured;

    int threshold;
    bool nonmax_suppression;

    FastDetector(void);
    int configure(int threshold, bool nonmax_suppression);
    int detect(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints);
    int detect(cv::Mat &image, std::vector<cv::Point2f> &points);
};

}  // end of slam namespace
#endif