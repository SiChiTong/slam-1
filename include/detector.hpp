#ifndef __SLAM_DETECTOR_HPP__
#define __SLAM_DETECTOR_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


class GoodDetector
{
public:
    bool configured;

    int max_corners;
    double quality_level;
    double min_dist;
    int block_size;
    bool use_harris;
    double k;

    cv::Mat mask;

    GoodDetector(void);
    int configure(void);
    int detect(cv::Mat &image, std::vector<cv::Point2f> &points);
};


class FastDetector
{
public:
    bool configured;

    int threshold;
    bool nonmax_suppression;

    FastDetector(void);
    int configure(int threshold, bool nonmax_suppression);
    int detect(cv::Mat &image, std::vector<cv::KeyPoint> &key_points);
    int detect(cv::Mat &image, std::vector<cv::Point2f> &points);
};

#endif
