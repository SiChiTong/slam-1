#ifndef __SLAM_DETECTOR_HPP__
#define __SLAM_DETECTOR_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace slam {

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
    int detect(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints);
    int detect(cv::Mat &image, std::vector<cv::Point2f> &points);
};


class ORB
{
public:
    bool configured;

    int nb_features;
    float scale_factor;
    int nb_levels;
    int edge_threshold;
    int first_level;
    int wta_k;
    int score_type;
    int patch_size;

    cv::ORB detector;

    ORB(void);
    int configure(void);
    int detect(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints);
    int compute(
        cv::Mat &image,
        std::vector<cv::KeyPoint> &keypoints,
        cv::Mat &descriptors
    );
};

} // end of slam namespace
#endif
