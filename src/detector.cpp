#include "detector.hpp"


// GOOD DETECTOR
GoodDetector::GoodDetector(void)
{
    this->configured = false;
    this->max_corners = 1000;
    this->quality_level = 0.1;
    this->min_dist = 10;
    this->block_size = 3;
    this->use_harris = false;
    this->k = 0.04;
}

int GoodDetector::configure(void)
{
    this->configured = true;

}

int GoodDetector::detect(cv::Mat &image, std::vector<cv::Point2f> &points)
{
    cv::goodFeaturesToTrack(
        image,
        points,
        this->max_corners,
        this->quality_level,
        this->min_dist,
        this->mask,
        this->block_size
    );

    return 0;
}



// FAST DETECTOR
FastDetector::FastDetector(void)
{
    this->configured = false;
    this->threshold = 20;
    this->nonmax_suppression = true;
}

int FastDetector::configure(int threshold, bool nonmax_suppression)
{
    this->configured = true;
    this->threshold = threshold;
    this->nonmax_suppression = nonmax_suppression;
}

int FastDetector::detect(cv::Mat &image, std::vector<cv::KeyPoint> &key_points)
{
    cv::FAST(image, key_points, this->threshold, this->nonmax_suppression);
    return 0;
}

int FastDetector::detect(cv::Mat &image, std::vector<cv::Point2f> &points)
{
    std::vector<cv::KeyPoint> key_points;
    cv::FAST(image, key_points, this->threshold, this->nonmax_suppression);
    cv::KeyPoint::convert(key_points, points, std::vector<int>());

    return 0;
}
