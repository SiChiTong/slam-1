#ifndef __SLAM_VISION_HPP__
#define __SLAM_VISION_HPP__

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "logging.hpp"


class Vision
{
public:
    bool initialized;

    int capture_index;
    int image_width;
    int image_height;
    cv::VideoCapture *capture;

    cv::Ptr<cv::FeatureDetector> fdetector;
    cv::Ptr<cv::DescriptorExtractor> fextractor;
    cv::Ptr<cv::DescriptorMatcher> fmatcher;

    int fdetector_type;
    int fextractor_type;
    int fmatcher_type;

    Vision(void);
    int init(void);
    int initCamera(void);
    int initFeatureDetector(std::string type);
    int initFeatureExtractor(std::string type);
    int initFeatureMatcher(std::string type);

    int detectFeatures(cv::Mat &image, std::vector<cv::KeyPoint> &key_pts);
    int extractFeatures(
        cv::Mat &image,
        std::vector<cv::KeyPoint> &key_pts,
        cv::Mat &descriptors
    );
    int matchFeaturesBF(cv::Mat d1, cv::Mat d2);
    int matchFeaturesFlann(cv::Mat d1, cv::Mat d2);
};

#endif
