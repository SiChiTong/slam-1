#ifndef __SLAM_VISION_UTILS_HPP__
#define __SLAM_VISION_UTILS_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

#include "slam/utils/utils.hpp"


namespace slam {

void convert_mat(MatX A, cv::Mat &B);
void convert_cvmat(cv::Mat A, MatX &B);
void convert_cvpts(std::vector<cv::Point2f> points, MatX &mat);
void combine_cvimgs(cv::Mat img1, cv::Mat img2, cv::Mat &out);

} // end of slam namespace
#endif
