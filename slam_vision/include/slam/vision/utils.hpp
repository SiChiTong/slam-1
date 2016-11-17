#ifndef __SLAM_VISION_UTILS_HPP__
#define __SLAM_VISION_UTILS_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>


namespace slam {

void cvmat2mat(cv::Mat A, MatX &B);
void cvpts2mat(std::vector<cv::Point2f> points, MatX &mat);

} // end of slam namespace
#endif
