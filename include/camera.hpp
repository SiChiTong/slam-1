#ifndef __SLAM_CAMERA_HPP__
#define __SLAM_CAMERA_HPP__

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "logging.hpp"


class CameraConfig
{
public:
	int camera_index;
	int image_width;
	int image_height;

	float exposure;
	float gain;

	cv::Mat camera_matrix;
	cv::Mat rectification_matrix;
	cv::Mat distortion_coefficients;
	cv::Mat projection_matrix;
};


class Camera
{
public:
    bool configured;
    cv::VideoCapture *capture;

    int capture_index;
    int image_width;
    int image_height;

    Camera(void);
    int configure(int capture_index, int image_width, int image_height);
	int getFrame(cv::Mat &image);
};


#endif
