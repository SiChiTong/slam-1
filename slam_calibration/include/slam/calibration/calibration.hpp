#ifndef __SLAM_CALIBRATION_HPP__
#define __SLAM_CALIBRATION_HPP__

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>

#include "slam/utils/utils.hpp"
#include "slam/utils/logging.hpp"


namespace slam {

// ERROR MESSAGES
#define ECALIBDIRPERM "permission denied! while creating [%s]!"
#define ECALIBNOTDIR "[%s] is not a dir!"
#define ECALIBDIREXIST "[%s] already exists!"
#define ECALIBDIR "failed to create [%s]!"

// CALIBRATION STATES
#define IDEL 0
#define CAPTURING 1
#define READY_TO_CALIBRATE 2


class Chessboard
{
public:
    bool configured;

    int nb_corners_rows;
    int nb_corners_columns;
    int nb_corners_total;
    cv::Size board_size;

    Chessboard(void);
    int configure(int nb_corners_rows, int nb_corners_columns);
};


class Calibration
{
public:
    bool configured;
    int state;

    Chessboard chessboard;
    int nb_samples;
    int nb_max_samples;
    std::string calibration_path;

    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    std::vector<cv::Mat> rotation_vectors;
    std::vector<cv::Mat> translation_vectors;

    Calibration(void);
    int configure(
        std::string calibration_path,
        Chessboard &chessboard,
        int nb_max_samples
    );
    bool findChessboardCorners(
        cv::Mat &image,
        std::vector<cv::Point2f> &corners
    );
    int saveImage(cv::Mat &image, std::vector<cv::Point2f> corners);
    int calibrate(
        std::vector<std::vector<cv::Point2f>> image_points,
        cv::Size image_size
    );
};

} // end of slam namespace
#endif
