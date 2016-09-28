#ifndef __SLAM_CALIBRATION_HPP__
#define __SLAM_CALIBRATION_HPP__

#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/opencv.hpp>

#include "util.hpp"
#include "logging.hpp"


// ERROR MESSAGES
#define ECALIBDIRPERM "permission denied! while creating [%s]!"
#define ECALIBNOTDIR "[%s] is not a dir!"
#define ECALIBDIREXIST "[%s] already exists!"
#define ECALIBDIR "failed to create [%s]!"


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

    Chessboard chessboard;
    int nb_samples;
    int nb_max_samples;
    std::string calibration_path;

    cv::Mat image_points;
    cv::Mat object_points;
    cv::Mat point_counts;

    cv::Mat intrinsic_matrix;
    cv::Mat distortion_coeffs;

    Calibration(void);
    int configure(
        std::string calibration_path,
        Chessboard &chessboard,
        int nb_max_samples
    );
    int saveImage(cv::Mat &image);
    bool findChessboardCorners(
        cv::Mat &image,
        std::vector<cv::Point2f> &detected_corners
    );
    void calibrate(
        std::vector<std::vector<cv::Point3f>> object_points,
        std::vector<std::vector<cv::Point2f>> image_points,
        cv::Size image_size
    );


};


#endif
