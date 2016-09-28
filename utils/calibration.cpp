#include <stdio.h>

#include "vision.hpp"


class ChessBoard
{
public:
    int nb_corners_rows;
    int nb_corners_columns;
    int nb_corners_total;
    cv::Size board_size;

    ChessBoard(void);
    void configure(int nb_corners_rows, int nb_corners_columns);
};

ChessBoard::ChessBoard(void)
{
    this->nb_corners_rows = 0;
    this->nb_corners_columns = 0;
    this->nb_corners_total = 0;
    this->board_size = cv::Size(0, 0);
}

void ChessBoard::configure(int nb_corners_rows, int nb_corners_columns)
{
    this->nb_corners_rows = nb_corners_rows;
    this->nb_corners_columns = nb_corners_columns;
    this->nb_corners_total = nb_corners_rows * nb_corners_columns;
    this->board_size = cv::Size(nb_corners_rows, nb_corners_columns);
}


class Calibration
{
public:
    ChessBoard chess_board;
    int nb_samples;
    cv::Mat image_points;
    cv::Mat object_points;
    cv::Mat point_counts;
    cv::Mat intrinsic_matrix;
    cv::Mat distortion_coeffs;

    Calibration(void);
    void configure(ChessBoard chess_board, int nb_samples);
};

Calibration::Calibration(void)
{
    this->nb_samples = 0;
}

void Calibration::configure(ChessBoard chess_board, int nb_samples)
{
    int total_corners;

    total_corners = nb_samples * chess_board.nb_corners_total;

    this->chess_board = chess_board;
    this->nb_samples = nb_samples;
    this->image_points = cv::Mat(total_corners, 2, CV_32FC1);
    this->object_points = cv::Mat(total_corners, 3, CV_32FC1);
    this->point_counts = cv::Mat(nb_samples, 1, CV_32SC1);
    this->intrinsic_matrix = cv::Mat(3, 3, CV_32FC1);
    this->distortion_coeffs = cv::Mat(5, 1, CV_32FC1);
}


int main(void)
{
    cv::Mat image;
    std::vector<cv::Point2f> corners;

    Vision vision;
    ChessBoard chess_board;
    Calibration calibration;

    chess_board.configure(5, 5);
    calibration.configure(chess_board, 10);


    // setup camera
    vision.capture_index = 0;
    vision.image_width = 320;
    vision.image_height = 240;
    vision.initCamera();

    // loop
    while (true) {
        vision.capture->read(image);
        cv::imshow("Video Capture", image);


        bool pattern_found = cv::findChessboardCorners(
            image,
            chess_board.board_size,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK
        );

        printf("pattern found: %d\n", (int) pattern_found);


        char c = cv::waitKey(1);
        if (c == 27) break;  // press ESC to stop
    }

    return 0;
}
