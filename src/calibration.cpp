#include "calibration.hpp"



// CHESSBOARD
Chessboard::Chessboard(void)
{
    this->configured = false;
    this->nb_corners_rows = 0;
    this->nb_corners_columns = 0;
    this->nb_corners_total = 0;
    this->board_size = cv::Size(0, 0);
}

int Chessboard::configure(int nb_corners_rows, int nb_corners_columns)
{
    this->configured = true;

    this->nb_corners_rows = nb_corners_rows;
    this->nb_corners_columns = nb_corners_columns;
    this->nb_corners_total = nb_corners_rows * nb_corners_columns;
    this->board_size = cv::Size(nb_corners_rows, nb_corners_columns);

	return 0;
}



// CALIBRATION
Calibration::Calibration(void)
{
    this->configured = false;
    this->nb_samples = 0;
}

int Calibration::configure(
    std::string calibration_path,
    Chessboard &chessboard,
    int nb_max_samples
)
{
    int retval;
    int total_corners;

    // setup
    total_corners = nb_samples * chessboard.nb_corners_total;
    rmtrailslash(calibration_path);

    // mkdir calibration directory
    retval = mkdir(calibration_path.c_str(), ACCESSPERMS);
    if (retval != 0) {
        switch (errno) {
        case EACCES:
            LOG_ERROR(ECALIBDIRPERM, calibration_path.c_str());
			break;
        case ENOTDIR:
            LOG_ERROR(ECALIBNOTDIR, calibration_path.c_str());
			break;
        case EEXIST:
            LOG_ERROR(ECALIBDIREXIST, calibration_path.c_str());
			break;
		default:
            LOG_ERROR(ECALIBDIR, calibration_path.c_str());
			break;
        }
        return -1;
    }

    // initialize variables
    this->chessboard = chessboard;
    this->nb_samples = 0;
    this->nb_max_samples = nb_max_samples;
    this->calibration_path = calibration_path;

    this->image_points = cv::Mat(total_corners, 2, CV_32FC1);
    this->object_points = cv::Mat(total_corners, 3, CV_32FC1);
    this->point_counts = cv::Mat(nb_samples, 1, CV_32SC1);
    this->intrinsic_matrix = cv::Mat(3, 3, CV_32FC1);
    this->distortion_coeffs = cv::Mat(5, 1, CV_32FC1);

	return 0;
}

int Calibration::saveImage(cv::Mat &image)
{
    std::string image_path;

    image_path = this->calibration_path + "/";
    image_path += "calib_" + std::to_string(this->nb_samples) + ".jpg";
    cv::imwrite(image_path, image);
    this->nb_samples++;

    return 0;
}

bool Calibration::findChessboardCorners(
    cv::Mat &image,
    std::vector<cv::Point2f> &detected_corners
)
{
    int flags;
    bool corners_found;
    cv::Mat image_gray;

    // setup
    detected_corners.clear();

    // detect chessboard corners
    flags = cv::CALIB_CB_ADAPTIVE_THRESH;
    flags += cv::CALIB_CB_NORMALIZE_IMAGE;
    flags += cv::CALIB_CB_FAST_CHECK;
    corners_found = cv::findChessboardCorners(
        image,
        this->chessboard.board_size,
        detected_corners,
        flags
    );

    // draw detected chessboard corners
    cv::drawChessboardCorners(
        image,
        this->chessboard.board_size,
        detected_corners,
        corners_found
    );

    return corners_found;
}

// void Calibration::calibrate(
//     std::vector<std::vector<cv::Point3f>> object_points,
//     std::vector<std::vector<cv::Point2f>> image_points,
//     cv::Size image_size
// )
// {
//     // cv::calibrateCamera(object_points,
//
// }
