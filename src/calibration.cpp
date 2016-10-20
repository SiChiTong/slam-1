#include "calibration.hpp"

namespace slam {

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
    this->state = IDEL;

    this->nb_samples = 0;
    this->nb_max_samples = 10;
    this->calibration_path = "./";
}

int Calibration::configure(
    std::string calibration_path,
    Chessboard &chessboard,
    int nb_max_samples
)
{
    int retval;

    // setup
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

    return 0;
}

bool Calibration::findChessboardCorners(
    cv::Mat &image,
    std::vector<cv::Point2f> &corners
)
{
    int flags;
    bool corners_found;
    cv::Mat image_gray;

    // setup
    this->state = CAPTURING;
    corners.clear();

    // detect chessboard corners
    flags = cv::CALIB_CB_ADAPTIVE_THRESH;
    flags += cv::CALIB_CB_NORMALIZE_IMAGE;
    flags += cv::CALIB_CB_FAST_CHECK;
    corners_found = cv::findChessboardCorners(
        image,
        this->chessboard.board_size,
        corners,
        flags
    );

    // draw detected chessboard corners
    cv::drawChessboardCorners(
        image,
        this->chessboard.board_size,
        corners,
        corners_found
    );

    return corners_found;
}

int Calibration::saveImage(cv::Mat &image, std::vector<cv::Point2f> image_points)
{
    std::string image_path;

    // pre-check
    if ((int) image_points.size() != this->chessboard.nb_corners_total) {
        LOG_INFO("failed to detect complete chessboard!");
        return -1;
    } else if (nb_samples >= nb_max_samples) {
        this->state = READY_TO_CALIBRATE;
        LOG_INFO("max calibration samples captured!");
        return -2;
    }

    // save image
    LOG_INFO("captured image [%d]", this->nb_samples);
    image_path = this->calibration_path + "/";
    image_path += "sample_" + std::to_string(this->nb_samples) + ".jpg";
    cv::imwrite(image_path, image);

    // record image points
    this->nb_samples++;

    return 0;
}

int Calibration::calibrate(
    std::vector<std::vector<cv::Point2f>> image_points,
    cv::Size image_size
)
{
    double rms;
    bool camera_matrix_ok;
    bool distortion_coefficients_ok;
    std::vector<std::vector<cv::Point3f>> object_points(1);

    // pre-check
    if (this->state != READY_TO_CALIBRATE) {
        LOG_INFO("calibrator is not ready to calibrate!");
        return -2;
    }

    // hard-coding the object points - assuming chessboard is origin by setting
    // chessboard in the x-y plane (where z = 0).
    for (int i = 0; i < chessboard.nb_corners_rows; i++) {
        for (int j = 0; j < chessboard.nb_corners_columns; j++) {
            object_points[0].push_back(cv::Point3f(i, j, 0.0f));
        }
    }
    object_points.resize(image_points.size(), object_points[0]);

    // calibrate camera
    rms = cv::calibrateCamera(
        object_points,
        image_points,
        image_size,
        this->camera_matrix,
        this->distortion_coefficients,
        this->rotation_vectors,
        this->translation_vectors
    );

    camera_matrix_ok = cv::checkRange(this->camera_matrix);
    distortion_coefficients_ok = cv::checkRange(this->distortion_coefficients);
    if (camera_matrix_ok && distortion_coefficients_ok) {
        std::cout << this->camera_matrix << std::endl;
        std::cout << std::endl;
        std::cout << this->distortion_coefficients << std::endl;
        return 0;
    } else {
        return -1;
    }
}

} // end of slam namespace
