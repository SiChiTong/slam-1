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
    this->nb_max_samples = 10;
    this->calibration_path = "./";
    this->image_points.clear();
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
    this->image_points.clear();

    return 0;
}

bool Calibration::findChessboardCorners(
    cv::Mat &image,
    std::vector<cv::Point2f> &image_points
)
{
    int flags;
    bool corners_found;
    cv::Mat image_gray;

    // setup
    image_points.clear();

    // detect chessboard corners
    flags = cv::CALIB_CB_ADAPTIVE_THRESH;
    flags += cv::CALIB_CB_NORMALIZE_IMAGE;
    flags += cv::CALIB_CB_FAST_CHECK;
    corners_found = cv::findChessboardCorners(
        image,
        this->chessboard.board_size,
        image_points,
        flags
    );

    // draw detected chessboard corners
    cv::drawChessboardCorners(
        image,
        this->chessboard.board_size,
        image_points,
        corners_found
    );

    return corners_found;
}

int Calibration::saveImage(cv::Mat &image, std::vector<cv::Point2f> image_points)
{
    std::string image_path;

    // pre-check
    if ((int) image_points.size() != this->chessboard.nb_corners_total) {
        LOG_WARN("failed to detect complete chessboard!");
        return -1;
    } else if (nb_samples >= nb_max_samples) {
        LOG_ERROR("max calibration samples captured!");
        return -2;
    }

    // save image
    LOG_INFO("captured image [%d]", this->nb_samples);
    image_path = this->calibration_path + "/";
    image_path += "sample_" + std::to_string(this->nb_samples) + ".jpg";
    cv::imwrite(image_path, image);

    // record image points
    this->image_points.push_back(image_points);
    this->nb_samples++;

    return 0;
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
