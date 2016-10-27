#include <fstream>

#include "slam/utils/munit.hpp"
#include "slam/calibration/calibration.hpp"


#define TEST_IMAGE "tests/data/chessboard.jpg"


// TESTS
int testCalibration(void);
int testCalibrationConfigure(void);
int testCalibrationConfigure(void);
int testCalibrationFindChessboardCorners(void);
int testCalibrationSaveImage(void);
int testCalibrationSaveCalibrationOutputs(void);
void testSuite(void);


int testCalibration(void)
{
    slam::Calibration calibration;

    mu_check(calibration.configured == false);
    mu_check(calibration.state == IDEL);

    mu_check(calibration.nb_samples == 0);
    mu_check(calibration.nb_max_samples == 10);
    mu_check(calibration.save_path == "./");

    return 0;
}

int testCalibrationConfigure(void)
{
    slam::Calibration calib;
    slam::Chessboard chess;

    // setup
    system("rm -rf /tmp/test");
    chess.configure(4, 4);
    calib.configure("/tmp/test", chess, cv::Size(600, 600), 10);

    mu_check(calib.nb_samples == 0);
    mu_check(calib.nb_max_samples == 10);
    mu_check(calib.save_path == "/tmp/test");

    return 0;
}

int testCalibrationFindChessboardCorners(void)
{
    bool retval;
    cv::Mat image;
    slam::Calibration calib;
    slam::Chessboard chess;
    std::vector<cv::Point2f> corners;

    // setup
    system("rm -rf /tmp/test");

    image = cv::imread(TEST_IMAGE);
    chess.configure(7, 7);
    calib.configure("/tmp/test", chess, cv::Size(600, 600), 10);

    // test and assert
    retval = calib.findChessboardCorners(image, corners);
    mu_check(retval == true);
    mu_check(corners.size() == 49);

    cv::imshow("test", image);
    cv::waitKey(1000);

    return 0;
}

int testCalibrationSaveImage(void)
{
    std::ifstream image_file;
    cv::Mat image;
    slam::Chessboard chess;
    slam::Calibration calib;
    std::vector<cv::Point2f> corners;

    // setup
    system("rm -rf /tmp/test");

    image = cv::imread(TEST_IMAGE);
    chess.configure(7, 7);
    calib.configure("/tmp/test", chess, cv::Size(600, 600), 10);
    calib.findChessboardCorners(image, corners);
    calib.saveImage(image, corners);

    // test and assert
    image_file.open("/tmp/test/sample_0.jpg", std::ifstream::in);
    mu_check(image_file.good());

    return 0;
}

int testCalibrationSaveCalibrationOutputs(void)
{
    int k;
    cv::Mat image;
    std::ifstream yaml_file;
    std::vector<cv::Point2f> corners;
    slam::Calibration calib;
    slam::Chessboard chess;

    // setup
    system("rm -rf /tmp/test");

    image = cv::imread(TEST_IMAGE);
    chess.configure(7, 7);
    calib.configure("/tmp/test", chess, cv::Size(600, 600), 1);
    calib.findChessboardCorners(image, corners);

    calib.camera_matrix = cv::Mat(3, 3, CV_32F);
    k = 0;
    for (int i = 0; i < calib.camera_matrix.rows; i++) {
        for (int j = 0; j < calib.camera_matrix.cols; j++) {
            calib.camera_matrix.at<float>(i, j) = k;
            k++;
        }
    }

    calib.distortion_coefficients = cv::Mat(1, 5, CV_32F);
    k = 0;
    for (int i = 0; i < calib.distortion_coefficients.rows; i++) {
        for (int j = 0; j < calib.distortion_coefficients.cols; j++) {
            calib.distortion_coefficients.at<float>(i, j) = k;
            k++;
        }
    }

    // test and assert
    calib.saveCalibrationOutputs();
    yaml_file.open("/tmp/test/calibration.yaml", std::ifstream::in);
    mu_check(yaml_file.good());

    return 0;
}

void testSuite(void)
{
    mu_add_test(testCalibration);
    mu_add_test(testCalibrationConfigure);
    mu_add_test(testCalibrationFindChessboardCorners);
    mu_add_test(testCalibrationSaveImage);
    mu_add_test(testCalibrationSaveCalibrationOutputs);
}

mu_run_tests(testSuite)
