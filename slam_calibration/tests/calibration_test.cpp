#include "slam/utils/munit.hpp"
#include "slam/calibration/calibration.hpp"


// TESTS
int testChessboard(void);
int testChessboardConfigure(void);
int testCalibration(void);
int testCalibrationConfigure(void);
void testSuite(void);


int testChessboard(void)
{
    slam::Chessboard chessboard;

    mu_check(chessboard.configured == false);

    mu_check(chessboard.nb_corners_rows == 0);
    mu_check(chessboard.nb_corners_columns == 0);
    mu_check(chessboard.nb_corners_total == 0);
    mu_check(chessboard.board_size == cv::Size(0, 0));

    return 0;
}

int testChessboardConfigure(void)
{
    slam::Chessboard chessboard;

    chessboard.configure(1, 2);

    mu_check(chessboard.configured == true);

    mu_check(chessboard.nb_corners_rows == 1);
    mu_check(chessboard.nb_corners_columns == 2);
    mu_check(chessboard.nb_corners_total == 2);
    mu_check(chessboard.board_size == cv::Size(1, 2));

    return 0;
}

int testCalibration(void)
{
    slam::Calibration calibration;

    mu_check(calibration.configured == false);
    mu_check(calibration.state == IDEL);

    mu_check(calibration.nb_samples == 0);
    mu_check(calibration.nb_max_samples == 10);
    mu_check(calibration.calibration_path == "./");

    return 0;
}

int testCalibrationConfigure(void)
{
    slam::Calibration calib;
    slam::Chessboard chess;

    // setup
    rmdir("/tmp/test");
    chess.configure(4, 4);
    calib.configure("/tmp/test", chess, 10);

    mu_check(calib.nb_samples == 0);
    mu_check(calib.nb_max_samples == 10);
    mu_check(calib.calibration_path == "/tmp/test");

    return 0;
}

void testSuite(void)
{
    mu_add_test(testChessboard);
    mu_add_test(testChessboardConfigure);
    mu_add_test(testCalibration);
    mu_add_test(testCalibrationConfigure);
}

mu_run_tests(testSuite)
