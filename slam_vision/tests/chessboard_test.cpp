#include "slam/utils/munit.hpp"
#include "slam/vision/chessboard.hpp"


// TESTS
int testChessboard(void);
int testChessboardConfigure(void);
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

void testSuite(void)
{
    mu_add_test(testChessboard);
    mu_add_test(testChessboardConfigure);
}

mu_run_tests(testSuite)
