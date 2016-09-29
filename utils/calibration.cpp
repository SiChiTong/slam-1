#include <stdio.h>

#include "camera.hpp"
#include "calibration.hpp"


int main(void)
{
    int retval;
    cv::Mat image;
    Camera camera;
    Chessboard chessboard;
    Calibration calibration;
    std::vector<cv::Point2f> image_points;

    // setup
    retval = chessboard.configure(9, 6);
    if (retval != 0) {
        LOG_ERROR("failed to configure chessboard!");
        return -1;
    }

    retval = calibration.configure("/tmp/calibration", chessboard, 10);
    if (retval != 0) {
        LOG_ERROR("failed to configure calibration!");
        return -1;
    }

    retval = camera.configure(0, 320, 240);
    if (retval != 0) {
        LOG_ERROR("failed to configure camera!");
        return -1;
    }

    // loop
    while (true) {
        camera.getFrame(image);
        calibration.findChessboardCorners(image, image_points);

        // show gui
        cv::imshow("Camera Calibration", image);

        // handle events
        char c = cv::waitKey(1);
        if (c == 27) {  // press ESC to stop
            return 0;
        } else if (c == 99) {  // capture image
            calibration.saveImage(image, image_points);
        }
    }

    return 0;
}
