#include <stdio.h>

#include "camera.hpp"
#include "calibration.hpp"


int main(void)
{
    int retval;
    char key_pressed;
    cv::Mat image;
    slam::Camera camera;
    slam::Chessboard chessboard;
    slam::Calibration calibration;
    std::vector<cv::Point2f> corners;
    std::vector<std::vector<cv::Point2f>> image_points;

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

    // capture chessboard images
    while (calibration.state != READY_TO_CALIBRATE) {
        camera.getFrame(image);
        if (calibration.findChessboardCorners(image, corners)) {
            image_points.push_back(corners);
        }

        // show gui
        cv::imshow("Camera Calibration", image);

        // handle events
        key_pressed = cv::waitKey(1);
        if (key_pressed == 27) {  // press ESC to stop
            return 0;
        } else if (key_pressed == 99) {  // capture image
            calibration.saveImage(image, corners);
        }
    }

    // calibrate
    calibration.calibrate(image_points, image.size());

    return 0;
}
