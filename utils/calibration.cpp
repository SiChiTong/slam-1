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

    std::vector<cv::Point2f> detected_corners;
    std::vector<std::vector<cv::Point2f>> overall_detected_corners;

    // loop
    while (true) {
        camera.capture->read(image);
        calibration.findChessboardCorners(image, detected_corners);
        calibration.saveImage(image);

        // show gui
        cv::imshow("Video Capture", image);

        // handle events
        char c = cv::waitKey(1);
        if (c == 27) {  // press ESC to stop
            break;
        }
    }

    return 0;
}
