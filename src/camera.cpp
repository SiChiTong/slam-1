#include "camera.hpp"


Camera::Camera(void)
{
    this->configured = false;

    this->capture_index = 0;
    this->image_width = 0;
    this->image_height = 0;
    this->capture = NULL;
}

int Camera::configure(int capture_index, int image_width, int image_height)
{
    // setup
	this->configured = true;
    this->capture = new cv::VideoCapture(capture_index);
    this->capture_index = capture_index;
    this->image_width = image_width;
    this->image_height = image_height;

    // open camera
    if (this->capture->isOpened() == 0) {
        LOG_ERROR("failed to open webcam!");
        return -1;
    }

    // configure image resolution
    this->capture->set(cv::CAP_PROP_FRAME_WIDTH, image_width);
    this->capture->set(cv::CAP_PROP_FRAME_HEIGHT, image_height);

    return 0;
}

int Camera::getFrame(cv::Mat &image)
{
	this->capture->read(image);
	return 0;
}
