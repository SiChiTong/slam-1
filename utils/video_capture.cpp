#include "vision.hpp"


int main(void)
{
    Vision vision;
    cv::Mat image;

	int fps;
	int fourcc;
	std::string file_name;
	cv::Size video_resolution;
	cv::VideoWriter video_writer;

    // setup camera
    vision.capture_index = 0;
    vision.image_width = 320;
    vision.image_height = 280;
    vision.initCamera();

	// setup video writer
	fps = 10;
	fourcc = CV_FOURCC('M', 'J', 'P', 'G');
	file_name = "test_video.avi";
	video_resolution = cv::Size(320, 280);

	video_writer.open(file_name, fourcc, fps, video_resolution);
	if (video_writer.isOpened() != true) {
		LOG_ERROR("cannot open video writer!");
		return -1;

	} else {
		LOG_INFO("video writer opened!");

	}

    // loop
    while (true) {
        vision.capture->read(image);
        cv::imshow("Video Capture", image);
		video_writer.write(image);

        char c = cv::waitKey(1);
		if (c == 27) break;  // press ESC to stop
    }

    return 0;
}
