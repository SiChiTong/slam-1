#include "odometry.hpp"
#include "detector.hpp"


int main(void)
{
    cv::Mat frame;
    cv::Mat img_1;
    cv::Mat img_2;
    std::vector<cv::Point2f> pts_1;
    std::vector<cv::Point2f> pts_2;
    std::vector<uchar> status;

    Camera camera;
    VisualOdometry vo;
    FastDetector fast;

    // setup
    vo.configure();
    camera.configure(0, 320, 280);
    fast.configure(20, true);

    camera.getFrame(frame);
    cv::cvtColor(frame, img_1, CV_BGR2GRAY);
    fast.detect(img_1, pts_1);

    while (1) {
        camera.getFrame(frame);
        cv::cvtColor(frame, img_2, CV_BGR2GRAY);

        fast.detect(img_2, pts_2);
        vo.featureTracking(img_1, img_2, pts_1, pts_2, status);
        vo.displayOpticalFlow(frame, pts_1, pts_2);
        vo.measure(pts_1, pts_2);

        img_2.copyTo(img_1);
        pts_1 = pts_2;
    }

    return 0;
}
