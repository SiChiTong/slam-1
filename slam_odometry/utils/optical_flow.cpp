#include "slam/odometry/odometry.hpp"
#include "slam/feature/fast.hpp"


int main(void)
{
    cv::Mat frame;
    cv::Mat img_1;
    cv::Mat img_2;
    std::vector<cv::Point2f> pts_1;
    std::vector<cv::Point2f> pts_2;
    std::vector<uchar> status;

    slam::Camera camera;
    slam::VisualOdometry vo;
    slam::FastDetector fast;

    // setup
    vo.configure();
    vo.focal_length = 820.0;
    vo.principle_point = cv::Point2f(290.9, 296.1);

    camera.configure(0, 640, 480);
    fast.configure(20, true);

    camera.getFrame(frame);
    cv::cvtColor(frame, img_1, cv::COLOR_BGR2GRAY);
    fast.detect(img_1, pts_1);

    float yaw;

    yaw = 0;

    float x;
    float y;
    float z;

    x = 0;
    y = 0;
    z = 0;

    while (1) {
        camera.getFrame(frame);
        cv::cvtColor(frame, img_2, cv::COLOR_BGR2GRAY);

        fast.detect(img_2, pts_2);
        vo.featureTracking(img_1, img_2, pts_1, pts_2, status);
        vo.displayOpticalFlow(frame, pts_1, pts_2);
        vo.measure(pts_1, pts_2);

        // yaw += atan2(vo.R.at<double>(1, 0), vo.R.at<double>(0, 0));
        // std::cout << yaw << std::endl;

        // x = vo.t.at<double>(0);
        // y = vo.t.at<double>(1);
        // z = vo.t.at<double>(2);
        // std::cout << x << " " << y << " " << z << std::endl;

        img_2.copyTo(img_1);
        pts_1 = pts_2;
    }

    return 0;
}
