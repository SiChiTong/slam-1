#include "odometry.hpp"


VisualOdometry::VisualOdometry(void)
{
    this->configured = false;
}

int VisualOdometry::configure(void)
{
    this->configured = true;

    this->focal_length = 1.0;
    this->principle_point = cv::Point2f(0.0, 0.0);

    return 0;
}

int VisualOdometry::featureTracking(
    cv::Mat img_1,
    cv::Mat img_2,
    std::vector<cv::Point2f> &pts_1,
    std::vector<cv::Point2f> &pts_2,
    std::vector<uchar> &status
)
{
    int correlation_index;
    std::vector<float> err;
    cv::Point2f pt;
    cv::Size win_size;
    cv::TermCriteria term_crit;

    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (pts_1.size() == 0 || pts_2.size() == 0) {
        return -2;
    }

    // setup
    correlation_index = 0;
    win_size = cv::Size(21, 21);
    term_crit = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
        20,
        0.3
    );

    // optical flow
    cv::calcOpticalFlowPyrLK(
        img_1,
        img_2,
        pts_1,
        pts_2,
        status,
        err,
        win_size,
        3,
        term_crit,
        0,
        0.001
    );

    // get rid of points for which the KLT tracking failed or those who
    // have gone outside the frame
    for (int i = 0; i < (int) status.size(); i++) {
        pt = pts_2.at(i - correlation_index);

        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
            if ((pt.x < 0) || (pt.y < 0)) {
                status.at(i) = 0;
            }
            pts_1.erase(pts_1.begin() + i - correlation_index);
            pts_2.erase(pts_2.begin() + i - correlation_index);
            correlation_index++;
        }
    }

}

int VisualOdometry::measure(
    std::vector<cv::Point2f> &pts_1,
    std::vector<cv::Point2f> &pts_2
)
{
    // pre-check
    if (this->configured == false) {
        return -1;
    } else if (pts_1.size() == 0 || pts_2.size() == 0) {
        return -2;
    }

    // essential matrix
    this->E = cv::findEssentialMat(
        pts_1,
        pts_2,
        this->focal_length,
        this->principle_point,
        cv::RANSAC,  // outlier rejection method
        0.999,  // threshold
        1.0,  // confidence level
        this->mask  // output array of N elements
    );
    if (this->E.rows != 3 || this->E.cols != 3) {
        return -3;
    }

    // recover pose
    cv::recoverPose(
        this->E,
        pts_1,
        pts_2,
        this->R,
        this->t,
        this->focal_length,
        this->principle_point,
        this->mask
    );

    return 0;
}

int VisualOdometry::displayOpticalFlow(
    cv::Mat &image,
    std::vector<cv::Point2f> &pts_1,
    std::vector<cv::Point2f> &pts_2
)
{
    cv::Point2f p;
    cv::Point2f q;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // draw flow lines
    for (int i = 0; i < pts_1.size(); i++) {
        p.x = pts_1[i].x;
        p.y = pts_1[i].y;

        q.x = pts_2[i].x;
        q.y = pts_2[i].y;

        cv::line(image, p, q, cv::Scalar(0, 0, 255), 1);
    }

    // display
    cv::imshow("Optical Flow", image);
    cv::waitKey(1);
}
