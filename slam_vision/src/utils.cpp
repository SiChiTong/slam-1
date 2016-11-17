#include "slam/vision/utils.hpp"


namespace slam {

void cvmat2mat(cv::Mat A, MatX &B)
{
    B.resize(A.rows, A.cols);
    for (int i = 0; i < A.rows; i++) {
        for (int j = 0; j < A.cols; j++) {
            B(i, j) = A.at<double>(i, j);
        }
    }
}

void cvpts2mat(std::vector<cv::Point2f> points, MatX &mat)
{
    cv::Point2f p;

    mat.resize(points.size(), 3);
    for (int i = 0; i < points.size(); i++) {
        p = points[i];
        mat(i, 0) = p.x;
        mat(i, 1) = p.y;
        mat(i, 2) = 1.0;
    }
}

} // end of slam namespace
