#include "slam/vision/utils.hpp"


namespace slam {

void convert_cvmat(cv::Mat A, MatX &B)
{
    B.resize(A.rows, A.cols);
    for (int i = 0; i < A.rows; i++) {
        for (int j = 0; j < A.cols; j++) {
            B(i, j) = A.at<double>(i, j);
        }
    }
}

void convert_cvpts(std::vector<cv::Point2f> points, MatX &mat)
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

void combine_cvimgs(cv::Mat img1, cv::Mat img2, cv::Mat &out)
{
    cv::Size size1;
    cv::Size size2;

    // setup
    size1 = img1.size();
    size2 = img2.size();
    out = cv::Mat(
        size1.height,
        size1.width + size2.width,
        img1.type()
    );

    // copy image 1 to the left
    out.adjustROI(0, 0, 0, -size2.width);
    img1.copyTo(out);

    // copy image 2 to the right
    out.adjustROI(0, 0, -size1.width, size2.width);
    img2.copyTo(out);

    // restore original roi
    out.adjustROI(0, 0, size1.width, 0);
}

} // end of slam namespace
