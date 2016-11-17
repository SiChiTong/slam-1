#ifndef __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__
#define __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__


#include <ceres/ceres.h>

#include "slam/utils/utils.hpp"


namespace slam {

class BAResidual
{
public:
    Mat3 K;
    Vec3 x1;
    Vec3 x2;

    BAResidual(void)
    {

    }

    BAResidual(Mat3 K, Vec3 x1, Vec3 x2)
    {
        this->K = K;
        this->x1 = x1;
        this->x2 = x2;
    }

    template <typename T>
    void setupRotationMatrix(const T * const q, Mat3 &R)
    {
        double qx, qy, qz, qw;

        // rotation matrix - parameterized quaternion
        qx = q[0];
        qy = q[1];
        qz = q[2];
        qw = q[3];

        R(0, 0) = 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2);
        R(0, 1) = 2 * qx * qy + 2 * qw * qz;
        R(0, 2) = 2 * qx * qz - 2 * qw * qy;

        R(1, 0) = 2 * qx * qy - 2 * qw * qz;
        R(1, 1) = 1 - 2 * pow(qx, 2) - 2 * pow(qz, 2);
        R(1, 2) = 2 * qy * qz + 2 * qw * qz;

        R(2, 0) = 2 * qx * qz - 2 * qw * qy;
        R(2, 1) = 2 * qy * qz - 2 * qw * qx;
        R(2, 2) = 1 - 2 * pow(qx, 2) - 2 * pow(qy, 2);
    }

    template <typename T>
    void setupCameraCenter(const T * const c, Vec3 &C)
    {
        C << c[0], c[1], c[2];
    }

    template <typename T>
    void setup3DPoint(const T * const x, Vec3 &X)
    {
        X << x[0], x[1], x[2];
    }

    double calcReprojectionError(Mat3 R, Vec3 C, Vec3 X, Vec2 m_tilde)
    {
        Vec3 x;
        Vec2 m;

        // 3D point
        X << x[0], x[1], x[2];

        // calculate reprojection error
        x = this->K * R * (X - C);
        m << x(0) / x(2),
             x(1) / x(2);

        // euclidean distance between observed and predicted
        return (m_tilde - m).norm();
    }

    template <typename T>
    bool operator()(
        const T * const q,
        const T * const c,
        const T * const x,
        T *residual
    )
    {
        double d1, d2;
        Mat3 R;
        Vec3 C, X;

        // setup
        R = this->setupRotationMatrix(q);
        C = this->setupCameraCenter(c);
        X = this->setup3DPoint(x);

        // calculate reprojection errors for x1 and x2 point correspondances
        d1 = this->calcReprojectionError(q, c, x, this->x1);
        d2 = this->calcReprojectionError(q, c, x, this->x2);

        // calculate error
        residual[0] = pow(d1, 2) + pow(d2, 2);

        return true;
    }
};

class BundleAdjustment
{
public:
    bool configured;
    // std::vector<cv::Point2f> x1_pts;
    // std::vector<cv::Point2f> x2_pts;

    BundleAdjustment(void);
    // int configure(
    //     std::vector<cv::Point2f> x1_pts,
    //     std::vector<cv::Point2f> x2_pts
    // );
    int solve(void);
};

}  // end of slam namespace
#endif
