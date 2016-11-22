#ifndef __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__
#define __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__

#include <typeinfo>

#include <ceres/ceres.h>

#include "slam/utils/utils.hpp"


namespace slam {

class BAResidual
{
public:
    double fx;
    double fy;
    double cx;
    double cy;

    double x1_x;
    double x1_y;
    double x2_x;
    double x2_y;

    BAResidual(void)
    {
        this->fx = 0.0;
        this->fy = 0.0;
        this->cx = 0.0;
        this->cy = 0.0;

        this->x1_x = 0.0;
        this->x1_y = 0.0;

        this->x2_x = 0.0;
        this->x2_y = 0.0;
    }

    BAResidual(Mat3 K, Vec2 x1, Vec2 x2)
    {
        this->fx = K(0, 0);
        this->fy = K(1, 1);
        this->cx = K(0, 2);
        this->cy = K(1, 2);

        this->x1_x = x1(0);
        this->x1_y = x1(1);

        this->x2_x = x2(0);
        this->x2_y = x2(1);
    }

    template <typename T>
    bool operator()(
        const T * const q,
        const T * const c,
        const T * const x,
        T *residual
    ) const
    {
        Eigen::Matrix<T, 3, 3> K, R;
        Eigen::Matrix<T, 3, 1> C, X;
        Eigen::Matrix<T, 3, 1> x1_est, x2_est;
        Eigen::Matrix<T, 2, 1> x1_est_pixel, x2_est_pixel, err1, err2;

        // camera intrinsics matrix
        K(0, 0) = T(this->fx);
        K(0, 1) = T(0.0);
        K(0, 2) = T(this->cx);

        K(1, 0) = T(0.0);
        K(1, 1) = T(this->fy);
        K(1, 2) = T(this->cy);

        K(2, 0) = T(0.0);
        K(2, 1) = T(0.0);
        K(2, 2) = T(1.0);

        // rotation matrix from quaternion q = (x, y, z, w)
        R(0, 0) = T(1) - T(2) * pow(q[1], 2) - T(2) * pow(q[2], 2);
        R(0, 1) = T(2) * q[0] * q[1] + T(2) * q[3] * q[2];
        R(0, 2) = T(2) * q[0] * q[2] - T(2) * q[3] * q[1];

        R(1, 0) = T(2) * q[0] * q[1] - T(2) * q[3] * q[2];
        R(1, 1) = T(1) - T(2) * pow(q[0], 2) - T(2) * pow(q[2], 2);
        R(1, 2) = T(2) * q[1] * q[2] + T(2) * q[3] * q[2];

        R(2, 0) = T(2) * q[0] * q[2] - T(2) * q[3] * q[1];
        R(2, 1) = T(2) * q[1] * q[2] - T(2) * q[3] * q[0];
        R(2, 2) = T(1) - T(2) * pow(q[0], 2) - T(2) * pow(q[1], 2);

        // camera center
        C << c[0], c[1], c[2];

        // 3D point
        X << x[0], x[1], x[2];

        // calculate reprojection error for camera 1
        x1_est = K * X;
        x1_est_pixel << x1_est(0) / x1_est(2), x1_est(1) / x1_est(2);
        err1 << abs(T(this->x1_x) - x1_est_pixel(0)),
                abs(T(this->x1_y) - x1_est_pixel(1));

        // calculate reprojection error for camera 2
        x2_est = K * R * (X - C);
        x2_est_pixel << x2_est(0) / x2_est(2), x2_est(1) / x2_est(2);
        err2 << abs(T(this->x2_x) - x2_est_pixel(0)),
                abs(T(this->x2_y) - x2_est_pixel(1));

        // calculate error
        residual[0] = err1(0);
        residual[1] = err1(1);

        return true;
    }
};

class BundleAdjustment
{
public:
    bool configured;
    Mat3 K;
    MatX x1_pts;
    MatX x2_pts;

    BundleAdjustment(void);
    int configure(Mat3 K, MatX x1_pts, MatX x2_pts);
    int solve(void);
};

}  // end of slam namespace
#endif
