#ifndef __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__
#define __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__


#include <ceres/ceres.h>

#include "slam/utils/utils.hpp"


namespace slam {

class BAResidual
{
public:
    double x;
    double y;

    BAResidual(Eigen::Vector2d x)
    {
        this->x = x(0);
        this->y = x(1);
    }

    template <typename T>
    bool operator()(const T * const m, const T * const c, T *residual) const
    {
        residual[0] = T(this->y) - exp(m[0] * T(this->x) + c[0]);
        return true;
    }
};

class BundleAdjustment
{
public:
    bool configured;

    BundleAdjustment(void);
    int configure(void);
    int solve(void);
};

}  // end of slam namespace
#endif
