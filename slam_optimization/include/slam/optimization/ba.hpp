#ifndef __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__
#define __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__


#include <ceres/ceres.h>

#include "slam/utils/utils.hpp"


namespace slam {

struct CostFunctor {
    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
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
