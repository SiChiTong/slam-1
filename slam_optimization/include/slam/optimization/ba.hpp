#ifndef __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__
#define __SLAM_OPTIMIZATION_BUNDLE_ADJUSTMENT_HPP__

#include <iostream>
#include <math.h>

#include <Eigen/Dense>

#include "slam/utils/utils.hpp"


namespace slam {

class BundleAdjustment
{
public:
    bool configured;

    BundleAdjustment(void);
    int configure(void);
};

}  // end of slam namespace
#endif
