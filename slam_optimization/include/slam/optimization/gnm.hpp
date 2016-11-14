#ifndef __SLAM_OPTIMIZATION_GNM_HPP__
#define __SLAM_OPTIMIZATION_GNM_HPP__

#include <cmath>

#include "slam/utils/utils.hpp"


namespace slam {

#define EGNMC "GNMOpt is not configured!"
#define EGNMF "Failed to execute GNMOpt.f() [%s]"
#define EGNMH "Hessian is not positive definite!"

class GNMOpt
{
public:
    bool configured;

    int max_iter;
    double step;

    VecX eta;
    VecX x;
    std::function<VecX (VecX x)> f;

    GNMOpt(void);
    int configure(
        int max_iter,
        VecX eta,
        VecX x,
        std::function<VecX (VecX x)> f
    );
    int calcJacobian(MatX &J);
    int optimize(void);
};

}  // end of slam namespace
#endif
