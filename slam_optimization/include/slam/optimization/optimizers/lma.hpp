#ifndef __SLAM_OPTIMIZATION_OPTIMIZERS_LMA_HPP__
#define __SLAM_OPTIMIZATION_OPTIMIZERS_LMA_HPP__

#include <cmath>

#include "slam/utils/utils.hpp"


namespace slam {

#define ELMAC "LMAOpt is not configured!"
#define ELMAF "Failed to execute LMAOpt.f() [%s]"
#define ELMAH "Hessian is not positive definite!"

class LMAOpt
{
public:
    bool configured;

    int max_iter;
    double step;

    VecX eta;
    VecX x;
    std::function<double (VecX x)> f;

    LMAOpt(void);
    int configure(
        int max_iter,
        VecX eta,
        VecX x,
        std::function<double (VecX x)> f
    );
    int calcGradient(VecX &df);
    int forwardInnerPartialDerviative(int i, int j, double &deriv);
    int backwardInnerPartialDerviative(int i, int j, double &deriv);
    int calcHessian(MatX &H);
    int optimize(void);
};

}  // end of slam namespace
#endif
