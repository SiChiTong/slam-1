#ifndef __SLAM_OPTIMIZATION_NM_HPP__
#define __SLAM_OPTIMIZATION_NM_HPP__

#include <cmath>

#include "slam/utils/utils.hpp"


namespace slam {

#define ENMC "NMOpt is not configured!"
#define ENMF "Failed to execute NMOpt.f() [%s]"
#define ENMH "Hessian is not positive definite!"

class NMOpt
{
public:
    bool configured;

    int max_iter;
    double step;

    VecX eta;
    VecX x;
    std::function<double (VecX x)> f;

    NMOpt(void);
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
