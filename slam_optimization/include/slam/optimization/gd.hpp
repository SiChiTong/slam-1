#ifndef __SLAM_OPTIMIZATION_GD_HPP__
#define __SLAM_OPTIMIZATION_GD_HPP__

#include <cmath>

#include "slam/utils/utils.hpp"


namespace slam {

class GDSolver
{
public:
    bool configured;

    int max_iter;
    int nb_functions;
    int nb_unknowns;
    VecX eta;
    VecX x;
    std::function<VecX (VecX x)> f;
    std::function<MatX (VecX x)> diff_func;

    GDSolver(void);
    int configure(int max_iter, VecX eta, VecX x);
    int calcJacobian(MatX &J);
    int solve(void);
};

}  // end of slam namespace
#endif
