#ifndef __SLAM_ESTIMATION_HPP__
#define __SLAM_ESTIMATION_HPP__

#include <iostream>
#include <functional>

#include "slam/utils/utils.hpp"


namespace slam {

class KalmanFilter
{
public:
    bool initialized;
    VecX mu;

    MatX B;
    MatX R;

    MatX C;
    MatX Q;

    MatX S;
    MatX I;
    MatX K;

    VecX mu_p;
    MatX S_p;

    KalmanFilter(void);
    int init(
        VecX mu,
        MatX R,
        MatX C,
        MatX Q
    );
    int estimate(MatX A, VecX y);
};


class ExtendedKalmanFilter
{
public:
    bool initialized;
    VecX mu;

    MatX R;
    MatX Q;

    MatX S;
    MatX I;
    MatX K;

    VecX mu_p;
    MatX S_p;

    ExtendedKalmanFilter(void);
    int init(VecX mu, MatX R, MatX Q);
    int predictionUpdate(VecX g, MatX G);
    int measurementUpdate(VecX h, MatX H, VecX y);
};


class ParticleFilter
{
public:
    bool initialized;
    VecX mu;

    int M;

    // VecX mu_p;
    // MatX S_p;

    ParticleFilter(void);
    int init(int M, VecX mu);
    int estimate(
        std::vector<VecX> X_p,
        std::vector<VecX> hX_p,
        VecX y
    );
};

} // end of slam namespace
#endif
