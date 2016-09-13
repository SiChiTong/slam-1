#ifndef __SLAM_ESTIMATION_HPP__
#define __SLAM_ESTIMATION_HPP__

#include <iostream>
#include <Eigen/Geometry>


class KalmanFilter
{
public:
    bool initialized;
    Eigen::VectorXd mu;

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd R;

    Eigen::MatrixXd C;
    Eigen::MatrixXd Q;

    Eigen::MatrixXd S;
    Eigen::MatrixXd I;
    Eigen::MatrixXd K;

    Eigen::VectorXd mu_p;
    Eigen::MatrixXd S_p;

    KalmanFilter(void);
    int setup(Eigen::VectorXd mu);
    int estimate(Eigen::VectorXd y, float dt);
};

#endif
