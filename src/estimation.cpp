#include "estimation.hpp"


int KalmanFilter::setup(Eigen::VectorXd mu)
{
    Eigen::MatrixXd A(9, 9);
    Eigen::MatrixXd B(9, 9);
    Eigen::MatrixXd R(9, 9);

    Eigen::MatrixXd C(3, 9);
    Eigen::MatrixXd Q(3, 3);

    Eigen::MatrixXd S(9, 9);
    Eigen::MatrixXd I(9, 9);
    Eigen::MatrixXd K(9, 9);

    Eigen::VectorXd mu_p(9);
    Eigen::MatrixXd S_p(9, 9);

    // transition matrix (assuming constant acceleration)
    A << 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1;

    // input matrix
    B = Eigen::MatrixXd::Zero(9, 9);

    // motion noise
    R << 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.5, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.5, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1.0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1.0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1.0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1.0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1.0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1.0;

    // measurement model
    C << 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0;

    // measurement noise
    Q << 20, 0, 0,
         0, 20, 0,
         0, 0, 20;

    // misc
    S = Eigen::MatrixXd::Identity(9, 9) * 100;
    I = Eigen::MatrixXd::Identity(9, 9);
    K = Eigen::MatrixXd::Zero(9, 9);
    // mu_p = Eigen::VectorXd::Zero(9);
    S_p = Eigen::MatrixXd::Zero(9, 9);

    // configure kalman filter
    this->mu = mu;

    this->A = A;
    this->B = B;
    this->R = R;

    this->C = C;
    this->Q = Q;

    this->S = S;
    this->I = I;
    this->K = K;

    this->mu_p = mu_p;
    this->S_p = S_p;

    return 0;
}

int KalmanFilter::estimate(Eigen::VectorXd y, float dt)
{
    // transition matrix (constant acceleration)
    this->A <<
            1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0, 0,
            0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0,
            0, 0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0,
            0, 0, 0, 1.0, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 1.0, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 1.0, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 1.0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1.0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1.0;

    // transition matrix (constant velocity)
    // this->A << 1, 0, 0, dt, 0, 0, 0, 0, 0,
    //   0, 1, 0, 0, dt, 0, 0, 0, 0,
    //   0, 0, 1, 0, 0, dt, 0, 0, 0,
    //   0, 0, 0, 1, 0, 0, 0, 0, 0,
    //   0, 0, 0, 0, 1, 0, 0, 0, 0,
    //   0, 0, 0, 0, 0, 1, 0, 0, 0,
    //   0, 0, 0, 0, 0, 0, 0, 0, 0,
    //   0, 0, 0, 0, 0, 0, 0, 0, 0,
    //   0, 0, 0, 0, 0, 0, 0, 0, 0;

    // prediction update
    this->mu_p = this->A * this->mu;
    this->S_p = this->A * this->S * this->A.transpose() + this->R;

    // measurement update
    this->K = this->S_p * this->C.transpose() * (this->C * this->S_p * this->C.transpose() + this->Q).inverse();
    this->mu = this->mu_p + this->K * (y - this->C * this->mu_p);
    this->S = (this->I - this->K * this->C) * this->S_p;

    return 0;
}
