#include "estimation.hpp"


KalmanFilter::KalmanFilter(void)
{
	this->initialized = false;

}

int KalmanFilter::init(
	Eigen::VectorXd mu,
	Eigen::MatrixXd R,
	Eigen::MatrixXd C,
	Eigen::MatrixXd Q
)
{
	int nb_states;

	nb_states = mu.size();
	this->initialized = true;
    this->mu = mu;

    this->B = Eigen::MatrixXd::Zero(nb_states, nb_states);
    this->R = R;

    this->C = C;
	this->Q = Q;

    this->S = Eigen::MatrixXd::Identity(nb_states, nb_states);
    this->I = Eigen::MatrixXd::Identity(nb_states, nb_states);
    this->K = Eigen::MatrixXd::Zero(nb_states, nb_states);

    this->mu_p = Eigen::VectorXd::Zero(nb_states);
    this->S_p = Eigen::MatrixXd::Zero(nb_states, nb_states);

    return 0;
}

int KalmanFilter::estimate(Eigen::MatrixXd A, Eigen::VectorXd y)
{
    // prediction update
    mu_p = A * mu;
    S_p = A * S * A.transpose() + R;

    // measurement update
    K = S_p * C.transpose() * (C * S_p * C.transpose() + Q).inverse();
    mu = mu_p + K * (y - C * mu_p);
    S = (I - K * C) * S_p;

    return 0;
}
