#include "estimation.hpp"


// KALMAN FILTER
KalmanFilter::KalmanFilter(void)
{
    this->initialized = false;
}

int KalmanFilter::init(VecXd mu, MatXd R, MatXd C, MatXd Q)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->mu = mu;

    this->B = MatXd::Zero(nb_states, nb_states);
    this->R = R;

    this->C = C;
    this->Q = Q;

    this->S = MatXd::Identity(nb_states, nb_states);
    this->I = MatXd::Identity(nb_states, nb_states);
    this->K = MatXd::Zero(nb_states, nb_states);

    this->mu_p = VecXd::Zero(nb_states);
    this->S_p = MatXd::Zero(nb_states, nb_states);

    return 0;
}

int KalmanFilter::estimate(MatXd A, VecXd y)
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



// EXTENDED KALMAN FILTER
ExtendedKalmanFilter::ExtendedKalmanFilter(void)
{
    this->initialized = false;
}

int ExtendedKalmanFilter::init(VecXd mu, MatXd R, MatXd Q)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->mu = mu;

    this->R = R;
    this->Q = Q;

    this->S = MatXd::Identity(nb_states, nb_states);
    this->I = MatXd::Identity(nb_states, nb_states);
    this->K = MatXd::Zero(nb_states, nb_states);

    this->mu_p = VecXd::Zero(nb_states);
    this->S_p = MatXd::Zero(nb_states, nb_states);

    return 0;
}

int ExtendedKalmanFilter::predictionUpdate(VecXd g, MatXd G, VecXd u, float dt)
{
    mu_p = g;
    S_p = G * S * G.transpose() + R;

    return 0;
}

int ExtendedKalmanFilter::measurementUpdate(VecXd, h, MatXd H, VecXd y)
{
    K = S_p * H.transpose() * (H * S_p * H.transpose() + Q).inverse();
    mu = mu_p + K * (y - h);
    S = (I - K * H) * S_p;

    return 0;
}



// Particle Filter
ParticleFilter::ParticleFilter(void)
{
    this->initialized = false;
}
