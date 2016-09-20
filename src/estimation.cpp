#include "estimation.hpp"


// KALMAN FILTER
KalmanFilter::KalmanFilter(void)
{
    this->initialized = false;
}

int KalmanFilter::init(VecXf mu, MatXf R, MatXf C, MatXf Q)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->mu = mu;

    this->B = MatXf::Zero(nb_states, nb_states);
    this->R = R;

    this->C = C;
    this->Q = Q;

    this->S = MatXf::Identity(nb_states, nb_states);
    this->I = MatXf::Identity(nb_states, nb_states);
    this->K = MatXf::Zero(nb_states, nb_states);

    this->mu_p = VecXf::Zero(nb_states);
    this->S_p = MatXf::Zero(nb_states, nb_states);

    return 0;
}

int KalmanFilter::estimate(MatXf A, VecXf y)
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

int ExtendedKalmanFilter::init(VecXf mu, MatXf R, MatXf Q)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->mu = mu;

    this->R = R;
    this->Q = Q;

    this->S = MatXf::Identity(nb_states, nb_states);
    this->I = MatXf::Identity(nb_states, nb_states);
    this->K = MatXf::Zero(nb_states, nb_states);

    this->mu_p = VecXf::Zero(nb_states);
    this->S_p = MatXf::Zero(nb_states, nb_states);

    return 0;
}

int ExtendedKalmanFilter::predictionUpdate(VecXf g, MatXf G)
{
    mu_p = g;
    S_p = G * S * G.transpose() + R;

    return 0;
}

int ExtendedKalmanFilter::measurementUpdate(VecXf h, MatXf H, VecXf y)
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

int ParticleFilter::init(int M, VecXf mu)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->M = M;
    this->mu = mu;

    // this->mu_p = VecXf::Zero(nb_states);
    // this->S_p = MatXf::Zero(nb_states, nb_states);

    return 0;
}

int ParticleFilter::estimate(
    std::vector<VecXf> X_p,
    std::vector<VecXf> hX_p,
    VecXf y
)
{
//     // % sampling
//     // for m = 1:pf.M
//     //     Xp = feval(Xp_func, pf.X(m), u, pf.R);
//     //     hXp = feval(hXp_func, Xp);
//     //
//     //     pf.Xp(:, m) = Xp;
//     //     pf.w(m) = max(1e-8, mvnpdf(y, hXp, pf.Q));
//     // end
//
//     // % importance resampling
//     // W = cumsum(pf.w);
//     // for m = 1:pf.M
//     //     seed = W(end) * rand(1);
//     //     pf.X(m) = pf.Xp(find(W > seed, 1));
//     // end
//
//     // % record mean particle
//     // pf.mu = mean(pf.X);
//     // pf.S = var(pf.X);
//
//     for (int i; i < this->M; i++) {
//
//     }

    VecXf x_p;
    VecXf hx_p;

    for (int i = 0; i < this->M; i++) {
        x_p = X_p[i];
        hx_p = hX_p[i];

    }


    return 0;
}
