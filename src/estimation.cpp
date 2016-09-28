#include "estimation.hpp"


// KALMAN FILTER
KalmanFilter::KalmanFilter(void)
{
    this->initialized = false;
}

int KalmanFilter::init(VecX mu, MatX R, MatX C, MatX Q)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->mu = mu;

    this->B = MatX::Zero(nb_states, nb_states);
    this->R = R;

    this->C = C;
    this->Q = Q;

    this->S = MatX::Identity(nb_states, nb_states);
    this->I = MatX::Identity(nb_states, nb_states);
    this->K = MatX::Zero(nb_states, nb_states);

    this->mu_p = VecX::Zero(nb_states);
    this->S_p = MatX::Zero(nb_states, nb_states);

    return 0;
}

int KalmanFilter::estimate(MatX A, VecX y)
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

int ExtendedKalmanFilter::init(VecX mu, MatX R, MatX Q)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->mu = mu;

    this->R = R;
    this->Q = Q;

    this->S = MatX::Identity(nb_states, nb_states);
    this->I = MatX::Identity(nb_states, nb_states);
    this->K = MatX::Zero(nb_states, nb_states);

    this->mu_p = VecX::Zero(nb_states);
    this->S_p = MatX::Zero(nb_states, nb_states);

    return 0;
}

int ExtendedKalmanFilter::predictionUpdate(VecX g, MatX G)
{
    mu_p = g;
    S_p = G * S * G.transpose() + R;

    return 0;
}

int ExtendedKalmanFilter::measurementUpdate(VecX h, MatX H, VecX y)
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

int ParticleFilter::init(int M, VecX mu)
{
    int nb_states;

    nb_states = mu.size();
    this->initialized = true;
    this->M = M;
    this->mu = mu;

    // this->mu_p = VecX::Zero(nb_states);
    // this->S_p = MatX::Zero(nb_states, nb_states);

    return 0;
}

int ParticleFilter::estimate(
    std::vector<VecX> X_p,
    std::vector<VecX> hX_p,
    VecX y
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

    VecX x_p;
    VecX hx_p;

    for (int i = 0; i < this->M; i++) {
        x_p = X_p[i];
        hx_p = hX_p[i];

    }


    return 0;
}
