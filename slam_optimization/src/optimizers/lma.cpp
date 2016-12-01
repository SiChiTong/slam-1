#include "slam/optimization/optimizers/lma.hpp"


namespace slam {

LMASettings::LMASettings(void)
{
    this->max_iter = 100;
    this->lambda = 0.01;
    this->function = nullptr;
    this->jacobian = nullptr;
    this->nb_inputs = 0;
    this->nb_params = 0;

    this->x = MatX::Zero(1, 1);
    this->y = MatX::Zero(1, 1);
    this->beta = MatX::Zero(1, 1);
}

LMA::LMA(void)
{
    this->configured = false;

    this->max_iter = 100;
    this->lambda = 0.01;
    this->function = nullptr;
    this->jacobian = nullptr;
    this->nb_inputs = 0;
    this->nb_params = 0;

    this->x = MatX::Zero(1, 1);
    this->y = MatX::Zero(1, 1);
    this->beta = MatX::Zero(1, 1);

    this->y_est = MatX::Zero(1, 1);
    this->diff = MatX::Zero(1, 1);

    this->J = MatX::Zero(1, 1);
    this->H = MatX::Zero(1, 1);

    this->error = FLT_MAX;
}

int LMA::configure(LMASettings settings)
{
    this->configured = true;

    this->max_iter = settings.max_iter;
    this->lambda = settings.lambda;
    this->function = settings.function;
    this->jacobian = settings.jacobian;
    this->nb_inputs = settings.nb_inputs;
    this->nb_params = settings.nb_params;

    this->x = settings.x;
    this->y = settings.y;
    this->beta = settings.beta;

    this->y_est = MatX::Zero(this->y.rows(), 1);
    this->diff = MatX::Zero(this->y.rows(), 1);

    this->J = MatX::Zero(this->y.rows(), this->nb_params);
    this->H = MatX::Zero(this->nb_params, this->nb_params);

    this->error = FLT_MAX;

    return 0;
}

int LMA::evalFunction(VecX beta, double &error)
{
    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // evaluate function
    for (int i = 0; i < this->y.rows(); i++) {
        this->y_est(i, 0) = this->function(
            this->x.block(i, 0, 1, this->nb_inputs),
            beta
        );
    }

    // calculate error
    this->diff = this->y - y_est;
    error = diff.dot(diff);

    return 0;
}

int LMA::calcGradients(VecX beta)
{
    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // calculate jacobian
    for (int i = 0; i < this->y.rows(); i++) {
        this->J.block(i, 0, 1, this->nb_params) = this->jacobian(
            this->x.block(i, 0, 1, this->nb_inputs),
            beta
        ).transpose();
    }

    // approximate Hessian
    this->H = this->J.transpose() * this->J;

    return 0;
}

int LMA::iterate(void)
{
    MatX I, H_est;
    VecX dp, beta_est;
    double error_est;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // update hessian
    I = MatX::Identity(this->nb_params, this->nb_params);
    H_est = this->H;
    H_est += this->lambda * I;

    // update params
    dp = -H_est.inverse() * (this->J.transpose() * this->diff);
    beta_est = this->beta;
    beta_est += dp;

    // evaluate new error
    this->evalFunction(beta_est, error_est);

    // adjust damping factor
    if (error_est > this->error) {
        this->lambda *= 10.0;

    } else {
        this->lambda /= 10.0;
        this->beta = beta_est;
        this->error = error_est;
        this->calcGradients(beta_est);

    }

    return 0;
}

int LMA::optimize(void)
{
    // pre-check
    if (this->configured == false) {
        LOG_ERROR(ELMAC);
        return -1;
    }

    // optimize
    try {
        // initialize error and gradients
        this->evalFunction(this->beta, this->error);
        this->calcGradients(this->beta);

        // iterate
        for (int i = 0; i < this->max_iter; i++) {
            printf("[%d]: %f\n", i, this->error);
            this->iterate();
        }

    } catch(const std::bad_function_call& e) {
        LOG_ERROR(ELMAF, e.what());
        return -2;
    }

    return 0;
}

}  // end of slam namespace
