#include "slam/optimization/gnm.hpp"


namespace slam {

GNMOpt::GNMOpt(void)
{
    this->configured = false;

    this->max_iter = 0;
    this->step = 0;

    this->eta;
    this->x;
    this->f = NULL;
}

int GNMOpt::configure(
    int max_iter,
    VecX eta,
    VecX x,
    std::function<VecX (VecX x)> f
)
{
    this->configured = true;

    this->max_iter = max_iter;
    this->step = 0.0001;

    this->eta = eta;
    this->x = x;
    this->f = f;

    return 0;
}

int GNMOpt::calcJacobian(MatX &J)
{
    double step;
    VecX px, nx;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // calculate jacobian
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < this->x.rows(); j++) {
            px = this->x;
            nx = this->x;
            px(j) += step;
            nx(j) -= step;

            J.block(i, j, 3, 1) = (this->f(px) - this->f(nx)) / (step * 2);
        }
    }

    return 0;
}

int GNMOpt::optimize(void)
{
    VecX df;
    VecX H_approx;
    VecX g;

    try {
        // pre-check
        if (this->configured == false) {
            LOG_ERROR(EGNMC);
            return -1;
        }

        // setup
        df.resize(this->x.rows(), 1);
        H_approx.resize(, 1);

        // optimize
        for (int i = 0; i < this->max_iter; i++) {
            this->calcGradient(df);
            H_approx = (df.transpose() * df).inverse();
            g = df.transpose() * this->f(this->x);

            this->x = this->x - this->eta.cwiseProduct(H_approx.inverse() * g);
        }

    } catch(const std::bad_function_call& e) {
        LOG_ERROR(EGNMF, e.what());
        return -2;
    }

    return 0;
}

}  // end of slam namespace
