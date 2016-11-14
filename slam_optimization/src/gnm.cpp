#include "slam/optimization/gnm.hpp"


namespace slam {

GNMOpt::GNMOpt(void)
{
    this->configured = false;

    this->max_iter = 0;
    this->step = 0;

    this->m = 0;
    this->eta;
    this->x;
    this->f = NULL;
}

int GNMOpt::configure(
    int max_iter,
    int m,
    VecX eta,
    VecX x,
    std::function<VecX (VecX x)> f
)
{
    this->configured = true;

    this->max_iter = max_iter;
    this->step = 0.1;

    this->m = m;
    this->eta = eta;
    this->x = x;
    this->f = f;

    return 0;
}

int GNMOpt::calcJacobian(MatX &J)
{
    VecX px, nx, cfd;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // calculate jacobian
    for (int j = 0; j < this->x.rows(); j++) {
        px = this->x;
        nx = this->x;
        px(j) += this->step;
        nx(j) -= this->step;

        cfd = (this->f(px) - this->f(nx)) / (this->step * 2);
        J.block(0, j, this->m, 1) = cfd;
    }

    return 0;
}

int GNMOpt::optimize(void)
{
    MatX J;
    MatX H_approx;
    VecX g;

    try {
        // pre-check
        if (this->configured == false) {
            LOG_ERROR(EGNMC);
            return -1;
        }

        // setup
        J.resize(this->m, this->x.rows());
        H_approx.resize(this->x.rows(), this->x.rows());

        // optimize
        for (int i = 0; i < this->max_iter; i++) {
            this->calcJacobian(J);
            H_approx = (J.transpose() * J).inverse();
            g = J.transpose() * this->f(this->x);
            this->x -= this->eta.cwiseProduct(H_approx.inverse() * g);
        }

    } catch(const std::bad_function_call& e) {
        LOG_ERROR(EGNMF, e.what());
        return -2;
    }

    return 0;
}

}  // end of slam namespace
