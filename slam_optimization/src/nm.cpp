#include "slam/optimization/nm.hpp"


namespace slam {

NMOpt::NMOpt(void)
{
    this->configured = false;

    this->max_iter = 0;
    this->step = 0;

    this->eta;
    this->x;
    this->f = NULL;
}

int NMOpt::configure(
    int max_iter,
    VecX eta,
    VecX x,
    std::function<double (VecX x)> f
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

int NMOpt::calcGradient(VecX &df)
{
    VecX px, nx;

    try {
        // pre-check
        if (this->configured == false) {
            LOG_ERROR(ENMC);
            return -1;
        }

        // calculate gradient using central finite difference
        for (int i = 0; i < this->x.rows(); i++) {
            px = this->x;
            nx = this->x;
            px(i) += this->step;
            nx(i) -= this->step;
            df(i) = (this->f(px) - this->f(nx)) / (this->step * 2);
        }

    } catch(const std::bad_function_call& e) {
        LOG_ERROR(ENMF, e.what());
        return -2;
    }

    return 0;
}

int NMOpt::forwardInnerPartialDerviative(int i, int j, double &deriv)
{
    VecX fp, bp, fd;

    // setup
    fp = this->x;
    bp = this->x;

    fp(i) += this->step;
    fp(j) += this->step;

    bp(i) += this->step;
    bp(j) -= this->step;

    deriv = (this->f(fp) - this->f(bp)) / (2 * this->step);

    return 0;
}

int NMOpt::backwardInnerPartialDerviative(int i, int j, double &deriv)
{
    VecX fp, bp, fd;

    fp = this->x;
    bp = this->x;

    fp(i) -= this->step;
    fp(j) += this->step;

    bp(i) -= this->step;
    bp(j) -= this->step;

    deriv = (this->f(fp) - this->f(bp)) / (2 * this->step);

    return 0;
}

int NMOpt::calcHessian(MatX &H)
{
    double ifd, ibd, cfd;

    // pre-check
    if (this->configured == false) {
        LOG_ERROR(ENMC);
        return -1;
    }

    // calculate hessian
    for (int i = 0; i < this->x.rows(); i++) {
        for (int j = 0; j < this->x.rows(); j++) {
            this->forwardInnerPartialDerviative(j, i, ifd);
            this->backwardInnerPartialDerviative(j, i, ibd);
            cfd = (ifd - ibd) / (2 * this->step);
            H(i, j) = cfd;
        }
    }

    return 0;
}

int NMOpt::optimize(void)
{
    VecX df;
    MatX H;
    MatX H_inv;

    try {
        // pre-check
        if (this->configured == false) {
            LOG_ERROR(ENMC);
            return -1;
        }

        // setup
        df.resize(this->x.rows(), 1);
        H.resize(this->x.rows(), this->x.rows());

        // calculate Hessian and inverse it
        this->calcHessian(H);
        if (isposdef(H) == false) {
            LOG_ERROR(ENMH);
            return -3;
        }
        H_inv = H.inverse();

        // optimize
        for (int i = 0; i < this->max_iter; i++) {
            this->calcGradient(df);
            this->x = this->x - this->eta.cwiseProduct(H_inv * df);
        }

    } catch(const std::bad_function_call& e) {
        LOG_ERROR(ENMF, e.what());
        return -2;
    }

    return 0;
}

}  // end of slam namespace
