#include "slam/optimization/optimizers/lma.hpp"


namespace slam {

LMAOpt::LMAOpt(void)
{
    this->configured = false;

    this->max_iter = 0;
    this->step = 0;

    this->eta;
    this->x;
    this->f = NULL;
}

int LMAOpt::configure(
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

int LMAOpt::calcGradient(VecX &df)
{
    VecX px, nx;

    try {
        // pre-check
        if (this->configured == false) {
            LOG_ERROR(ELMAC);
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
        LOG_ERROR(ELMAF, e.what());
        return -2;
    }

    return 0;
}

int LMAOpt::forwardInnerPartialDerviative(int i, int j, double &deriv)
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

int LMAOpt::backwardInnerPartialDerviative(int i, int j, double &deriv)
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

int LMAOpt::calcHessian(MatX &H)
{
    double ifd, ibd, cfd;

    // pre-check
    if (this->configured == false) {
        LOG_ERROR(ELMAC);
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

int LMAOpt::optimize(void)
{
    VecX df;
    VecX H_approx;
    VecX g;

    try {
        // pre-check
        if (this->configured == false) {
            LOG_ERROR(ELMAC);
            return -1;
        }

        // setup
        df.resize(this->x.rows(), 1);
        H_approx.resize(1, 1);

        // optimize
        for (int i = 0; i < this->max_iter; i++) {
            this->calcGradient(df);
            H_approx = (df.transpose() * df).inverse();
            g = df.transpose() * this->f(this->x);

            this->x = this->x - this->eta.cwiseProduct(H_approx.inverse() * g);
        }

    } catch(const std::bad_function_call& e) {
        LOG_ERROR(ELMAF, e.what());
        return -2;
    }

    return 0;
}

}  // end of slam namespace
