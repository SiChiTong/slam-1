#include "slam/optimization/gd.hpp"


namespace slam {

slam::VecX beale(slam::VecX x)
{
    slam::VecX y(1);

    y << pow((1.5 - x(0) + x(0) * x(1)), 2)
         + pow((2.25 - x(0) + x(0) * pow(x(1), 2)), 2)
         + pow((2.625 - x(0) + x(0) * pow(x(1), 3)), 2);

    return y;
}

GDSolver::GDSolver(void)
{
    this->configured = false;

    this->max_iter = 1000;
}

int GDSolver::configure(int max_iter, VecX eta, VecX x)
{
    this->configured = true;

    this->max_iter = max_iter;
    this->eta = eta;
    this->x = x;

    return 0;
}

int GDSolver::calcJacobian(MatX &J)
{
    double step;
    VecX px, nx;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // setup
    step = 0.001;
    px.resize(this->nb_unknowns);
    nx.resize(this->nb_unknowns);

    // calculate jacobian
    for (int i = 0; i < this->nb_functions; i++) {
        for (int j = 0; j < this->nb_unknowns; j++) {
            px = this->x;
            nx = this->x;
            px(j) += step;
            nx(j) -= step;

            J.block(i, j, 1, 1) = (this->f(px) - this->f(nx)) / (step * 2);
        }
    }

    return 0;
}

int GDSolver::solve(void)
{
    MatX J;
    VecX G;

    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // setup
    J.resize(this->nb_functions, this->nb_unknowns);

    // optimize
    for (int i = 0; i < this->max_iter; i++) {
        this->calcJacobian(J);
        G = this->f(this->x);
        this->x = this->x - this->eta.cwiseProduct(J.transpose() * G);
    }

    return 0;
}

}  // end of slam namespace
