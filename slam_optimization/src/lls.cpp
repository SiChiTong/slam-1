#include "slam/optimization/lls.hpp"


namespace slam {

LLSSolver::LLSSolver(void)
{
    this->configured = false;
}

int LLSSolver::configure(void)
{
    this->configured = true;

    return 0;
}

int LLSSolver::solve(void)
{

    return 0;
}

}  // end of slam namespace

