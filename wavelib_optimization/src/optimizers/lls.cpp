#include "slam/optimization/optimizers/lls.hpp"


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

int LLSSolver::solve(MatX A, MatX b, VecX &x)
{
    // pre-check
    if (this->configured == false) {
        return -1;
    }

    // perform linear least squares
    x = (A.transpose() * A).inverse() * A.transpose() * b;

    return 0;
}

}  // end of slam namespace
