#ifndef __wavelib_OPTIMIZATION_OPTIMIZERS_LLS_HPP__
#define __wavelib_OPTIMIZATION_OPTIMIZERS_LLS_HPP__

#include "wavelib/utils/utils.hpp"


namespace wavelib {

class LLSSolver
{
public:
    bool configured;

    LLSSolver(void);
    int configure(void);
    int solve(MatX A, MatX b, VecX &x);
};

}  // end of wavelib namespace
#endif
