#ifndef __SLAM_OPTIMIZATION_OPTIMIZERS_LLS_HPP__
#define __SLAM_OPTIMIZATION_OPTIMIZERS_LLS_HPP__

#include "slam/utils/utils.hpp"


namespace slam {

class LLSSolver
{
public:
    bool configured;

    LLSSolver(void);
    int configure(void);
    int solve(MatX A, MatX b, VecX &x);
};

}  // end of slam namespace
#endif
