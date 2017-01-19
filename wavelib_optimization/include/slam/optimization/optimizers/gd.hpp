#ifndef __SLAM_OPTIMIZATION_OPTIMIZERS_GD_HPP__
#define __SLAM_OPTIMIZATION_OPTIMIZERS_GD_HPP__

#include <cmath>

#include "slam/utils/utils.hpp"


namespace slam {

#define EGDC "GDOpt is not configured!"
#define EGDF "Failed to execute GDOpt.f() [%s]"

class GDOpt
{
public:
    bool configured;

    int max_iter;
    VecX eta;
    VecX x;
    std::function<double (VecX x)> f;

    GDOpt(void);
    int configure(
        int max_iter,
        VecX eta,
        VecX x,
        std::function<double (VecX x)> f
    );
    int calcGradient(VecX &df);
    int optimize(void);
};

}  // end of slam namespace
#endif
