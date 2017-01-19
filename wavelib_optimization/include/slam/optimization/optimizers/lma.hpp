#ifndef __SLAM_OPTIMIZATION_OPTIMIZERS_LMA_HPP__
#define __SLAM_OPTIMIZATION_OPTIMIZERS_LMA_HPP__

#include <cmath>
#include <float.h>

#include "slam/utils/utils.hpp"


namespace slam {

#define ELMAC "LMA is not configured!"
#define ELMAF "Failed to execute LMA.f() [%s]"
#define ELMAH "Hessian is not positive definite!"

#define LMA_BIND(X) \
    std::bind(X, std::placeholders::_1, std::placeholders::_2)

class LMASettings
{
public:
    int max_iter;
    double lambda;
    std::function<double (VecX x, VecX beta)> function;
    std::function<VecX (VecX x, VecX beta)> jacobian;
    int nb_inputs;
    int nb_params;

    MatX x;
    VecX y;
    VecX beta;

    LMASettings(void);
};

class LMAOpt
{
public:
    bool configured;

    int max_iter;
    double lambda;
    std::function<double (VecX x, VecX beta)> function;
    std::function<VecX (VecX x, VecX beta)> jacobian;
    int nb_inputs;
    int nb_params;

    MatX x;
    VecX y;
    VecX beta;

    VecX y_est;
    VecX diff;

    MatX J;
    MatX H;

    double error;

    LMAOpt(void);
    int configure(LMASettings settings);
    int evalFunction(VecX beta, double &error);
    int calcGradients(VecX beta);
    int iterate(void);
    int optimize(void);
};

}  // end of slam namespace
#endif
