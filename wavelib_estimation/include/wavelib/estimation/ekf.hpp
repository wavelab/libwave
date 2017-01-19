#ifndef __wavelib_ESTIMATION_EKF_HPP__
#define __wavelib_ESTIMATION_EKF_HPP__

#include <iostream>
#include <functional>

#include "wavelib/utils/utils.hpp"


namespace wavelib {

class ExtendedKalmanFilter
{
public:
    bool initialized;
    VecX mu;

    MatX R;
    MatX Q;

    MatX S;
    MatX I;
    MatX K;

    VecX mu_p;
    MatX S_p;

    ExtendedKalmanFilter(void);
    int init(VecX mu, MatX R, MatX Q);
    int predictionUpdate(VecX g, MatX G);
    int measurementUpdate(VecX h, MatX H, VecX y);
};

} // end of wavelib namespace
#endif
