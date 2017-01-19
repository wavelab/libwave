#ifndef __wavelib_ESTIMATION_KF_HPP__
#define __wavelib_ESTIMATION_KF_HPP__

#include <iostream>
#include <functional>

#include "wavelib/utils/utils.hpp"


namespace wavelib {

class KalmanFilter
{
public:
    bool initialized;
    VecX mu;

    MatX B;
    MatX R;

    MatX C;
    MatX Q;

    MatX S;
    MatX I;
    MatX K;

    VecX mu_p;
    MatX S_p;

    KalmanFilter(void);
    int init(
        VecX mu,
        MatX R,
        MatX C,
        MatX Q
    );
    int estimate(MatX A, VecX y);
};

} // end of wavelib namespace
#endif
