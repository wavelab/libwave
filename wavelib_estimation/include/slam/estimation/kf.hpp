#ifndef __SLAM_ESTIMATION_KF_HPP__
#define __SLAM_ESTIMATION_KF_HPP__

#include <iostream>
#include <functional>

#include "slam/utils/utils.hpp"


namespace slam {

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

} // end of slam namespace
#endif
