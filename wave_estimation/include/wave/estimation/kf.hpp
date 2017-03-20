#ifndef WAVE_ESTIMATION_KF_HPP
#define WAVE_ESTIMATION_KF_HPP

#include <iostream>
#include <functional>

#include "wave/utils/utils.hpp"


namespace wave {

class KalmanFilter {
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
    int init(VecX mu, MatX R, MatX C, MatX Q);
    int estimate(MatX A, VecX y);
};

}  // end of wave namespace
#endif
