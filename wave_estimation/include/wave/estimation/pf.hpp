#ifndef __wave_ESTIMATION_PF_HPP__
#define __wave_ESTIMATION_PF_HPP__

#include <iostream>
#include <functional>

#include "wave/utils/utils.hpp"


namespace wave {

class ParticleFilter
{
public:
    bool initialized;
    VecX mu;

    int M;

    // VecX mu_p;
    // MatX S_p;

    ParticleFilter(void);
    int init(int M, VecX mu);
    int estimate(
        std::vector<VecX> X_p,
        std::vector<VecX> hX_p,
        VecX y
    );
};

} // end of wave namespace
#endif
