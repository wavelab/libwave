#ifndef __wavelib_ESTIMATION_PF_HPP__
#define __wavelib_ESTIMATION_PF_HPP__

#include <iostream>
#include <functional>

#include "wavelib/utils/utils.hpp"


namespace wavelib {

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

} // end of wavelib namespace
#endif
