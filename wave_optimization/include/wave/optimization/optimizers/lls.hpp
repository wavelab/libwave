#ifndef __wave_OPTIMIZATION_OPTIMIZERS_LLS_HPP__
#define __wave_OPTIMIZATION_OPTIMIZERS_LLS_HPP__

#include "wave/utils/utils.hpp"


namespace wave {

class LLSSolver
{
public:
    bool configured;

    LLSSolver(void);
    int configure(void);
    int solve(MatX A, MatX b, VecX &x);
};

}  // end of wave namespace
#endif
