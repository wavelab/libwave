#ifndef __wave_KINEMATICS_TWOWHEEL_HPP__
#define __wave_KINEMATICS_TWOWHEEL_HPP__

#include "wave/utils/utils.hpp"


namespace wave {

class TwoWheelRobotModel
{
public:
    bool initialized;

    TwoWheelRobotModel(void);
    VecX gFunc(VecX x, VecX u, float dt);
    MatX GFunc(VecX x, VecX u, float dt);
    VecX hFunc(VecX x);
    MatX HFunc(VecX y);
};

} // end of wave namespace
#endif
