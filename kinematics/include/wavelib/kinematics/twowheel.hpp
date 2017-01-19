#ifndef __wavelib_KINEMATICS_TWOWHEEL_HPP__
#define __wavelib_KINEMATICS_TWOWHEEL_HPP__

#include "wavelib/utils/utils.hpp"


namespace wavelib {

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

} // end of wavelib namespace
#endif
