#ifndef __SLAM_KINEMATICS_TWOWHEEL_HPP__
#define __SLAM_KINEMATICS_TWOWHEEL_HPP__

#include "slam/utils/utils.hpp"


namespace slam {

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

} // end of slam namespace
#endif
