#ifndef WAVE_KINEMATICS_TWOWHEEL_HPP
#define WAVE_KINEMATICS_TWOWHEEL_HPP

#include <float.h>
#include <iostream>

#include "wave/utils/utils.hpp"

namespace wave {

class TwoWheelRobot2DModel {
 public:
    Vec3 pose;

    TwoWheelRobot2DModel() : pose{0.0, 0.0, 0.0} {}
    TwoWheelRobot2DModel(Vec3 pose) : pose{pose} {}
    Vec3 update(Vec2 inputs, float dt);
};

}  // end of wave namespace
#endif
