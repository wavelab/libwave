/** @file
 * @ingroup kinematics
 */

#ifndef WAVE_KINEMATICS_TWOWHEEL_HPP
#define WAVE_KINEMATICS_TWOWHEEL_HPP

#include <float.h>
#include <iostream>

#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup kinematics
 *  @{ */

/** Generic two wheel robot motion model */
class TwoWheelRobot2DModel {
 public:
    Vec3 pose;

    TwoWheelRobot2DModel() : pose{0.0, 0.0, 0.0} {}
    TwoWheelRobot2DModel(Vec3 pose) : pose{pose} {}

    /** Update two wheel model
     * @param inputs Model input vector where first input is wheel velocity in
     * $ms^-1$ and the second input is steering angular velocity $rads^-1$
     * @param dt Update time step in seconds
     */
    Vec3 update(Vec2 inputs, float dt);
};

/** @} end of group */
}  // end of wave namespace
#endif
