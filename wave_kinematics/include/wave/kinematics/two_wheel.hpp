/** @file
 * @ingroup kinematics
 */

#ifndef WAVE_KINEMATICS_TWOWHEEL_HPP
#define WAVE_KINEMATICS_TWOWHEEL_HPP

#include <iostream>

#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup kinematics
 *  @{ */

/** Generic two wheel robot motion model */
class TwoWheelRobot2DModel {
 public:
    Vec3 pose;  /// Pose of the two wheel robot position in x, y (meters) and
                /// heading theta (radians)

    TwoWheelRobot2DModel() : pose{0.0, 0.0, 0.0} {}
    TwoWheelRobot2DModel(const Vec3 &pose) : pose{pose} {}

    /** Update two wheel model
     *
     * @param inputs Model input vector where first input is wheel velocity in
     * $ms^-1$ and the second input is steering angular velocity $rads^-1$
     *
     * @param dt Update time step in seconds
     *
     * @returns Updated pose of two wheel robot
     */
    Vec3 update(const Vec2 &inputs, double dt);
};

/** @} end of group */
}  // end of wave namespace
#endif
