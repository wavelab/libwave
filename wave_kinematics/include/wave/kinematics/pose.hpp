/** @file
 * @ingroup kinematics
 */

#ifndef WAVE_KINEMATICS_POSE_HPP
#define WAVE_KINEMATICS_POSE_HPP

#include <iostream>
#include <iomanip>

#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup kinematics
 *  @{ */

/**
 * A generic pose class that stores orientation and position information in a
 * quaternion and vector of size 3.
 */
class Pose {
 public:
    Vec3 position;
    Quaternion orientation;

    Pose() : position{0.0, 0.0, 0.0}, orientation{1.0, 0.0, 0.0, 0.0} {}
    Pose(Vec3 position, Quaternion orientation)
        : position{position}, orientation{orientation} {}
    Pose(double roll, double pitch, double yaw, double x, double y, double z);

    /// Obtain orientation as a rotation matrix
    Mat3 rotationMatrix();

    /// Print position
    void printPosition();

    /// Print orientation
    void printOrientation();

    /// Print quaternion
    void printQuaternion();

    /// Print pose
    void print();
};

/** @} end of group */
}  // namespace wave

#endif  // WAVE_KINEMATICS_POSE_HPP
