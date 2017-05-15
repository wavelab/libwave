#ifndef WAVE_KINEMATICS_POSE_HPP
#define WAVE_KINEMATICS_POSE_HPP

#include <iostream>
#include <iomanip>

#include "wave/utils/utils.hpp"

namespace wave {

/**
 * A generic pose class that stores orientation and position information in a
 * quaternion and vector of size 3.
 */
class Pose {
 public:
    Quaternion q;
    Vec3 position;

    Pose() : q{1.0, 0.0, 0.0, 0.0}, position{0.0, 0.0, 0.0} {}
    Pose(Quaternion q, Vec3 position) : q{q}, position{position} {}
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

}  // end of wave namespace
#endif
