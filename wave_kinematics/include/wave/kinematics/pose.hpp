#ifndef WAVE_KINEMATICS_POSE_HPP
#define WAVE_KINEMATICS_POSE_HPP

#include <iostream>
#include <iomanip>

#include "wave/utils/utils.hpp"

namespace wave {

class Pose {
 public:
    Quaternion q;
    Vec3 position;

    Pose() : q{1.0, 0.0, 0.0, 0.0}, position{0.0, 0.0, 0.0} {}
    Pose(Quaternion q, Vec3 position) : q{q}, position{position} {}
    Pose(double roll, double pitch, double yaw, double x, double y, double z);

    Mat3 rotationMatrix();
    void printPosition();
    void printOrientation();
    void printQuaternion();
    void print();
};

}  // end of wave namespace
#endif
