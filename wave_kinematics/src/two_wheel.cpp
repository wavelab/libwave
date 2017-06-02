#include "wave/kinematics/two_wheel.hpp"

namespace wave {

Vec3 TwoWheelRobot2DModel::update(const Vec2 &inputs, double dt) {
    this->pose(0) += inputs(0) * cos(this->pose(2)) * dt;
    this->pose(1) += inputs(0) * sin(this->pose(2)) * dt;
    this->pose(2) += inputs(1) * dt;

    return this->pose;
}

}  // namespace wave
