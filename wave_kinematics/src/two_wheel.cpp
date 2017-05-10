#include "wave/kinematics/two_wheel.hpp"

namespace wave {

Vec3 TwoWheelRobot2DModel::update(Vec2 inputs, float dt) {
    Vec3 g;

    // clang-format off
		g << this->pose(0) + inputs(0) * cos(this->pose(2)) * dt,
				this->pose(1) + inputs(0) * sin(this->pose(2)) * dt,
				this->pose(2) + inputs(1) * dt;
    // clang-format on

    return g;
}

}  // end of wave namespace
