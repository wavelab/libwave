#include "wave/kinematics/twowheel.hpp"

namespace wave {

TwoWheelRobotModel::TwoWheelRobotModel(void) {
    this->initialized = false;
}

VecX TwoWheelRobotModel::gFunc(Vec3 x, Vec2 u, float dt) {
    Vec3 g;

    // clang-format off
  g << x(0) + u(0) * cos(x(2)) * dt,
       x(1) + u(0) * sin(x(2)) * dt,
       x(2) + u(1) * dt;
    // clang-format on

    return g;
}

MatX TwoWheelRobotModel::GFunc(Vec3 x, Vec2 u, float dt) {
    MatX G;

    // clang-format off
  G << 1.0, 0.0, (-u(0) * sin(x(2)) * dt),
       0.0, 1.0, (u(0) * cos(x(2)) * dt),
       0.0, 0.0, 1.0;
    // clang-format on

    return G;
}

VecX TwoWheelRobotModel::hFunc(VecX x) {
    VecX h;
    MatX H;

    H = MatX::Identity(3, 3);
    h = H * x;

    return h;
}

MatX TwoWheelRobotModel::HFunc(VecX y) {
    MatX H;
    // UNUSED(y);

    H = MatX::Identity(3, 3);

    return H;
}

}  // end of wave namespace
