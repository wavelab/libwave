#include "slam/kinematics/twowheel.hpp"


namespace slam {

TwoWheelRobotModel::TwoWheelRobotModel(void)
{
    this->initialized = false;
}

VecX TwoWheelRobotModel::gFunc(VecX x, VecX u, float dt)
{
    VecX g;

    g << x(1) + u(1) * cos(x(3)) * dt,
         x(2) + u(1) * sin(x(3)) * dt,
         x(3) + u(2) * dt;

    return g;
}

MatX TwoWheelRobotModel::GFunc(VecX x, VecX u, float dt)
{
    MatX G;

    G << 1.0, 0.0, (-u(1) * sin(x(3)) * dt),
         0.0, 1.0, (u(1) * cos(x(3)) * dt),
         0.0, 0.0, 1.0;

    return G;
}

VecX TwoWheelRobotModel::hFunc(VecX x)
{
    VecX h;
    MatX H;

    H = Eigen::MatrixXf::Identity(3, 3);
    h = H * x;

    return h;
}

MatX TwoWheelRobotModel::HFunc(VecX y)
{
    MatX H;
    UNUSED(y);

    H = Eigen::MatrixXf::Identity(3, 3);

    return H;
}

} // end of slam namespace
