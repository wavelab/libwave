#include "wave/kinematics/pose.hpp"

namespace wave {

Pose::Pose(
  double roll, double pitch, double yaw, double x, double y, double z) {
    Vec3 euler;
    euler << roll, pitch, yaw;
    euler2quat(euler, 321, this->orientation);
    this->position << x, y, z;
}

Mat3 Pose::rotationMatrix() {
    return this->orientation.toRotationMatrix();
}

void Pose::printPosition() {
    std::cout << "position [";
    std::cout << std::setprecision(2) << this->position(0);
    std::cout << std::setprecision(2) << this->position(1);
    std::cout << std::setprecision(2) << this->position(2);
    std::cout << "]" << std::endl;
}

void Pose::printOrientation() {
    std::cout << "quaternion[";
    std::cout << std::setprecision(2) << this->orientation.w();
    std::cout << std::setprecision(2) << this->orientation.x();
    std::cout << std::setprecision(2) << this->orientation.y();
    std::cout << std::setprecision(2) << this->orientation.z();
    std::cout << "]" << std::endl;
}

void Pose::print() {
    this->printPosition();
    this->printOrientation();
}

}  // namespace wave
