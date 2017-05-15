#include "wave/kinematics/pose.hpp"

namespace wave {

Pose::Pose(
  double roll, double pitch, double yaw, double x, double y, double z) {
    Vec3 euler;
    euler << roll, pitch, yaw;
    euler2quat(euler, 321, this->q);
    this->position << x, y, z;
}

Mat3 Pose::rotationMatrix() {
    return this->q.toRotationMatrix();
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
    std::cout << std::setprecision(2) << this->q.w();
    std::cout << std::setprecision(2) << this->q.x();
    std::cout << std::setprecision(2) << this->q.y();
    std::cout << std::setprecision(2) << this->q.z();
    std::cout << "]" << std::endl;
}

void Pose::print() {
    this->printPosition();
    this->printOrientation();
}

}  // end of wave namespace
