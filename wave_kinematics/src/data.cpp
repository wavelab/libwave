#include "wave/kinematics/data.hpp"

namespace wave {

Pose::Pose(void) {
  Quaternion q;

  this->q = q.setIdentity();
  this->position = Vec3::Zero(3, 1);
}

Pose::Pose(Quaternion q, Vec3 position) {
  this->q = q;
  this->position = position;
}

// clang-format off
Pose::Pose(double roll, double pitch, double yaw,
           double x, double y, double z) {
  Vec3 euler;
  euler << roll, pitch, yaw;
  euler2quat(euler, 321, this->q);
  this->position << x, y, z;
}
// clang-format on

Mat3 Pose::rotationMatrix(void) {
  return this->q.toRotationMatrix();
}

void Pose::printPosition(void) {
  printf("position: [");
  printf("%.2f, ", this->position(0));
  printf("%.2f, ", this->position(1));
  printf("%.2f", this->position(2));
  printf("]\n");
}

void Pose::printOrientation(void) {
  printf("quaternion: [");
  printf("%.2f, ", this->q.w());
  printf("%.2f, ", this->q.x());
  printf("%.2f,", this->q.y());
  printf("%.2f", this->q.z());
  printf("]\n");
}

void Pose::print(void) {
  this->printPosition();
  this->printOrientation();
}

}  // end of wave namespace
