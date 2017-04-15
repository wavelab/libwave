#include "wave/kinematics/rotation.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

// Default constructor.
Rotation::Rotation() {}

// Constructor taking Euler angles.
Rotation::Rotation(const Vec3 &input_vector) {
    this->setFromEulerXYZ(input_vector);
}

// Constructor taking in an angle magnitude and rotation axis.
Rotation::Rotation(const double angle_magnitude, const Vec3 &rotation_axis) {
    this->setFromAngleAxis(angle_magnitude, rotation_axis);
}

Rotation &Rotation::setIdentity() {
    this->rotation_object.setIdentity();
    return *this;
}

void Rotation::invert() {
    this->rotation_object.invert();
}

Vec3 Rotation::rotate(const Vec3 &input_vector) const {
    return this->rotation_object.rotate(input_vector);
}

Vec3 Rotation::inverseRotate(const Vec3 &input_vector) const {
    return this->rotation_object.inverseRotate(input_vector);
}

Vec3 Rotation::rotateAndJacobian(const Vec3 &input_vector,
                                 Mat3 &Jpoint,
                                 Mat3 &Jparam) const {
    // Calculate the Jacobian quantities.
    // Jacobian wrt the point is simply the rotation matrix.
    Jpoint = this->toRotationMatrix();

    // Jacobian wrt to the lie algebra parameters is the rotated vector
    // in skew symmetric form.
    Vec3 point_rotated = this->rotate(input_vector);
    Jparam = -1.0 * kindr::getSkewMatrixFromVector(point_rotated);

    return point_rotated;
}

Rotation &Rotation::setFromEulerXYZ(const Vec3 &input_vector) {
    // Rotate about X, then Y, then Z, by input_vector[0],input_vector[1],
    // input_vector[2], respectively.

    Mat3 rotation_matrix;
    rotation_matrix =
      Eigen::AngleAxisd(input_vector[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(input_vector[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(input_vector[0], Eigen::Vector3d::UnitX());


    this->rotation_object.setMatrix(rotation_matrix);

    return *this;
}

Rotation &Rotation::setFromAngleAxis(const double angle_magnitude,
                                     const Vec3 &rotation_axis) {
    kindr::AngleAxisD angle_axis(angle_magnitude, rotation_axis);
    this->rotation_object = angle_axis;
    return *this;
}

Rotation &Rotation::setFromExpMap(const Vec3 &se3_vector) {
    this->rotation_object = this->rotation_object.exponentialMap(se3_vector);
    return *this;
}

Rotation &Rotation::setFromMatrix(const Mat3 &input_matrix) {
    this->rotation_object.setMatrix(input_matrix);
    return *this;
}

Vec3 Rotation::logMap() const {
    return this->rotation_object.logarithmicMap();
}

Rotation &Rotation::manifoldPlus(const Vec3 &omega) {
    this->rotation_object = this->rotation_object.boxPlus(omega);
    return *this;
}

bool Rotation::isNear(const Rotation &R,
                      const double comparison_threshold) const {
    return this->rotation_object.isNear(R.rotation_object,
                                        comparison_threshold);
}

Mat3 Rotation::toRotationMatrix() const {
    return this->rotation_object.matrix();
}

Rotation Rotation::operator*(const Rotation &R) const {
    Rotation composed;
    composed.rotation_object = this->rotation_object * R.rotation_object;
    return composed;
}

std::ostream &operator<<(std::ostream &stream, const Rotation &R) {
    stream << R.rotation_object;
    return stream;
}

}  // end of wave namespace
