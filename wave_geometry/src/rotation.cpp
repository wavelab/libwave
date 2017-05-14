#include "wave/utils/math.hpp"

#include "wave/geometry/exception_helpers.hpp"
#include "wave/geometry/rotation.hpp"

namespace wave {

Rotation::Rotation() {}

Rotation::Rotation(const Vec3 &input_vector) {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(input_vector);

    this->setFromEulerXYZ(input_vector);
}

Rotation::Rotation(const double angle_magnitude, const Vec3 &rotation_axis) {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(rotation_axis);
    checkScalarFinite(angle_magnitude);

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
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(input_vector);

    return this->rotation_object.rotate(input_vector);
}

Vec3 Rotation::inverseRotate(const Vec3 &input_vector) const {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(input_vector);

    return this->rotation_object.inverseRotate(input_vector);
}

Vec3 Rotation::rotateAndJacobian(const Vec3 &input_vector,
                                 Mat3 &Jpoint,
                                 Mat3 &Jparam) const {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(input_vector);

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

    // Check if the input is finite, throw error otherwise
    checkArrayFinite(input_vector);

    Mat3 rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(input_vector[2], Vec3::UnitZ()) *
                      Eigen::AngleAxisd(input_vector[1], Vec3::UnitY()) *
                      Eigen::AngleAxisd(input_vector[0], Vec3::UnitX());


    this->rotation_object.setMatrix(rotation_matrix);

    return *this;
}

Rotation &Rotation::setFromAngleAxis(const double angle_magnitude,
                                     const Vec3 &rotation_axis) {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(rotation_axis);
    checkScalarFinite(angle_magnitude);

    // Check if the input rotation axis is normalized,
    // throw error otherwise.

    checkVectorNormalized(rotation_axis);

    kindr::AngleAxisD angle_axis(angle_magnitude, rotation_axis);
    this->rotation_object = angle_axis;
    return *this;
}

Rotation &Rotation::setFromExpMap(const Vec3 &se3_vector) {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(se3_vector);

    this->rotation_object = this->rotation_object.exponentialMap(se3_vector);
    return *this;
}

Rotation &Rotation::setFromMatrix(const Mat3 &input_matrix) {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(input_matrix);

    // Check if the input is a valid rotation matrix,
    // throw error otherwise.

    checkValidRotation(input_matrix);

    this->rotation_object.setMatrix(input_matrix);
    return *this;
}

Vec3 Rotation::logMap() const {
    return this->rotation_object.logarithmicMap();
}

Rotation &Rotation::manifoldPlus(const Vec3 &omega) {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(omega);

    this->rotation_object = this->rotation_object.boxPlus(omega);
    return *this;
}

bool Rotation::isNear(const Rotation &R,
                      const double comparison_threshold = 1e-6) const {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(R.toRotationMatrix());
    checkScalarFinite(comparison_threshold);

    return this->rotation_object.isNear(R.rotation_object,
                                        comparison_threshold);
}

Mat3 Rotation::toRotationMatrix() const {
    return this->rotation_object.matrix();
}

Rotation Rotation::operator*(const Rotation &R) const {
    // Check if the input is finite, throw error otherwise
    checkArrayFinite(R.toRotationMatrix());

    Rotation composed;
    composed.rotation_object = this->rotation_object * R.rotation_object;
    return composed;
}

std::ostream &operator<<(std::ostream &stream, const Rotation &R) {
    stream << R.rotation_object;
    return stream;
}

bool Rotation::isValidRotationMatrix(const Mat3 &R) {
    // For a rotation to be valid, R*inv(R)==I,
    // and det(R) = 1;

    Mat3 eye3;
    eye3.setIdentity();
    Mat3 ortho_matrix = R * R.inverse();
    if (ortho_matrix.isApprox(eye3) && !fltcmp(R.determinant(), 1))
        return true;
    else
        return false;
}

}  // end of wave namespace
