#include "wave/utils/math.hpp"

#include "wave/geometry/exception_helpers.hpp"
#include "wave/geometry/rotation.hpp"

namespace wave {

Rotation::Rotation() {}

Rotation::Rotation(const Vec3 &input_vector) {
    // Check if the input is finite, throw error otherwise
    checkMatrixFinite(input_vector);

    this->setFromEulerXYZ(input_vector);
}

Rotation::Rotation(const double angle_magnitude, const Vec3 &rotation_axis) {
    // Check if the input is finite, throw error otherwise
    checkMatrixFinite(rotation_axis);
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
    checkMatrixFinite(input_vector);

    return this->rotation_object.rotate(input_vector);
}

Vec3 Rotation::inverseRotate(const Vec3 &input_vector) const {
    // Check if the input is finite, throw error otherwise
    checkMatrixFinite(input_vector);

    return this->rotation_object.inverseRotate(input_vector);
}

Vec3 Rotation::rotateAndJacobian(const Vec3 &input_vector,
                                 Mat3 &Jpoint,
                                 Mat3 &Jparam) const {
    // Check if the input is finite, throw error otherwise
    checkMatrixFinite(input_vector);

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
    checkMatrixFinite(input_vector);

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
    checkMatrixFinite(rotation_axis);
    checkScalarFinite(angle_magnitude);

    // Check if the input rotation axis is normalized,
    // throw error otherwise.

    checkVectorNormalized(rotation_axis);

    kindr::AngleAxisD angle_axis(angle_magnitude, rotation_axis);
    this->rotation_object = angle_axis;
    return *this;
}

Rotation &Rotation::setFromExpMap(const Vec3 &se3_vector) {
    checkMatrixFinite(se3_vector);

    this->rotation_object = this->rotation_object.exponentialMap(se3_vector);
    return *this;
}

Rotation &Rotation::setFromMatrix(const Mat3 &input_matrix) {
    checkMatrixFinite(input_matrix);
    checkValidRotation(input_matrix);

    this->rotation_object.setMatrix(input_matrix);
    return *this;
}

Vec3 Rotation::logMap() const {
    return this->rotation_object.logarithmicMap();
}

Mat3 Rotation::expMapAndJacobian(const Vec3& Vec, Mat3 &J_expmap) {
    Rotation rot;
    rot.setFromExpMap(Vec);

    J_expmap = kindr::getJacobianOfExponentialMap(Vec);
    return rot.toRotationMatrix();
}

Vec3 Rotation::logMapAndJacobian(const Rotation &R, Mat3 &J_logmap) {
    Vec3 log_params = R.logMap();
    // compute the derivative. First get the Jacobian of the expMap
    // for log(R).
    Mat3 J_exp = kindr::getJacobianOfExponentialMap(log_params);

    // Inverse of logMap is just inverse of J_exp.
    J_logmap = J_exp.inverse();

    return log_params;
}


Rotation &Rotation::manifoldPlus(const Vec3 &omega) {
    checkMatrixFinite(omega);

    this->rotation_object = this->rotation_object.boxPlus(omega);
    return *this;
}

Vec3 Rotation::manifoldMinus(const Rotation &R) const {
    checkMatrixFinite(R.toRotationMatrix());

    return this->rotation_object.boxMinus(R.rotation_object);
}

Vec3 Rotation::manifoldMinusAndJacobian(const Rotation &R_right,
                                        Mat3 &J_left,
                                        Mat3 &J_right) const {
    checkMatrixFinite(R_right.toRotationMatrix());
    // Compute manifoldMinus.
    Vec3 manifold_difference = this->manifoldMinus(R_right);

    // Compute the Jacobian using chain rule.  Recall the form
    // of manifoldMinus (R_left = "this")

    // R_left \boxminus R_right
    // = logm(R_left*inv(R_right))
    // J_left = J_logm * J_compose_R_left
    // J_right = J_logm * J_compose_inv_R_right * J_inv_R_right

    Mat3 J_inv_R_right;
    Rotation inv_R_left = R_right.inverseAndJacobian(J_inv_R_right);

    Mat3 J_compose_R_left, J_compose_inv_R_right;
    Rotation R_composed = this->composeAndJacobian(
      inv_R_left, J_compose_R_left, J_compose_inv_R_right);

    Mat3 J_logm;
    Rotation::logMapAndJacobian(R_composed, J_logm);

    J_left = J_logm * J_compose_R_left;
    J_right = J_logm * J_compose_inv_R_right * J_inv_R_right;

    return manifold_difference;
}

Rotation Rotation::composeAndJacobian(const Rotation &R_right,
                                      Mat3 &J_left,
                                      Mat3 &J_right) const {
    checkMatrixFinite(R_right.toRotationMatrix());

    // Compute the rotation composition.
    Rotation R_out;
    R_out = (*this) * R_right;

    // Jacobian wrt left rotation is identity.
    J_left = MatX::Identity(3, 3);

    // Jacobian wrt right rotation is the left rotation matrix.
    J_right = this->toRotationMatrix();

    return R_out;
}

bool Rotation::isNear(const Rotation &R,
                      const double comparison_threshold = 1e-6) const {
    checkMatrixFinite(R.toRotationMatrix());
    checkScalarFinite(comparison_threshold);

    return this->rotation_object.isNear(R.rotation_object,
                                        comparison_threshold);
}

Rotation Rotation::inverseAndJacobian(Mat3 &J_rotation) const {
    Rotation inverted;
    inverted.rotation_object = this->rotation_object.inverted();
    J_rotation = -1.0 * inverted.toRotationMatrix();
    return inverted;
}

Mat3 Rotation::toRotationMatrix() const {
    return this->rotation_object.matrix();
}

Rotation Rotation::operator*(const Rotation &R) const {
    checkMatrixFinite(R.toRotationMatrix());

    Rotation composed;
    composed.rotation_object = this->rotation_object * R.rotation_object;
    return composed;
}

Vec3 Rotation::operator-(const Rotation &R) const {
    checkMatrixFinite(R.toRotationMatrix());

    return this->manifoldMinus(R);
}

std::ostream &operator<<(std::ostream &stream, const Rotation &R) {
    stream << R.rotation_object;
    return stream;
}


}  // namespace wave
