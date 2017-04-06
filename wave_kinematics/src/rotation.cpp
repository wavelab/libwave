#include "wave/kinematics/rotation.hpp"
#include "wave/utils/utils.hpp"

double wave::Rotation::comparision_threshold = 1e-8;

wave::Rotation::Rotation() {}

void wave::Rotation::setToIdentity() {
    this->rotation_object.setIdentity();
}

void wave::Rotation::compose(const Rotation &R) {
    this->rotation_object = this->rotation_object * R.rotation_object;
}

wave::Mat3 wave::Rotation::invert() {
    this->rotation_object.invert();
}

wave::Vec3 wave::Rotation::rotate(const Vec3 &input_vector) {
    return this->rotation_object.rotate(input_vector);
}

wave::Vec3 wave::Rotation::inverseRotate(const Vec3 &input_vector) {
    return this->rotation_object.inverseRotate(input_vector);
}

wave::Vec3 wave::Rotation::rotateAndJacobian(const Vec3 &input_vector,
                                             Mat3 &Jpoint,
                                             Mat3 &Jparam) {
    // Calculate the Jacobian quantities.
    // Jacobian wrt the point is simply the rotation matrix.
    Jpoint = this->rotation_object.matrix();

    // Jacobian wrt to the lie algebra parameters is the rotated vector
    // in skew symmetric form.
    Vec3 point_rotated = this->rotation_object.rotate(input_vector);
    Jparam = -1.0 * skew_symmetric_matrix(point_rotated);

    return point_rotated;
}

bool wave::Rotation::fromEulerXYZ(const double rotation_x,
                                  const double rotation_y,
                                  const double rotation_z,
                                  Rotation &R) {
    double sx = sin(rotation_x);
    double cx = cos(rotation_x);
    double sy = sin(rotation_y);
    double cy = cos(rotation_y);
    double sz = sin(rotation_z);
    double cz = cos(rotation_z);

    double r00 = cz * cy;
    double r01 = cz * sy * sx - sz * cx;
    double r02 = cz * sy * cx + sz * sx;
    double r10 = sz * cy;
    double r11 = sz * sy * sx + cz * cx;
    double r12 = sz * sy * cx - cz * sx;
    double r20 = -sy;
    double r21 = cy * sx;
    double r22 = cy * cx;


    R.rotation_object.setMatrix(r00, r01, r02, r10, r11, r12, r20, r21, r22);

    return true;
}

bool wave::Rotation::fromAngleAxis(const double angle_magnitude,
                                   const Vec3 rotation_axis,
                                   Rotation &R) {
    kindr::AngleAxisD angle_axis(angle_magnitude, rotation_axis);
    R.rotation_object = angle_axis;

    return true;
}

bool wave::Rotation::fromExpMap(const Vec3 se3_vector, Rotation &R) {
    R.rotation_object = R.rotation_object.exponentialMap(se3_vector);
    return true;
}

wave::Vec3 wave::Rotation::logMap() {
    return this->rotation_object.logarithmicMap();
}

void wave::Rotation::manifoldPlus(const wave::Vec3 omega) {
    this->rotation_object = this->rotation_object.boxPlus(omega);
}

bool wave::Rotation::isNear(const Rotation &R) {
    return this->rotation_object.isNear(R.rotation_object,
                                        this->comparision_threshold);
}

wave::Mat3 wave::Rotation::getRotationMatrix() {
    return this->rotation_object.matrix();
}