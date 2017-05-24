#ifndef WAVE_KINEMATICS_ROTATION_HPP
#define WAVE_KINEMATICS_ROTATION_HPP

#include <functional>
#include <iostream>
#include <ostream>

#include <kindr/Core>

#include "wave/utils/utils.hpp"

namespace wave {

class Rotation {
 private:
    // Internal storage as a kindr Rotation object.
    kindr::RotationMatrixD rotation_object;

 public:
    // Default Constructor.
    Rotation();

    // Constructor taking euler angles.
    explicit Rotation(const Vec3 &);

    // Constructor taking in an angle magnitude and rotation axis.
    explicit Rotation(const double angle_magnitude, const Vec3 &rotation_axis);

    // Set the rotation from Euler angles and return a reference to it.
    Rotation &setFromEulerXYZ(const Vec3 &input_vector);

    // Set the rotation from angle-axis and return a reference to it.
    Rotation &setFromAngleAxis(const double angle_magnitude,
                               const Vec3 &rotation_axis);

    // Set the rotation from exp map and return a reference to it.
    Rotation &setFromExpMap(const Vec3 &se3_vector);

    // Set the rotation from a matrix and return a reference to it.
    Rotation &setFromMatrix(const Mat3 &input_matrix);

    // Set the rotation to identity.
    Rotation &setIdentity();

    // Module functions.

    // Returns the log map of R.
    Vec3 logMap() const;

    // Returns the log map of R and the associated Jacobian
    // of the mapping.
    static Vec3 logMapAndJacobian(const Rotation &R, Mat3 &J_logmap);

    // Performs the coordinate mapping operation.
    Vec3 rotate(const Vec3 &input_vector) const;

    // Performs the coordinate mapping operation, and the Jacobians
    // wrt the point and rotation.
    Vec3 rotateAndJacobian(const Vec3 &input_vector,
                           Mat3 &J_point,
                           Mat3 &J_rot) const;

    // Performs the inverse coordinate mapping.
    Vec3 inverseRotate(const Vec3 &input_vector) const;

    // Inverts "this".
    void invert();

    // Checks if R is sufficiently close to "this".
    bool isNear(const Rotation &R, double comparison_threshold) const;

    // Returns the boxplus of a rotation and a vector.
    Rotation &manifoldPlus(const Vec3 &omega);

    // Returns the boxminus of two rotations.
    Vec3 manifoldMinus(const Rotation &R) const;

    // Returns the boxminus of two rotations, and the Jacobians of boxminus
    // wrt the left and right rotations.
    Vec3 manifoldMinusAndJacobian(const Rotation &R,
                                  Mat3 &J_left,
                                  Mat3 &J_right) const;

    // Composes two rotations and computes the Jacobians wrt the left
    // and right rotations.
    // R_out = R_left*R_right.  Note that "this" corresponds to
    // R_left.
    Rotation composeAndJacobian(const Rotation &rotation_right,
                                Mat3 &J_left,
                                Mat3 &J_right) const;

    // Returns the inverse of "this" and the Jacobian of the inverse
    // mapping.
    Rotation inverseAndJacobian(Mat3 &J_rotation) const;

    Mat3 toRotationMatrix() const;

    // Operator overloads.

    // Implements rotation multiplication.
    Rotation operator*(const Rotation &R) const;

    // Implements manifoldMinus using - operator.
    Vec3 operator-(const Rotation &R) const;
    friend std::ostream &operator<<(std::ostream &stream, const Rotation &R);
};

// Other helper functions.
bool isValidRotationMatrix(const Mat3 &input_matrix);


}  // end of wave namespace
#endif
