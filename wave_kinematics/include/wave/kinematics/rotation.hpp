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
    explicit Rotation(const double angle_magnitude, const Vec3 rotation_axis);

    // Set the rotation from Euler angles and return a reference to it.
    Rotation &setFromEulerXYZ(const Vec3 &input_vector);

    // Set the rotation from angle-axis and return a reference to it.
    Rotation &setFromAngleAxis(const double angle_magnitude,
                               const Vec3 rotation_axis);

    // Set the rotation from exp map and return a reference to it.
    Rotation &setFromExpMap(const Vec3 se3_vector);

    // Set the rotation from a matrix and return a reference to it.
    Rotation &setFromMatrix(const Mat3 input_matrix);

    // Set the rotation to identity.
    Rotation &setIdentity();

    // Module functions.
    Vec3 logMap() const;
    Vec3 rotate(const Vec3 &input_vector) const;
    Vec3 rotateAndJacobian(const Vec3 &input_vector,
                           Mat3 &Jpoint,
                           Mat3 &Jparam) const;
    Vec3 inverseRotate(const Vec3 &input_vector) const;
    void invert();
    bool isNear(const Rotation &R, double comparison_threshold) const;
    Rotation &manifoldPlus(const Vec3 &omega);

    Mat3 toRotationMatrix() const;

    // Operator overloads.
    Rotation operator*(const Rotation &R) const;
    friend std::ostream &operator<<(std::ostream &stream, const Rotation &R);
};


}  // end of wave namespace
#endif
