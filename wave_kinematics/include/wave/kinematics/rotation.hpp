#ifndef __WAVE_KINEMATICS_ROTATION_HPP__
#define __WAVE_KINEMATICS_ROTATION_HPP__

#include <iostream>
#include <functional>

#include "wave/utils/utils.hpp"
#include <kindr/Core>

namespace wave {

class Rotation {
public:
  // Internal storage as a kindr Rotation object.
  kindr::RotationMatrixD rotation_object;
  // The comparision threshold used to test if rotation objects are
  // "close enough" to each other.  Used in unit testing and the "isNear"
  // function.
  static double comparision_threshold;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Rotation Object Initializers.
  Rotation();
  static bool fromEulerXYZ(const double rotation_x,
                           const double rotation_y,
                           const double rotation_z,
                           Rotation &R);
  static bool fromAngleAxis(const double angle_magnitude,
                            const Vec3 rotation_axis,
                            Rotation &R);
  static bool fromExpMap(const Vec3 se3_vector, Rotation &R);

  // Module functions.
  Vec3 logMap();
  void setToIdentity();
  Vec3 rotate(const Vec3 &input_vector);
  Vec3 rotateAndJacobian(const Vec3 &input_vector,
                         Mat3 &Jpoint,
                         Mat3 &Jparam);
  Vec3 inverseRotate(const Vec3 &input_vector);
  void compose(const Rotation &R);
  Mat3 invert();
  bool isNear(const Rotation &R);
  void manifoldPlus(const Vec3 omega);

  // Set and get methods.
  Mat3 getRotationMatrix();
};

}  // end of wave namespace
#endif