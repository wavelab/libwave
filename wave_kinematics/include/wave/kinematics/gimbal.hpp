#ifndef __WAVE_KINEMATICS_GIMBAL_HPP__
#define __WAVE_KINEMATICS_GIMBAL_HPP__

#include <float.h>
#include <iostream>
#include <iomanip>

#include "wave/utils/utils.hpp"
#include "wave/kinematics/data.hpp"
#include "wave/kinematics/control.hpp"


namespace wave {
namespace gimbal {

class AttitudeController {
public:
  control::PID roll_controller;
  control::PID pitch_controller;
  double dt;
  Vec3 outputs;

  AttitudeController(void);
  VecX calculate(Vec3 setpoints, Vec3 actual, double dt);
};

class GimbalModel {
public:
  VecX states;

  double Ix;
  double Iy;
  Pose camera_offset;

  Vec3 joint_setpoints;
  AttitudeController joint_controller;

  Quaternion frame_orientation;
  Quaternion joint_orientation;
  Vec3 target_attitude_if;

  GimbalModel(void);
  GimbalModel(VecX pose);
  int update(Vec3 motor_inputs, double dt);
  Vec3 attitudeControllerControl(double dt);
  void setFrameOrientation(Quaternion frame_if);
  void setAttitude(Vec3 euler_if);
  Vec3 getTargetInBF(Vec3 target_cf);
  Vec3 getTargetInBPF(Vec3 target_cf,
                      Quaternion body_if,
                      Quaternion joint_bf);
  void trackTarget(Vec3 target_cf);
  Vec4 getState(void);
  void printState(void);
};

}  // end of gimbal namespace
}  // end of wave namespace
#endif
