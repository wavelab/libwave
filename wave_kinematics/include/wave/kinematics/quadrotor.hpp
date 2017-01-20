#ifndef __WAVE_KINEMATICS_QUADROTOR_HPP__
#define __WAVE_KINEMATICS_QUADROTOR_HPP__

#include <float.h>
#include <iostream>

#include "wave/utils/utils.hpp"
#include "wave/kinematics/control.hpp"

namespace wave {
namespace quadrotor {

class AttitudeController {
public:
  double dt;
  Vec4 outputs;

  control::PID roll_controller;
  control::PID pitch_controller;
  control::PID yaw_controller;

  AttitudeController(void);
  Vec4 calculate(Vec4 setpoints, Vec4 actual, double dt);
  Vec4 calculate(Vec4 psetpoints, Vec4 vsetpoints, Vec4 actual, double dt);
};

class PositionController {
public:
  double dt;
  Vec4 outputs;

  control::PID x_controller;
  control::PID y_controller;
  control::PID z_controller;

  PositionController(void);
  Vec4 calculate(Vec3 setpoints, Vec4 actual, double yaw, double dt);
};

class VelocityController {
public:
  control::PID vx_controller;
  control::PID vy_controller;
  control::PID vz_controller;

  VelocityController(void);
  VecX calculate(Vec3 setpoints, Vec3 actual, double yaw, double dt);
};

class QuadrotorModel {
public:
  VecX states;
  double Ix;
  double Iy;
  double Iz;

  double kr;
  double kt;

  double l;
  double d;

  double m;
  double g;

  Vec4 attitude_setpoints;
  VecX position_setpoints;
  VecX velocity_setpoints;

  AttitudeController attitude_controller;
  PositionController position_controller;
  VelocityController velocity_controller;

  QuadrotorModel(void);
  QuadrotorModel(VecX pose);
  int update(VecX motor_inputs, double dt);
  VecX attitudeControllerControl(double dt);
  VecX positionControllerControl(double dt);
  VecX velocityControllerControl(double dt);
  // VecX trackerControllerControl(double dt);
  void setAttitude(double roll, double pitch, double yaw, double z);
  void setPosition(double x, double y, double z);
  void setVelocity(double vx, double vy, double vz);
  VecX getPose(void);
  VecX getVelocity(void);
  void printState(void);
};

}  // end of quadrotor namespace
}  // end of wave namespace
#endif
