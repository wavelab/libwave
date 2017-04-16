#ifndef WAVE_QUADROTOR_QUADROTOR_HPP
#define WAVE_QUADROTOR_QUADROTOR_HPP

#include <float.h>
#include <iostream>

#include "wave/utils/utils.hpp"
#include "wave/controls/pid.hpp"

namespace wave {

class AttitudeController {
 public:
    double dt;
    Vec4 outputs;

    PID roll_controller;
    PID pitch_controller;
    PID yaw_controller;

    AttitudeController()
        : dt(0.0),
          outputs(),
          roll_controller(200.0, 0.5, 10.0),
          pitch_controller(200.0, 0.5, 10.0),
          yaw_controller(200.0, 0.5, 10.0) {}

    Vec4 update(const Vec4 &setpoints, const Vec4 &actual, double dt);
    Vec4 update(const Vec4 &psetpoints,
                const Vec4 &vsetpoints,
                const Vec4 &actual,
                double dt);
};

class PositionController {
 public:
    double dt;
    Vec4 outputs;

    PID x_controller;
    PID y_controller;
    PID z_controller;

    PositionController()
        : dt(0.0),
          outputs(),
          x_controller(0.5, 0.0, 0.035),
          y_controller(0.5, 0.0, 0.035),
          z_controller(0.5, 0.0, 0.018) {}

    Vec4 update(const Vec3 &setpoints,
                const Vec4 &actual,
                double yaw,
                double dt);
};

class QuadrotorModel {
 public:
    Vec3 attitude;
    Vec3 angular_velocity;
    Vec3 position;
    Vec3 linear_velocity;

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
    Vec3 position_setpoints;

    AttitudeController attitude_controller;
    PositionController position_controller;

    QuadrotorModel()
        : attitude(0, 0, 0),
          angular_velocity(0, 0, 0),
          position(0, 0, 0),
          linear_velocity(0, 0, 0),
          Ix(0.0963),  // inertial x
          Iy(0.0963),  // inertial y
          Iz(0.1927),  // inertial z
          kr(0.1),     // rotation drag constant
          kt(0.2),     // translation drag constant
          l(0.9),      // arm length
          d(1.0),      // drag
          m(1.0),      // mass of quad
          g(10.0),     // gravitational constant
          attitude_setpoints(0, 0, 0, 0),
          position_setpoints(0, 0, 0),
          attitude_controller(),
          position_controller() {}

    QuadrotorModel(const VecX &pose)
        : attitude(pose(3), pose(4), pose(5)),
          angular_velocity(0, 0, 0),
          position(pose(0), pose(1), pose(2)),
          linear_velocity(0, 0, 0),
          Ix(0.0963),  // inertial x
          Iy(0.0963),  // inertial y
          Iz(0.1927),  // inertial z
          kr(0.1),     // rotation drag constant
          kt(0.2),     // translation drag constant
          l(0.9),      // arm length
          d(1.0),      // drag
          m(1.0),      // mass of quad
          g(10.0),     // gravitational constant
          attitude_setpoints(0, 0, 0, 0),
          position_setpoints(0, 0, 0),
          attitude_controller(),
          position_controller() {}

    int update(const VecX &motor_inputs, double dt);
    Vec4 attitudeControllerControl(double dt);
    Vec4 positionControllerControl(double dt);
    void setAttitude(double roll, double pitch, double yaw, double z);
    void setPosition(double x, double y, double z);
    void setVelocity(double vx, double vy, double vz);
    VecX getPose();
    VecX getVelocity();
    void printState();
};

}  // end of wave namespace
#endif
