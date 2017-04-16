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

    AttitudeController(void);
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

    PositionController(void);
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

    QuadrotorModel(void);
    QuadrotorModel(const VecX &pose);
    int update(const VecX &motor_inputs, double dt);
    Vec4 attitudeControllerControl(double dt);
    Vec4 positionControllerControl(double dt);
    void setAttitude(double roll, double pitch, double yaw, double z);
    void setPosition(double x, double y, double z);
    void setVelocity(double vx, double vy, double vz);
    VecX getPose(void);
    VecX getVelocity(void);
    void printState(void);
};

}  // end of wave namespace
#endif
