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
    Vec4 update(Vec4 setpoints, Vec4 actual, double dt);
    Vec4 update(Vec4 psetpoints, Vec4 vsetpoints, Vec4 actual, double dt);
};

class PositionController {
 public:
    double dt;
    Vec4 outputs;

    PID x_controller;
    PID y_controller;
    PID z_controller;

    PositionController(void);
    Vec4 update(Vec3 setpoints, Vec4 actual, double yaw, double dt);
};

/*class VelocityController {
 public:
    PID vx_controller;
    PID vy_controller;
    PID vz_controller;

    VelocityController(void);
    VecX update(Vec3 setpoints, Vec3 actual, double yaw, double dt);
};
*/
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

    AttitudeController attitude_controller;
    PositionController position_controller;

    QuadrotorModel(void);
    QuadrotorModel(VecX pose);
    int update(VecX motor_inputs, double dt);
    VecX attitudeControllerControl(double dt);
    VecX positionControllerControl(double dt);
    VecX velocityControllerControl(double dt);
    void setAttitude(double roll, double pitch, double yaw, double z);
    void setPosition(double x, double y, double z);
    void setVelocity(double vx, double vy, double vz);
    VecX getPose(void);
    VecX getVelocity(void);
    void printState(void);
};

}  // end of wave namespace
#endif
