#ifndef WAVE_KINEMATICS_GIMBAL_HPP
#define WAVE_KINEMATICS_GIMBAL_HPP

#include <float.h>
#include <iostream>
#include <iomanip>

#include "wave/utils/utils.hpp"
#include "wave/controls/pid.hpp"
#include "wave/kinematics/pose.hpp"

namespace wave {

/**
 * Attitude Controller for a 2-Axis gimbal, this controller implements 2 PID
 * controllers for both roll and pitch and has a fixed control rate of 100Hz.
 */
class AttitudeController {
 public:
    PID roll_controller;
    PID pitch_controller;
    double dt;
    Vec3 outputs;

    AttitudeController()
        : roll_controller{0.3, 0.0, 0.2},
          pitch_controller{0.3, 0.0, 0.2},
          dt{0.0},
          outputs{0.0, 0.0, 0.0} {}

    /**
     * Update gimbal attitude controller.
     *
     * Where `setpoints`, `actual` are vector of size 3 with roll, pitch and
     * yaw setpoints and actual attitudes in radians, with `dt` denoting the
     * update
     * time step. Returns motor inputs in roll, pitch, and yaw as a vector of
     * size 3 for a gimbal.
     */
    VecX update(Vec3 setpoints, Vec3 actual, double dt);
};

/**
 * 2-axis Gimbal model with PID attitude controllers
 *
 * This model has the following assumptions:
 *
 * - gimbal motion is linear
 * - no friction
 * - no gimbal lock protection
 * - running at a rate of 100hz
 * - donwload facing (origin attitude is downwards)
 */
class GimbalModel {
 public:
    Vec4 states;

    double Ix;
    double Iy;
    Pose camera_offset;

    Vec3 joint_setpoints;
    AttitudeController joint_controller;

    Quaternion frame_orientation;
    Quaternion joint_orientation;
    Vec3 target_attitude_if;

    GimbalModel()
        : states{0.0, 0.0, 0.0, 0.0},
          Ix{0.01},
          Iy{0.01},
          camera_offset{0.0, deg2rad(90.0), 0.0, 0.0, 0.0, 0.0},
          joint_setpoints{0.0, 0.0, 0.0},
          joint_controller{},
          frame_orientation{},
          joint_orientation{},
          target_attitude_if{0.0, 0.0, 0.0} {}

    GimbalModel(Vec4 pose)
        : states{pose},
          Ix{0.01},
          Iy{0.01},
          camera_offset{0.0, deg2rad(90.0), 0.0, 0.0, 0.0, 0.0},
          joint_setpoints{0.0, 0.0, 0.0},
          joint_controller{},
          frame_orientation{},
          joint_orientation{},
          target_attitude_if{0.0, 0.0, 0.0} {}

    /**
     * Update gimbal model
     */
    int update(Vec3 motor_inputs, double dt);

    /**
     * Update gimbal attitude controller
     */
    Vec3 attitudeControllerControl(double dt);

    /**
     * Set gimbal frame orientation
     */
    void setFrameOrientation(Quaternion frame_if);

    /**
     * Set gimbal joint attitude
     */
    void setAttitude(Vec3 euler_if);

    /**
     * Obtain gimbal target in body frame
     */
    Vec3 getTargetInBF(Vec3 target_cf);

    /**
     * Obtain gimbal target in body planar frame
     */
    Vec3 getTargetInBPF(Vec3 target_cf,
                        Quaternion body_if,
                        Quaternion joint_bf);

    /**
     * Track target
     */
    void trackTarget(Vec3 target_cf);

    /**
     * Get gimbal state
     */
    Vec4 getState();

    /**
     * Print gimbal state
     */
    void printState();
};

}  // end of wave namespace
#endif
