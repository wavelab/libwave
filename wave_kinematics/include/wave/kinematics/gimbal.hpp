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
class Gimbal2AxisController {
 public:
    PID roll_controller;
    PID pitch_controller;
    double dt;
    Vec2 outputs;

    Gimbal2AxisController()
        : roll_controller{0.3, 0.0, 0.2},
          pitch_controller{0.3, 0.0, 0.2},
          dt{0.0},
          outputs{0.0, 0.0} {}

    /**
     * Update gimbal attitude controller.
     *
     * Where `setpoints`, `actual` are vector of size 3 with roll, pitch and
     * yaw setpoints and actual attitudes in radians, with `dt` denoting the
     * update
     * time step. Returns motor inputs in roll, pitch, and yaw as a vector of
     * size 3 for a gimbal.
     */
    Vec2 update(Vec2 setpoints, Vec2 actual, double dt);
};

/**
 * 2-axis Gimbal model with PID attitude controllers
 *
 * This model has the following assumptions:
 *
 * - gimbal motion is linear
 * - gimbal is downward facing (origin is downwards)
 * - no friction
 * - no gimbal lock protection
 * - running at a rate of 100hz
 */
class Gimbal2AxisModel {
 public:
    Vec4 states;

    double Ix;
    double Iy;
    Pose camera_offset;

    Vec2 joint_setpoints;
    Gimbal2AxisController joint_controller;

    Quaternion frame_orientation;
    Quaternion joint_orientation;
    Vec2 target_attitude_if;

    Gimbal2AxisModel()
        : states{0.0, 0.0, 0.0, 0.0},
          Ix{0.01},
          Iy{0.01},
          camera_offset{0.0, deg2rad(90.0), 0.0, 0.0, 0.0, 0.0},
          joint_setpoints{0.0, 0.0},
          joint_controller{},
          frame_orientation{},
          joint_orientation{},
          target_attitude_if{0.0, 0.0} {}

    Gimbal2AxisModel(Vec4 pose)
        : states{pose},
          Ix{0.01},
          Iy{0.01},
          camera_offset{0.0, deg2rad(90.0), 0.0, 0.0, 0.0, 0.0},
          joint_setpoints{0.0, 0.0},
          joint_controller{},
          frame_orientation{},
          joint_orientation{},
          target_attitude_if{0.0, 0.0} {}

    /**
     * Update gimbal model
     *
     * Args:
     * - `motor_inputs`: a vector of roll pitch and yaw in radians
     * - `dt`: simulation time step or time difference when the model was last
     *         updated
     */
    void update(Vec2 motor_inputs, double dt);

    /**
     * Update gimbal attitude controller
     *
     * Args:
     * - `dt` is time step or time difference when the model was last updated
     */
    Vec2 attitudeControllerControl(double dt);

    /**
     * Set gimbal frame orientation
     *
     * Args:
     * - `frame_if`: frame orientation in inertial frame (NWU coordinate system)
     */
    void setFrameOrientation(Quaternion frame_if);

    /**
     * Set gimbal joint attitude
     *
     * Args:
     * - `euler_if`: gimbal attitude in inertial frame (NWU coordinate system)
     */
    void setAttitude(Vec2 euler_if);

    /**
     * Obtain gimbal target in body frame
     *
     * Args:
     * - `target_cf`: target in camera frame (EDN coordinate system)
     */
    Vec3 getTargetInBF(Vec3 target_cf);

    /**
     * Obtain gimbal target in body planar frame
     *
     * Args:
     * - `target_cf`: target in camera frame (EDN coordinate system)
     * - `body_if`: gimbal body in inertial frame (NWU coordinate system)
     * - `joint_bf`: gimbal body in body frame (NWU coordinate system)
     */
    Vec3 getTargetInBPF(Vec3 target_cf,
                        Quaternion body_if,
                        Quaternion joint_bf);

    /**
     * Track target
     *
     * Args:
     * - `target_cf`: target in camera frame (EDN coordinate system)
     */
    void trackTarget(Vec3 target_cf);

    /**
     * Get gimbal state
     *
     * The state is:
     *
     * - roll
     * - roll velocity
     * - pitch
     * - pitch velocity
     *
     * as a vector of size 4.
     */
    Vec4 getState();

    /**
     * Print gimbal state
     */
    void printState();
};

}  // end of wave namespace
#endif
