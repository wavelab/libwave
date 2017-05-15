#include "wave/kinematics/gimbal.hpp"

namespace wave {

// Attitude Controller
Vec2 Gimbal2AxisController::update(Vec2 setpoints, Vec2 actual, double dt) {
    Vec2 outputs;

    // check rate
    // controller is running at 100Hz
    this->dt += dt;
    if (this->dt < 0.001) {
        // not time to update controller yet
        // returning last controller outputs
        return this->outputs;
    }

    // roll pitch yaw
    outputs(0) = this->roll_controller.update(setpoints(0), actual(0), dt);
    outputs(1) = this->pitch_controller.update(setpoints(1), actual(1), dt);

    // keep track of outputs
    this->outputs = outputs;
    this->dt = 0.0;  // reset controller timer

    return outputs;
}

// GIMBAL MODEL
void Gimbal2AxisModel::update(Vec2 motor_inputs, double dt) {
    // setup
    float ph = this->states(0);
    float ph_vel = this->states(1);
    float th = this->states(2);
    float th_vel = this->states(3);

    // roll
    this->states(0) = ph + ph_vel * dt;
    this->states(1) = ph_vel + (motor_inputs(0) / this->Ix) * dt;

    // pitch
    this->states(2) = th + th_vel * dt;
    this->states(3) = th_vel + (motor_inputs(1) / this->Ix) * dt;

    // update joint orientation
    Vec3 euler, target;
    euler << this->states(0), this->states(2), 0.0;
    euler2quat(euler, 321, this->joint_orientation);

    // track target attitude
    quat2euler(this->frame_orientation, 321, euler);
    this->joint_setpoints(0) = target_attitude_if(0) - euler(0);
    this->joint_setpoints(1) = target_attitude_if(1) - euler(1);
}

Vec2 Gimbal2AxisModel::attitudeControllerControl(double dt) {
    Vec2 actual_attitude;
    Vec2 motor_inputs;

    // attitude controller
    actual_attitude << this->states(0), this->states(2);
    motor_inputs =
      this->joint_controller.update(this->joint_setpoints, actual_attitude, dt);

    return motor_inputs;
}

void Gimbal2AxisModel::setFrameOrientation(Quaternion frame_if) {
    Vec3 euler;

    // filter out yaw - we do not need it
    quat2euler(frame_if, 321, euler);
    euler(2) = 0.0;

    // set gimbal frame orientation
    euler2quat(euler, 321, this->frame_orientation);
}

void Gimbal2AxisModel::setAttitude(Vec2 euler_if) {
    this->target_attitude_if(0) = euler_if(0);
    this->target_attitude_if(1) = euler_if(1);
}

Vec3 Gimbal2AxisModel::getTargetInBF(Vec3 target_cf) {
    Vec3 target_nwu;
    Mat3 R;
    Vec3 t;

    // transform camera frame to NWU frame
    // camera frame:  (z - forward, x - right, y - down)
    // NWU frame:  (x - forward, y - left, z - up)
    target_nwu(0) = target_cf(2);
    target_nwu(1) = -target_cf(0);
    target_nwu(2) = -target_cf(1);

    // camera mount offset
    R = this->camera_offset.rotationMatrix();
    t = this->camera_offset.position;

    // transform target from camera frame to body frame
    return (R * target_nwu + t);
}

Vec3 Gimbal2AxisModel::getTargetInBPF(Vec3 target_cf,
                                      Quaternion body_if,
                                      Quaternion joint_bf) {
    Vec3 p, target_bpf;
    Mat3 R_body, R_joint;

    // body is assumed to be NWU frame
    R_body = body_if.toRotationMatrix();

    // joint is assumed to be NWU frame
    R_joint = joint_bf.toRotationMatrix();

    // transform target in camera frame to body frame
    p = this->getTargetInBF(target_cf);

    // transform target in camera frame to body planar frame
    target_bpf = R_body * R_joint * p;

    return target_bpf;
}

void Gimbal2AxisModel::trackTarget(Vec3 target_cf) {
    double dist;
    Vec3 target;

    // obtain target in body planar frame
    target = this->getTargetInBPF(
      target_cf, this->frame_orientation, this->joint_orientation);

    // update gimbal setpoints
    dist = target.norm();
    this->target_attitude_if(0) = asin(target(1) / dist);
    this->target_attitude_if(1) = -asin(target(0) / dist);
}

Vec4 Gimbal2AxisModel::getState(void) {
    Vec4 pose;

    pose(0) = this->states(0);
    pose(1) = this->states(1);
    pose(2) = this->states(2);
    pose(3) = this->states(3);

    return pose;
}

void Gimbal2AxisModel::printState(void) {
    std::cout << "roll: ";
    std::cout << std::setprecision(2) << this->states(0) << "\t";

    std::cout << "pitch: ";
    std::cout << std::setprecision(2) << this->states(2) << "\t";

    std::cout << "roll vel: ";
    std::cout << std::setprecision(2) << this->states(1) << "\t";

    std::cout << "pitch vel: ";
    std::cout << std::setprecision(2) << this->states(3) << std::endl;
}

}  // end of wave namespace
