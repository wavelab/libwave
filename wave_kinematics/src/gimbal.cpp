#include "wave/kinematics/gimbal.hpp"

namespace wave {

// Attitude Controller
AttitudeController::AttitudeController(void) {
    this->roll_controller = PID(0.3, 0.0, 0.2);
    this->pitch_controller = PID(0.3, 0.0, 0.2);
}

VecX AttitudeController::update(Vec3 setpoints, Vec3 actual, double dt) {
    Vec3 outputs;

    // check rate
    this->dt += dt;
    if (this->dt < 0.001) {  // controller is running at 100Hz
        return this->outputs;
    }

    // roll pitch yaw
    outputs(0) = this->roll_controller.update(setpoints(0), actual(0), dt);
    outputs(1) = this->pitch_controller.update(setpoints(1), actual(1), dt);
    outputs(2) = 0.0;

    // keep track of outputs
    this->outputs = outputs;
    this->dt = 0.0;  // reset controller timer

    return outputs;
}

// GIMBAL MODEL
GimbalModel::GimbalModel(void) {
    this->states = VecX(4);
    this->states(0) = 0.0;  // roll position
    this->states(1) = 0.0;  // roll velocity
    this->states(2) = 0.0;  // pitch position
    this->states(3) = 0.0;  // pitch velocity

    this->Ix = 0.01;
    this->Iy = 0.01;

    // downward facing camera (gimbal is NWU frame)
    // NWU frame: (x - forward, y - left, z - up)
    this->camera_offset = Pose(0.0, deg2rad(90.0), 0.0, 0.0, 0.0, 0.0);

    this->joint_setpoints = Vec3();
    this->joint_controller = AttitudeController();

    this->frame_orientation = Quaternion();
    this->joint_orientation = Quaternion();
    this->target_attitude_if << 0.0, 0.0, 0.0;
}

GimbalModel::GimbalModel(VecX pose) {
    this->states = VecX(4);
    this->states(0) = pose(0);  // roll position
    this->states(1) = pose(1);  // roll velocity
    this->states(2) = pose(3);  // pitch position
    this->states(3) = pose(4);  // pitch velocity

    this->Ix = 0.01;
    this->Iy = 0.01;

    // downward facing camera (gimbal is NWU frame)
    // NWU frame: (x - forward, y - left, z - up)
    this->camera_offset = Pose(0.0, deg2rad(90.0), 0.0, 0.0, 0.0, 0.0);

    this->joint_setpoints = Vec3();
    this->joint_controller = AttitudeController();

    this->frame_orientation = Quaternion();
    this->joint_orientation = Quaternion();
    this->target_attitude_if << 0.0, 0.0, 0.0;
}

int GimbalModel::update(Vec3 motor_inputs, double dt) {
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
    this->joint_setpoints(2) = 0.0;  // 0 because this is a 2-axis gimbal

    return 0;
}

Vec3 GimbalModel::attitudeControllerControl(double dt) {
    Vec3 actual_attitude;
    Vec3 motor_inputs;

    // attitude controller
    actual_attitude << this->states(0), this->states(2), 0.0;
    motor_inputs = this->joint_controller.update(
      this->joint_setpoints,
      actual_attitude,
      dt
    );

    return motor_inputs;
}

void GimbalModel::setFrameOrientation(Quaternion frame_if) {
    Vec3 euler;

    // filter out yaw - we do not need it
    quat2euler(frame_if, 321, euler);
    euler(2) = 0.0;

    // set gimbal frame orientation
    euler2quat(euler, 321, this->frame_orientation);
}

void GimbalModel::setAttitude(Vec3 euler_if) {
    this->target_attitude_if(0) = euler_if(0);
    this->target_attitude_if(1) = euler_if(1);
    this->target_attitude_if(2) = 0.0;
}

Vec3 GimbalModel::getTargetInBF(Vec3 target_cf) {
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

Vec3 GimbalModel::getTargetInBPF(Vec3 target_cf,
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

void GimbalModel::trackTarget(Vec3 target_cf) {
    double dist;
    Vec3 target;

    // obtain target in body planar frame
    target = this->getTargetInBPF(
      target_cf, this->frame_orientation, this->joint_orientation);

    // update gimbal setpoints
    dist = target.norm();
    this->target_attitude_if(0) = asin(target(1) / dist);
    this->target_attitude_if(1) = -asin(target(0) / dist);
    this->target_attitude_if(2) = 0.0;
}

Vec4 GimbalModel::getState(void) {
    Vec4 pose;

    pose(0) = this->states(0);
    pose(1) = this->states(1);
    pose(2) = this->states(2);
    pose(3) = this->states(3);

    return pose;
}

void GimbalModel::printState(void) {
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
