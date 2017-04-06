#include "wave/kinematics/quadrotor.hpp"

namespace wave {

// ATTITUDE CONTROLLER
AttitudeController::AttitudeController(void) {
    this->dt = 0.0;
    this->outputs << 0.0, 0.0, 0.0, 0.0;

    this->roll_controller = PID(200.0, 0.5, 10.0);
    this->pitch_controller = PID(200.0, 0.5, 10.0);
    this->yaw_controller = PID(200.0, 0.5, 10.0);
}

Vec4 AttitudeController::update(Vec4 setpoints, Vec4 actual, double dt) {
    double r, p, y, t;
    Vec4 outputs;
    double actual_yaw, setpoint_yaw, error_yaw;
    double max_thrust;

    // check rate
    this->dt += dt;
    if (this->dt < 0.001) {
        return this->outputs;
    }

    // calculate yaw error
    actual_yaw = rad2deg(actual(2));
    setpoint_yaw = rad2deg(setpoints(2));
    error_yaw = setpoint_yaw - actual_yaw;
    if (error_yaw > 180.0) {
        error_yaw -= 360.0;
    } else if (error_yaw < -180.0) {
        error_yaw += 360.0;
    }
    error_yaw = deg2rad(error_yaw);

    // roll pitch yaw
    r = this->roll_controller.update(setpoints(0), actual(0), this->dt);
    p = this->pitch_controller.update(setpoints(1), actual(1), this->dt);
    y = this->yaw_controller.update(error_yaw, 0.0, this->dt);

    // thrust
    max_thrust = 5.0;
    t = max_thrust * setpoints(3);  // convert relative thrust to true thrust
    t = (t > max_thrust) ? max_thrust : t;  // limit thrust
    t = (t < 0) ? 0.0 : t;                  // limit thrust

    // map roll, pitch, yaw and thrust to motor outputs
    outputs(0) = -p - y + t;
    outputs(1) = -r + y + t;
    outputs(2) = p - y + t;
    outputs(3) = r + y + t;

    // limit outputs
    for (int i = 0; i < 4; i++) {
        if (outputs(i) > max_thrust) {
            outputs(i) = max_thrust;
        } else if (outputs(i) < 0.0) {
            outputs(i) = 0.0;
        }
    }

    // keep track of outputs
    this->outputs = outputs;
    this->dt = 0.0;

    return outputs;
}

Vec4 AttitudeController::update(Vec4 psetpoints,
                                   Vec4 vsetpoints,
                                   Vec4 actual,
                                   double dt) {
    Vec4 setpoints;
    setpoints = psetpoints + vsetpoints;
    return this->update(setpoints, actual, dt);
}


// POSITION CONTROLLER
PositionController::PositionController(void) {
    this->dt = 0.0;
    this->outputs << 0.0, 0.0, 0.0, 0.0;

    this->x_controller = PID(0.5, 0.0, 0.035);
    this->y_controller = PID(0.5, 0.0, 0.035);
    this->z_controller = PID(0.3, 0.0, 0.018);
}

Vec4 PositionController::update(Vec3 setpoints,
                                   Vec4 actual,
                                   double yaw,
                                   double dt) {
    double r, p, y, t;
    Vec3 errors, euler;
    Vec4 outputs;
    Mat3 R;

    // check rate
    this->dt += dt;
    if (this->dt < 0.01) {
        return this->outputs;
    }

    // calculate RPY errors relative to quadrotor by incorporating yaw
    errors(0) = setpoints(0) - actual(0);
    errors(1) = setpoints(1) - actual(1);
    errors(2) = setpoints(2) - actual(2);
    euler << 0.0, 0.0, actual(3);
    euler2rot(euler, 123, R);
    errors = R * errors;

    // roll, pitch, yaw and thrust
    r = -this->y_controller.update(errors(1), 0.0, dt);
    p = this->x_controller.update(errors(0), 0.0, dt);
    y = yaw;
    t = 0.5 + this->z_controller.update(errors(2), 0.0, dt);
    outputs << r, p, y, t;

    // limit roll, pitch
    for (int i = 0; i < 2; i++) {
        if (outputs(i) > deg2rad(30.0)) {
            outputs(i) = deg2rad(30.0);
        } else if (outputs(i) < deg2rad(-30.0)) {
            outputs(i) = deg2rad(-30.0);
        }
    }

    // limit yaw
    while (outputs(2) > deg2rad(360.0)) {
        outputs(2) -= deg2rad(360.0);
    }
    while (outputs(2) < deg2rad(0.0)) {
        outputs(2) += deg2rad(360.0);
    }

    // limit thrust
    if (outputs(3) > 1.0) {
        outputs(3) = 1.0;
    } else if (outputs(3) < 0.0) {
        outputs(3) = 0.0;
    }

    // yaw first if threshold reached
    if (fabs(yaw - actual(3)) > deg2rad(2)) {
        outputs(0) = 0.0;
        outputs(1) = 0.0;
    }

    // keep track of outputs
    this->outputs = outputs;
    this->dt = 0.0;

    return outputs;
}


// QUADROTOR MODEL
QuadrotorModel::QuadrotorModel(void) {
    this->states = VecX(12);
    this->states(0) = 0.0;   // roll
    this->states(1) = 0.0;   // pitch
    this->states(2) = 0.0;   // yaw
    this->states(3) = 0.0;   // angular velocity x
    this->states(4) = 0.0;   // angular velocity y
    this->states(5) = 0.0;   // angular velocity z
    this->states(6) = 0.0;   // x
    this->states(7) = 0.0;   // y
    this->states(8) = 0.0;   // z
    this->states(9) = 0.0;   // linear velocity x
    this->states(10) = 0.0;  // linear velocity y
    this->states(11) = 0.0;  // linear velocity z

    this->Ix = 0.0963;  // inertial x
    this->Iy = 0.0963;  // inertial y
    this->Iz = 0.1927;  // inertial z

    this->kr = 0.1;  // rotation drag constant
    this->kt = 0.2;  // translation drag constant

    this->l = 0.9;  // arm length
    this->d = 1.0;  // drag

    this->m = 1.0;   // mass of quad
    this->g = 10.0;  // gravitational constant

    this->attitude_setpoints = Vec4();
    this->position_setpoints = VecX(3);

    this->attitude_controller = AttitudeController();
    this->position_controller = PositionController();
}

QuadrotorModel::QuadrotorModel(VecX pose) {
    this->states = VecX(12);
    this->states(0) = pose(3);  // roll
    this->states(1) = pose(4);  // pitch
    this->states(2) = pose(5);  // yaw
    this->states(3) = 0.0;      // angular velocity x
    this->states(4) = 0.0;      // angular velocity y
    this->states(5) = 0.0;      // angular velocity z
    this->states(6) = pose(0);  // x
    this->states(7) = pose(1);  // y
    this->states(8) = pose(2);  // z
    this->states(9) = 0.0;      // linear velocity x
    this->states(10) = 0.0;     // linear velocity y
    this->states(11) = 0.0;     // linear velocity z

    this->Ix = 0.0963;  // inertial x
    this->Iy = 0.0963;  // inertial y
    this->Iz = 0.1927;  // inertial z

    this->kr = 0.1;  // rotation drag constant
    this->kt = 0.2;  // translation drag constant;

    this->l = 0.9;  // arm length
    this->d = 1.0;  // drag

    this->m = 1.0;   // mass of quad
    this->g = 10.0;  // gravitational constant

    this->attitude_setpoints = Vec4();
    this->attitude_setpoints << 0.0, 0.0, 0.0, 0.5;

    this->position_setpoints = VecX(3);
    this->position_setpoints << pose(0), pose(1), pose(2);

    this->attitude_controller = AttitudeController();
    this->position_controller = PositionController();
}

int QuadrotorModel::update(VecX motor_inputs, double dt) {
    double ph, th, ps;
    double p, q, r;
    double x, y, z;
    double vx, vy, vz;

    double Ix, Iy, Iz;
    double kr, kt;
    double tauf, taup, tauq, taur;
    double m, g;

    Mat4 A;
    Vec4 tau;

    // setup
    ph = this->states(0);
    th = this->states(1);
    ps = this->states(2);

    p = this->states(3);
    q = this->states(4);
    r = this->states(5);

    x = this->states(6);
    y = this->states(7);
    z = this->states(8);

    vx = this->states(9);
    vy = this->states(10);
    vz = this->states(11);

    Ix = this->Ix;
    Iy = this->Iy;
    Iz = this->Iz;

    kr = this->kr;
    kt = this->kt;

    m = this->m;
    g = this->g;

    // convert motor inputs to angular p, q, r and total thrust
    // clang-format off
  A << 1.0, 1.0, 1.0, 1.0,
       0.0, -this->l, 0.0, this->l,
       -this->l, 0.0, this->l, 0.0,
       -this->d, this->d, -this->d, this->d;
    // clang-format on

    tau = A * motor_inputs;
    tauf = tau(0);
    taup = tau(1);
    tauq = tau(2);
    taur = tau(3);

    // update
    // clang-format off
  this->states(0) = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
  this->states(1) = th + (q * cos(ph) - r * sin(ph)) * dt;
  this->states(2) = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;
  this->states(3) = p + (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
  this->states(4) = q + (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
  this->states(5) = r + (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;
  this->states(6) = x + vx * dt;
  this->states(7) = y + vy * dt;
  this->states(8) = z + vz * dt;
  this->states(9) = vx + ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
  this->states(10) = vy + ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
  this->states(11) = vz + (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;
    // clang-format on

    // constrain yaw to be [-180, 180]
    this->states(2) = rad2deg(this->states(2));
    this->states(2) = wrapTo180(this->states(2));
    this->states(2) = deg2rad(this->states(2));

    return 0;
}

VecX QuadrotorModel::attitudeControllerControl(double dt) {
    Vec4 actual_attitude;
    Vec4 motor_inputs;

    // attitude controller
    // clang-format off
  actual_attitude << this->states(0),  // roll
                     this->states(1),  // pitch
                     this->states(2),  // yaw
                     this->states(8);  // z
  motor_inputs = this->attitude_controller.update(this->attitude_setpoints,
                                                     actual_attitude,
                                                     dt);
    // clang-format on

    return motor_inputs;
}

VecX QuadrotorModel::positionControllerControl(double dt) {
    Vec4 motor_inputs;
    Vec4 actual_position;
    Vec4 actual_attitude;

    // position controller
    // clang-format off
  actual_position(0) = this->states(6);  // x
  actual_position(1) = this->states(7);  // y
  actual_position(2) = this->states(8);  // z
  actual_position(3) = this->states(2);  // yaw
  this->attitude_setpoints = this->position_controller.update(
    this->position_setpoints,
    actual_position,
    0.0,
    dt
  );

  // attitude controller
  actual_attitude(0) = this->states(0);  // roll
  actual_attitude(1) = this->states(1);  // pitch
  actual_attitude(2) = this->states(2);  // yaw
  actual_attitude(3) = this->states(8);  // z
  motor_inputs = this->attitude_controller.update(
    this->attitude_setpoints,
    actual_attitude,
    dt
  );
    // clang-format on

    return motor_inputs;
}

void QuadrotorModel::setAttitude(double roll,
                                 double pitch,
                                 double yaw,
                                 double z) {
    this->attitude_setpoints(0) = roll;
    this->attitude_setpoints(1) = pitch;
    this->attitude_setpoints(2) = yaw;
    this->attitude_setpoints(3) = z;
}

void QuadrotorModel::setPosition(double x, double y, double z) {
    this->position_setpoints(0) = x;
    this->position_setpoints(1) = y;
    this->position_setpoints(2) = z;
}

VecX QuadrotorModel::getPose(void) {
    VecX pose(6);

    // x, y, z
    pose(0) = this->states(6);
    pose(1) = this->states(7);
    pose(2) = this->states(8);

    // phi, theta, psi
    pose(3) = this->states(0);
    pose(4) = this->states(1);
    pose(5) = this->states(2);

    return pose;
}

VecX QuadrotorModel::getVelocity(void) {
    VecX velocities(6);

    // vx, vy, vz
    velocities(0) = this->states(9);
    velocities(1) = this->states(10);
    velocities(2) = this->states(11);

    // phi_dot, theta_dot, psi_dot
    velocities(3) = this->states(3);
    velocities(4) = this->states(4);
    velocities(5) = this->states(5);

    return velocities;
}

void QuadrotorModel::printState(void) {
    float x, y, z;
    float phi, theta, psi;

    // phi, theta, psi
    phi = this->states(0);
    theta = this->states(1);
    psi = this->states(2);

    // x, y, z
    x = this->states(6);
    y = this->states(7);
    z = this->states(8);

    // print states
    printf("x: %f\t", x);
    printf("y: %f\t", y);
    printf("z: %f\t\t", z);

    printf("phi: %f\t", phi);
    printf("theta: %f\t", theta);
    printf("psi: %f\n", psi);
}

}  // end of wave namespace
