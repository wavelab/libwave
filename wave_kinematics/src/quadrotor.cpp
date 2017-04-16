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

Vec4 AttitudeController::update(const Vec4 &setpoints,
                                const Vec4 &actual,
                                double dt) {
    // check rate
    this->dt += dt;
    if (this->dt < 0.001) {
        return this->outputs;
    }

    // calculate yaw error
    double actual_yaw = rad2deg(actual(2));
    double setpoint_yaw = rad2deg(setpoints(2));
    double error_yaw = setpoint_yaw - actual_yaw;
    if (error_yaw > 180.0) {
        error_yaw -= 360.0;
    } else if (error_yaw < -180.0) {
        error_yaw += 360.0;
    }
    error_yaw = deg2rad(error_yaw);

    // roll pitch yaw
    double r = this->roll_controller.update(setpoints(0), actual(0), this->dt);
    double p = this->pitch_controller.update(setpoints(1), actual(1), this->dt);
    double y = this->yaw_controller.update(error_yaw, 0.0, this->dt);

    // thrust
    double max_thrust = 5.0;
    double t = max_thrust * setpoints(3);   // convert relative to true thrust
    t = (t > max_thrust) ? max_thrust : t;  // limit thrust
    t = (t < 0) ? 0.0 : t;                  // limit thrust

    // map roll, pitch, yaw and thrust to motor outputs
    Vec4 outputs;
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

Vec4 AttitudeController::update(const Vec4 &psetpoints,
                                const Vec4 &vsetpoints,
                                const Vec4 &actual,
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

Vec4 PositionController::update(const Vec3 &setpoints,
                                const Vec4 &actual,
                                double yaw,
                                double dt) {
    // check rate
    this->dt += dt;
    if (this->dt < 0.01) {
        return this->outputs;
    }

    // calculate RPY errors relative to quadrotor by incorporating yaw
    Vec3 errors;
    errors(0) = setpoints(0) - actual(0);
    errors(1) = setpoints(1) - actual(1);
    errors(2) = setpoints(2) - actual(2);

    Vec3 euler;
    Mat3 R;
    euler << 0.0, 0.0, actual(3);
    euler2rot(euler, 123, R);
    errors = R * errors;

    // roll, pitch, yaw and thrust
    double r = -this->y_controller.update(errors(1), 0.0, dt);
    double p = this->x_controller.update(errors(0), 0.0, dt);
    double y = yaw;
    double t = 0.5 + this->z_controller.update(errors(2), 0.0, dt);
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
    this->attitude << 0.0, 0.0, 0.0;
    this->angular_velocity << 0.0, 0.0, 0.0;
    this->position << 0.0, 0.0, 0.0;
    this->linear_velocity << 0.0, 0.0, 0.0;

    this->Ix = 0.0963;  // inertial x
    this->Iy = 0.0963;  // inertial y
    this->Iz = 0.1927;  // inertial z

    this->kr = 0.1;  // rotation drag constant
    this->kt = 0.2;  // translation drag constant

    this->l = 0.9;  // arm length
    this->d = 1.0;  // drag

    this->m = 1.0;   // mass of quad
    this->g = 10.0;  // gravitational constant

    this->attitude_setpoints << 0.0, 0.0, 0.0, 0.5;
    this->position_setpoints << 0.0, 0.0, 0.0;

    this->attitude_controller = AttitudeController();
    this->position_controller = PositionController();
}

QuadrotorModel::QuadrotorModel(const VecX &pose) {
    this->attitude << pose(3), pose(4), pose(5);
    this->angular_velocity << 0.0, 0.0, 0.0;
    this->position << pose(0), pose(1), pose(2);
    this->linear_velocity << 0.0, 0.0, 0.0;

    this->Ix = 0.0963;  // inertial x
    this->Iy = 0.0963;  // inertial y
    this->Iz = 0.1927;  // inertial z

    this->kr = 0.1;  // rotation drag constant
    this->kt = 0.2;  // translation drag constant;

    this->l = 0.9;  // arm length
    this->d = 1.0;  // drag

    this->m = 1.0;   // mass of quad
    this->g = 10.0;  // gravitational constant

    this->attitude_setpoints << 0.0, 0.0, 0.0, 0.5;
    this->position_setpoints << pose(0), pose(1), pose(2);

    this->attitude_controller = AttitudeController();
    this->position_controller = PositionController();
}

int QuadrotorModel::update(const VecX &motor_inputs, double dt) {
    double ph = this->attitude(0);
    double th = this->attitude(1);
    double ps = this->attitude(2);

    double p = this->angular_velocity(0);
    double q = this->angular_velocity(1);
    double r = this->angular_velocity(2);

    double x = this->position(0);
    double y = this->position(1);
    double z = this->position(2);

    double vx = this->linear_velocity(0);
    double vy = this->linear_velocity(1);
    double vz = this->linear_velocity(2);

    double Ix = this->Ix;
    double Iy = this->Iy;
    double Iz = this->Iz;

    double kr = this->kr;
    double kt = this->kt;

    double m = this->m;
    double g = this->g;

    // convert motor inputs to angular p, q, r and total thrust
    // clang-format off
    Mat4 A;
    A << 1.0, 1.0, 1.0, 1.0,
         0.0, -this->l, 0.0, this->l,
         -this->l, 0.0, this->l, 0.0,
         -this->d, this->d, -this->d, this->d;
    // clang-format on

    Vec4 tau = A * motor_inputs;
    double tauf = tau(0);
    double taup = tau(1);
    double tauq = tau(2);
    double taur = tau(3);

    // update
    // clang-format off
    this->attitude(0) = ph + (p + q * sin(ph) * tan(th) + r * cos(ph) * tan(th)) * dt;
    this->attitude(1) = th + (q * cos(ph) - r * sin(ph)) * dt;
    this->attitude(2) = ps + ((1 / cos(th)) * (q * sin(ph) + r * cos(ph))) * dt;
    this->angular_velocity(0) = p + (-((Iz - Iy) / Ix) * q * r - (kr * p / Ix) + (1 / Ix) * taup) * dt;
    this->angular_velocity(1) = q + (-((Ix - Iz) / Iy) * p * r - (kr * q / Iy) + (1 / Iy) * tauq) * dt;
    this->angular_velocity(2) = r + (-((Iy - Ix) / Iz) * p * q - (kr * r / Iz) + (1 / Iz) * taur) * dt;
    this->position(0) = x + vx * dt;
    this->position(1) = y + vy * dt;
    this->position(2) = z + vz * dt;
    this->linear_velocity(0) = vx + ((-kt * vx / m) + (1 / m) * (cos(ph) * sin(th) * cos(ps) + sin(ph) * sin(ps)) * tauf) * dt;
    this->linear_velocity(1) = vy + ((-kt * vy / m) + (1 / m) * (cos(ph) * sin(th) * sin(ps) - sin(ph) * cos(ps)) * tauf) * dt;
    this->linear_velocity(2) = vz + (-(kt * vz / m) + (1 / m) * (cos(ph) * cos(th)) * tauf - g) * dt;
    // clang-format on

    // constrain yaw to be [-180, 180]
    this->attitude(2) = rad2deg(this->attitude(2));
    this->attitude(2) = wrapTo180(this->attitude(2));
    this->attitude(2) = deg2rad(this->attitude(2));

    return 0;
}

Vec4 QuadrotorModel::attitudeControllerControl(double dt) {
    Vec4 actual_attitude;
    Vec4 motor_inputs;

    // attitude controller
    // clang-format off
    actual_attitude << this->attitude(0),  // roll
                       this->attitude(1),  // pitch
                       this->attitude(2),  // yaw
                       this->position(2);  // z
    motor_inputs = this->attitude_controller.update(this->attitude_setpoints,
                                                    actual_attitude,
                                                    dt);
    // clang-format on

    return motor_inputs;
}

Vec4 QuadrotorModel::positionControllerControl(double dt) {
    // position controller
    // clang-format off
    Vec4 actual_position;
    actual_position(0) = this->position(0);  // x
    actual_position(1) = this->position(1);  // y
    actual_position(2) = this->position(2);  // z
    actual_position(3) = this->attitude(2);  // yaw
    this->attitude_setpoints = this->position_controller.update(
        this->position_setpoints,
        actual_position,
        0.0,
        dt
    );

    // attitude controller
    Vec4 actual_attitude;
    Vec4 motor_inputs;
    actual_attitude(0) = this->attitude(0);  // roll
    actual_attitude(1) = this->attitude(1);  // pitch
    actual_attitude(2) = this->attitude(2);  // yaw
    actual_attitude(3) = this->position(2);  // z
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
    pose(0) = this->position(0);
    pose(1) = this->position(1);
    pose(2) = this->position(2);

    // phi, theta, psi
    pose(3) = this->attitude(0);
    pose(4) = this->attitude(1);
    pose(5) = this->attitude(2);

    return pose;
}

VecX QuadrotorModel::getVelocity(void) {
    VecX velocities(6);

    // vx, vy, vz
    velocities(0) = this->linear_velocity(0);
    velocities(1) = this->linear_velocity(1);
    velocities(2) = this->linear_velocity(2);

    // phi_dot, theta_dot, psi_dot
    velocities(3) = this->angular_velocity(0);
    velocities(4) = this->angular_velocity(1);
    velocities(5) = this->angular_velocity(2);

    return velocities;
}

void QuadrotorModel::printState(void) {
    printf("x: %f\t", this->position(0));
    printf("y: %f\t", this->position(1));
    printf("z: %f\t\t", this->position(2));

    printf("phi: %f\t", this->attitude(0));
    printf("theta: %f\t", this->attitude(1));
    printf("psi: %f\n", this->attitude(2));
}

}  // end of wave namespace
