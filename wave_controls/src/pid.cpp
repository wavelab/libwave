#include "wave/controls/pid.hpp"

namespace wave {

PID::PID(void) {
    this->error_prev = 0.0;
    this->error_sum = 0.0;

    this->error_p = 0.0;
    this->error_i = 0.0;
    this->error_d = 0.0;

    this->k_p = 0.0;
    this->k_i = 0.0;
    this->k_d = 0.0;
}

PID::PID(double k_p, double k_i, double k_d) {
    this->error_prev = 0.0;
    this->error_sum = 0.0;

    this->error_p = 0.0;
    this->error_i = 0.0;
    this->error_d = 0.0;

    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
}

double PID::update(double setpoint, double actual, double dt) {
    double error;
    double output;

    // calculate errors
    error = setpoint - actual;
    this->error_sum += error * dt;

    // calculate output
    this->error_p = this->k_p * error;
    this->error_i = this->k_i * this->error_sum;
    this->error_d = this->k_d * (error - this->error_prev) / dt;
    output = this->error_p + this->error_i + this->error_d;

    // update error
    this->error_prev = error;

    return output;
}

}  // end of wave namespace
