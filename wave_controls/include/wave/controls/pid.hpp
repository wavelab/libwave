#ifndef WAVE_PID_HPP
#define WAVE_PID_HPP

namespace wave {

class PID {
 public:
    double error_prev;
    double error_sum;

    double error_p;
    double error_i;
    double error_d;

    double k_p;
    double k_i;
    double k_d;

    PID(void);
    PID(double k_p, double k_i, double k_d);
    double update(double setpoint, double actual, double dt);
};

}  // end of wave namespace
#endif
