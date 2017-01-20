#ifndef __WAVE_KINEMATICS_CONTROL_HPP__
#define __WAVE_KINEMATICS_CONTROL_HPP__

#include <float.h>
#include <math.h>

namespace wave {
namespace control {

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
  double calculate(double setpoint, double input, double dt);
};

}  // end of control namespace
}  // end of wave namespace
#endif
