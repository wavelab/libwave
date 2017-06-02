/** @file
 * @ingroup controls
 */

#ifndef WAVE_CONTROLS_PID_HPP
#define WAVE_CONTROLS_PID_HPP

namespace wave {
/** @addtogroup controls
 *  @{ */

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

    PID()
        : error_prev(0.0),
          error_sum(0.0),
          error_p(0.0),
          error_i(0.0),
          error_d(0.0),
          k_p(0.0),
          k_i(0.0),
          k_d(0.0) {}

    PID(double k_p, double k_i, double k_d)
        : error_prev(0.0),
          error_sum(0.0),
          error_p(0.0),
          error_i(0.0),
          error_d(0.0),
          k_p(k_p),
          k_i(k_i),
          k_d(k_d) {}

    double update(double setpoint, double actual, double dt);
};

/** @} group controls */
}  // namespace wave

#endif  // WAVE_CONTROLS_PID_HPP
