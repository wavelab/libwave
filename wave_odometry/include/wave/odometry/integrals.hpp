#ifndef WAVE_INTEGRALS_HPP
#define WAVE_INTEGRALS_HPP

#include <Eigen/Core>
#include "wave/utils/utils.hpp"

namespace wave {

/**
 * This class implements the integrand for the covariance of constant acceleration priors
 */
class ConstantAccelerationCovariance {
 private:
    /// Power spectral density matrix on acceleration noise
    const Mat6 Qc;
    const double tmax;
    /// Rotational velocity
    const Vec6 w;

    /**
     * This is to calculate what I've been calling gamma in my head.
     * It's similar to the Rodrigez formula, and pops out of the NL-SDE that
     * models constant acceleration.
     * @param deltaT required
     * @return
     */
    void evaluateGamma(const double &deltaT, Mat6 &gamma);

 public:
    ConstantAccelerationCovariance(Mat6 QC, double Tmax, Vec6 omega) : Qc(QC), tmax(Tmax), w(omega) {}

    /**
     * Numerically integrate using trapezoidal rule to get Qtotal
     * @param Qtotal Covariance
     * @param slices How many discrete chunks to use when integrating
     */
    void calculateCovariance(Mat6 &Qtotal, const int& slices = 50);
};
}

#endif  // WAVE_INTEGRALS_HPP
