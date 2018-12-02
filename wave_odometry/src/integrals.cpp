#include "wave/odometry/integrals.hpp"
#include "wave/geometry_og/transformation.hpp"

namespace wave {

void ConstantAccelerationCovariance::evaluateGamma(const double &deltaT, Mat6 &gamma) {
    Vec6 scaled = deltaT * this->w;
    Mat6 scaled_adjoint;
    scaled_adjoint.setZero();
    scaled_adjoint.block<3, 3>(0, 0) = Transformation<>::skewSymmetric3(scaled.block<3, 1>(0, 0));
    scaled_adjoint.block<3, 3>(3, 3) = Transformation<>::skewSymmetric3(scaled.block<3, 1>(0, 0));
    scaled_adjoint.block<3, 3>(3, 0) = Transformation<>::skewSymmetric3(scaled.block<3, 1>(3, 0));

    double theta = scaled.block<3, 1>(0, 0).norm();
    if (theta < 1e-6) {
        gamma = 0.5 * Mat6::Identity() + 0.166666666666667 * scaled_adjoint +
                0.041666666666667 * scaled_adjoint * scaled_adjoint;
    } else {
        double A = (2*cos(theta) + theta*theta - 2)/(2*theta*theta*theta*theta);
        double B = (6*sin(theta) - 6*theta + theta*theta*theta) / (theta*theta*theta*theta*theta);

        gamma = 0.5 * Mat6::Identity() + 0.166666666666667 * scaled_adjoint +
                A * scaled_adjoint * scaled_adjoint + B * scaled_adjoint * scaled_adjoint * scaled_adjoint;
    }
}


void ConstantAccelerationCovariance::calculateCovariance(Mat6 &Qtotal, const int &slices) {
    Qtotal.setZero();
    double step_size = this->tmax / (double) slices;
    Mat6 gamma, C;
    double deltaT;
    for(int i = 0; i < slices; i++) {
        deltaT = this->tmax - ((double) i) * step_size;
        this->evaluateGamma(deltaT, gamma);
        C = deltaT * deltaT * gamma;
        if (i == 0 || i + 1 == slices) {
            Qtotal += 0.5 * C * this->Qc * C.transpose();
        } else {
            Qtotal += C * this->Qc * C.transpose();
        }
    }
    Qtotal = step_size * Qtotal;
}

}  // namespace wave