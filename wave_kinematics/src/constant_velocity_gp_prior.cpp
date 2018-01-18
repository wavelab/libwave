#include "wave/kinematics/constant_velocity_gp_prior.hpp"

namespace wave_kinematics {

void ConstantVelocityPrior::calculateTransitionMatrix(const double &t1, const double &t2, Mat12 &transition) {
    transition.block<6,6>(6,6).setIdentity();
    transition.block<6,6>(6,0).setZero();

    transition.block<6,6>(0,0) = wave::Transformation::expMapAdjoint((t2-t1)*this->omega, 1e-2);
    transition.block<6,6>(0,6) = (t2 - t1) * wave::Transformation::SE3LeftJacobian((t2-t1)*this->omega, 1e-2);
}

void ConstantVelocityPrior::calculateLinCovariance(Mat12 &covariance, const double &t1, const double &t2) {
    double dT = t2 - t1;
    auto skew = wave::Transformation::skewSymmetric6(this->omega);
    covariance.block<6,6>(0,0) = 0.3333333333333 * dT * dT * dT * this->Qc +
                                 0.125 * dT * dT * dT * dT * (skew * this->Qc + this->Qc * skew.transpose()) +
                                 0.05 * dT * dT * dT * dT * dT * skew * this->Qc * skew.transpose();
    covariance.block<6,6>(0,6) = 0.5 * dT * dT * Qc + 0.166666666666666 * dT * dT * dT * skew * this->Qc;
    covariance.block<6,6>(6,0) = covariance.block<6,6>(0,6).transpose();
    covariance.block<6,6>(6,6) = dT * this->Qc;
}

void ConstantVelocityPrior::calculateStuff(Mat12 &hat, Mat12 &candle) {
    Mat12 trans_tau_k, trans_kp1_tau, trans_kp1_k;
    this->calculateTransitionMatrix(this->tk, this->tau, this->omega, trans_tau_k);
    this->calculateTransitionMatrix(this->tau, this->tkp1, this->omega, trans_kp1_tau);
    this->calculateTransitionMatrix(this->tk, this->tkp1, this->omega, trans_kp1_k);

    Mat12 Q, Qtau;
    this->calculateLinCovariance(Q, this->tk, this->tkp1);
    this->calculateLinCovariance(Qtau, this->tk, this->tau);

    hat = trans_tau_k - Qtau * trans_kp1_tau.transpose() * Q.inverse() * trans_kp1_k;
    candle = Qtau * trans_kp1_tau.transpose() * Q.inverse();
}

}
