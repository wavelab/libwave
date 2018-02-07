#include "wave/kinematics/constant_velocity_gp_prior.hpp"

namespace wave_kinematics {

ConstantVelocityPrior::ConstantVelocityPrior(const double &tk, const double &tkp1, const double *tau,
                                             const wave::Mat6 &Qc, const wave::Mat6 &inv_Qc) :
                                            tk(tk), tkp1(tkp1), dT(tkp1 - tk), tau(tau), Qc(Qc), inv_Qc(inv_Qc) {}

void ConstantVelocityPrior::calculateTransitionMatrix(Mat12 &transition, const double &t1, const double &t2) {
    transition.block<6,6>(6,6).setIdentity();
    transition.block<6,6>(6,0).setZero();

    transition.block<6,6>(0,0).setIdentity();
    transition.block<6,6>(0,6) = (t2 - t1) * Eigen::Matrix<double, 6, 6>::Identity();
}

void ConstantVelocityPrior::calculateTransitionMatrix() {
    this->calculateTransitionMatrix(this->t_mat, this->tk, this->tkp1);
}

void ConstantVelocityPrior::calculateLinInvCovariance(Mat12 &covariance, const double &t1, const double &t2) {
    double dT = t2 - t1;
    dT = 1.0 / dT;
    double dT2 = dT * dT;
    covariance.block<6,6>(6,6) = 4.0 * dT * this->inv_Qc;

    covariance.block<6,6>(0,6) = -6.0 * dT2 * this->inv_Qc;
    covariance.block<6,6>(6,0) = covariance.block<6,6>(0,6);

    covariance.block<6,6>(0,0) = 12.0 * dT2 * dT * this->inv_Qc;
}

void ConstantVelocityPrior::calculateLinCovariance(Mat12 &covariance, const double &t1, const double &t2) {
    double dT = t2 - t1;
    double dT2 = dT * dT;

    covariance.block<6,6>(6,6) = dT * this->Qc;

    covariance.block<6,6>(0,6) = 0.5 * dT2 * this->Qc;
    covariance.block<6,6>(6,0) = covariance.block<6,6>(0,6);

    covariance.block<6,6>(0,0) = 0.333333333333333333 * dT2 * dT * this->Qc;
}

void ConstantVelocityPrior::calculateLinInvCovariance() {
    this->calculateLinInvCovariance(this->inv_covar, this->tk, this->tkp1);
}

void ConstantVelocityPrior::calculateCandle(Mat12 &candle) {
    double t1 = *(this->tau) - this->tk;
    double t2 = this->tkp1 - *(this->tau);
    double invt = 1.0 / (this->dT);

    candle.block<6,6>(0,0).noalias() = (t1*t1*(4*t1 - 3*this->dT + 6*t2)) * invt * invt * invt * Mat6::Identity();
    candle.block<6,6>(6,0).noalias() = (6*t1*(t1 - this->dT + 2*t2))* invt * invt * invt * Mat6::Identity();
    candle.block<6,6>(0,6).noalias() = -(t1*t1*(2*t1 - 2*this->dT + 3*t2)) * invt * invt * Mat6::Identity();
    candle.block<6,6>(6,6).noalias() = -(t1*(3*t1 - 4*this->dT + 6*t2))* invt * invt * Mat6::Identity();
}

void ConstantVelocityPrior::calculateHat(Mat12 &hat) {
    double t1 = *(this->tau) - this->tk;
    double t2 = this->tkp1 - *(this->tau);
    double invt = 1.0 / (this->dT);

    hat.block<6,6>(0,0).noalias() = (1 - ((t1*t1*(4*t1 - 3*this->dT + 6*t2)) * invt * invt * invt)) * Mat6::Identity();
    hat.block<6,6>(6,0).noalias() = -(6*t1*(t1 - this->dT + 2*t2))* invt * invt * invt * Mat6::Identity();
    hat.block<6,6>(0,6).noalias() = (t1*(this->dT * this->dT + this->dT*t1 - 2*t1*t1 - 3*t2*t1)) * invt * invt * Mat6::Identity();
    hat.block<6,6>(6,6).noalias() = (this->dT * this->dT + 2*this->dT*t1 - 3*t1*t1*t1 - 6*t2*t1)* invt * invt * Mat6::Identity();
}

void ConstantVelocityPrior::calculateStuff(Mat12 &hat, Mat12 &candle) {
    this->calculateCandle(candle);

    hat.block<6,6>(0,0).noalias() = Mat6::Identity() - candle.block<6,6>(0,0);
    hat.block<6,6>(6,0).noalias() = - candle.block<6,6>(6,0);
    hat.block<6,6>(0,6).noalias() = (*(this->tau) - this->tk) * Mat6::Identity() - this->dT * candle.block<6,6>(0,0) - candle.block<6,6>(0,6);
    hat.block<6,6>(6,6).noalias() = Mat6::Identity() - this->dT * candle.block<6,6>(6,0) - candle.block<6,6>(6,6);
}

}
