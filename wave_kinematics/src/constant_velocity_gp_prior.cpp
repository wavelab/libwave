#include "wave/kinematics/constant_velocity_gp_prior.hpp"

namespace wave_kinematics {

ConstantVelocityPrior::ConstantVelocityPrior(const double &tk, const double &tkp1, const double *tau,
                                             const wave::Mat6 &Qc) :
                                            tk(tk), tkp1(tkp1), tau(tau), Qc(Qc) {
    this->inv_Qc = this->Qc.llt().solve(wave::Mat6::Identity());
}

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
    covariance.block<6,6>(6,6) = 4.0 * dT * this->inv_Qc;
    dT *= dT;
    covariance.block<6,6>(0,6) = -6.0 * dT * this->inv_Qc;
    covariance.block<6,6>(6,0) = covariance.block<6,6>(0,6);
    dT *= dT;
    covariance.block<6,6>(0,0) = 12.0 * dT * this->inv_Qc;
}

void ConstantVelocityPrior::calculateLinCovariance(Mat12 &covariance, const double &t1, const double &t2) {
    double dT = t2 - t1;

    covariance.block<6,6>(6,6) = dT * this->Qc;
    dT *= dT;
    covariance.block<6,6>(0,6) = 0.5 * dT * this->Qc;
    covariance.block<6,6>(6,0) = covariance.block<6,6>(0,6);
    dT *= dT;
    covariance.block<6,6>(0,0) = 0.333333333333333333 * dT * this->Qc;
}

void ConstantVelocityPrior::calculateLinInvCovariance() {
    this->calculateLinInvCovariance(this->inv_covar, this->tk, this->tkp1);
}

void ConstantVelocityPrior::calculateCandle(Mat12 &candle) {
    this->calculateLinInvCovariance();

    Mat12 covar, tran_kp1_tau;
    this->calculateTransitionMatrix(tran_kp1_tau, *(this->tau), this->tkp1);
    this->calculateLinCovariance(covar, this->tk, *(this->tau));
    candle = covar * tran_kp1_tau.transpose() * this->inv_covar;
}

void ConstantVelocityPrior::calculateStuff(Mat12 &hat, Mat12 &candle) {
    Mat12 trans_tau_k;
    this->calculateCandle(candle);
    this->calculateTransitionMatrix(trans_tau_k, this->tk, *(this->tau));
    this->calculateTransitionMatrix();

    hat = trans_tau_k - candle * this->t_mat;
}

}
