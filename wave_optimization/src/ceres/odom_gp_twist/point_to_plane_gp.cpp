#include "wave/optimization/ceres/odom_gp_twist/point_to_plane_gp.hpp"

namespace wave_optimization {

SE3PointToPlaneGP::SE3PointToPlaneGP(const double *const pA,
                                     const double *const pB,
                                     const double *const pC,
                                     SE3PointToPlaneGPObjects &objects,
                                     const wave::Mat3 &covZ,
                                     bool use_weighting)
    : ptA(pA), ptB(pB), ptC(pC), objects(objects) {
    this->objects.JP_T.setZero();
    this->objects.JP_T.block<3, 3>(0, 3).setIdentity();

    this->calculateJr_P(this->objects.Jr_P);

    if (use_weighting) {
        double covR = (objects.Jr_P * covZ * objects.Jr_P.transpose())(0);
        this->weight = std::sqrt(1.0 / covR);
    } else {
        this->weight = 1;
    }

    double dBA[3] = {ptB[0] - ptA[0], ptB[1] - ptA[1], ptB[2] - ptA[2]};
    double dBC[3] = {ptB[0] - ptC[0], ptB[1] - ptC[1], ptB[2] - ptC[2]};

    ceres::CrossProduct(dBA, dBC, this->cBA_BC);

    this->inv_den = 1.0 / ceres::sqrt(cBA_BC[0] * cBA_BC[0] + cBA_BC[1] * cBA_BC[1] + cBA_BC[2] * cBA_BC[2]);
}

bool SE3PointToPlaneGP::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const wave::Vec12> state_tk_map(parameters[0]);
    Eigen::Map<const wave::Vec12> state_tkp1_map(parameters[1]);

    this->objects.T_cur_twist =
            this->objects.hat * state_tk_map + this->objects.candle * state_tkp1_map;

    this->objects.T_current.setFromExpMap(this->objects.T_cur_twist);

    wave::Vec3 point = this->objects.T_current.transform(this->objects.T0_pt);

    // point is the transformed point.
    double d_B[3] = {point(0) - ptB[0], point(1) - ptB[1], point(2) - ptB[2]};

    double num = this->cBA_BC[0] * d_B[0] + this->cBA_BC[1] * d_B[1] + this->cBA_BC[2] * d_B[2];

    residuals[0] = this->weight * (num * (this->inv_den));

    if (jacobians) {
        this->objects.JP_T(0, 1) = point(2);
        this->objects.JP_T(0, 2) = -point(1);
        this->objects.JP_T(1, 0) = -point(2);
        this->objects.JP_T(1, 2) = point(0);
        this->objects.JP_T(2, 0) = point(1);
        this->objects.JP_T(2, 1) = -point(0);

        wave::Transformation<>::SE3ApproxLeftJacobian(this->objects.T_cur_twist, this->objects.Jexp);
        this->objects.Jr_T = this->weight * this->objects.Jr_P * this->objects.JP_T * this->objects.Jexp;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> Jr_Tk(jacobians[0], 1, 12);
            Jr_Tk = this->objects.Jr_T * this->objects.hat;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> Jr_Tkp1(jacobians[1], 1, 12);
            Jr_Tkp1 = this->objects.Jr_T * this->objects.candle;
        }
    }

    return true;
}

/// Jacobian of the residual wrt the transformed point
void SE3PointToPlaneGP::calculateJr_P(Eigen::Matrix<double, 1, 3> &Jr_P) const {
    const double &XA1 = this->ptA[0], &XA2 = this->ptA[1], &XA3 = this->ptA[2], &XB1 = this->ptB[0],
                 &XB2 = this->ptB[1], &XB3 = this->ptB[2], &XC1 = this->ptC[0], &XC2 = this->ptC[1],
                 &XC3 = this->ptC[2];

    Jr_P << -((XA2 - XB2) * (XB3 - XC3) - (XA3 - XB3) * (XB2 - XC2)) * 1.0 /
              sqrt(pow(((XA1 - XB1) * (XB2 - XC2) - (XA2 - XB2) * (XB1 - XC1)), 2) +
                   pow(((XA1 - XB1) * (XB3 - XC3) - (XA3 - XB3) * (XB1 - XC1)), 2) +
                   pow(((XA2 - XB2) * (XB3 - XC3) - (XA3 - XB3) * (XB2 - XC2)), 2)),
      ((XA1 - XB1) * (XB3 - XC3) - (XA3 - XB3) * (XB1 - XC1)) * 1.0 /
        sqrt(pow(((XA1 - XB1) * (XB2 - XC2) - (XA2 - XB2) * (XB1 - XC1)), 2) +
             pow(((XA1 - XB1) * (XB3 - XC3) - (XA3 - XB3) * (XB1 - XC1)), 2) +
             pow(((XA2 - XB2) * (XB3 - XC3) - (XA3 - XB3) * (XB2 - XC2)), 2)),
      -((XA1 - XB1) * (XB2 - XC2) - (XA2 - XB2) * (XB1 - XC1)) * 1.0 /
        sqrt(pow(((XA1 - XB1) * (XB2 - XC2) - (XA2 - XB2) * (XB1 - XC1)), 2) +
             pow(((XA1 - XB1) * (XB3 - XC3) - (XA3 - XB3) * (XB1 - XC1)), 2) +
             pow(((XA2 - XB2) * (XB3 - XC3) - (XA3 - XB3) * (XB2 - XC2)), 2));
}

}  // namespace wave
