#include "wave/optimization/ceres/odom_gp/point_to_plane_gp.hpp"

namespace wave {

SE3PointToPlaneGP::SE3PointToPlaneGP(const double *const p,
                                     const double *const pA,
                                     const double *const pB,
                                     const double *const pC,
                                     SE3PointToPlaneGPObjects &objects,
                                     const Mat3 &covZ,
                                     bool use_weighting)
    : pt(p), ptA(pA), ptB(pB), ptC(pC), objects(objects) {
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
    Eigen::Map<const Mat34> tk_map(parameters[0], 3, 4);
    Eigen::Map<const Mat34> tkp1_map(parameters[1], 3, 4);

    Transformation<Eigen::Map<const Mat34>, true> Tk(tk_map);
    Transformation<Eigen::Map<const Mat34>, true> Tkp1(tkp1_map);

    Eigen::Map<const Vec6> vel_k(parameters[2], 6, 1);
    Eigen::Map<const Vec6> vel_kp1(parameters[3], 6, 1);

    if (jacobians) {
        Transformation<Mat34, true>::interpolateAndJacobians<>(Tk,
                                                               Tkp1,
                                                               vel_k,
                                                               vel_kp1,
                                                               this->objects.hat,
                                                               this->objects.candle,
                                                               this->objects.T_current,
                                                               this->objects.JT_Ti,
                                                               this->objects.JT_Tip1,
                                                               this->objects.JT_Wi,
                                                               this->objects.JT_Wip1);
    } else {
        Transformation<Mat34, true>::interpolate<>(
          Tk, Tkp1, vel_k, vel_kp1, this->objects.hat, this->objects.candle, this->objects.T_current);
    }

    Eigen::Map<const Vec3> PT(this->pt, 3, 1);
    Vec3 point = this->objects.T_current.transform(PT);

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


        this->objects.Jr_T = this->objects.Jr_P * this->objects.JP_T;
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> Jr_Tk(jacobians[0], 1, 12);
            Jr_Tk.setZero();
            Jr_Tk.block<1, 6>(0, 0) = this->weight * this->objects.Jr_T * this->objects.JT_Ti;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> Jr_Tkp1(jacobians[1], 1, 12);
            Jr_Tkp1.setZero();
            Jr_Tkp1.block<1, 6>(0, 0) = this->weight * this->objects.Jr_T * this->objects.JT_Tip1;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jac_map(jacobians[2], 1, 6);
            jac_map = this->weight * this->objects.Jr_T * this->objects.JT_Wi;
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> jac_map(jacobians[3], 1, 6);
            jac_map = this->weight * this->objects.Jr_T * this->objects.JT_Wip1;
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
