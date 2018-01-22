#include "wave/optimization/ceres/point_to_line_gp.hpp"
#include <Eigen/QR>

namespace wave {

SE3PointToLineGP::SE3PointToLineGP(const double *const p,
                                   const double *const pA,
                                   const double *const pB,
                                   const Transformation &prior,
                                   const Transformation &T_k_inverse_prior,
                                   const Transformation &T_kp1_inverse_prior,
                                   const Vec6 &vel_k_prior,
                                   const Vec6 &vel_kp1_prior,
                                   const Eigen::Matrix<double, 6, 12> &JT_Tk,
                                   const Eigen::Matrix<double, 6, 12> &JT_Tkp1,
                                   const Mat3 &CovZ,
                                   bool calculate_weight)
    : pt(p),
      ptA(pA),
      ptB(pB),
      T_prior(prior),
      T_k_inverse_prior(T_k_inverse_prior),
      T_kp1_inverse_prior(T_kp1_inverse_prior),
      vel_k_prior(vel_k_prior),
      vel_kp1_prior(vel_kp1_prior),
      JT_Tk(JT_Tk),
      JT_Kp1(JT_Tkp1) {
    this->JP_T.setZero();
    this->JP_T.block<3, 3>(0, 3).setIdentity();
    this->Jr_Tk.block<2, 6>(0, 6).setZero();
    this->Jr_Tkp1.block<2, 6>(0, 6).setZero();

    this->diff[0] = this->ptB[0] - this->ptA[0];
    this->diff[1] = this->ptB[1] - this->ptA[1];
    this->diff[2] = this->ptB[2] - this->ptA[2];
    this->bottom = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];

    if (this->bottom < 1e-10) {
        // The points defining the line are too close to each other
        throw std::out_of_range("Points defining line are too close!");
    }

    this->Jres_P(0, 0) = 1 - (diff[0] * diff[0] / bottom);
    this->Jres_P(0, 1) = -(diff[0] * diff[1] / bottom);
    this->Jres_P(0, 2) = -(diff[0] * diff[2] / bottom);
    this->Jres_P(1, 0) = -(diff[1] * diff[0] / bottom);
    this->Jres_P(1, 1) = 1 - (diff[1] * diff[1] / bottom);
    this->Jres_P(1, 2) = -(diff[1] * diff[2] / bottom);
    this->Jres_P(2, 0) = -(diff[2] * diff[0] / bottom);
    this->Jres_P(2, 1) = -(diff[2] * diff[1] / bottom);
    this->Jres_P(2, 2) = 1 - (diff[2] * diff[2] / bottom);

    Eigen::Vector3d unitdiff;
    double invlength = 1.0 / sqrt(this->bottom);
    if (this->diff[2] > 0) {
        unitdiff[0] = this->diff[0] * invlength;
        unitdiff[1] = this->diff[1] * invlength;
        unitdiff[2] = this->diff[2] * invlength;
    } else {
        unitdiff[0] = -this->diff[0] * invlength;
        unitdiff[1] = -this->diff[1] * invlength;
        unitdiff[2] = -this->diff[2] * invlength;
    }

    Eigen::Vector3d unitz;
    unitz << 0, 0, 1;

    auto v = unitdiff.cross(unitz);
    auto s = v.norm();
    auto c = unitz.dot(unitdiff);
    auto skew = Transformation::skewSymmetric3(v);
    this->rotation = Eigen::Matrix3d::Identity() + skew + skew * skew * ((1 - c) / (s * s));

    this->Jres_P = this->rotation * this->Jres_P;

    if (calculate_weight) {
        auto rotated = this->Jres_P * CovZ * this->Jres_P.transpose();
        this->weight_matrix = rotated.block<2, 2>(0, 0).inverse().sqrt();
    } else {
        this->weight_matrix.setIdentity();
    }
}

bool SE3PointToLineGP::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Matrix<double, 3, 4>> Tk_mat(parameters[0], 3, 4);
    Eigen::Map<const Eigen::Matrix<double, 3, 4>> Tkp1_mat(parameters[1], 3, 4);
    Eigen::Map<const Vec6> vel_k(parameters[2], 6, 1);
    Eigen::Map<const Vec6> vel_kp1(parameters[3], 6, 1);

    // todo(ben) Template transformation class to avoid copy operation
    Transformation Tk(Tk_mat);
    Transformation Tkp1(Tkp1_mat);

    Vec6 op_point;
    op_point = this->JT_Tk.block<6,6>(0,0) * (Tk * this->T_k_inverse_prior).logMap(0.1) +
               this->JT_Tk.block<6,6>(0,6) * (vel_k - this->vel_k_prior) +
               this->JT_Kp1.block<6,6>(0,0) * (Tkp1 * this->T_kp1_inverse_prior).logMap(0.1) +
               this->JT_Kp1.block<6,6>(0,6) * (vel_kp1 - this->vel_kp1_prior);

    this->T_current = this->T_prior;
    this->T_current.manifoldPlus(op_point);

    Eigen::Map<const Vec3> PT(this->pt, 3, 1);
    Vec3 point = this->T_current.transform(PT);

    double p_A[3] = {point(0) - this->ptA[0], point(1) - this->ptA[1], point(2) - this->ptA[2]};

    double scaling = ceres::DotProduct(p_A, diff);
    // point on line closest to point
    double p_Tl[3] = {this->ptA[0] + (scaling / bottom) * diff[0],
                      this->ptA[1] + (scaling / bottom) * diff[1],
                      this->ptA[2] + (scaling / bottom) * diff[2]};

    double residual_o[3];
    residual_o[0] = point(0) - p_Tl[0];
    residual_o[1] = point(1) - p_Tl[1];
    residual_o[2] = point(2) - p_Tl[2];
    Eigen::Map<Vec3> res(residual_o, 3, 1);
    Eigen::Vector2d reduced = (this->rotation * res).block<2, 1>(0, 0);
    reduced = this->weight_matrix * reduced;
    Eigen::Map<Eigen::Vector2d>(residuals, 2, 1) = reduced;

    if (jacobians != NULL) {
        this->JP_T(0,1) = point(2);
        this->JP_T(0,2) = -point(1);
        this->JP_T(1,0) = -point(2);
        this->JP_T(1,2) = point(0);
        this->JP_T(2,0) = point(1);
        this->JP_T(2,1) = -point(0);

        // Jres_P already has rotation incorporated during construction
        this->Jr_T = this->Jres_P * this->JP_T;

        if (jacobians[0]) {
            this->Jr_Tk.block<2,6>(0,0) = this->weight_matrix * this->Jr_T.block<2,6>(0,0) * this->JT_Tk.block<6,6>(0,0);
            Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>>(jacobians[0], 2, 12) = this->Jr_Tk;
        }
        if (jacobians[1]) {
            this->Jr_Tkp1.block<2,6>(0,0) = this->weight_matrix * this->Jr_T.block<2,6>(0,0) * this->JT_Kp1.block<6,6>(0,0);
            Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>>(jacobians[1], 2, 12) = this->Jr_Tkp1;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jac_map(jacobians[2], 2, 6);
            jac_map = this->weight_matrix * this->Jr_T.block<2,6>(0,0) * this->JT_Tk.block<6,6>(0,6);
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> jac_map(jacobians[3], 2, 6);
            jac_map = this->weight_matrix * this->Jr_T.block<2,6>(0,0) * this->JT_Kp1.block<6,6>(0,6);
        }
    }

    return true;
}

}  // namespace wave
