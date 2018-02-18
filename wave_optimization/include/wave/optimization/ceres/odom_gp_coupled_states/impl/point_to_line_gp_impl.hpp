#ifndef WAVE_POINT_TO_LINE_GP_COUPLED_IMPL_HPP
#define WAVE_POINT_TO_LINE_GP_COUPLED_IMPL_HPP

#include "wave/optimization/ceres/odom_gp_coupled_states/point_to_line_gp.hpp"

namespace wave {

template<int DIM>
SE3PointToLineGPCoupled<DIM>::SE3PointToLineGPCoupled(const double *const p,
                                         const double *const pA,
                                         const double *const pB,
                                         const Eigen::Matrix<double, 6, 12> &hat,
                                         const Eigen::Matrix<double, 6, 12> &candle,
                                       const int &idx_k,
                                         const Mat3 &CovZ,
                                         bool calculate_weight)
        : idx_k(idx_k),
          pt(p),
          ptA(pA),
          ptB(pB),
          hat(hat),
          candle(candle) {
    this->JP_T.setZero();
    this->JP_T.template block<3, 3>(0, 3).setIdentity();

    this->diff[0] = this->ptB[0] - this->ptA[0];
    this->diff[1] = this->ptB[1] - this->ptA[1];
    this->diff[2] = this->ptB[2] - this->ptA[2];
    double bottom = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];

    if (bottom < 1e-10) {
        // The points defining the line are too close to each other
        throw std::out_of_range("Points defining line are too close!");
    }
    this->inv_bottom = 1.0 / bottom;

    this->Jres_P(0, 0) = 1 - (diff[0] * diff[0] * this->inv_bottom);
    this->Jres_P(0, 1) = -(diff[0] * diff[1] * this->inv_bottom);
    this->Jres_P(0, 2) = -(diff[0] * diff[2] * this->inv_bottom);
    this->Jres_P(1, 0) = -(diff[1] * diff[0] * this->inv_bottom);
    this->Jres_P(1, 1) = 1 - (diff[1] * diff[1] * this->inv_bottom);
    this->Jres_P(1, 2) = -(diff[1] * diff[2] * this->inv_bottom);
    this->Jres_P(2, 0) = -(diff[2] * diff[0] * this->inv_bottom);
    this->Jres_P(2, 1) = -(diff[2] * diff[1] * this->inv_bottom);
    this->Jres_P(2, 2) = 1 - (diff[2] * diff[2] * this->inv_bottom);

    Eigen::Vector3d unitdiff;
    double invlength = sqrt(this->inv_bottom);
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
    auto skew = Transformation<void>::skewSymmetric3(v);
    this->rotation = Eigen::Matrix3d::Identity() + skew + skew * skew * ((1 - c) / (s * s));

    this->Jres_P = this->rotation * this->Jres_P;

    if (calculate_weight) {
        auto rotated = this->Jres_P * CovZ * this->Jres_P.transpose();
        this->weight_matrix = rotated.template block<2, 2>(0, 0).inverse().sqrt();
    } else {
        this->weight_matrix.setIdentity();
    }
}

template<int DIM>
bool SE3PointToLineGPCoupled<DIM>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Vec6> vel_k(parameters[0], 6, 1);
    auto tk_ptr = std::make_shared<Eigen::Map<const Eigen::Matrix<double, 3, 4>>>(parameters[0] + 6 + 12*this->idx_k, 3, 4);
    auto tkp1_ptr = std::make_shared<Eigen::Map<const Eigen::Matrix<double, 3, 4>>>(parameters[0] + 6 + 12*(this->idx_k + 1), 3, 4);

    Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>, true> Tk(tk_ptr);
    Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>, true> Tkp1(tkp1_ptr);

    if (jacobians) {
        this->T_current = Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>, true>::interpolateAndJacobians(
                Tk, Tkp1, vel_k, vel_k, this->hat, this->candle,
                this->JT_Ti, this->JT_Tip1, this->JT_Wi, this->JT_Wip1);
    } else {
        this->T_current = Transformation<Eigen::Map<const Eigen::Matrix<double, 3, 4>>, true>::interpolate(Tk, Tkp1,
                                                                                                           vel_k, vel_k,
                                                                                                           this->hat,
                                                                                                           this->candle);
    }

    Eigen::Map<const Vec3> PT(this->pt, 3, 1);
    Vec3 point = this->T_current.transform(PT);

    double p_A[3] = {point(0) - this->ptA[0], point(1) - this->ptA[1], point(2) - this->ptA[2]};

    double scaling = ceres::DotProduct(p_A, diff);
    // point on line closest to point
    double p_Tl[3] = {this->ptA[0] + (scaling * this->inv_bottom) * diff[0],
                      this->ptA[1] + (scaling * this->inv_bottom) * diff[1],
                      this->ptA[2] + (scaling * this->inv_bottom) * diff[2]};

    Eigen::Map<const Vec3> pt_Tl(p_Tl, 3, 1);
    Eigen::Map<Eigen::Vector2d> reduced(residuals, 2, 1);

    reduced = this->weight_matrix * (this->rotation * (point - pt_Tl)).template block<2, 1>(0, 0);

    if (jacobians != nullptr) {
        this->JP_T(0, 1) = point(2);
        this->JP_T(0, 2) = -point(1);
        this->JP_T(1, 0) = -point(2);
        this->JP_T(1, 2) = point(0);
        this->JP_T(2, 0) = point(1);
        this->JP_T(2, 1) = -point(0);

        // Jres_P already has rotation incorporated during construction
        this->Jr_T = this->Jres_P * this->JP_T;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, DIM, Eigen::RowMajor>> jac_map(jacobians[0], 2, DIM);
            jac_map.setZero();

            jac_map.template block<2, 6>(0,0) = this->weight_matrix * this->Jr_T.template block<2, 6>(0, 0) * (this->JT_Wi + this->JT_Wip1);
            jac_map.template block<2, 6>(0,6 + 12*this->idx_k) = this->weight_matrix * this->Jr_T.template block<2, 6>(0, 0) * this->JT_Ti;
            jac_map.template block<2, 6>(0,6 + 12*(1 + this->idx_k)) = this->weight_matrix * this->Jr_T.template block<2, 6>(0, 0) * this->JT_Tip1;
        }
    }

    return true;
}

}

#endif //WAVE_POINT_TO_LINE_GP_COUPLED_IMPL_HPP
