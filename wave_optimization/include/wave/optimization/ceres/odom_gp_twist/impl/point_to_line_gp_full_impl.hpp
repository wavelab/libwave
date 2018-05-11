#ifndef WAVE_POINT_TO_LINE_GP_TWIST_FULL_IMPL_HPP
#define WAVE_POINT_TO_LINE_GP_TWIST_FULL_IMPL_HPP

#include "wave/optimization/ceres/odom_gp_twist/point_to_line_gp_full.hpp"

template<int... idx>
SE3PointToLineGPFull<idx...>::SE3PointToLineGPFull(const float *const pt, const float *const ptA,
                                                   const float *const ptB, SE3PointToLineGPFullObject &object,
                                                   const wave::Mat3 &CovZ, bool calculate_weight)
    : pt(pt), ptA(ptA), ptB(ptB), object(object) {
    this->object.JP_T.setZero();
    this->object.JP_T.block<3, 3>(0, 3).setIdentity();

    this->diff[0] = this->ptB[0] - this->ptA[0];
    this->diff[1] = this->ptB[1] - this->ptA[1];
    this->diff[2] = this->ptB[2] - this->ptA[2];
    this->bottom = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2];

    if (this->bottom < 1e-10) {
        throw std::out_of_range("Points defining line are too close!");
    }

    this->object.Jres_P(0, 0) = 1 - (diff[0] * diff[0] / bottom);
    this->object.Jres_P(0, 1) = -(diff[0] * diff[1] / bottom);
    this->object.Jres_P(0, 2) = -(diff[0] * diff[2] / bottom);
    this->object.Jres_P(1, 0) = -(diff[1] * diff[0] / bottom);
    this->object.Jres_P(1, 1) = 1 - (diff[1] * diff[1] / bottom);
    this->object.Jres_P(1, 2) = -(diff[1] * diff[2] / bottom);
    this->object.Jres_P(2, 0) = -(diff[2] * diff[0] / bottom);
    this->object.Jres_P(2, 1) = -(diff[2] * diff[1] / bottom);
    this->object.Jres_P(2, 2) = 1 - (diff[2] * diff[2] / bottom);

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
    auto skew = wave::Transformation<>::skewSymmetric3(v);
    this->object.rotation = Eigen::Matrix3d::Identity() + skew + skew * skew * ((1 - c) / (s * s));

    this->object.Jres_P = this->object.rotation * this->object.Jres_P;

    if (calculate_weight) {
        auto rotated = this->object.Jres_P * CovZ * this->object.Jres_P.transpose();
        this->weight_matrix = rotated.block<2, 2>(0, 0).inverse().sqrt();
    } else {
        this->weight_matrix.setIdentity();
    }
}

template<int... idx>
bool SE3PointToLineGPFull<idx...>::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    constexpr size_t n_state = sizeof...(idx);
    std::vector<Eigen::Map<const wave::Vec12>> states;
    for (uint32_t i; i < n_state; i++) {
        states.emplace_back(Eigen::Map<const wave::Vec12>(parameters[i]));
    }

    /// 1.

    this->object.T_cur_twist =
      this->object.hat * state_tk_map + this->object.candle * state_tkp1_map;

    wave::Transformation::expMap1st(this->object.T_cur_twist, this->object.T_current);

    wave::Vec3 point = this->object.T_current.block<3,3>(0,0) * this->object.T0_pt + this->object.T_current.block<3,1>(0,3);

    double p_A[3] = {point(0) - this->ptA[0], point(1) - this->ptA[1], point(2) - this->ptA[2]};

    double scaling = ceres::DotProduct(p_A, diff);
    // point on line closest to point
    double p_Tl[3] = {this->ptA[0] + (scaling / bottom) * diff[0],
                      this->ptA[1] + (scaling / bottom) * diff[1],
                      this->ptA[2] + (scaling / bottom) * diff[2]};

    Eigen::Map<const wave::Vec3> pt_Tl(p_Tl, 3, 1);
    Eigen::Map<Eigen::Vector2d> reduced(residuals, 2, 1);

    reduced = this->weight_matrix * (this->object.rotation * (point - pt_Tl)).block<2, 1>(0, 0);

    if (jacobians != nullptr) {
        this->object.JP_T(0, 1) = point(2);
        this->object.JP_T(0, 2) = -point(1);
        this->object.JP_T(1, 0) = -point(2);
        this->object.JP_T(1, 2) = point(0);
        this->object.JP_T(2, 0) = point(1);
        this->object.JP_T(2, 1) = -point(0);

        this->object.Jexp = wave::Transformation<>::SE3ApproxLeftJacobian(this->object.T_cur_twist);

        // Jres_P already has rotation incorporated during construction
        this->object.Jr_T = this->weight_matrix * this->object.Jres_P.block<2, 3>(0, 0) * this->object.JP_T * this->object.Jexp;

        // Remains to be seen whether the jacobian of the exponential map matters
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> Jr_Tk(jacobians[0], 2, 12);
            Jr_Tk = this->object.Jr_T * this->object.hat;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> Jr_Tkp1(jacobians[1], 2, 12);
            Jr_Tkp1 = this->object.Jr_T * this->object.candle;
        }
    }

    return true;

}  // namespace wave_optimization

#endif //WAVE_POINT_TO_LINE_GP_TWIST_FULL_IMPL_HPP
