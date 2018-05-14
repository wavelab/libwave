#ifndef WAVE_POINT_TO_LINE_GP_TWIST_FULL_IMPL_HPP
#define WAVE_POINT_TO_LINE_GP_TWIST_FULL_IMPL_HPP

#include "wave/optimization/ceres/odom_gp_twist/point_to_line_gp_full.hpp"

template<int... idx>
SE3PointToLineGPFull<idx...>::SE3PointToLineGPFull(const float *const pt, const float *const ptA, const float *const ptB,
                                                   SE3PointToLineGPFullObject &object, const wave::Mat3 &CovZ, bool calculate_weight, bool ahead)
    : pt(pt), ptA(ptA), ptB(ptB), ahead(ahead), object(object) {
    this->object.JP_T.setZero();
    this->object.JP_T.block<3, 3>(0, 3).setIdentity();

    if (calculate_weight) {
        this->weight_matrix = CovZ;
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

    // Creating references to sort out possibly twice referenced states
    // Frames M, Mp1 correspond to pt, Frames C, Cp1 the rest.
    const auto &SM = states.at(0);
    const auto &SMp1 = states.at(1);
    const auto &SC = (n_state == 4) ? states.at(2) : ((this->ahead) ? states.at(1) : states.at(2));
    const auto &SCp1 = (n_state == 4) ? states.at(3) : ((this->ahead) ? states.at(2) : states.at(0));

    // Vectorize points to work with them a bit easier
    Eigen::Map<const wave::Vec3f> Pt0(this->pt);
    Eigen::Map<const wave::Vec3f> PtA0(this->ptA);
    Eigen::Map<const wave::Vec3f> PtB0(this->ptB);

    // Update each point location

    wave::Mat34 Tp_inc, TpA_inc, TpB_inc;
    wave::Vec3 Pt, PtA, PtB;

    this->object.twist.noalias() = this->object.hat.at(0)(0, 1) * SM.block<6, 1>(6,0) +
                         this->object.candle.at(0)(0, 0) * this->object.Jlog.at(0) * (SMp1.block<6, 1>(0,0) - this->object.AdTMp1 * SM.block<6, 1>(0,0)) +
                        this->object.candle.at(0)(0,1) * this->object.Jlog.at(0) * SMp1.block<6, 1>(6,0);
    wave::Transformation::expMap1st(this->object.twist, Tp_inc);

    Pt = Tp_inc.block<3,3>(0,0) * Pt0.cast<double>() + Tp_inc.block<3,1>(0,3);

    this->object.twist.noalias() = this->object.hat.at(1)(0, 1) * SC.block<6, 1>(6,0) +
                         this->object.candle.at(1)(0, 0) * this->object.Jlog.at(1) * (SCp1.block<6, 1>(0,0) - this->object.AdTCp1 * SC.block<6, 1>(0,0)) +
                         this->object.candle.at(1)(0,1) * this->object.Jlog.at(1) * SCp1.block<6, 1>(6,0);
    this->object.twist = this->object.AdTMinv * (SC.block<6, 1>(0,0) - SM.block<6, 1>(0,0) + this->object.AdTC * this->object.twist);

    wave::Transformation::expMap1st(this->object.twist, TpA_inc);
    PtA = TpA_inc.block<3,3>(0,0) * PtA0.cast<double>() + TpA_inc.block<3,1>(0,3);

    this->object.twist.noalias() = this->object.hat.at(2)(0, 1) * SC.block<6, 1>(6,0) +
                                   this->object.candle.at(2)(0, 0) * this->object.Jlog.at(1) * (SCp1.block<6, 1>(0,0) - this->object.AdTCp1 * SC.block<6, 1>(0,0)) +
                                   this->object.candle.at(2)(0,1) * this->object.Jlog.at(1) * SCp1.block<6, 1>(6,0);
    this->object.twist = this->object.AdTMinv * (SC.block<6, 1>(0,0) - SM.block<6, 1>(0,0) + this->object.AdTC * this->object.twist);

    wave::Transformation::expMap1st(this->object.twist, TpB_inc);
    PtB = TpB_inc.block<3,3>(0,0) * PtB0.cast<double>() + TpB_inc.block<3,1>(0,3);

    wave::Vec3 p_A = Pt - PtA;
    wave::Vec3 diff = PtB - PtA;

    double scaling = p_A.transpose() * diff;
    double divisor = diff.transpose() * diff;
    // point on line closest to point
    wave::Vec3 p_Tl = PtA + (scaling / divisor) * diff;

    Eigen::Map<wave::Vec3> err(residuals);

    err = this->weight_matrix * (Pt - p_Tl);

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

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> Jr_SM(jacobians[0], 3, 12);
            Jr_Tk = this->object.Jr_T * this->object.hat;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> Jr_Tkp1(jacobians[1], 3, 12);
            Jr_Tkp1 = this->object.Jr_T * this->object.candle;
        }
    }

    return true;

}  // namespace wave_optimization

#endif //WAVE_POINT_TO_LINE_GP_TWIST_FULL_IMPL_HPP
