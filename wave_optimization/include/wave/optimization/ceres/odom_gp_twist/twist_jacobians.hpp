/**
 * Some common calculations are defined here
 */

#ifndef WAVE_TWIST_JACOBIANS_HPP
#define WAVE_TWIST_JACOBIANS_HPP

#include <Eigen/Core>

namespace wave_optimization {

template<typename Derived, typename MDerived, typename VDerived, typename V1Derived, typename JDerived>
inline void calcInFrameInterpolation(const Eigen::MatrixBase<Derived> &hat,
                                     const Eigen::MatrixBase<Derived> &candle,
                                     const Eigen::MatrixBase<MDerived> &Jlog,
                                     const Eigen::MatrixBase<MDerived> &AdTkp1,
                                     const Eigen::MatrixBase<VDerived> &Xk,
                                     const Eigen::MatrixBase<VDerived> &Xkp1,
                                     Eigen::MatrixBase<V1Derived> &Tint,
                                     Eigen::MatrixBase<JDerived> *JT_Xk = nullptr,
                                     Eigen::MatrixBase<JDerived> *JT_Xkp1 = nullptr) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 1, 2)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<MDerived>, 6, 6)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<VDerived>, 12, 1)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<V1Derived>, 6, 1)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<JDerived>, 6, 12)

    Tint.noalias() = hat(0, 1) * Xk.template block<6, 1>(6, 0) +
    candle(0, 0) * Jlog * (Xkp1.template block<6, 1>(0, 0) - AdTkp1 * Xk.template block<6, 1>(0, 0)) +
    candle(0, 1) * Jlog * Xkp1.template block<6, 1>(6, 0);

    if(JT_Xk) {
        JT_Xk->template block<6, 6>(0,0).noalias() = (candle(0,0) * Jlog) *(-AdTkp1);
        JT_Xk->template block<6, 6>(0,6).noalias() = hat(0,1) * Eigen::MatrixBase<MDerived>::Identity();
    }
    if(JT_Xkp1) {
        JT_Xkp1->template block<6, 6>(0,0).noalias() = candle(0,0) * Jlog;
        JT_Xkp1->template block<6, 6>(0,6).noalias() = candle(0,1) * Jlog;
    }
}

}

#endif //WAVE_TWIST_JACOBIANS_HPP
