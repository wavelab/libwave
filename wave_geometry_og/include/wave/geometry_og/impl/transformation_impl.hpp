#ifndef WAVE_TRANSFORMATION_IMPL_HPP
#define WAVE_TRANSFORMATION_IMPL_HPP

#include "wave/geometry_og/exception_helpers.hpp"
#include "wave/geometry_og/transformation.hpp"

namespace wave {

template <typename Derived, bool approximate>
Transformation<Derived, approximate> &Transformation<Derived, approximate>::setIdentity() {
    this->storage.template block<3, 3>(0, 0).setIdentity();
    this->storage.template block<3, 1>(0, 3).setZero();
    return *this;
}

template <typename Derived, bool approximate>
Transformation<Derived, approximate>::Transformation(const Vec3 &eulers, const Vec3 &translation) {
    this->setFromEulerXYZ(eulers, translation);
}

template <typename Derived, bool approximate>
Transformation<Derived, approximate> &Transformation<Derived, approximate>::setFromEulerXYZ(const Vec3 &eulers,
                                                                                            const Vec3 &translation) {
    checkMatrixFinite(eulers);
    checkMatrixFinite(translation);

    Mat3 rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(eulers[2], Vec3::UnitZ()) * Eigen::AngleAxisd(eulers[1], Vec3::UnitY()) *
                      Eigen::AngleAxisd(eulers[0], Vec3::UnitX());

    this->storage.template block<3, 3>(0, 0) = rotation_matrix;
    this->storage.template block<3, 1>(0, 3) = translation;

    return *this;
}

template <typename Derived, bool approximate>
Transformation<Derived, approximate> &Transformation<Derived, approximate>::setFromMatrix(const Mat4 &input_matrix) {
    checkMatrixFinite(input_matrix);

    this->storage = input_matrix.block<3, 4>(0, 0);

    return *this;
}

template <typename Derived, bool approximate>
template <typename V_Type, typename M_Type>
void Transformation<Derived, approximate>::skewSymmetric3(const Eigen::MatrixBase<V_Type> &V,
                                                          Eigen::MatrixBase<M_Type> &retval) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<M_Type>, 3, 3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<V_Type>, 3)
    retval.derived()(0, 0) = 0.0;
    retval.derived()(0, 1) = -V.derived()(2);
    retval.derived()(0, 2) = V.derived()(1);
    retval.derived()(1, 0) = V.derived()(2);
    retval.derived()(1, 1) = 0.0;
    retval.derived()(1, 2) = -V.derived()(0);
    retval.derived()(2, 0) = -V.derived()(1);
    retval.derived()(2, 1) = V.derived()(0);
    retval.derived()(2, 2) = 0.0;
}

template <typename Derived, bool approximate>
template <typename V_Type, typename M_Type>
void Transformation<Derived, approximate>::skewSymmetric3(const Eigen::MatrixBase<V_Type> &V,
                                                          Eigen::MatrixBase<M_Type> &&retval) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<M_Type>, 3, 3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<V_Type>, 3)
    retval.derived()(0, 0) = 0.0;
    retval.derived()(0, 1) = -V.derived()(2);
    retval.derived()(0, 2) = V.derived()(1);
    retval.derived()(1, 0) = V.derived()(2);
    retval.derived()(1, 1) = 0.0;
    retval.derived()(1, 2) = -V.derived()(0);
    retval.derived()(2, 0) = -V.derived()(1);
    retval.derived()(2, 1) = V.derived()(0);
    retval.derived()(2, 2) = 0.0;
}

template <typename Derived, bool approximate>
template <typename V_Type>
Mat3 Transformation<Derived, approximate>::skewSymmetric3(const Eigen::MatrixBase<V_Type> &V) {
    Mat3 retval;
    skewSymmetric3(V, retval);
    return retval;
}

template <typename Derived, bool approximate>
template <typename V_Type, typename M_Type>
void Transformation<Derived, approximate>::skewSymmetric6(const Eigen::MatrixBase<V_Type> &W,
                                                          Eigen::MatrixBase<M_Type> &retval) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<M_Type>, 6, 6)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<V_Type>, 6)

    skewSymmetric3(W.template block<3, 1>(0, 0), retval.template block<3, 3>(0, 0));
    skewSymmetric3(W.template block<3, 1>(0, 0), retval.template block<3, 3>(3, 3));
    skewSymmetric3(W.template block<3, 1>(3, 0), retval.template block<3, 3>(3, 0));
    retval.template block<3, 3>(0, 3).setZero();
}

template <typename Derived, bool approximate>
template <typename InType, typename Other, bool InApprox, bool O_approx>
void Transformation<Derived, approximate>::interpolate(const Transformation<InType, InApprox> &T_k,
                                                       const Transformation<InType, InApprox> &T_kp1,
                                                       const Vec6 &twist_k,
                                                       const Vec6 &twist_kp1,
                                                       const Mat12 &hat,
                                                       const Mat12 &candle,
                                                       Transformation<Other, O_approx> &T_int,
                                                       Vec6 *interpTwist) {
    Mat6 J_left;
    auto eps = T_kp1.manifoldMinus(T_k);

    if (approximate) {
        J_left = Transformation<>::SE3ApproxInvLeftJacobian(eps);
    } else {
        J_left = Transformation<>::SE3LeftJacobian(eps).inverse();
    }

    T_int = T_k;
    T_int.manifoldPlus(hat.block<6, 6>(0, 6) * twist_k + candle.block<6, 6>(0, 0) * eps +
                       candle.block<6, 6>(0, 6) * J_left * twist_kp1);
    if (interpTwist) {
        *interpTwist = hat.block<6, 6>(6, 6) * twist_k + candle.block<6, 6>(6, 0) * eps +
                       candle.block<6, 6>(6, 6) * J_left * twist_kp1;
    }
}

template <typename Derived, bool approximate>
template <typename InType, typename Other, bool InApprox, bool O_approx>
void Transformation<Derived, approximate>::interpolateReduced(const Transformation<InType, InApprox> &T_k,
                                                       const Transformation<InType, InApprox> &T_kp1,
                                                       const Vec6 &twist_k,
                                                       const Vec6 &twist_kp1,
                                                       const Mat2 &hat,
                                                       const Mat2 &candle,
                                                       Transformation<Other, O_approx> &T_int,
                                                       Vec6 *interpTwist) {
    Mat6 J_left;
    auto eps = T_kp1.manifoldMinus(T_k);

    if (approximate) {
        J_left = Transformation<>::SE3ApproxInvLeftJacobian(eps);
    } else {
        J_left = Transformation<>::SE3LeftJacobian(eps).inverse();
    }

    T_int = T_k;
    T_int.manifoldPlus(hat(0, 1) * twist_k + candle(0, 0) * eps + candle(0, 1) * J_left * twist_kp1);
    if (interpTwist) {
        *interpTwist = hat(1, 1) * twist_k + candle(1, 0) * eps + candle(1, 1) * J_left * twist_kp1;
    }
}

template <typename Derived, bool approximate>
template <typename InType, typename Other, bool InApprox, bool O_approx>
void Transformation<Derived, approximate>::interpolateAndJacobians(const Transformation<InType, InApprox> &T_k,
                                                                   const Transformation<InType, InApprox> &T_kp1,
                                                                   const Vec6 &twist_k,
                                                                   const Vec6 &twist_kp1,
                                                                   const Mat12 &hat,
                                                                   const Mat12 &candle,
                                                                   Transformation<Other, O_approx> &T_int,
                                                                   Mat6 &J_Tk,
                                                                   Mat6 &J_Tkp1,
                                                                   Mat6 &J_twist_k,
                                                                   Mat6 &J_twist_kp1) {
    Mat6 J_left, J_right;
    auto eps = T_kp1.manifoldMinusAndJacobian(T_k, &J_left, &J_right);
    Vec6 int_twist =
      hat.block<6, 6>(0, 6) * twist_k + candle.block<6, 6>(0, 0) * eps + candle.block<6, 6>(0, 6) * J_left * twist_kp1;

    T_int.setFromExpMap(int_twist);

    Mat6 J_comp_left, J_comp_right;

    T_int = T_int.composeAndJacobian(T_k, J_comp_left, J_comp_right);

    Mat6 Jexp;

    if (approximate) {
        Transformation<Derived>::SE3ApproxLeftJacobian(int_twist,  Jexp);
    } else {
        Jexp = Transformation<Derived>::SE3LeftJacobian(int_twist);
    };

    Mat6 bsfactor;
    bsfactor << (eps(1) * twist_kp1(1)) * 0.08333333333333333333 + (eps(2) * twist_kp1(2)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(1)) * 0.08333333333333333333 - twist_kp1(2) * 0.5 -
        (eps(1) * twist_kp1(0)) * 0.1666666666666666666666,
      twist_kp1(1) * 0.5 + (eps(0) * twist_kp1(2)) * 0.08333333333333333333 -
        (eps(2) * twist_kp1(0)) * 0.1666666666666666666666,
      0, 0, 0, twist_kp1(2) * 0.5 - (eps(0) * twist_kp1(1)) * 0.1666666666666666666666 +
                 (eps(1) * twist_kp1(0)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(0)) * 0.08333333333333333333 + (eps(2) * twist_kp1(2)) * 0.08333333333333333333,
      (eps(1) * twist_kp1(2)) * 0.08333333333333333333 - twist_kp1(0) * 0.5 -
        (eps(2) * twist_kp1(1)) * 0.1666666666666666666666,
      0, 0, 0, (eps(2) * twist_kp1(0)) * 0.08333333333333333333 - (eps(0) * twist_kp1(2)) * 0.1666666666666666666666 -
                 twist_kp1(1) * 0.5,
      twist_kp1(0) * 0.5 - (eps(1) * twist_kp1(2)) * 0.1666666666666666666666 +
        (eps(2) * twist_kp1(1)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(0)) * 0.08333333333333333333 + (eps(1) * twist_kp1(1)) * 0.08333333333333333333, 0, 0, 0,
      (eps(1) * twist_kp1(4)) * 0.08333333333333333333 + (eps(4) * twist_kp1(1)) * 0.08333333333333333333 +
        (eps(2) * twist_kp1(5)) * 0.08333333333333333333 + (eps(5) * twist_kp1(2)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(4)) * 0.08333333333333333333 - twist_kp1(5) * 0.5 -
        (eps(1) * twist_kp1(3)) * 0.1666666666666666666666 + (eps(3) * twist_kp1(1)) * 0.08333333333333333333 -
        (eps(4) * twist_kp1(0)) * 0.1666666666666666666666,
      twist_kp1(4) * 0.5 + (eps(0) * twist_kp1(5)) * 0.08333333333333333333 -
        (eps(2) * twist_kp1(3)) * 0.1666666666666666666666 + (eps(3) * twist_kp1(2)) * 0.08333333333333333333 -
        (eps(5) * twist_kp1(0)) * 0.1666666666666666666666,
      (eps(1) * twist_kp1(1)) * 0.08333333333333333333 + (eps(2) * twist_kp1(2)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(1)) * 0.08333333333333333333 - twist_kp1(2) * 0.5 -
        (eps(1) * twist_kp1(0)) * 0.1666666666666666666666,
      twist_kp1(1) * 0.5 + (eps(0) * twist_kp1(2)) * 0.08333333333333333333 -
        (eps(2) * twist_kp1(0)) * 0.1666666666666666666666,
      twist_kp1(5) * 0.5 - (eps(0) * twist_kp1(4)) * 0.1666666666666666666666 +
        (eps(1) * twist_kp1(3)) * 0.08333333333333333333 - (eps(3) * twist_kp1(1)) * 0.1666666666666666666666 +
        (eps(4) * twist_kp1(0)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(3)) * 0.08333333333333333333 + (eps(3) * twist_kp1(0)) * 0.08333333333333333333 +
        (eps(2) * twist_kp1(5)) * 0.08333333333333333333 + (eps(5) * twist_kp1(2)) * 0.08333333333333333333,
      (eps(1) * twist_kp1(5)) * 0.08333333333333333333 - twist_kp1(3) * 0.5 -
        (eps(2) * twist_kp1(4)) * 0.1666666666666666666666 + (eps(4) * twist_kp1(2)) * 0.08333333333333333333 -
        (eps(5) * twist_kp1(1)) * 0.1666666666666666666666,
      twist_kp1(2) * 0.5 - (eps(0) * twist_kp1(1)) * 0.1666666666666666666666 +
        (eps(1) * twist_kp1(0)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(0)) * 0.08333333333333333333 + (eps(2) * twist_kp1(2)) * 0.08333333333333333333,
      (eps(1) * twist_kp1(2)) * 0.08333333333333333333 - twist_kp1(0) * 0.5 -
        (eps(2) * twist_kp1(1)) * 0.1666666666666666666666,
      (eps(2) * twist_kp1(3)) * 0.08333333333333333333 - (eps(0) * twist_kp1(5)) * 0.1666666666666666666666 -
        twist_kp1(4) * 0.5 - (eps(3) * twist_kp1(2)) * 0.1666666666666666666666 +
        (eps(5) * twist_kp1(0)) * 0.08333333333333333333,
      twist_kp1(3) * 0.5 - (eps(1) * twist_kp1(5)) * 0.1666666666666666666666 +
        (eps(2) * twist_kp1(4)) * 0.08333333333333333333 - (eps(4) * twist_kp1(2)) * 0.1666666666666666666666 +
        (eps(5) * twist_kp1(1)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(3)) * 0.08333333333333333333 + (eps(3) * twist_kp1(0)) * 0.08333333333333333333 +
        (eps(1) * twist_kp1(4)) * 0.08333333333333333333 + (eps(4) * twist_kp1(1)) * 0.08333333333333333333,
      (eps(2) * twist_kp1(0)) * 0.08333333333333333333 - (eps(0) * twist_kp1(2)) * 0.1666666666666666666666 -
        twist_kp1(1) * 0.5,
      twist_kp1(0) * 0.5 - (eps(1) * twist_kp1(2)) * 0.1666666666666666666666 +
        (eps(2) * twist_kp1(1)) * 0.08333333333333333333,
      (eps(0) * twist_kp1(0)) * 0.08333333333333333333 + (eps(1) * twist_kp1(1)) * 0.08333333333333333333;

    J_Tk = Jexp * (candle.block<6, 6>(0, 0) * J_right + candle.block<6, 6>(0, 6) * bsfactor * J_right) + J_comp_right;
    J_Tkp1 = Jexp * (candle.block<6, 6>(0, 0) * J_left + candle.block<6, 6>(0, 6) * bsfactor * J_left);
    J_twist_k = Jexp * hat.block<6, 6>(0, 6);
    J_twist_kp1 = Jexp * candle.block<6, 6>(0, 6) * J_left;
}

template <typename Derived, bool approximate>
Mat6 Transformation<Derived, approximate>::adjointRep() const {
    Mat6 retval;
    retval.template block<3, 3>(0, 0) = this->storage.template block<3, 3>(0, 0);
    retval.template block<3, 3>(3, 3) = this->storage.template block<3, 3>(0, 0);
    Mat3 skew;
    skewSymmetric3(this->storage.template block<3, 1>(0, 3), skew);
    retval.template block<3, 3>(3, 0) = skew * this->storage.template block<3, 3>(0, 0);
    retval.template block<3, 3>(0, 3).setZero();
    return retval;
}

template <typename Derived, bool approximate>
void Transformation<Derived, approximate>::Jinterpolated(const Vec6 &twist, const double &alpha, Mat6 &retval) {
    // 3rd order approximation

    double A, B, C;
    A = (alpha * (alpha - 1.0)) * 0.5;
    B = (alpha * (alpha - 1.0) * (2.0 * alpha - 1.0)) * 0.0833333333333333333;
    C = (alpha * alpha * (alpha - 1) * (alpha - 1.0)) * 0.0416666666666666667;

    Mat6 adjoint;
    Transformation<Derived>::Adjoint(twist, adjoint);

    retval.noalias() = alpha * Mat6::Identity() + A * adjoint + B * adjoint * adjoint + C * adjoint * adjoint * adjoint;
}

template <typename Derived, bool approximate>
void Transformation<Derived, approximate>::Adjoint(const Vec6 &twist, Mat6 &retval) {
    retval.setZero();

    retval(0, 1) = -twist(2);
    retval(0, 2) = twist(1);
    retval(1, 0) = twist(2);
    retval(1, 2) = -twist(0);
    retval(2, 0) = -twist(1);
    retval(2, 1) = twist(0);

    retval(3, 4) = -twist(2);
    retval(3, 5) = twist(1);
    retval(4, 3) = twist(2);
    retval(4, 5) = -twist(0);
    retval(5, 3) = -twist(1);
    retval(5, 4) = twist(0);

    retval(3, 1) = -twist(5);
    retval(3, 2) = twist(4);
    retval(4, 0) = twist(5);
    retval(4, 2) = -twist(3);
    retval(5, 0) = -twist(4);
    retval(5, 1) = twist(3);
}

template <typename Derived, bool approximate>
void Transformation<Derived, approximate>::J_lift(Eigen::Matrix<double, 12, 6> &retval) const {
    retval.setZero();

    retval(0, 1) = this->storage.operator()(2, 0);
    retval(0, 2) = -this->storage.operator()(1, 0);
    retval(1, 0) = -this->storage.operator()(2, 0);
    retval(1, 2) = this->storage.operator()(0, 0);
    retval(2, 0) = this->storage.operator()(1, 0);
    retval(2, 1) = -this->storage.operator()(0, 0);

    retval(3, 1) = this->storage.operator()(2, 1);
    retval(3, 2) = -this->storage.operator()(1, 1);
    retval(4, 0) = -this->storage.operator()(2, 1);
    retval(4, 2) = this->storage.operator()(0, 1);
    retval(5, 0) = this->storage.operator()(1, 1);
    retval(5, 1) = -this->storage.operator()(0, 1);

    retval(6, 1) = this->storage.operator()(2, 2);
    retval(6, 2) = -this->storage.operator()(1, 2);
    retval(7, 0) = -this->storage.operator()(2, 2);
    retval(7, 2) = this->storage.operator()(0, 2);
    retval(8, 0) = this->storage.operator()(1, 2);
    retval(8, 1) = -this->storage.operator()(0, 2);

    retval(9, 1) = this->storage.operator()(2, 3);
    retval(9, 2) = -this->storage.operator()(1, 3);
    retval(10, 0) = -this->storage.operator()(2, 3);
    retval(10, 2) = this->storage.operator()(0, 3);
    retval(11, 0) = this->storage.operator()(1, 3);
    retval(11, 1) = -this->storage.operator()(0, 3);

    retval(9, 3) = 1;
    retval(10, 4) = 1;
    retval(11, 5) = 1;
}

template <typename Derived, bool approximate>
template <typename VType>
Transformation<Derived, approximate> &Transformation<Derived, approximate>::setFromExpMap(
  const Eigen::MatrixBase<VType> &se3_vector) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<VType>, 6)
    checkMatrixFinite(se3_vector);

    this->expMap(se3_vector.eval(), this->storage);

    return *this;
}

template <typename T_str, bool approximate>
template <typename VType, typename MType>
void Transformation<T_str, approximate>::expMap(const Eigen::MatrixBase<VType> &W, Eigen::MatrixBase<MType> &retval) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<VType>, 6)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<MType>, 3, 4)

    using VScalar = typename VType::Scalar;
    using MScalar = typename MType::Scalar;

    Eigen::Matrix<VScalar, 3, 3> wx;
    skewSymmetric3(W.template block<3, 1>(0, 0), wx);
    VScalar wn = W.template block<3, 1>(0, 0).norm();

    if (approximate) {
        // 2nd order taylor expansion
        retval.template block<3, 3>(0, 0).noalias() = (Eigen::Matrix<VScalar, 3, 3>::Identity() +
                                                      (1.0 - 0.16666666666666667 * (wn * wn)) * wx +
                                                      (0.5 - 4.166666666666666667e-2 * (wn * wn)) * wx * wx).template cast<MScalar>();
        retval.template block<3, 1>(0, 3).noalias() =
                ((Eigen::Matrix<VScalar, 3, 3>::Identity() + (0.5 - 4.166666666666666667e-2 * (wn * wn)) * wx +
           (0.16666666666666667 - 8.33333333333333333e-3 * (wn * wn)) * wx * wx) *
          W.template block<3, 1>(3, 0)).template cast<MScalar>();
    } else {
        VScalar A, B, C;
        if (wn > tol) {
            A = std::sin(wn) / wn;
            B = (1.0 - std::cos(wn)) / (wn * wn);
            C = (1.0 - A) / (wn * wn);
        } else {
            // Use taylor expansion
            A = 1.0 - 0.16666666666666667 * (wn * wn) + 8.33333333333333333e-3 * (wn * wn * wn * wn);
            B = 0.5 - 4.166666666666666667e-2 * (wn * wn) + 1.38888888888888888889e-3 * (wn * wn * wn * wn);
            C = 0.16666666666666667 - 8.33333333333333333e-3 * (wn * wn) + 1.984126984126984e-04 * (wn * wn * wn * wn);
        }
        retval.template block<3, 3>(0, 0).noalias() = (Eigen::Matrix<VScalar, 3, 3>::Identity() + A * wx + B * wx * wx).template cast<MScalar>();
        retval.template block<3, 1>(0, 3).noalias() =
                ((Eigen::Matrix<VScalar, 3, 3>::Identity() + B * wx + C * wx * wx) * W.template block<3, 1>(3, 0)).template cast<MScalar>();
    }
}

template <typename T_str, bool approximate>
template <typename VType, typename MType>
void Transformation<T_str, approximate>::expMap1st(const Eigen::MatrixBase<VType> &W, Eigen::MatrixBase<MType> &retval) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<VType>, 6)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<MType>, 3, 4)

    using VScalar = typename VType::Scalar;
    using MScalar = typename MType::Scalar;

    Eigen::Matrix<VScalar, 3, 3> wx;
    skewSymmetric3(W.template block<3, 1>(0, 0), wx);

    // 1st order taylor expansion
    retval.template block<3, 3>(0, 0).noalias() = (Eigen::Matrix<VScalar, 3, 3>::Identity() + wx).template cast<MScalar>();
    retval.template block<3, 1>(0, 3).noalias() = ((Eigen::Matrix<VScalar, 3, 3>::Identity() + 0.5 * wx) * W.template block<3, 1>(3, 0)).template cast<MScalar>();
}

template <typename Derived, bool approximate>
Mat6 Transformation<Derived, approximate>::expMapAdjoint(const Vec6 &W) {
    double wn = W.template block<3, 1>(0, 0).norm();

    Mat6 skew;
    skewSymmetric6(W, skew);

    double s = std::sin(wn);
    double c = std::cos(wn);

    double A, B, C, D;
    if (wn > tol) {
        A = (3.0 * s - wn * c) / (2.0 * wn);
        B = (4.0 - wn * s - 4.0 * c) / (2.0 * wn * wn);
        C = (s - wn * c) / (2.0 * wn * wn * wn);
        D = (2.0 - wn * s - 2.0 * c) / (2.0 * wn * wn * wn * wn);

        return Mat6::Identity() + A * skew + B * skew * skew + C * skew * skew * skew + D * skew * skew * skew * skew;
    } else {
        // Fudge it
        return Mat6::Identity() + skew;
    }
}

template <typename Derived, bool approximate>
Mat6 Transformation<Derived, approximate>::SE3LeftJacobian(const Vec6 &W) {
    Mat3 wx;
    Transformation<Derived>::skewSymmetric3(W.template block<3, 1>(0, 0), wx);
    double wn = W.template block<3, 1>(0, 0).norm();

    Mat6 retval, adj;
    retval.setZero();
    adj.setZero();

    // This is applying the adjoint operator to the se(3) vector: se(3) -> adj(se(3))
    adj.template block<3, 3>(0, 0) = wx;
    adj.template block<3, 3>(3, 3) = wx;
    skewSymmetric3(W.template block<3, 1>(3, 0), adj.template bottomLeftCorner<3, 3>());
    //    adj.template block<3, 3>(3, 0) = Transformation<Derived>::skewSymmetric3(W.template block<3, 1>(3, 0));

    double A, B, C, D;
    if (wn > tol) {
        A = ((4.0 - wn * std::sin(wn) - 4.0 * cos(wn)) / (2.0 * wn * wn));
        B = (((4.0 * wn - 5.0 * std::sin(wn) + wn * std::cos(wn))) / (2.0 * wn * wn * wn));
        C = ((2.0 - wn * std::sin(wn) - 2.0 * std::cos(wn)) / (2.0 * wn * wn * wn * wn));
        D = ((2.0 * wn - 3.0 * std::sin(wn) + wn * std::cos(wn)) / (2.0 * wn * wn * wn * wn * wn));

        retval.noalias() = Mat6::Identity() + A * adj + B * adj * adj + C * adj * adj * adj + D * adj * adj * adj * adj;
    } else {
        // First order taylor expansion
        retval.noalias() = Mat6::Identity() + 0.5 * adj;  // + 0.1666666666666666666666 * adj * adj;
    }
    return retval;
}

template <typename Derived, bool approximate>
template <typename VType, typename MType>
void Transformation<Derived, approximate>::SE3ApproxLeftJacobian(const Eigen::MatrixBase<VType> &W, Eigen::MatrixBase<MType> &J) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<VType>, 6)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<MType>, 6, 6)

    using VScalar = typename VType::Scalar;
    using MScalar = typename MType::Scalar;

    Eigen::Matrix<VScalar, 3, 3> wx;
    skewSymmetric3(W.template block<3, 1>(0, 0), wx);

    J.setIdentity();

    J.template block<3, 3>(0, 0).noalias() += 0.5 * wx.template cast<MScalar>();
    J.template block<3, 3>(3, 3).noalias() += 0.5 * wx.template cast<MScalar>();
    J.template block<3, 3>(3, 0).noalias() += 0.5 * skewSymmetric3(W.template block<3, 1>(3, 0)).template cast<MScalar>();
}

template <typename Derived, bool approximate>
template <typename OtherDerived>
Eigen::Matrix<typename OtherDerived::Scalar, 6, 6> Transformation<Derived, approximate>::SE3ApproxInvLeftJacobian(
  const Eigen::MatrixBase<OtherDerived> &W) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<OtherDerived>, 6)

    using Scalar = typename OtherDerived::Scalar;

    Eigen::Matrix<Scalar, 3, 3> wx;
    skewSymmetric3(W.template block<3, 1>(0, 0), wx);

    Eigen::Matrix<Scalar, 6, 6> retval, adj;
    retval.setZero();
    adj.setZero();

    // This is applying the adjoint operator to the se(3) vector: se(3) -> adj(se(3))
    adj.template block<3, 3>(0, 0) = wx;
    adj.template block<3, 3>(3, 3) = wx;
    skewSymmetric3(W.template block<3, 1>(3, 0), adj.template block<3, 3>(3, 0));
    //    adj.template block<3, 3>(3, 0) = skewSymmetric3(W.template block<3, 1>(3, 0));

    retval.noalias() = Eigen::Matrix<Scalar, 6, 6>::Identity() - 0.5 * adj +
                       0.0833333333333333333 * adj * adj;  // + C*adj*adj*adj + D*adj*adj*adj*adj;

    return retval;
}

template <typename Derived, bool approximate>
Vec6 Transformation<Derived, approximate>::logMap(double tolerance) const {
    Vec6 retval;

    double wn;
    // Need to pander to ceres gradient checker a bit here
    if ((this->storage.template block<3, 3>(0, 0).trace() - 1.0) / 2.0 > 1.0) {
        wn = 0;
    } else {
        wn = std::acos((this->storage.template block<3, 3>(0, 0).trace() - 1.0) / 2.0);
    }

    if (approximate) {
        // 2nd order taylor expansion
        Mat3 skew = (0.5 + wn * wn * 0.083333333333333) *
                    (this->storage.template block<3, 3>(0, 0) - this->storage.template block<3, 3>(0, 0).transpose());
        retval(0) = skew(2, 1);
        retval(1) = skew(0, 2);
        retval(2) = skew(1, 0);

        retval.template block<3, 1>(3, 0) = (Mat3::Identity() - 0.5 * skew) * this->storage.template block<3, 1>(0, 3);
    } else {
        double A, B;
        Mat3 skew, Vinv;
        if (wn > tolerance) {
            A = wn / (2 * std::sin(wn));
            B = (1 - std::cos(wn)) / (wn * wn);
            skew =
              A * (this->storage.template block<3, 3>(0, 0) - this->storage.template block<3, 3>(0, 0).transpose());
            Vinv = Mat3::Identity() - 0.5 * skew + (1 / (wn * wn)) * (1 - (1 / (4 * A * B))) * skew * skew;
        } else {
            // Third order taylor expansion
            A = 0.5 + wn * wn / 12.0 + wn * wn * wn * wn * (7.0 / 720.0);
            B = 0.5 - wn * wn / 24.0 + wn * wn * wn * wn * (7.0 / 720.0);
            skew =
              A * (this->storage.template block<3, 3>(0, 0) - this->storage.template block<3, 3>(0, 0).transpose());
            Vinv = Mat3::Identity() - 0.5 * skew;
        }

        retval(0) = skew(2, 1);
        retval(1) = skew(0, 2);
        retval(2) = skew(1, 0);

        retval.template block<3, 1>(3, 0) = Vinv * this->storage.template block<3, 1>(0, 3);
    }

    return retval;
}

template <typename Derived, bool approximate>
Vec6 Transformation<Derived, approximate>::logMap(const Transformation &T) {
    return T.logMap();
}

template <typename Derived, bool approximate>
Vec3 Transformation<Derived, approximate>::transform(const Vec3 &input_vector) const {
    Vec3 retval;
    this->transform(input_vector, retval);
    return retval;
}

template <typename Derived, bool approximate>
template <typename IP_T, typename OP_T>
void Transformation<Derived, approximate>::transform(const Eigen::MatrixBase<IP_T> &ip_vec,
                                                     Eigen::MatrixBase<OP_T> &op_vec) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<IP_T>, 3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<OP_T>, 3)
    op_vec = this->storage.template block<3, 3>(0, 0) * ip_vec + this->storage.template block<3, 1>(0, 3);
}

template <typename Derived, bool approximate>
template <typename IP_T, typename OP_T, typename M_T1, typename M_T2>
void Transformation<Derived, approximate>::transformAndJacobian(const Eigen::MatrixBase<IP_T> &input_vector,
                                                                Eigen::MatrixBase<OP_T> &output_vector,
                                                                Eigen::MatrixBase<M_T1> *Jpoint,
                                                                Eigen::MatrixBase<M_T2> *Jparam) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<IP_T>, 3)
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<OP_T>, 3)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<M_T1>, 3, 3)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<M_T2>, 3, 6)

    using M1_Type = typename M_T1::Scalar;

    this->transform(input_vector, output_vector);

    if(Jpoint) {
        *Jpoint = this->storage.template block<3, 3>(0, 0).template cast<M1_Type>();
    }

    if(Jparam) {
        skewSymmetric3(output_vector, Jparam->template block<3, 3>(0, 0));
        Jparam->template block<3, 3>(0, 0) *= -1;
        Jparam->template block<3, 3>(0, 3).setIdentity();
    }
}

template <typename Derived, bool approximate>
Vec3 Transformation<Derived, approximate>::inverseTransform(const Vec3 &input_vector) const {
    Vec3 retval;
    this->inverseTransform(input_vector, retval);
    return retval;
}

template <typename Derived, bool approximate>
template <typename IP_T, typename OP_T>
void Transformation<Derived, approximate>::inverseTransform(const Eigen::MatrixBase<IP_T> &ip_vec,
                                                            Eigen::MatrixBase<OP_T> &op_vec) const {
    op_vec = this->storage.template block<3, 3>(0, 0).transpose() * ip_vec -
             this->storage.template block<3, 3>(0, 0).transpose() * this->storage.template block<3, 1>(0, 3);
}

template <typename Derived, bool approximate>
Transformation<Derived, approximate> &Transformation<Derived, approximate>::invert() {
    this->storage.template block<3, 3>(0, 0) = this->storage.template block<3,3>(0,0).transpose().eval();
    this->storage.template block<3, 1>(0, 3) =
      -this->storage.template block<3, 3>(0, 0) * this->storage.template block<3, 1>(0, 3);

    return *this;
}

template <typename Derived, bool approximate>
template <typename T_OUT, bool approx>
void Transformation<Derived, approximate>::transformInverse(Transformation<T_OUT, approx> &T_inv) const {
    T_inv.storage = this->storage;
    T_inv.invert();
}

template <typename Derived, bool approximate>
Transformation<Mat34, approximate> Transformation<Derived, approximate>::transformInverse() const {
    Transformation<Mat34, approximate> retval;
    this->transformInverse(retval);
    return retval;
}

template <typename Derived, bool approximate>
bool Transformation<Derived, approximate>::isNear(const Transformation &other, double comparison_threshold) const {
    Vec6 diff = this->manifoldMinus(other);
    if (diff.norm() > comparison_threshold) {
        return false;
    } else {
        return true;
    }
}

template <typename Derived, bool approximate>
template <typename VType>
Transformation<Derived, approximate> &Transformation<Derived, approximate>::manifoldPlus(
  const Eigen::MatrixBase<VType> &omega) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<VType>, 6)

    Mat34 incremental;
    expMap(omega, incremental);

    this->storage.template block<3, 1>(0, 3) =
      incremental.template block<3, 3>(0, 0) * this->storage.template block<3, 1>(0, 3) +
      incremental.template block<3, 1>(0, 3);
    this->storage.template block<3, 3>(0, 0) =
      incremental.template block<3, 3>(0, 0) * this->storage.template block<3, 3>(0, 0);

    return *this;
}

template <typename Derived, bool approximate>
template <typename Other, bool OtherApprox>
Vec6 Transformation<Derived, approximate>::manifoldMinus(const Transformation<Other, OtherApprox> &T) const {
    // Need a temporary transform object
    Transformation<Mat34, approximate> Ttemp;
    T.transformInverse(Ttemp);

    return ((*this) * Ttemp).logMap();
}

template <typename Derived, bool approximate>
template <typename Other, bool OtherApprox, typename OtherDerived>
void Transformation<Derived, approximate>::manifoldMinus(const Transformation<Other, OtherApprox> &T, Eigen::MatrixBase<OtherDerived>& retval) const {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<OtherDerived>, 6)
    retval = ((*this) * T.transformInverse()).logMap();
}

template <typename Derived, bool approximate>
template <typename otherStorage, bool otherApprox>
Vec6 Transformation<Derived, approximate>::manifoldMinusAndJacobian(const Transformation<otherStorage, otherApprox> &T,
                                                                    Mat6 *J_left,
                                                                    Mat6 *J_right) const {
    // logmap(T1 * inv(T2))
    Mat6 J_logm;
    Transformation<Mat34, approximate> T2inv;

    T.transformInverse(T2inv);
    Transformation<Mat34, approximate> diff;
    diff = (*this) * T2inv;
    auto manifold_difference = diff.logMap();

    if constexpr (approximate) {
        J_logm = Transformation<>::SE3ApproxInvLeftJacobian(manifold_difference);
    } else {
        J_logm = Transformation<>::SE3LeftJacobian(manifold_difference).inverse();
    }


    Mat6 J_comp_inv;
    auto R1R2t = this->storage.template block<3, 3>(0, 0) * T.storage.template block<3, 3>(0, 0).transpose();
    auto t1 = this->storage.template block<3, 1>(0, 3);

    J_comp_inv.template block<3, 3>(0, 0) = -R1R2t;
    J_comp_inv.template block<3, 3>(3, 3) = -R1R2t;
    J_comp_inv.template block<3, 3>(3, 0) = -skewSymmetric3(t1) * R1R2t -
                                            this->storage.template block<3, 3>(0, 0) *
                                              skewSymmetric3(T2inv.storage.template block<3, 1>(0, 3)) *
                                              T.storage.template block<3, 3>(0, 0).transpose();
    J_comp_inv.template block<3, 3>(0, 3).setZero();

    if (J_left) {
        *J_left = J_logm;
    }
    if (J_right) {
        *J_right = J_logm * J_comp_inv;
    }

    return manifold_difference;
}

template <typename T_str, bool approximate>
template <typename Other, typename OutT>
void Transformation<T_str, approximate>::compose(const Other &T_Right, OutT &T_out) const {
    T_out.storage.template block<3, 3>(0, 0).noalias() =
      this->storage.template block<3, 3>(0, 0) * T_Right.storage.template block<3, 3>(0, 0);
    T_out.storage.template block<3, 1>(0, 3).noalias() =
      this->storage.template block<3, 3>(0, 0) * T_Right.storage.template block<3, 1>(0, 3) +
      this->storage.template block<3, 1>(0, 3);
}

template <typename T_str, bool approximate>
template <typename Other, bool approx>
Transformation<T_str, approximate> Transformation<T_str, approximate>::composeAndJacobian(
  const Transformation<Other, approx> &T_right, Mat6 &J_left, Mat6 &J_right) const {
    J_left.setIdentity();
    J_right.setZero();

    J_right.template block<3, 3>(0, 0) = this->storage.template block<3, 3>(0, 0);
    J_right.template block<3, 3>(3, 3) = this->storage.template block<3, 3>(0, 0);
    J_right.template block<3, 3>(3, 0) =
      skewSymmetric3(this->storage.template block<3, 1>(0, 3)) * this->storage.template block<3, 3>(0, 0);

    return (*this) * T_right;
}

template <typename Derived, bool approximate>
Transformation<Derived, approximate> &Transformation<Derived, approximate>::normalize() {
    // Check if R has strayed too far outside SO(3)
    // and if so normalize
    Mat3 R = this->storage.template block<3, 3>(0, 0);
    Mat3 temp = R * R.transpose();
//    if (std::abs(R.determinant() - 1) > tolerance ||
//            ((temp - Mat3::Identity()).norm() > tolerance)) {
        temp = temp.sqrt().inverse();
        this->storage.template block<3, 3>(0, 0) = temp * R;
//    }

    return *this;
}

template <typename Derived, bool approximate>
Transformation<Derived, approximate> Transformation<Derived, approximate>::inverseAndJacobian(
  Mat6 &J_transformation) const {
    Transformation<Mat34, approximate> retval;
    this->transformInverse(retval);
    auto R = retval.getRotationMatrix();

    J_transformation.setZero();
    J_transformation.template block<3, 3>(0, 0) = -R;
    J_transformation.template block<3, 3>(3, 3) = -R;
    skewSymmetric3(retval.getTranslation(), J_transformation.template block<3, 3>(3, 0));
    J_transformation.template block<3, 3>(3, 0) = -J_transformation.template block<3, 3>(3, 0) * R;
    //    J_transformation.template block<3, 3>(3, 0) = -skewSymmetric3(retval.getTranslation()) * R;

    return retval;
}

template <typename Derived, bool approximate>
Mat3 Transformation<Derived, approximate>::getRotationMatrix() const {
    return this->storage.template block<3, 3>(0, 0);
}

template <typename Derived, bool approximate>
Vec3 Transformation<Derived, approximate>::getTranslation() const {
    return this->storage.template block<3, 1>(0, 3);
}

template <typename Derived, bool approximate>
Mat4 Transformation<Derived, approximate>::getMatrix() const {
    Mat4 retval = Mat4::Identity();
    retval.template block<3, 4>(0, 0) = this->storage;
    return retval;
}

template <typename Derived, bool approximate>
template <typename Other, bool OtherApprox>
Transformation<Mat34, approximate> Transformation<Derived, approximate>::operator*(
  const Transformation<Other, OtherApprox> &T) const {
    Transformation<Mat34, approximate> composed;
    composed.storage.template block<3, 3>(0, 0).noalias() =
      this->storage.template block<3, 3>(0, 0) * T.storage.template block<3, 3>(0, 0);
    composed.storage.template block<3, 1>(0, 3).noalias() =
      this->storage.template block<3, 3>(0, 0) * T.storage.template block<3, 1>(0, 3) +
      this->storage.template block<3, 1>(0, 3);

    return composed;
}

template <typename Derived, bool approximate>
Vec6 Transformation<Derived, approximate>::operator-(const Transformation &T) const {
    return this->manifoldMinus(T);
}
}


#endif  // WAVE_TRANSFORMATION_HPP