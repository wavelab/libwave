#ifndef WAVE_TRANSFORMATION_IMPL_HPP
#define WAVE_TRANSFORMATION_IMPL_HPP

#include "wave/geometry/exception_helpers.hpp"
#include "wave/geometry/transformation.hpp"

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
Mat3 Transformation<Derived, approximate>::skewSymmetric3(const Vec3 &V) {
    Mat3 retval;
    retval << 0, -V(2), V(1), V(2), 0, -V(0), -V(1), V(0), 0;
    return retval;
}

template <typename Derived, bool approximate>
Mat6 Transformation<Derived, approximate>::skewSymmetric6(const Vec6 &W) {
    Mat6 retval = Mat6::Zero();
    retval.template block<3, 3>(0, 0) = skewSymmetric3(W.block<3, 1>(0, 0));
    retval.block<3, 3>(3, 3) = skewSymmetric3(W.block<3, 1>(0, 0));
    retval.block<3, 3>(3, 0) = skewSymmetric3(W.block<3, 1>(3, 0));
    return retval;
}

template <typename Derived, bool approximate>
template <typename Other, bool O_approx>
void Transformation<Derived, approximate>::interpolate(const Transformation &T_k,
                                                       const Transformation &T_kp1,
                                                       const Vec6 &twist_k,
                                                       const Vec6 &twist_kp1,
                                                       const Eigen::Matrix<double, 6, 12> &hat,
                                                       const Eigen::Matrix<double, 6, 12> &candle,
                                                       Transformation<Other, O_approx> &T_int) {
    Mat6 J_left, J_right;
    auto eps = T_kp1.manifoldMinusAndJacobian(T_k, J_left, J_right);

//    T_int.deepCopy(T_k);
    T_int = T_k;
    T_int.manifoldPlus(hat.block<6, 6>(0, 6) * twist_k + candle.block<6, 6>(0, 0) * eps +
                       candle.block<6, 6>(0, 6) * J_left * twist_kp1);
}

template <typename Derived, bool approximate>
template <typename Other, bool O_approx>
void Transformation<Derived, approximate>::interpolateAndJacobians(const Transformation &T_k,
                                                                   const Transformation &T_kp1,
                                                                   const Vec6 &twist_k,
                                                                   const Vec6 &twist_kp1,
                                                                   const Eigen::Matrix<double, 6, 12> &hat,
                                                                   const Eigen::Matrix<double, 6, 12> &candle,
                                                                   Transformation<Other, O_approx> &T_int,
                                                                   Mat6 &J_Tk,
                                                                   Mat6 &J_Tkp1,
                                                                   Mat6 &J_twist_k,
                                                                   Mat6 &J_twist_kp1) {
    Mat6 J_left, J_right;
    auto eps = T_kp1.manifoldMinusAndJacobian(T_k, J_left, J_right);

    T_int.setFromExpMap(hat.block<6, 6>(0, 6) * twist_k + candle.block<6, 6>(0, 0) * eps +
                        candle.block<6, 6>(0, 6) * J_left * twist_kp1);

    Mat6 J_comp_left, J_comp_right;

    T_int = T_int.composeAndJacobian(T_k, J_comp_left, J_comp_right);

    Mat6 Jexp;

    if (approximate) {
        Jexp = Transformation<Derived>::SE3ApproxLeftJacobian(hat.block<6, 6>(0, 6) * twist_k +
                                                              candle.block<6, 6>(0, 0) * eps +
                                                              candle.block<6, 6>(0, 6) * J_left * twist_kp1);
    } else {
        Jexp =
          Transformation<Derived>::SE3LeftJacobian(hat.block<6, 6>(0, 6) * twist_k + candle.block<6, 6>(0, 0) * eps +
                                                     candle.block<6, 6>(0, 6) * J_left * twist_kp1);
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
    retval.block<3, 3>(3, 3) = this->storage.template block<3, 3>(0, 0);
    retval.block<3, 3>(3, 0) =
      skewSymmetric3(this->storage.template block<3, 1>(0, 3)) * this->storage.template block<3, 3>(0, 0);
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
void Transformation<T_str, approximate>::expMap(const Eigen::MatrixBase<VType> &W,
                                                  Eigen::MatrixBase<MType> &retval) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<VType>, 6)
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<MType>, 3, 4)

    Mat3 wx = skewSymmetric3(W.template block<3, 1>(0, 0));
    double wn = W.template block<3, 1>(0, 0).norm();

    if (approximate) {
        // 2nd order taylor expansion
        retval.template block<3, 3>(0, 0).noalias() = Mat3::Identity() + (1.0 - 0.16666666666666667 * (wn * wn)) * wx +
                                                      (0.5 - 4.166666666666666667e-2 * (wn * wn)) * wx * wx;
        retval.template block<3, 1>(0, 3).noalias() =
          (Mat3::Identity() + (0.5 - 4.166666666666666667e-2 * (wn * wn)) * wx +
           (0.16666666666666667 - 8.33333333333333333e-3 * (wn * wn)) * wx * wx) *
          W.template block<3, 1>(3, 0);
    } else {
        double A, B, C;
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
        retval.template block<3, 3>(0, 0).noalias() = Mat3::Identity() + A * wx + B * wx * wx;
        retval.template block<3, 1>(0, 3).noalias() =
          (Mat3::Identity() + B * wx + C * wx * wx) * W.template block<3, 1>(3, 0);
    }
}

template <typename Derived, bool approximate>
Mat6 Transformation<Derived, approximate>::expMapAdjoint(const Vec6 &W) {
    double wn = W.template block<3, 1>(0, 0).norm();

    auto skew = skewSymmetric6(W);

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
    Mat3 wx = Transformation<Derived>::skewSymmetric3(W.template block<3, 1>(0, 0));
    double wn = W.template block<3, 1>(0, 0).norm();

    Mat6 retval, adj;
    retval.setZero();
    adj.setZero();

    // This is applying the adjoint operator to the se(3) vector: se(3) -> adj(se(3))
    adj.template block<3, 3>(0, 0) = wx;
    adj.template block<3, 3>(3, 3) = wx;
    adj.template block<3, 3>(3, 0) = Transformation<Derived>::skewSymmetric3(W.template block<3, 1>(3, 0));

    double A, B, C, D;
    if (wn > tol) {
        A = ((4.0 - wn * std::sin(wn) - 4.0 * cos(wn)) / (2.0 * wn * wn));
        B = (((4.0 * wn - 5.0 * std::sin(wn) + wn * std::cos(wn))) / (2.0 * wn * wn * wn));
        C = ((2.0 - wn * std::sin(wn) - 2.0 * std::cos(wn)) / (2.0 * wn * wn * wn * wn));
        D = ((2.0 * wn - 3.0 * std::sin(wn) + wn * std::cos(wn)) / (2.0 * wn * wn * wn * wn * wn));

        retval.noalias() = Mat6::Identity() + A * adj + B * adj * adj + C * adj * adj * adj + D * adj * adj * adj * adj;
    } else {
        // First order taylor expansion
        retval.noalias() = Mat6::Identity() + 0.5 * adj;
    }
    return retval;
}

template <typename Derived, bool approximate>
Mat6 Transformation<Derived, approximate>::SE3ApproxLeftJacobian(const Vec6 &W) {
    Mat3 wx = Transformation<Derived>::skewSymmetric3(W.template block<3, 1>(0, 0));

    Mat6 retval, adj;
    retval.setZero();
    adj.setZero();

    // This is applying the adjoint operator to the se(3) vector: se(3) -> adj(se(3))
    adj.template block<3, 3>(0, 0) = wx;
    adj.template block<3, 3>(3, 3) = wx;
    adj.template block<3, 3>(3, 0) = skewSymmetric3(W.template block<3, 1>(3, 0));

    // double A;
    // Fourth order terms are shown should you ever want to used them.
    // A = 0.5;  // - wn*wn*wn*wn/720;
    // B = 1 / 6;  // - wn*wn*wn*wn/5040;
    // C = 1/24 - wn*wn/360 + wn*wn*wn*wn/13440;
    // D = 1/120 -wn*wn/2520 + wn*wn*wn*wn/120960;

    retval.noalias() =
      Mat6::Identity() + 0.5 * adj + 0.1666666666666666666666 * adj * adj;  // + *adj*adj*adj; // + D*adj*adj*adj*adj;

    return retval;
}

template <typename Derived, bool approximate>
Mat6 Transformation<Derived, approximate>::SE3ApproxInvLeftJacobian(const Vec6 &W) {
    Mat3 wx = Transformation<Derived>::skewSymmetric3(W.template block<3, 1>(0, 0));

    Mat6 retval, adj;
    retval.setZero();
    adj.setZero();

    // This is applying the adjoint operator to the se(3) vector: se(3) -> adj(se(3))
    adj.template block<3, 3>(0, 0) = wx;
    adj.template block<3, 3>(3, 3) = wx;
    adj.template block<3, 3>(3, 0) = skewSymmetric3(W.template block<3, 1>(3, 0));

    retval.noalias() =
      Mat6::Identity() - 0.5 * adj + 0.0833333333333333333 * adj * adj;  // + C*adj*adj*adj + D*adj*adj*adj*adj;

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
    op_vec = this->storage.template block<3, 3>(0, 0) * ip_vec + this->storage.template block<3, 1>(0, 3);
}

template <typename Derived, bool approximate>
Vec3 Transformation<Derived, approximate>::transformAndJacobian(const Vec3 &input_vector,
                                                                Mat3 &Jpoint,
                                                                Eigen::Matrix<double, 3, 6> &Jparam) const {
    Vec3 retval;
    this->transform(input_vector, retval);

    Jpoint = this->storage.template block<3, 3>(0, 0);

    Eigen::Matrix<double, 3, 6> Tpdonut;
    Tpdonut.setZero();
    Tpdonut.template block<3, 3>(0, 0) = -skewSymmetric3(retval);
    Tpdonut.template block<3, 3>(0, 3) = Mat3::Identity();

    Jparam.noalias() = Tpdonut;

    return retval;
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
    this->storage.template block<3, 3>(0, 0).transposeInPlace();
    this->storage.template block<3, 1>(0, 3) =
      -this->storage.template block<3, 3>(0, 0) * this->storage.template block<3, 1>(0, 3);

    return *this;
}

template <typename Derived, bool approximate>
template <bool approx>
Transformation<Derived, approximate> Transformation<Derived, approximate>::transformInverse() const {
    Transformation<Derived, approximate> inverse;
    inverse.storage.derived() = this->storage.derived();
//    inverse = *this;
    inverse.invert();
    return inverse;
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

    Eigen::Matrix<double, 3, 4> incremental;
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
    return ((*this) * T.transformInverse()).logMap();
}

template <typename Derived, bool approximate>
Vec6 Transformation<Derived, approximate>::manifoldMinusAndJacobian(const Transformation &T,
                                                                    Mat6 &J_left,
                                                                    Mat6 &J_right) const {
    // logmap(T1 * inv(T2))
    Mat6 J_logm;
    Transformation<TransformStorage, approximate> T2inv;
//    T2inv.deepCopy(T.transformInverse());
    T2inv = T.transformInverse();
    Transformation<TransformStorage, approximate> diff;
    diff = (*this) * T2inv;
    auto manifold_difference = diff.logMap();

    // todo: Change to constexpr if when switch to C++17
    if (approximate) {
        J_logm = Transformation<>::SE3ApproxInvLeftJacobian(manifold_difference);
    } else {
        J_logm = Transformation<>::SE3LeftJacobian(manifold_difference).inverse();
    }


    Mat6 J_comp_inv;
    auto R1R2t = this->storage.template block<3, 3>(0, 0) * T.storage.template block<3, 3>(0, 0).transpose();
    auto t1 = this->storage.template block<3, 1>(0, 3);

    J_comp_inv.template block<3, 3>(0, 0) = -R1R2t;
    J_comp_inv.template block(3, 3, 3, 3) = -R1R2t;
    J_comp_inv.template block(3, 0, 3, 3) = -skewSymmetric3(t1) * R1R2t -
                                            this->storage.template block<3, 3>(0, 0) *
                                              skewSymmetric3(T2inv.storage.template block<3, 1>(0, 3)) *
                                              T.storage.template block<3, 3>(0, 0).transpose();
    J_comp_inv.template block(0, 3, 3, 3).setZero();

    J_left = J_logm;
    J_right = J_logm * J_comp_inv;

    return manifold_difference;
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
Transformation<Derived, approximate> &Transformation<Derived, approximate>::normalizeMaybe(double tolerance) {
    // Check if R has strayed too far outside SO(3)
    // and if so normalize
    if ((this->storage.template block<3, 3>(0, 0).determinant() - 1) > tolerance) {
        Mat3 R = this->storage.template block<3, 3>(0, 0);
        Mat3 temp = R * R.transpose();
        temp = temp.sqrt().inverse();
        this->storage.template block<3, 3>(0, 0) = temp * R;
    }

    return *this;
}

template <typename Derived, bool approximate>
Transformation<Derived, approximate> Transformation<Derived, approximate>::inverseAndJacobian(
  Mat6 &J_transformation) const {
    Transformation retval = this->transformInverse();
    auto R = retval.getRotationMatrix();

    J_transformation.setZero();
    J_transformation.template block<3, 3>(0, 0) = -R;
    J_transformation.template block<3, 3>(3, 3) = -R;
    J_transformation.template block<3, 3>(3, 0) = -skewSymmetric3(retval.getTranslation()) * R;

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
Transformation<Derived, approximate> Transformation<Derived, approximate>::operator*(
  const Transformation<Other, OtherApprox> &T) const {
    Transformation<TransformStorage, approximate> composed;
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

//template <typename Derived, bool approximate>
//template <typename Other, bool OtherApprox>
//Transformation<Derived, approximate> &Transformation<Derived, approximate>::deepCopy(
//  const Transformation<Other, OtherApprox> &T) {
//    *this = T;
//
//    return *this;
//}
}


#endif  // WAVE_TRANSFORMATION_HPP