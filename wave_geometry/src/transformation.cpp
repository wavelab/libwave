#include "wave/geometry/exception_helpers.hpp"
#include "wave/geometry/transformation.hpp"

namespace wave {

Transformation::Transformation() {
    this->matrix.block(0, 0, 3, 3).setIdentity();
    this->matrix.block(0, 3, 3, 1).setZero();
}

Transformation &Transformation::setIdentity() {
    this->matrix.block(0, 0, 3, 3).setIdentity();
    this->matrix.block(0, 3, 3, 1).setZero();
    return *this;
}

Transformation::Transformation(const Vec3 &eulers, const Vec3 &translation) {
    this->setFromEulerXYZ(eulers, translation);
}

Transformation &Transformation::setFromEulerXYZ(const Vec3 &eulers, const Vec3 &translation) {
    checkMatrixFinite(eulers);
    checkMatrixFinite(translation);

    this->matrix.setIdentity();

    Mat3 rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(eulers[2], Vec3::UnitZ()) * Eigen::AngleAxisd(eulers[1], Vec3::UnitY()) *
                      Eigen::AngleAxisd(eulers[0], Vec3::UnitX());

    this->matrix.block(0, 0, 3, 3) = rotation_matrix;
    this->matrix.block(0, 3, 3, 1) = translation;

    return *this;
}

Transformation &Transformation::setFromMatrix(const Mat4 &input_matrix) {
    checkMatrixFinite(input_matrix);

    this->matrix = input_matrix.block(0, 0, 3, 4);

    return *this;
}

Mat3 Transformation::skewSymmetric3(const Vec3 &V) {
    Mat3 retval;
    retval << 0, -V(2), V(1), V(2), 0, -V(0), -V(1), V(0), 0;
    return retval;
}

Mat6 Transformation::Jinterpolated(const Vec6 &twist, const double &alpha) {
    // 3rd order approximation

    double A, B, C;
    A = (alpha*(alpha-1.0))*0.5;
    B = (alpha*(alpha-1.0)*(2.0*alpha-1.0))*0.0833333333333333333;
    C = (alpha*alpha*(alpha-1)*(alpha-1))*0.0416666666666666667;

    //todo(ben) make an adjoint function
    Mat6 adjoint = Mat6::Zero();
    adjoint.block(0,0,3,3) = skewSymmetric3(Vec3(twist(0), twist(1), twist(2)));
    adjoint.block(3,3,3,3) = skewSymmetric3(Vec3(twist(0), twist(1), twist(2)));
    adjoint.block(3,0,3,3) = skewSymmetric3(Vec3(twist(3), twist(4), twist(5)));

    return alpha * Mat6::Identity() + A * adjoint + B * adjoint * adjoint + C * adjoint * adjoint * adjoint;
}

Transformation &Transformation::setFromExpMap(const Vec6 &se3_vector) {
    checkMatrixFinite(se3_vector);

    Mat4 transform = this->expMap(se3_vector, this->TOL);

    this->matrix = transform.block(0, 0, 3, 4);

    return *this;
}

Mat4 Transformation::expMap(const Vec6 &W, double TOL) {
    Mat3 wx = skewSymmetric3(W.block(0, 0, 3, 1));
    double wn = W.block(0, 0, 3, 1).norm();

    double A, B, C;
    if (wn > TOL) {
        A = std::sin(wn) / wn;
        B = (1 - std::cos(wn)) / (wn * wn);
        C = (1 - A) / (wn * wn);
    } else {
        // Use taylor expansion
        A = 1 - (wn * wn) / 6 + (wn * wn * wn * wn) / 120;
        B = 0.5 - (wn * wn) / 24 + (wn * wn * wn * wn) / 720;
        C = 1 / 6 - (wn * wn / 120) + (wn * wn * wn * wn) / 5040;
    }
    Mat3 V;
    V = Mat3::Identity() + B * wx + C * wx * wx;

    Mat4 retval;
    retval.setIdentity();

    retval.block(0, 0, 3, 3).noalias() = Mat3::Identity() + A * wx + B * wx * wx;
    retval.block(0, 3, 3, 1).noalias() = V * W.block(3, 0, 3, 1);

    return retval;
}

Mat6 Transformation::SE3LeftJacobian(const Vec6 &W, double TOL) {
    Mat3 wx = Transformation::skewSymmetric3(W.block(0, 0, 3, 1));
    double wn = W.block(0, 0, 3, 1).norm();

    Mat6 retval, adj;
    retval.setZero();
    adj.setZero();

    // This is applying the adjoint operator to the se(3) vector: se(3) -> adj(se(3))
    adj.block(0, 0, 3, 3) = wx;
    adj.block(3, 3, 3, 3) = wx;
    adj.block(3, 0, 3, 3) = Transformation::skewSymmetric3(W.block(3, 0, 3, 1));

    double A, B, C, D;
    if (wn > TOL) {
        A = ((4 - wn * std::sin(wn) - 4 * cos(wn)) / (2 * wn * wn));
        B = (((4 * wn - 5 * std::sin(wn) + wn * std::cos(wn))) / (2 * wn * wn * wn));
        C = ((2 - wn * std::sin(wn) - 2 * std::cos(wn)) / (2 * wn * wn * wn * wn));
        D = ((2 * wn - 3 * std::sin(wn) + wn * std::cos(wn)) / (2 * wn * wn * wn * wn * wn));

        retval.noalias() = Mat6::Identity() + A * adj + B * adj * adj + C * adj * adj * adj + D * adj * adj * adj * adj;
    } else {
        // First order taylor expansion
        retval.noalias() = Mat6::Identity() + 0.5 * adj;
    }
    return retval;
}

Mat6 Transformation::SE3ApproxLeftJacobian(const Vec6 &W) {
    Mat3 wx = Transformation::skewSymmetric3(W.block(0, 0, 3, 1));

    Mat6 retval, adj;
    retval.setZero();
    adj.setZero();

    // This is applying the adjoint operator to the se(3) vector: se(3) -> adj(se(3))
    adj.block(0, 0, 3, 3) = wx;
    adj.block(3, 3, 3, 3) = wx;
    adj.block(3, 0, 3, 3) = skewSymmetric3(W.block(3, 0, 3, 1));

    double A, B;
    // Fourth order terms are shown should you ever want to used them.
    A = 1 / 2;  // - wn*wn*wn*wn/720;
    B = 1 / 6;  // - wn*wn*wn*wn/5040;
    // C = 1/24 - wn*wn/360 + wn*wn*wn*wn/13440;
    // D = 1/120 -wn*wn/2520 + wn*wn*wn*wn/120960;

    retval.noalias() = Mat6::Identity() + A * adj + B * adj * adj;  // + C*adj*adj*adj + D*adj*adj*adj*adj;

    return retval;
}

Vec6 Transformation::logMap() const {
    Mat3 R = this->matrix.block(0, 0, 3, 3);
    double wn;
    // Need to pander to ceres gradient checker a bit here
    if (R.trace() > 1) {
        wn = 0;
    } else {
        wn = std::acos((R.trace() - 1.0) / 2.0);
    }
    double A, B;
    Mat3 skew, Vinv;
    if (wn > this->TOL) {
        A = wn / (2 * std::sin(wn));
        B = (1 - std::cos(wn)) / (wn * wn);
        skew = A * (R - R.transpose());
        Vinv = Mat3::Identity() - 0.5 * skew + (1 / (wn * wn)) * (1 - (1 / (4 * A * B))) * skew * skew;
    } else {
        // Third order taylor expansion
        A = 0.5 + wn * wn / 12;  // + wn*wn*wn*wn*(7.0/720.0)
        B = 0.5 - wn * wn / 24;  // + wn*wn*wn*wn*(7.0/720.0)
        skew = A * (R - R.transpose());
        Vinv = Mat3::Identity() - 0.5 * skew;
    }
    Vec6 retval;
    retval(0) = skew(2, 1);
    retval(1) = skew(0, 2);
    retval(2) = skew(1, 0);

    retval.block(3, 0, 3, 1) = Vinv * this->matrix.block(0, 3, 3, 1);

    return retval;
}

Vec6 Transformation::logMap(const Transformation &T) {
    return T.logMap();
}

Vec3 Transformation::transform(const Vec3 &input_vector) const {
    return this->matrix.block(0, 0, 3, 3) * input_vector + this->matrix.block(0, 3, 3, 1);
}

Vec3 Transformation::transformAndJacobian(const Vec3 &input_vector,
                                          Mat3 &Jpoint,
                                          Eigen::Matrix<double, 3, 6> &Jparam) const {
    Vec3 retval = this->transform(input_vector);

    Jpoint = this->matrix.block(0, 0, 3, 3);

    Eigen::Matrix<double, 3, 6> Tpdonut;
    Tpdonut.setZero();
    Tpdonut.block(0, 0, 3, 3) = -skewSymmetric3(retval);
    Tpdonut.block(0, 3, 3, 3) = Mat3::Identity();

    Jparam.noalias() = Tpdonut;

    return retval;
}

Vec3 Transformation::inverseTransform(const Vec3 &input_vector) const {
    return this->matrix.block(0, 0, 3, 3).transpose() * input_vector -
           this->matrix.block(0, 0, 3, 3).transpose() * this->matrix.block(0, 3, 3, 1);
}

Transformation &Transformation::invert() {
    this->matrix.block(0, 0, 3, 3).transposeInPlace();
    this->matrix.block(0, 3, 3, 1) = -this->matrix.block(0, 0, 3, 3) * this->matrix.block(0, 3, 3, 1);

    return *this;
}

Transformation Transformation::inverse() const {
    Transformation inverse;
    Mat4 t_matrix = Mat4::Identity();
    t_matrix.block(0, 0, 3, 4) = this->matrix;
    inverse.setFromMatrix(t_matrix);
    inverse.invert();
    return inverse;
}

bool Transformation::isNear(const Transformation &other, double comparison_threshold) const {
    Vec6 diff = this->manifoldMinus(other);
    if (diff.norm() > comparison_threshold) {
        return false;
    } else {
        return true;
    }
}

Transformation &Transformation::manifoldPlus(const Vec6 &omega) {
    Mat4 incremental = expMap(omega, this->TOL);

    this->matrix.block(0, 3, 3, 1) =
      incremental.block(0, 0, 3, 3) * this->matrix.block(0, 3, 3, 1) + incremental.block(0, 3, 3, 1);
    this->matrix.block(0, 0, 3, 3) = incremental.block(0, 0, 3, 3) * this->matrix.block(0, 0, 3, 3);

    return *this;
}

Vec6 Transformation::manifoldMinus(const Transformation &T) const {
    Transformation delta = (*this) * T.inverse();
    return delta.logMap();
}

Vec6 Transformation::manifoldMinusAndJacobian(const Transformation &T, Mat6 &J_left, Mat6 &J_right) const {
    // logmap(T1 * inv(T2))
    Mat6 J_inv, J_l_compose, J_r_compose, J_logm;
    Transformation T2inv = T.inverseAndJacobian(J_inv);
    Transformation diff = this->composeAndJacobian(T2inv, J_l_compose, J_r_compose);
    auto manifold_difference = diff.logMap();

    J_logm = SE3LeftJacobian(manifold_difference, this->TOL).inverse();

    J_left = J_logm * J_l_compose;
    J_right = J_logm * J_r_compose * J_inv;

    return this->manifoldMinus(T);
}

Transformation Transformation::composeAndJacobian(const Transformation &T_right, Mat6 &J_left, Mat6 &J_right) const {
    J_left.setIdentity();
    J_right.setZero();

    J_right.block(0, 0, 3, 3) = this->matrix.block(0, 0, 3, 3);
    J_right.block(3, 3, 3, 3) = this->matrix.block(0, 0, 3, 3);
    J_right.block(3, 0, 3, 3) = skewSymmetric3(this->matrix.block(0, 3, 3, 1)) * this->matrix.block(0, 0, 3, 3);

    return (*this) * T_right;
}

Transformation Transformation::inverseAndJacobian(Mat6 &J_transformation) const {
    Transformation retval = this->inverse();
    auto R = retval.getRotationMatrix();

    J_transformation.setZero();
    J_transformation.block(0, 0, 3, 3) = -R;
    J_transformation.block(3, 3, 3, 3) = -R;
    J_transformation.block(3, 0, 3, 3) = -skewSymmetric3(retval.getTranslation()) * R;

    return retval;
}

Mat3 Transformation::getRotationMatrix() const {
    return this->matrix.block(0, 0, 3, 3);
}

Vec3 Transformation::getTranslation() const {
    return this->matrix.block(0, 3, 3, 1);
}

Mat4 Transformation::getMatrix() const {
    Mat4 retval = Mat4::Identity();
    retval.block(0, 0, 3, 4) = this->matrix;
    return retval;
}

Transformation Transformation::operator*(const Transformation &T) const {
    Transformation composed;
    composed.matrix.block(0, 0, 3, 3).noalias() = this->matrix.block(0, 0, 3, 3) * T.getRotationMatrix();
    composed.matrix.block(0, 3, 3, 1).noalias() =
      this->matrix.block(0, 0, 3, 3) * T.getTranslation() + this->matrix.block(0, 3, 3, 1);

    return composed;
}

Vec6 Transformation::operator-(const Transformation &T) const {
    return this->manifoldMinus(T);
}
}