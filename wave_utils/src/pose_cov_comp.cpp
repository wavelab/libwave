/* Copyright (c) 2017, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: pose_cov_comp.cpp
 * Desc: File containing the impl. for pose composition with uncertainty
 * Auth: Chunshang Li and Jordan Hu
 *
 * ############################################################################
 */

#include "wave/utils/pose_cov_comp.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

namespace wave {

PoseWithCovariance::PoseWithCovariance() {
    this->position.setZero();
    this->rotation_matrix.setIdentity();
    this->covariance.setZero();
}

PoseWithCovariance::PoseWithCovariance(Vector6 &p, Matrix6x6 &cov) {
    Vector3 ypr;

    ypr << p[5], p[4], p[3];

    this->position = p.block<3, 1>(0, 0);
    this->rotation_matrix = yprToRotMatrix(ypr);
    this->covariance = cov;
}

PoseWithCovariance::PoseWithCovariance(Vector3 &p,
                                       Matrix3x3 &r,
                                       Matrix6x6 &cov) {
    this->position = p;
    this->rotation_matrix = r;
    this->covariance = cov;
}


Vector3 PoseWithCovariance::getPosition() const {
    return this->position;
}

Vector3 PoseWithCovariance::getYPR() const {
    Vector3 ypr = rotMatrixToYPR(this->rotation_matrix);
    return ypr;
}

Vector4 PoseWithCovariance::getQuaternion() const {
    Vector4 q = rotMatrixToQuat(this->rotation_matrix);

    q = normalizeQuat(q);

    return q;
}

Vector7 PoseWithCovariance::getPoseQuaternion() const {
    Vector7 pose;
    Vector4 q = this->getQuaternion();

    pose << this->position, q;

    return pose;
}

Eigen::Affine3d PoseWithCovariance::getTransformMatrix() const {
    Eigen::Affine3d T_m;

    T_m.translation() = this->position;
    T_m.linear() = this->rotation_matrix;

    return T_m;
}

PoseWithCovariance composePose(PoseWithCovariance &p1, PoseWithCovariance &p2) {
    PoseWithCovariance r;  // store all the results

    // Use Eigen Transform to transform poses
    // This is the same as implementing Equation (5.5)
    Eigen::Affine3d T_p1, T_p2, T_r;
    T_p1 = p1.getTransformMatrix();
    T_p2 = p2.getTransformMatrix();
    T_r = T_p1 * T_p2;

    r.position = T_r.translation();
    r.rotation_matrix = T_r.rotation();

    // compute the covariances
    // p6 = [x y z roll pitch yaw]', transformation using YPR
    // p7 = [x y z qr, qx, qy, qz]', transformation using Quaternion, qr is qw

    // get covariances from PoseWithCovariance objects
    Matrix6x6 cov_p1 = p1.covariance, cov_p2 = p2.covariance;

    // pR7 is the composed pose in p7 form
    // p17 is the p1 in p7 form, p27 is p2 in p7 form, p16 is p1 in p6 form,
    // p26 is p2 in p6 form
    Vector7 pR7, p17, p27;
    Vector6 p16, p26;

    pR7 = r.getPoseQuaternion();
    p17 = p1.getPoseQuaternion();
    p27 = p2.getPoseQuaternion();

    p16 << p1.getPosition(), quatToYPR(p17.block<4, 1>(3, 0));
    p26 << p2.getPosition(), quatToYPR(p27.block<4, 1>(3, 0));

    // Equation (5.3)
    Matrix6x7 jacobian_p7_to_p6 = jacobian_p7_to_p6_wrt_p(pR7);
    Matrix7x7 jacobian_p7_p7_composition =
      jacobian_p7_p7_Composition_wrt_p1(p17, p27);
    Matrix7x6 jacobian_p6_to_p7 = jacobian_p6_to_p7_wrt_p(p16);
    Matrix6x6 dfpc_dp =
      jacobian_p7_to_p6 * jacobian_p7_p7_composition * jacobian_p6_to_p7;

    // Equation (5.4)
    jacobian_p7_p7_composition = jacobian_p7_p7_Composition_wrt_p2(p17, p27);
    jacobian_p6_to_p7 = jacobian_p6_to_p7_wrt_p(p26);
    Matrix6x6 dfpc_dq =
      jacobian_p7_to_p6 * jacobian_p7_p7_composition * jacobian_p6_to_p7;

    // Equation (5.2)
    // putting everything together
    Matrix6x6 final_cov = dfpc_dp * cov_p1 * dfpc_dp.transpose() +
                          dfpc_dq * cov_p2 * dfpc_dq.transpose();

    r.covariance = final_cov;

    return r;
}

Vector4 normalizeQuat(const Vector4 q) {
    Vector4 norm_q;
    double norm_coeff;
    double qr = q(0), qx = q(1), qy = q(2), qz = q(3);

    norm_coeff = 1 / sqrt(qr * qr + qx * qx + qy * qy + qz * qz);
    norm_q = norm_coeff * q;
    return norm_q;
}

Vector3 quatToYPR(const Vector4 &q) {
    Vector3 v;
    double qr = q(0), qx = q(1), qy = q(2), qz = q(3);
    double delta = qr * qy - qx * qz;  // discriminant of normalized quaternion
    double roll, pitch, yaw;

    // Check for special cases when abs(delta) =~ 0.5 (Equation (2.10))
    if (fabs(delta - 0.5) < 1e-6) {
        yaw = -2 * atan2(qx, qr);
        pitch = M_PI / 2;
        roll = 0;

    } else if (fabs(delta + 0.5) < 1e-6) {
        yaw = 2 * atan2(qx, qr);
        pitch = -M_PI / 2;
        roll = 0;
    } else {
        yaw = atan2(2 * (qr * qz + qx * qy), (1 - 2 * (qy * qy + qz * qz)));
        pitch = asin(2 * delta);
        roll = atan2(2 * (qr * qx + qy * qz), (1 - 2 * (qx * qx + qy * qy)));
    }

    v(0) = yaw;
    v(1) = pitch;
    v(2) = roll;

    return v;
}

Vector4 yprToQuat(const Vector3 &ypr) {
    Vector4 q;
    double qr, qx, qy, qz;
    double half_y = ypr(0) / 2, half_p = ypr(1) / 2, half_r = ypr(2) / 2;
    double cy = cos(half_y), cp = cos(half_p), cr = cos(half_r),
           sy = sin(half_y), sp = sin(half_p), sr = sin(half_r);

    qr = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;

    q << qr, qx, qy, qz;

    return q;
}

Matrix3x3 yprToRotMatrix(const Vector3 &ypr) {
    double y = ypr(0), p = ypr(1), r = ypr(2);
    Matrix3x3 rotMatrix;
    double cy = cos(y), cp = cos(p), cr = cos(r), sy = sin(y), sp = sin(p),
           sr = sin(r);

    rotMatrix << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr,
      cp * cr;

    return rotMatrix;
}

Vector3 rotMatrixToYPR(const Matrix3x3 &p) {
    Vector3 ypr;
    double yaw, pitch, roll;
    double p11 = p(0, 0), p12 = p(0, 1), p13 = p(0, 2), p21 = p(1, 0),
           p22 = p(1, 1), p23 = p(1, 2), p31 = p(2, 0), p32 = p(2, 1),
           p33 = p(2, 2);

    pitch = atan2(-p31, sqrt(p11 * p11 + p21 * p21));

    if (fabs(pitch + M_PI * 1.0 / 2) < 1e-6) {
        yaw = atan2(-p23, -p13);
        roll = 0;
    } else if (fabs(pitch - M_PI * 1.0 / 2) < 1e-6) {
        yaw = atan2(p23, p13);
        roll = 0;
    } else {
        yaw = atan2(p21, p11);
        roll = atan2(p32, p33);
    }

    ypr << yaw, pitch, roll;

    // To avoid warnings
    (void) p12;
    (void) p22;

    return ypr;
}

Vector4 rotMatrixToQuat(const Matrix3x3 &p) {
    Vector4 q;
    Vector3 ypr;

    ypr = rotMatrixToYPR(p);
    q = yprToQuat(ypr);

    return q;
}

/// the jacobian of quaternion normalization function
/// quat in the form of [qr, qx, qt, qz]
/// Equation (1.7)
Matrix4x4 jacobian_Quat_Norm_wrt_q(const Vector4 &q) {
    const double &qr = q(0), &qx = q(1), &qy = q(2), &qz = q(3);
    Matrix4x4 m;

    // See Equation (1.7)
    double k = 1 / pow(qr * qr + qx * qx + qy * qy + qz * qz, 1.5);

    // clang-format off
    m << qx * qx + qy * qy + qz * qz, -qr * qx, -qr * qy, -qr * qz,
        -qx * qr, qr * qr + qy * qy + qz * qz, -qx * qy, -qx * qz,
        -qy * qr, -qy * qx, qr * qr + qx * qx + qz * qz, -qy * qz,
        -qz * qr, -qz * qx, -qz * qy, qr * qr + qx * qx + qy * qy;
    // clang-format on

    m = m * k;

    return m;
}

// the jacobian of normalized quaternion to rpy function
// Equation (2.9) to Equation (2.10)
Matrix3x4 jacobian_Quat_Norm_to_Rpy_wrt_q(const Vector4 &q) {
    const double &qr = q(0), &qx = q(1), &qy = q(2), &qz = q(3);

    Matrix3x4 m;

    // Equation (2.9)
    double delta = qr * qy - qx * qz;

    // handle special (rare) cases for when |delta| = 0.5
    if (fabs(delta - 0.5) < 1e-10) {  // delta = 0.5
                                      // clang-format off
        m << (2*qx)/(qr*qr + qx*qx), -(2*qr)/(qr*qr + qx*qx), 0, 0,
                                    0,                     0, 0, 0,
                                    0,                     0, 0, 0;

        // clang-format on

    } else if (fabs(delta + 0.5) < 1e-10) {  // delta = -0.5
                                             // clang-format off
        m << -(2*qx)/(qr*qr + qx*qx), (2*qr)/(qr*qr + qx*qx), 0, 0,
                                     0,                    0, 0, 0,
                                     0,                    0, 0, 0;
                                             // clang-format on
    } else {
        // Equation (2.10)
        // Jacobian obtained symbolically from SymPy by taking the derivative
        // of this below
        //     roll  = [[        2*(qr*qz + qx*qy)/(-2*qy*qy - 2*qz*qz + 1)],
        //     pitch =  [                           asin(2*qr*qy - 2*qx*qz)],
        //     yaw   =  [atan((2*qr*qx + 2*qy*qz)/(-2*qx*qx - 2*qy*qy + 1))]]
        // with respect to qr, qx, qy, qz using SymPy

        // just for convenience
        auto sq = [](double x) { return x * x; };

        // clang-format off
        m << 
            2*qz*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)),
            2*qy*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)),
            2*qx*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)) - 4*qy*(-2*qr*qz - 2*qx*qy)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)),
            2*qr*(-2*qy*qy - 2*qz*qz + 1)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)) - 4*qz*(-2*qr*qz - 2*qx*qy)/(sq(2*qr*qz + 2*qx*qy) + sq(-2*qy*qy - 2*qz*qz + 1)),

            2*qy/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1),
            -2*qz/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1),
            2*qr/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1),
            -2*qx/sqrt(-sq(2*qr*qy - 2*qx*qz) + 1),

            2*qx*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)),
            2*qr*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)) - 4*qx*(-2*qr*qx - 2*qy*qz)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)),
            -4*qy*(-2*qr*qx - 2*qy*qz)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)) + 2*qz*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1)),
            2*qy*(-2*qx*qx - 2*qy*qy + 1)/(sq(2*qr*qx + 2*qy*qz) + sq(-2*qx*qx - 2*qy*qy + 1));

        // clang-format on
    }

    return m;
}

/// the jacobian of p7 to p6 conversion
/// Equation (2.12)
Matrix6x7 jacobian_p7_to_p6_wrt_p(const Vector7 &p) {
    Matrix6x7 r = Matrix6x7::Zero();

    // Equation (2.12)
    // bottom left 3x3 is zero
    // top right 3x4 is zero
    // top left 3x3 is identity
    r(0, 0) = 1;
    r(1, 1) = 1;
    r(2, 2) = 1;

    // This function takes the un-normalized quaternion
    Matrix4x4 jacobian_quat_norm =
      jacobian_Quat_Norm_wrt_q(p.block<4, 1>(3, 0));

    Matrix3x4 jacobian_quat_norm_to_rpy =
      jacobian_Quat_Norm_to_Rpy_wrt_q(p.block<4, 1>(3, 0));

    r.block<3, 4>(3, 3) = jacobian_quat_norm_to_rpy * jacobian_quat_norm;

    return r;
}

// jacobian of composing a point to a p7
// Equation (3.8)
Matrix3x7 jacobian_p7_Point_Composition_wrt_p(const Vector7 &p,
                                              const Vector3 &a) {
    Matrix3x7 m = Matrix3x7::Zero();

    // The 3x3 on the top left is the identity matrix
    m.block<3, 3>(0, 0) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    // Equation (3.9)
    double ax = a(0), ay = a(1), az = a(2);
    double qr = p(3), qx = p(4), qy = p(5), qz = p(6);

    // clang-format off
    m.block<3, 4>(0, 3) <<
        -qz*ay+qy*az, qy*ay+qz*az, -2*qy*ax+qx*ay+qr*az, -2*qz*ax-qr*ay+qx*az,
        qz*ax-qx*az, qy*ax-2*qx*ay-qr*az, qx*ax+qz*az, qr*ax-2*qz*ay+qy*az,
        -qy*ax+qx*ay, qz*ax+qr*ay-2*qx*az, -qr*ax+qz*ay-2*qy*az, qx*ax+qy*ay;
    // clang-format on

    m.block<3, 4>(0, 3) *= 2;

    // applying jacobian normalization
    Matrix4x4 jacobian_quat_norm =
      jacobian_Quat_Norm_wrt_q(p.block<4, 1>(3, 0));
    m.block<3, 4>(0, 3) *= jacobian_quat_norm;

    return m;
}

// jacobian of the composition of p7 poses
// Equation (5.8)
Matrix7x7 jacobian_p7_p7_Composition_wrt_p1(const Vector7 &p1,
                                            const Vector7 &p2) {
    Matrix7x7 m = Matrix7x7::Zero();

    Vector4 q2 = p2.block<4, 1>(3, 0);
    double qr2 = q2(0), qx2 = q2(1), qy2 = q2(2), qz2 = q2(3);

    // clang-format off
    m.block<4, 4>(3, 3) << qr2, -qx2, -qy2, -qz2,
                           qx2, qr2, qz2, -qy2,
                           qy2, -qz2, qr2, qx2,
                           qz2, qy2, -qx2, qr2;
    // clang-format on

    // Note: this quaternion normalization jacobian matrix is not present in
    // the book's formulation, but it should be there if a jacobian
    // normalization is performed on the input
    Matrix4x4 jacobian_quat_norm =
      jacobian_Quat_Norm_wrt_q(p1.block<4, 1>(3, 0));
    m.block<4, 4>(3, 3) *= jacobian_quat_norm;

    m.block<3, 7>(0, 0) =
      jacobian_p7_Point_Composition_wrt_p(p1, p2.block<3, 1>(0, 0));

    return m;
}

// jacobian of composing a point to a p7
// Equation (3.10)
Matrix3x3 jacobian_p7_Point_Composition_wrt_a(const Vector7 &p,
                                              const Vector3 &a) {
    Matrix3x3 m = Matrix3x3::Zero();

    double qr = p(3), qx = p(4), qy = p(5), qz = p(6);

    // clang-format off
    m << 0.5 - qy*qy - qz*qz , qx*qy - qr*qz, qr*qy + qx*qz,
         qr*qz + qx*qy, 0.5 - qx*qx - qz*qz, qy*qz - qr*qx,
         qx*qz - qr*qy, qr*qx + qy*qz, 0.5 - qx*qx - qy*qy;
    // clang-format on

    m *= 2;

    // To avoid warnings
    (void) a;

    return m;
}

// jacobian of the composition of p7 poses
// Equation (5.9)
Matrix7x7 jacobian_p7_p7_Composition_wrt_p2(const Vector7 &p1,
                                            const Vector7 &p2) {
    Matrix7x7 m = Matrix7x7::Zero();

    Vector4 q1 = p1.block<4, 1>(3, 0);
    double qr1 = q1(0), qx1 = q1(1), qy1 = q1(2), qz1 = q1(3);

    // clang-format off
    m.block<4, 4>(3, 3) << qr1, -qx1, -qy1, -qz1,
                           qx1, qr1, -qz1, qy1,
                           qy1, qz1, qr1, -qx1,
                           qz1, -qy1, qx1, qr1;
    // clang-format on

    // Note: this quaternion normalization jacobian matrix is not present in
    // the book's formulation, but it should be there if a jacobian
    // normalization is performed on the input
    Matrix4x4 jacobian_quat_norm =
      jacobian_Quat_Norm_wrt_q(p2.block<4, 1>(3, 0));
    m.block<4, 4>(3, 3) *= jacobian_quat_norm;

    m.block<3, 3>(0, 0) =
      jacobian_p7_Point_Composition_wrt_a(p1, p2.block<3, 1>(0, 0));

    return m;
}

// jacobian of converting a p6 to a p7
// Equation (2.8)
Matrix7x6 jacobian_p6_to_p7_wrt_p(const Vector6 &p) {
    Matrix7x6 m = Matrix7x6::Zero();

    // top left corner is a identity matrix
    // the bottom left 4x3 and top right 3x3 is zero
    m.block<3, 3>(0, 0) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    double ccc, ccs, csc, scc, ssc, sss, scs, css;
    double roll = p(5), pitch = p(4), yaw = p(3);
    ccc = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2);
    ccs = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    csc = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    scs = sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
    css = cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
    scc = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2);
    ssc = sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
    sss = sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

    // clang-format off
    m.block<4, 3>(3, 3) <<
        (ssc - ccs) / 2.0, (scs - csc) / 2.0, (css - scc) / 2.0,
        -(csc + scs) / 2.0, -(ssc + ccs) / 2.0, (ccc + sss) / 2.0,
        (scc - css) / 2.0, (ccc - sss) / 2.0, (ccs - ssc) / 2.0,
        (ccc + sss) / 2.0, -(css + scc) / 2.0, -(csc + scs) / 2.0;
    // clang-format on

    return m;
}
}
