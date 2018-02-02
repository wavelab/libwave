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
 * File: RefEkf_test.cpp
 * Desc: Contains the unit tests for the pose_cov_comp functions.
 * Auth: Chunshang Li and Jordan Hu
 *
 * ############################################################################
*/

#include "wave/utils/pose_cov_comp.hpp"
#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

using namespace wave;

bool CheckPose(PoseWithCovariance expected, PoseWithCovariance actual) {
    return (actual.position.isApprox(expected.position, 1e-10),
            actual.rotation_matrix.isApprox(expected.rotation_matrix, 1e-10));
}

/// Tests if the normal of the generated Quaternion is 1
TEST(PoseCovComp, normalizeQuat_test) {
    Vector4 q, norm_q;
    double mag_norm_q;

    q << 1, 0, 0, 0;
    norm_q = normalizeQuat(q);
    mag_norm_q = sqrt(norm_q(0) * norm_q(0) + norm_q(1) * norm_q(1) +
                      norm_q(2) * norm_q(2) + norm_q(3) * norm_q(3));
    ASSERT_TRUE(fabs(mag_norm_q - 1) < 1e-6);

    q << 1, 0, 1, 0;
    norm_q = normalizeQuat(q);
    mag_norm_q = sqrt(norm_q(0) * norm_q(0) + norm_q(1) * norm_q(1) +
                      norm_q(2) * norm_q(2) + norm_q(3) * norm_q(3));
    ASSERT_TRUE(fabs(mag_norm_q - 1) < 1e-6);

    q << 1, 3, 4, 3;
    norm_q = normalizeQuat(q);
    mag_norm_q = sqrt(norm_q(0) * norm_q(0) + norm_q(1) * norm_q(1) +
                      norm_q(2) * norm_q(2) + norm_q(3) * norm_q(3));
    ASSERT_TRUE(fabs(mag_norm_q - 1) < 1e-6);
}

/// Test against MATLAB output of quat2angle
TEST(PoseCovComp, QuatToRPY_test) {
    Vector4 q;
    Vector3 r;

    q << 1, 0, 0, 0;
    r = quatToYPR(q);
    ASSERT_TRUE(r.isApprox(Vector3({0, 0, 0})));

    q << 0.182574185835055, 0.365148371670111, 0.547722557505166,
      0.730296743340221;
    r = quatToYPR(q);
    ASSERT_TRUE(r.isApprox(
      Vector3({2.356194490192345, -0.339836909454122, 1.428899272190733})));

    q << 0.5, 0.5, 0.5, 0.5;
    r = quatToYPR(q);
    ASSERT_TRUE(r.isApprox(Vector3({1.570796326794897, 0, 1.570796326794897})));
}

/// Test against MATLAB output of eul2rotm
TEST(PoseCovComp, yprToRotMatrix_test) {
    Vector3 ypr;
    Matrix3x3 rot_matrix, rot_matrix_expected;

    ypr << 0, M_PI * 1.0 / 2, 0;
    rot_matrix = yprToRotMatrix(ypr);
    rot_matrix_expected << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    ASSERT_TRUE(rot_matrix.isApprox(rot_matrix_expected, 1e-4));

    ypr << 0.78544, 0.1, 0;
    rot_matrix = yprToRotMatrix(ypr);
    rot_matrix_expected << 0.7035, -0.7071, 0.0706, 0.7036, 0.7071, 0.0706,
      -0.0998, 0, 0.9950;
    ASSERT_TRUE(rot_matrix.isApprox(rot_matrix_expected, 1e-4));

    ypr << M_PI * 1.0 / 3, 1, -2;
    rot_matrix = yprToRotMatrix(ypr);
    rot_matrix_expected << 0.2702, -0.0222, -0.9626, 0.4679, -0.8707, 0.1514,
      -0.8415, -0.4913, -0.2248;
    ASSERT_TRUE(rot_matrix.isApprox(rot_matrix_expected, 1e-4));
}

/// Test against MATLAB output of rotm2eul
TEST(PoseCovComp, rotMatrixToYPR_test) {
    Matrix3x3 rot_matrix;
    Vector3 ypr, ypr_expected;

    rot_matrix << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    ypr = rotMatrixToYPR(rot_matrix);
    ypr_expected << 0, M_PI * 1.0 / 2, 0;
    ASSERT_TRUE(ypr.isApprox(ypr_expected, 1e-4));

    rot_matrix << 0.7035, -0.7071, 0.0706, 0.7036, 0.7071, 0.0706, -0.0998, 0,
      0.9950;
    ypr = rotMatrixToYPR(rot_matrix);
    ypr_expected << 0.78544, 0.1, 0;
    ASSERT_TRUE(ypr.isApprox(ypr_expected, 1e-4));

    rot_matrix << 0.2702, -0.0222, -0.9626, 0.4679, -0.8707, 0.1514, -0.8415,
      -0.4913, -0.2248;
    ypr = rotMatrixToYPR(rot_matrix);
    ypr_expected << M_PI * 1.0 / 3, 1, -2;
    ASSERT_TRUE(ypr.isApprox(ypr_expected, 1e-4));
}

/// Test against MATLAB output of rotm2eul
/// Degenerate case where pitch = M_PI/2
TEST(PoseCovComp, rotMatrixToYPR_degenerate_test) {
    Matrix3x3 rot_matrix;
    Vector3 ypr, ypr_expected;

    rot_matrix << 0.0000, -0.8415, 0.5403, 0.0000, 0.5403, 0.8415, -1.0000, 0,
      0.0000;
    ypr = rotMatrixToYPR(rot_matrix);
    ypr_expected << 1, M_PI * 1.0 / 2, 0;
    ASSERT_TRUE(ypr.isApprox(ypr_expected, 1e-4));

    rot_matrix << 0.0000, -0.8415, -0.5403, 0.0000, 0.5403, -0.8415, 1.0000, 0,
      0.0000;
    ypr = rotMatrixToYPR(rot_matrix);
    ypr_expected << 1, -M_PI * 1.0 / 2, 0;
    ASSERT_TRUE(ypr.isApprox(ypr_expected, 1e-4));
}

/// Test against MATLAB eul2quat
TEST(PoseCovComp, yprToQuat_test) {
    Vector3 ypr;
    Vector4 q, q_expected;

    ypr << 0, M_PI * 1.0 / 2, 0;
    q = yprToQuat(ypr);
    q_expected << 0.7071, 0, 0.7071, 0;
    ASSERT_TRUE(q.isApprox(q_expected, 1e-4));

    ypr << 0.78544, 0.1, 0;
    q = yprToQuat(ypr);
    q_expected << 0.9227, -0.0191, 0.0462, 0.3822;
    ASSERT_TRUE(q.isApprox(q_expected, 1e-4));

    ypr << M_PI * 1.0 / 3, 1, -2;
    q = yprToQuat(ypr);
    q_expected << 0.2089, -0.7690, -0.1449, 0.5865;
    ASSERT_TRUE(q.isApprox(q_expected, 1e-4));
}

/// Test against MATLAB rotm2quat
TEST(PoseCovComp, rotMatrixToQuat_test) {
    Matrix3x3 rot_matrix;
    Vector4 q, q_expected;

    rot_matrix << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    q = rotMatrixToQuat(rot_matrix);
    q_expected << 0.7071, 0, 0.7071, 0;
    ASSERT_TRUE(q.isApprox(q_expected, 1e-4));

    rot_matrix << 0.7035, -0.7071, 0.0706, 0.7036, 0.7071, 0.0706, -0.0998, 0,
      0.9950;
    q = rotMatrixToQuat(rot_matrix);
    q_expected << 0.9227, -0.0191, 0.0462, 0.3822;
    ASSERT_TRUE(q.isApprox(q_expected, 1e-4));

    rot_matrix << 0.2702, -0.0222, -0.9626, 0.4679, -0.8707, 0.1514, -0.8415,
      -0.4913, -0.22484;
    q = rotMatrixToQuat(rot_matrix);
    q_expected << 0.2089, -0.7690, -0.1449, 0.5865;
    ASSERT_TRUE(q.isApprox(q_expected, 1e-4));
}

/// Test against numerical Jacobian
TEST(PoseCovComp, jacobian_Quat_Norm_wrt_q) {
    Vector4 q;
    Matrix4x4 m, m_exp;

    q << 1, 0, 0, 0;
    m_exp << 0, -0.000005000000414, -0.000005000000414, -0.000005000000414, 0,
      0.999999999950000, 0, 0, 0, 0, 0.999999999950000, 0, 0, 0, 0,
      0.999999999950000;
    m = jacobian_Quat_Norm_wrt_q(q);
    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));

    q << 0.182574185835055, 0.365148371670111, 0.547722557505166,
      0.730296743340221;
    m_exp << 0.966664019300123, -0.066667214382310, -0.100000091279862,
      -0.133332785609563, -0.066668309828266, 0.866661919723954,
      -0.200000182559723, -0.266665571219127, -0.100002464742399,
      -0.200001643135828, 0.699994248931635, -0.399998356825915,
      -0.133336619656532, -0.266668857529240, -0.400000365119446,
      0.466661554621162;
    m = jacobian_Quat_Norm_wrt_q(q);
    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));

    q << -1, 3, -10, -5;
    m_exp << 0.085428777950314, 0.001912586919395, -0.006375285124816,
      -0.003187639197044, 0.001912575015028, 0.080328516699479,
      0.019125855371671, 0.009562917585582, -0.006375250061197,
      0.019125869188397, 0.022313509095984, -0.031876391959340,
      -0.003187625030598, 0.009562934594198, -0.031876425621302,
      0.070128132473313;
    m = jacobian_Quat_Norm_wrt_q(q);
    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));
}

/// Test against numerical Jacobian
TEST(PoseCovComp, jacobian_Quat_Norm_to_Rpy_wrt_q) {
    Vector4 q;
    Matrix3x4 m, m_exp;

    q << 1, 0, 0, 0;
    m_exp << 0, 0, 0, 2.000000000133333, 0, 0, 2.000000000133333, 0, 0,
      2.000000000133333, 0, 0;
    m = jacobian_Quat_Norm_to_Rpy_wrt_q(q);
    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));

    q << 0.182574185835055, 0.365148371670111, 0.547722557505166,
      0.730296743340221;
    m_exp << -1.095433115105848, -0.821577086274061, 1.095436115017279,
      1.916996701556428, 1.161892617412841, -1.549197581207640,
      0.387298069459696, -0.774597729907844, 0.109543671511503,
      1.588413806663524, 2.519514404175638, 0.164314877282301;
    m = jacobian_Quat_Norm_to_Rpy_wrt_q(q);
    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));
}

/// Test against numerical Jacobian
TEST(PoseCovComp, jacobian_p7_to_p6_wrt_p) {
    Vector7 p;
    Matrix6x7 m, m_exp;

    p << 10000, -1234, -0.01, 0.182574185835055, 0.365148371670111,
      0.547722557505166, 0.730296743340221;
    m_exp << 1.000000000000000, 0, 0, 0, 0, 0, 0, 0, 1.000000000000000, 0, 0, 0,
      0, 0, 0, 0, 1.000000000000000, 0, 0, 0, 0, 0, 0, 0, -1.369302643849934,
      -1.369310143761737, 0.273865028654896, 0.821580086185491, 0, 0, 0,
      1.290990323876695, -1.290984431351339, 0.774590658825236,
      -0.258191700902577, 0, 0, 0, -0.273869528699677, 0.821592086253098,
      1.369298143827357, -1.369298143760744;
    m = jacobian_p7_to_p6_wrt_p(p);
    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));
}

/// Test against numerical Jacobian
TEST(PoseCovComp, jacobian_p7_p7_Composition_wrt_p1) {
    Vector7 p, q;
    Matrix7x7 m, m_exp;

    p << 4, 3, -1, 0.475528258147577, -0.154508497187474, 0.267616567329817,
      -0.823639103546332;
    q << 1, 2, 3, 0.182574185835055, 0.365148371670111, 0.547722557505166,
      0.730296743340221;

    m = jacobian_p7_p7_Composition_wrt_p1(p, q);


    m_exp << 0.999999999962142, 0, 0, 3.541747805702044, -3.430002809601262,
      0.400122223354060, 2.818326357090228, 0, 0.999999999962142, 0,
      3.613482488606933, -2.489974567865261, -2.811883320585906,
      1.639715318901835, 0, 0, 1.000000000006551, 0.509686399086107,
      1.568576876809224, -6.521153764671438, -2.118866245615081, 0, 0, 0,
      -0.101869168256385, -0.272731058870157, -0.707800707555606,
      -0.237632491062456, 0, 0, 0, -0.011470322458518, 0.304940645234897,
      0.518340875876877, 0.104594530259128, 0, 0, 0, 0.489988722461587,
      -0.711540484793494, 0.150083497435505, 0.465145093747621, 0, 0, 0,
      0.723372429568825, 0.549972046151592, -0.369042331477276,
      0.194562972621881;

    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));
}

/// Test against numerical Jacobian
TEST(PoseCovComp, jacobian_p7_p7_Composition_wrt_p2) {
    Vector7 p, q;
    Matrix7x7 m, m_exp;

    p << 4, 3, -1, 0.475528258147577, -0.154508497187474, 0.267616567329817,
      -0.823639103546332;
    q << 1, 2, 3, 0.182574185835055, 0.365148371670111, 0.547722557505166,
      0.730296743340221;

    m = jacobian_p7_p7_Composition_wrt_p2(p, q);
    m_exp << -0.500000000069889, 0.700629269179131, 0.509036960405496, 0, 0, 0,
      0, -0.866025403833959, -0.404508497275913, -0.293892626146430, 0, 0, 0, 0,
      0, -0.587785252292861, 0.809016994374190, 0, 0, 0, 0, 0, 0, 0,
      0.366316299404978, -0.063910659098276, -0.595240597056623,
      0.386801287743932, 0, 0, 0, -0.299109499013461, 0.186328705875649,
      0.389841036418925, -0.310773890943583, 0, 0, 0, 0.245450294959759,
      -0.867966935115538, 0.409029880529821, 0.065846783250945, 0, 0, 0,
      -0.826294842010133, -0.272929987309156, -0.162479188463294,
      0.464896122014113;

    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));
}

/// Test against numerical Jacobian
TEST(PoseCovComp, jacobian_p6_to_p7_wrt_p) {
    Vector6 p;
    Matrix7x6 m, m_exp;

    p << 4, 3, -1, 0.942477796076938, -15.707963267948966, -0.031415926535898;
    m = jacobian_p6_to_p7_wrt_p(p);

    m_exp << 0.999999999962142, 0, 0, 0, 0, 0, 0, 1.000000000006551, 0, 0, 0, 0,
      0, 0, 0.999999999995449, 0, 0, 0, 0, 0, 0, 0.006997652187476,
      0.445448292529880, -0.226967254941899, 0, 0, 0, 0.445447734043203,
      -0.006998228518906, 0.003564918998888, 0, 0, 0, 0.226968359651192,
      -0.003564372796916, -0.006996547485816, 0, 0, 0, 0.003565503912083,
      0.226967263513601, 0.445448318954662;

    ASSERT_TRUE(m.isApprox(m_exp, 1e-4));
}

/// Test with all zero inputs, and make sure the output is all zeros
TEST(PoseCovComp, all_zero_test) {
    PoseWithCovariance p1, p2;

    PoseWithCovariance r = composePose(p1, p2);

    PoseWithCovariance p;

    ASSERT_TRUE(CheckPose(p, r));

    Matrix6x6 m_exp = Matrix6x6::Zero();

    ASSERT_TRUE(r.covariance.isApprox(m_exp, 1e-15));
}

/// Test just the pose part of the composition is correct, the same ROS tf
/// function is used here, but this checks that function from top to bottom
/// returns the correct composition
TEST(PoseCovComp, pose_test_A) {
    Vector6 p1, p2;
    Matrix6x6 cov;
    cov.setZero();

    p1 << 1, 2, 3, 0, 0, 0;
    p2 << 4, 5, 6, 0, 0, 0.785398;
    PoseWithCovariance pc1(p1, cov), pc2(p2, cov);

    PoseWithCovariance r = composePose(pc1, pc2);

    Vector6 p;
    p << 5, 7, 9, 0, 0, 0.785398;

    PoseWithCovariance pc(p, cov);

    ASSERT_TRUE(CheckPose(pc, r));
}

/// Test just the pose part of the composition is correct, the same ROS tf
/// function is used here, but this checks that function from top to bottom
/// returns the correct composition
TEST(PoseCovComp, pose_test_B) {
    Vector6 p1, p2;
    Matrix6x6 cov;

    p1 << -1, 2, -3, M_PI * 10, M_PI * 0.99, M_PI;
    p2 << 123, -0.03, -11, M_PI * 0.11, M_PI * 3.5, M_PI * -1;
    cov.setZero();
    PoseWithCovariance pc1(p1, cov), pc2(p2, cov);

    PoseWithCovariance r1 = composePose(pc1, pc2);

    Eigen::Affine3d T_p1, T_p2, T_r2;

    T_p1.linear() = pc1.rotation_matrix;
    T_p1.translation() = pc1.position;

    T_p2.linear() = pc2.rotation_matrix;
    T_p2.translation() = pc2.position;

    T_r2 = T_p1 * T_p2;
    Vector3 r2_trans = T_r2.translation();
    Matrix3x3 r2_rot = T_r2.rotation();
    PoseWithCovariance r2(r2_trans, r2_rot, cov);

    ASSERT_TRUE(CheckPose(r2, r1));
}

/// This test checks the covariances are the expected values, this ensures
/// that no changes accidentally modifies the computation results
/// Pose covariances are the same
TEST(PoseCovComp, covariance_test_A) {
    Vector6 p1, p2;
    Matrix6x6 cov1, cov2;

    p1 << 0, 0, 0, 0, 0, 0;
    p2 << 0, 0, 0, 0, 0, 0;

    cov1 << 1, 0.1, -0.2, 0.1, 0.3, 0.1, 0.1, 1, -0.3, -0.4, 0.1, 0.3, -0.2,
      -0.3, 1, 0.3, -0.4, 0.4, 0.1, -0.4, 0.3, 1, -0.3, 0.5, 0.3, 0.1, -0.4,
      -0.3, 1, 0.3, 0.1, 0.3, 0.4, 0.5, 0.3, 1;
    cov1 = 1.0e-3 * (cov1.transpose() * cov1);
    cov2 = cov1;

    PoseWithCovariance pc1(p1, cov1), pc2(p2, cov2);
    PoseWithCovariance r1 = composePose(pc1, pc2);

    // check with the computed results
    Matrix6x6 m_exp;
    m_exp << 0.00232, 0.00056, -0.00096, 0.00012, 0.00138, 0.00058, 0.00056,
      0.00272, -0.00132, -0.00152, 0.00112, 0.00064, -0.00096, -0.00132,
      0.00308, 0.00204, -0.00172, 0.00144, 0.00012, -0.00152, 0.00204, 0.0032,
      -0.00116, 0.00184, 0.00138, 0.00112, -0.00172, -0.00116, 0.00288, 0.0007,
      0.00058, 0.00064, 0.00144, 0.00184, 0.0007, 0.0032;

    Matrix6x6 covar(r1.covariance);
    EXPECT_TRUE(covar.isApprox(m_exp, 1e-5));
}


/// This test checks the covariances are the expected values, this ensures
/// that no changes accidentally modifies the computation results
/// Pose covariances are different
TEST(PoseCovComp, covariance_test_B) {
    Vector6 p1, p2;
    Matrix6x6 cov1, cov2;

    p1 << 0, 0, 0, 0, 0, 0;
    p2 << 0, 0, 0, 0, 0, 0;

    cov1 << 1, 0.1, -0.2, 0.1, 0.3, 0.1, 0.1, 1, -0.3, -0.4, 0.1, 0.3, -0.2,
      -0.3, 1, 0.3, -0.4, 0.4, 0.1, -0.4, 0.3, 1, -0.3, 0.5, 0.3, 0.1, -0.4,
      -0.3, 1, 0.3, 0.1, 0.3, 0.4, 0.5, 0.3, 1;
    cov1 = 1.0e-3 * (cov1.transpose() * cov1);
    cov2 = cov1 + cov1;

    PoseWithCovariance pc1(p1, cov1), pc2(p2, cov2);
    PoseWithCovariance r1 = composePose(pc1, pc2);

    // check with the computed results
    Matrix6x6 m_exp;
    m_exp << 0.00348, 0.00084, -0.00144, 0.00018, 0.00207, 0.00087, 0.00084,
      0.00408, -0.00198, -0.00228, 0.00168, 0.00096, -0.00144, -0.00198,
      0.00462, 0.00306, -0.00258, 0.00216, 0.00018, -0.00228, 0.00306, 0.0048,
      -0.00174, 0.00276, 0.00207, 0.00168, -0.00258, -0.00174, 0.00432, 0.00105,
      0.00087, 0.00096, 0.00216, 0.00276, 0.00105, 0.0048;

    Matrix6x6 covar(r1.covariance);
    EXPECT_TRUE(covar.isApprox(m_exp, 1e-6));
}

/// This test checks the covariances are the expected values, this ensures
/// that no changes accidentally modifies the computation results
/// Note: Degenerate case due to pitch of M_PI/2
TEST(PoseCovComp, covariance_test_C) {
    Vector6 p1, p2;
    Matrix6x6 cov1, cov2;


    p1 << -1, 2, -3, M_PI * 10, M_PI * 0.99, M_PI;
    p2 << 2, -0.5, 0, M_PI * 0.11, M_PI * 3.5, M_PI * -1;

    cov1 << 0.0854, -0.0113, 0.0153, 0.0191, -0.0074, 0.0271, -0.0113, 0.0081,
      -0.0002, -0.0069, 0.0014, -0.0111, 0.0153, -0.0002, 0.0212, 0.0062,
      -0.0025, 0.0040, 0.0191, -0.0069, 0.0062, 0.0177, -0.0081, 0.0003,
      -0.0074, 0.0014, -0.0025, -0.0081, 0.0573, -0.0019, 0.0271, -0.0111,
      0.0040, 0.0003, -0.0019, 0.0347;

    cov2 << 0.0216, -0.0126, -0.0011, -0.0107, 0.0020, -0.0154, -0.0126, 0.0425,
      0.0174, 0.0069, 0.0085, 0.0152, -0.0011, 0.0174, 0.0389, 0.0046, -0.0004,
      0.0082, -0.0107, 0.0069, 0.0046, 0.0427, 0.0054, 0.0251, 0.0020, 0.0085,
      -0.0004, 0.0054, 0.0114, -0.0032, -0.0154, 0.0152, 0.0082, 0.0251,
      -0.0032, 0.0257;

    PoseWithCovariance pc1(p1, cov1), pc2(p2, cov2);
    PoseWithCovariance r1 = composePose(pc1, pc2);

    // check with the computed results
    Matrix6x6 m_exp;
    m_exp << 0.0939170077, 0.02654455951, 0.04181333697, -0.8900658294,
      0.005086858698, -0.9268740694, 0.02654455951, 0.09374376512,
      0.05351137717, 0.4544374846, 0.02278949281, 0.4036447693, 0.04181333697,
      0.05351137717, 0.3154435083, -0.7814409313, 0.117675924, -0.8155600097,
      -0.8900658294, 0.4544374846, -0.7814409313, 36.49441063, 0.06328226214,
      36.44453263, 0.005086858698, 0.02278949281, 0.117675924, 0.06328226214,
      0.06739192548, 0.05308509534, -0.9268740694, 0.4036447693, -0.8155600097,
      36.44453263, 0.05308509534, 36.53097751;

    Matrix6x6 covar(r1.covariance);
    EXPECT_TRUE(covar.isApprox(m_exp, 1e-6));
}

/// This test checks the covariances are the expected values, this ensures
/// that no changes accidentally modifies the computation results
/// Note: Degenerate case due to pitch of M_PI/2
TEST(PoseCovComp, covariance_test_D) {
    Vector6 p1, p2;
    Matrix6x6 cov1, cov2;

    p1 << -1, 2, -3, M_PI * 10, M_PI * 0.99, M_PI;
    p2 << 2, -0.5, 0, M_PI * 0.11, M_PI * 3.5, M_PI * -1;

    cov1 << 5.1469, -3.9470, -2.8670, 2.7500, -0.5647, -0.4096, -3.9470,
      14.8769, 2.0875, -2.2614, 1.5113, -1.7985, -2.8670, 2.0875, 3.5990,
      -4.1143, -1.3928, -0.0248, 2.7500, -2.2614, -4.1143, 7.7574, 4.2143,
      -0.2528, -0.5647, 1.5113, -1.3928, 4.2143, 6.5188, -2.2745, -0.4096,
      -1.7985, -0.0248, -0.2528, -2.2745, 1.9454;

    cov2 << 4.9694, -0.9063, 4.7049, -0.1580, -0.9065, -4.8257, -0.9063, 3.4400,
      -0.9033, 0.6171, -0.3010, 0.4286, 4.7049, -0.9033, 13.8381, 4.8507,
      -0.7857, -4.9432, -0.1580, 0.6171, 4.8507, 11.4157, 2.9465, -0.9057,
      -0.9065, -0.3010, -0.7857, 2.9465, 2.5863, -0.1871, -4.8257, 0.4286,
      -4.9432, -0.9057, -0.1871, 7.2824;

    PoseWithCovariance pc1(p1, cov1), pc2(p2, cov2);
    PoseWithCovariance r1 = composePose(pc1, pc2);

    // check with the computed results
    Matrix6x6 m_exp;
    m_exp << 9.376475663, -4.855467832, 1.036334379, 11.60174971, 3.946359952,
      8.014419096, -4.855467832, 40.27474854, -28.081395, 83.34562314,
      -10.21894611, 69.01958869, 1.036334379, -28.081395, 54.34729134,
      -196.3548644, 14.79497982, -183.3501498, 11.60174971, 83.34562314,
      -196.3548644, 2296.385192, -50.38367668, 2249.696519, 3.946359952,
      -10.21894611, 14.79497982, -50.38367668, 8.8083392, -48.74286127,
      8.014419096, 69.01958869, -183.3501498, 2249.696519, -48.74286127,
      2227.689809;

    Matrix6x6 covar(r1.covariance);
    EXPECT_TRUE(covar.isApprox(m_exp, 1e-6));
}
