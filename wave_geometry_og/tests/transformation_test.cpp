#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/geometry_og/transformation.hpp"
#include "wave/geometry_og/numerical_test_functors.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

namespace {

using T_Type = Transformation<Mat34, false>;

}

class TransformationTestFixture : public ::testing::Test {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    T_Type transform_expected;
    Vec6 transformation_twist_parameters;
    double comparison_threshold = 1e-6;

    void SetUp() {
        Mat4 transform_matrix;
        transform_matrix << 0.936293363584199, -0.275095847318244, 0.218350663146334, 1, 0.289629477625516,
          0.956425085849232, -0.036957013524625, 2, -0.198669330795061, 0.097843395007256, 0.975170327201816, 3, 0, 0,
          0, 1;
        transformation_twist_parameters << 0.068924613882066, 0.213225926957886, 0.288748939228676, 0.965590777183138,
          1.960945901104432, 3.037052911306709;
        this->transform_expected.setFromMatrix(transform_matrix);
    }
};

TEST(TransformationTest, Destructor) {
    T_Type transform;
//    transform.~T_Type();
}

// Test that fromEulerXYZ returns the correct matrix.
TEST_F(TransformationTestFixture, testFromEulerXYZ) {
    T_Type transformation(Vec3(0.1, 0.2, 0.3), Vec3(1, 2, 3));

    ASSERT_TRUE(this->transform_expected.isNear(transformation, this->comparison_threshold));

    // Test non-finite input arguments
    Vec3 ip_nan(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    ASSERT_THROW(T_Type t_nan(ip_nan, Vec3(1, 2, 3)), std::invalid_argument);
    Vec3 ip_inf(std::numeric_limits<double>::infinity(), 0, 0);
    ASSERT_THROW(T_Type t_inf(ip_inf, Vec3(1, 2, 3)), std::invalid_argument);
}

TEST_F(TransformationTestFixture, testFromExpMap) {
    T_Type transform;
    transform.setFromExpMap(this->transformation_twist_parameters);

    ASSERT_TRUE(this->transform_expected.isNear(transform, this->comparison_threshold));

    // Test non-finite input arguments
    transformation_twist_parameters(0) = std::numeric_limits<double>::quiet_NaN();
    ASSERT_THROW(transform.setFromExpMap(transformation_twist_parameters), std::invalid_argument);
    transformation_twist_parameters(0) = std::numeric_limits<double>::infinity();
    ASSERT_THROW(transform.setFromExpMap(transformation_twist_parameters), std::invalid_argument);
}

TEST_F(TransformationTestFixture, testLogMap) {
    Vec6 copy = this->transform_expected.logMap();
    Vec6 err = copy - this->transformation_twist_parameters;
    ASSERT_LE(err.norm(), this->comparison_threshold);
}

// Test to ensure default constructor produces identity.
TEST_F(TransformationTestFixture, testDefaultConstructor) {
    T_Type R;
    Mat4 eye;
    // Use Eigen's identity.
    eye.setIdentity();
    T_Type Q;
    Q.setFromMatrix(eye);
    ASSERT_TRUE(R.isNear(Q, this->comparison_threshold));
}

TEST_F(TransformationTestFixture, testSetIdentity) {
    T_Type transform;
    transform.setFromExpMap(this->transformation_twist_parameters);
    transform.setIdentity();
    Mat4 eye;
    // Use Eigen's identity.
    eye.setIdentity();
    T_Type Q;
    Q.setFromMatrix(eye);
    ASSERT_TRUE(transform.isNear(Q, this->comparison_threshold));
}

TEST_F(TransformationTestFixture, testCompose) {
    Vec6 T2_twist;
    T2_twist << -0.1, -0.2, -0.3, 2, 1, 2;
    T_Type T2;
    T2.setFromExpMap(T2_twist);

    Vec6 T3_twist;
    T3_twist << -0.033491978759741, 0.008765073464665, -0.007426741800595, 3.011657415278366, 3.184233419918160,
      4.863413030607249;

    T_Type T3;
    T3.setFromExpMap(T3_twist);

    T_Type composed = this->transform_expected * T2;
    ASSERT_TRUE(T3.isNear(composed, this->comparison_threshold));
}

TEST_F(TransformationTestFixture, testInverse) {
    Vec6 T_inv_twist;
    T_inv_twist << -0.068924613882065, -0.213225926957886, -0.288748939228676, -0.965590777183138, -1.960945901104432,
      -3.037052911306709;

    T_Type T_inv, T_expect;
    T_inv.setFromExpMap(T_inv_twist);

    this->transform_expected.transformInverse(T_expect);
    ASSERT_TRUE(T_expect.isNear(T_inv, this->comparison_threshold));
}

TEST_F(TransformationTestFixture, testTransform) {
    Vec3 v1(1, 1, 1);
    Vec3 v1_transformed(1.879548179412289, 3.209097549950123, 3.874344391414011);
    Vec3 v1_test;
    this->transform_expected.transform(v1, v1_test);
    Vec3 err = v1_test - v1_transformed;

    ASSERT_LE(err.norm(), this->comparison_threshold);
}

TEST_F(TransformationTestFixture, testInverseTransform) {
    Vec3 v1(1, 1, 1);
    Vec3 v1_transformed(0.107709183964606, -1.152111875863744, -1.913383640879007);
    Vec3 v1_test = this->transform_expected.inverseTransform(v1);
    Vec3 err = v1_test - v1_transformed;

    ASSERT_LE(err.norm(), this->comparison_threshold);
}

TEST_F(TransformationTestFixture, testManifoldPlus) {
    Vec6 T2_twist;
    T2_twist << -0.1, -0.2, -0.3, 2, 1, 2;
    T_Type T2;
    T2.setFromExpMap(T2_twist);

    // expm(t1) * T2
    T2.manifoldPlus(this->transformation_twist_parameters);

    // This is T1 * T2
    Vec6 T3_twist;
    T3_twist << -0.033491978759741, 0.008765073464665, -0.007426741800595, 3.011657415278366, 3.184233419918160,
      4.863413030607249;
    T_Type T3;
    T3.setFromExpMap(T3_twist);

    ASSERT_TRUE(T2.isNear(T3, this->comparison_threshold));
}

TEST_F(TransformationTestFixture, testManifoldMinus) {
    Vec6 T3_twist;
    T3_twist << -0.033491978759741, 0.008765073464665, -0.007426741800595, 3.011657415278366, 3.184233419918160,
      4.863413030607249;
    T_Type T3;
    T3.setFromExpMap(T3_twist);

    Vec6 T2_twist;
    T2_twist << -0.1, -0.2, -0.3, 2, 1, 2;
    T_Type T2;
    T2.setFromExpMap(T2_twist);

    Vec6 params = T3.manifoldMinus(T2);

    Vec6 err = params - this->transformation_twist_parameters;
    ASSERT_LE(err.norm(), this->comparison_threshold);
}

// Jacobian testing
TEST_F(TransformationTestFixture, testTransformAndJacobian) {
    Vec3 v(1, 1, 1);
    Vec3 dead_end;
    Eigen::Matrix<double, 3, 6> Jparam_analytical, Jparam_numerical;
    Mat3 Jpoint_analytical, Jpoint_numerical;
    // get analytical jacobians
    this->transform_expected.transformAndJacobian(v, dead_end, &Jpoint_analytical, &Jparam_analytical);

    TransformAndJacobianJpointFunctor Jpoint_functor(this->transform_expected);
    numerical_jacobian(Jpoint_functor, v, Jpoint_numerical);

    TransformAndJacobianJparamFunctor Jparam_functor(this->transform_expected, v);
    Vec6 perturb_vec = Vec6::Zero();
    numerical_jacobian(Jparam_functor, perturb_vec, Jparam_numerical);

    MatX err_mat = Jpoint_analytical - Jpoint_numerical;
    ASSERT_LE(err_mat.norm(), this->comparison_threshold);

    MatX err_mat2 = Jparam_analytical - Jparam_numerical;
    ASSERT_LE(err_mat2.norm(), this->comparison_threshold);
}

TEST_F(TransformationTestFixture, testComposeAndJacobian) {
    Vec6 T3_twist;
    T3_twist << -0.033491978759741, 0.008765073464665, -0.007426741800595, 3.011657415278366, 3.184233419918160,
      4.863413030607249;
    T_Type T3;
    T3.setFromExpMap(T3_twist);

    Vec6 T2_twist;
    T2_twist << -0.1, -0.2, -0.3, 2, 1, 2;
    T_Type T2;
    T2.setFromExpMap(T2_twist);

    Mat6 J_left_analytical, J_right_analytical;
    T_Type composed = this->transform_expected.composeAndJacobian(T2, J_left_analytical, J_right_analytical);
    ASSERT_TRUE(composed.isNear(T3, this->comparison_threshold));

    Mat6 J_left_numerical, J_right_numerical;
    Vec6 perturbation_vec = Vec6::Zero();

    // Jacobian wrt the left .
    TComposeAndJacobianJLeftFunctor J_left_functor(this->transform_expected, T2);
    numerical_jacobian(J_left_functor, perturbation_vec, J_left_numerical);

    wave::MatX error_matrix = J_left_analytical - J_left_numerical;
    ASSERT_LE(error_matrix.norm(), this->comparison_threshold);


    // Jacobian wrt the right .
    TComposeAndJacobianJRightFunctor J_right_functor(this->transform_expected, T2);
    numerical_jacobian(J_right_functor, perturbation_vec, J_right_numerical);

    error_matrix = J_right_analytical - J_right_numerical;
    ASSERT_LE(error_matrix.norm(), this->comparison_threshold);
}

TEST_F(TransformationTestFixture, testInverseAndJacoban) {
    Vec6 T_inv_twist;
    T_inv_twist << -0.068924613882065, -0.213225926957886, -0.288748939228676, -0.965590777183138, -1.960945901104432,
      -3.037052911306709;

    T_Type T_inv;
    T_inv.setFromExpMap(T_inv_twist);

    Mat6 J_analytical;
    T_Type T1inv = this->transform_expected.inverseAndJacobian(J_analytical);
    ASSERT_TRUE(T1inv.isNear(T_inv, this->comparison_threshold));

    Mat6 J_numerical;
    Vec6 perturbation_vec = Vec6::Zero();

    TInverseAndJacobianFunctor J_inverse_functor(this->transform_expected);
    numerical_jacobian(J_inverse_functor, perturbation_vec, J_numerical);

    wave::MatX err_mat = J_analytical - J_numerical;
    ASSERT_LE(err_mat.norm(), this->comparison_threshold);
}

TEST_F(TransformationTestFixture, testLogMapAndJacobian) {
    Mat6 J_analytical;
    Vec6 log_params = this->transform_expected.logMap();
    J_analytical = T_Type::SE3LeftJacobian(log_params);
    J_analytical = J_analytical.inverse();
    ASSERT_LE((log_params - this->transformation_twist_parameters).norm(), this->comparison_threshold);

    Mat6 J_numerical;
    Vec6 perturbation_vec = Vec6::Zero();

    TLogMapAndJacobianFunctor J_log_functor(this->transform_expected);
    numerical_jacobian(J_log_functor, perturbation_vec, J_numerical);

    wave::MatX err_mat = J_analytical - J_numerical;
    ASSERT_LE(err_mat.norm(), this->comparison_threshold);
}

TEST_F(TransformationTestFixture, testManifoldMinusAndJacobian) {
    Vec6 T3_twist;
    T3_twist << -0.033491978759741, 0.008765073464665, -0.007426741800595, 3.011657415278366, 3.184233419918160,
      4.863413030607249;
    T_Type T3;
    T3.setFromExpMap(T3_twist);

    Vec6 T2_twist;
    T2_twist << -0.1, -0.2, -0.3, 2, 1, 2;
    T_Type T2;
    T2.setFromExpMap(T2_twist);

    // T3 - T2 = T1 (transform_expected)

    Mat6 J_left_analytical, J_right_analytical;
    Vec6 log_params = T3.manifoldMinusAndJacobian(T2, &J_left_analytical, &J_right_analytical);
    Vec6 err = log_params - this->transformation_twist_parameters;
    ASSERT_LE(err.norm(), this->comparison_threshold);

    Mat6 J_left_numerical, J_right_numerical;
    Vec6 perturbation_vec = Vec6::Zero();

    TManifoldMinusAndJacobianJLeftFunctor J_left_functor(T3, T2);
    numerical_jacobian(J_left_functor, perturbation_vec, J_left_numerical);
    TManifoldMinusAndJacobianJRightFunctor J_right_functor(T3, T2);
    numerical_jacobian(J_right_functor, perturbation_vec, J_right_numerical);

    wave::MatX errmat = J_left_analytical - J_left_numerical;
    ASSERT_LE(errmat.norm(), this->comparison_threshold);
    errmat = J_right_analytical - J_right_numerical;
    ASSERT_LE(errmat.norm(), this->comparison_threshold);
}

TEST_F(TransformationTestFixture, MappingArrays) {
    Mat34 memblock;
    Eigen::Map<Mat34> mapped(memblock.data());
    Transformation<Eigen::Map<Mat34>, true> inter(mapped);

    inter.setIdentity();

    const double permanent[12] = {0};

    Eigen::Map<const Mat34> pmap(permanent);
    Transformation<Eigen::Map<const Mat34>, true> ptra(pmap);
}

}