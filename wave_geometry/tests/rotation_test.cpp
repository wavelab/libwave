#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/geometry/numerical_test_functors.hpp"
#include "wave/geometry/rotation.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

class RotationTestFixture : public ::testing::Test {
 public:
    // Data members used for testing.
    Rotation rotation1_expected;
    // Comparision threshold for matrices.
    double comparision_threshold = 1e-5;

    void SetUp() {
        // It is the expected rotation matrix, using the parameters from
        // rotation1.
        Mat3 rotation_matrix;
        rotation_matrix << 0.936293363584199, -0.275095847318244,
          0.218350663146334, 0.289629477625516, 0.956425085849232,
          -0.036957013524625, -0.198669330795061, 0.097843395007256,
          0.975170327201816;
        rotation1_expected.setFromMatrix(rotation_matrix);
    }
};

// Construction tests.

// Test that fromEulerXYZ returns the correct matrix.
TEST_F(RotationTestFixture, testFromEulerXyz) {
    Rotation rotation1(Vec3(0.1, 0.2, 0.3));
    ASSERT_TRUE(
      this->rotation1_expected.isNear(rotation1, this->comparision_threshold));

    // Test non-finite input values.
    Vec3 input_vector_nan(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    ASSERT_THROW(Rotation rotation_nan(input_vector_nan),
                 std::invalid_argument);
    Vec3 input_vector_inf(0, 0, std::numeric_limits<double>::infinity());
    ASSERT_THROW(Rotation rotation_inf(input_vector_inf),
                 std::invalid_argument);
}


// Test that fromAngleAxis returns the correct matrix.
TEST_F(RotationTestFixture, testFromAngleAxis) {
    // True angle axis params for rotation1.
    Vec3 rotation1_angleaxis_params(
      0.068924613882066, 0.213225926957886, 0.288748939228675);

    Rotation rotation_from_angle_axis(rotation1_angleaxis_params.norm(),
                                      rotation1_angleaxis_params.normalized());
    ASSERT_TRUE(rotation1_expected.isNear(rotation_from_angle_axis,
                                          this->comparision_threshold));

    // Test non-finite input values.
    Vec3 input_vector_nan(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    ASSERT_THROW(Rotation rotation_nan(1, input_vector_nan),
                 std::invalid_argument);
    double input_mag = std::numeric_limits<double>::infinity();
    ASSERT_THROW(
      Rotation rotation_inf(input_mag, rotation1_angleaxis_params.normalized()),
      std::invalid_argument);

    // Test passing non normalized axis
    ASSERT_THROW(Rotation rotation_inf(1, rotation1_angleaxis_params),
                 std::invalid_argument);
}

// Test that setFromExpMap returns the correct matrix.
TEST_F(RotationTestFixture, testSetFromExpMap) {
    // True angle axis params for rotation1.
    Vec3 rotation1_angleaxis_params(
      0.068924613882066, 0.213225926957886, 0.288748939228675);

    Rotation rotation_from_exp_map;
    rotation_from_exp_map.setFromExpMap(rotation1_angleaxis_params);
    ASSERT_TRUE(rotation1_expected.isNear(rotation_from_exp_map,
                                          this->comparision_threshold));

    // Test non-finite input values.
    Vec3 input_vector_nan(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    ASSERT_THROW(rotation_from_exp_map.setFromExpMap(input_vector_nan),
                 std::invalid_argument);
    Vec3 input_vector_inf(std::numeric_limits<double>::infinity(), 0, 0);
    ASSERT_THROW(rotation_from_exp_map.setFromExpMap(input_vector_inf),
                 std::invalid_argument);

    // Test passing non normalized axis
    ASSERT_THROW(Rotation rotation_inf(1, rotation1_angleaxis_params),
                 std::invalid_argument);
}

// Test input checking for setFromMatrix.
TEST_F(RotationTestFixture, testFromMatrix) {
    Rotation rotation1;
    Mat3 rotation_matrix;
    rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0,
      std::numeric_limits<double>::infinity();
    ASSERT_THROW(rotation1.setFromMatrix(rotation_matrix),
                 std::invalid_argument);

    rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0,
      std::numeric_limits<double>::quiet_NaN();
    ASSERT_THROW(rotation1.setFromMatrix(rotation_matrix),
                 std::invalid_argument);

    // Invalid rotation matrix.
    rotation_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 10;
    ASSERT_THROW(rotation1.setFromMatrix(rotation_matrix),
                 std::invalid_argument);
}

// Test that logMap returns the correct parameters.
TEST_F(RotationTestFixture, testLogMap) {
    // True angle axis params for rotation1.
    Vec3 rotation1_angleaxis_params(
      0.068924613882066, 0.213225926957886, 0.288748939228675);
    Vec3 so3_params = this->rotation1_expected.logMap();
    Vec3 error = rotation1_angleaxis_params - so3_params;
    ASSERT_LE(error.norm(), this->comparision_threshold);
}

// Test to ensure default constructor produces identity.
TEST_F(RotationTestFixture, testDefaultConstructor) {
    Rotation R;
    Mat3 eye;
    // Use Eigen's identity.
    eye.setIdentity();
    Rotation Q;
    Q.setFromMatrix(eye);
    ASSERT_TRUE(R.isNear(Q, this->comparision_threshold));
}

// Test to ensure setIdentity returns identity.
TEST_F(RotationTestFixture, testSetIdentity) {
    Rotation rotation1(Vec3(0.1, 0.2, 0.3));
    rotation1.setIdentity();
    Mat3 eye;
    // Use Eigen's identity.
    eye.setIdentity();
    Rotation Q;
    Q.setFromMatrix(eye);
    ASSERT_TRUE(rotation1.isNear(Q, this->comparision_threshold));
}

// Test that rotation composition returns expected product.
TEST_F(RotationTestFixture, testCompose) {
    Rotation rotation1(Vec3(0.1, 0.2, 0.3));
    Rotation rotation2(Vec3(-0.1, -0.2, -0.3));
    // rotation3 is the composition of rotation1 and rotation2.
    Rotation rotation3(
      Vec3(-0.065223037770317, 0.020616131403543, -0.013176146513587));
    Rotation composed_rotation = rotation1 * rotation2;
    ASSERT_TRUE(
      composed_rotation.isNear(rotation3, this->comparision_threshold));
}

// Test that rotation inverse returns correct matrix.
TEST_F(RotationTestFixture, testInverse) {
    Rotation rotation1(Vec3(0.1, 0.2, 0.3));
    // rotation1_inverse is the inverse of rotation1.
    Rotation rotation1_inverse(
      Vec3(-0.037879880513201, -0.220124031212965, -0.285771700628461));
    rotation1.invert();
    ASSERT_TRUE(
      rotation1.isNear(rotation1_inverse, this->comparision_threshold));
}

// Test that rotation of a vector returns correct value.
TEST_F(RotationTestFixture, testRotate) {
    Rotation rotation1(Vec3(0.1, 0.2, 0.3));
    Vec3 v1(1, 1, 1);
    Vec3 v1_rotated(0.879548179412290, 1.209097549950123, 0.874344391414010);
    Vec3 v1_rotated_test = rotation1.rotate(v1);
    Vec3 error = v1_rotated_test - v1_rotated;
    ASSERT_LE(error.norm(), this->comparision_threshold);

    // Test non-finite input values.
    Vec3 input_vector_nan(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    ASSERT_THROW(rotation1.rotate(input_vector_nan), std::invalid_argument);
    Vec3 input_vector_inf(std::numeric_limits<double>::infinity(), 0, 0);
    ASSERT_THROW(rotation1.rotate(input_vector_inf), std::invalid_argument);
}

// Test that inverse rotation of a vector returns correct value.
TEST_F(RotationTestFixture, testInverseRotate) {
    Rotation rotation1_inverse(
      Vec3(-0.037879880513201, -0.220124031212965, -0.285771700628461));
    Vec3 v1(1, 1, 1);
    Vec3 v1_inverse_rotated_test = rotation1_inverse.inverseRotate(v1);
    Vec3 v1_rotated(0.879548179412290, 1.209097549950123, 0.874344391414010);
    Vec3 error = v1_inverse_rotated_test - v1_rotated;
    ASSERT_LE(error.norm(), this->comparision_threshold);

    // Test non-finite input values.
    Vec3 input_vector_nan(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    ASSERT_THROW(rotation1_inverse.inverseRotate(input_vector_nan),
                 std::invalid_argument);
    Vec3 input_vector_inf(std::numeric_limits<double>::infinity(), 0, 0);
    ASSERT_THROW(rotation1_inverse.inverseRotate(input_vector_inf),
                 std::invalid_argument);
}

// Test functionality of the boxplus operator.
TEST_F(RotationTestFixture, testManifoldPlus) {
    // We know that rotation3 = rotation1*rotation2
    // Therefore rotation3 = exp(rotation1_angleaxis_params)*rotation2,
    // which is exactly the boxplus operation.

    // True angle axis params for rotation1.
    Vec3 rotation1_angleaxis_params(
      0.068924613882066, 0.213225926957886, 0.288748939228675);
    Rotation rotation2(Vec3(-0.1, -0.2, -0.3));
    // rotation3 is the composition of rotation1 and rotation2.
    Rotation rotation3(
      Vec3(-0.065223037770317, 0.020616131403543, -0.013176146513587));

    // Use manifold plus to make rotation2==rotation3.
    rotation2.manifoldPlus(rotation1_angleaxis_params);
    ASSERT_TRUE(rotation2.isNear(rotation3, this->comparision_threshold));

    // Test non-finite input values.
    Vec3 input_vector_nan(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    ASSERT_THROW(rotation2.manifoldPlus(input_vector_nan),
                 std::invalid_argument);
    Vec3 input_vector_inf(std::numeric_limits<double>::infinity(), 0, 0);
    ASSERT_THROW(rotation2.manifoldPlus(input_vector_inf),
                 std::invalid_argument);
}

// Test functionality of the boxminus operator.
TEST_F(RotationTestFixture, testManifoldMinus) {
    // We know that rotation3 = rotation1*rotation2
    // Therefore rotation3 * inv(rotation2) = rotation1
    // log(rotation3 * inv(rotation2)) = log(rotation1)
    // which is exactly rotation3 \boxminus rotation2.

    // True angle axis params for rotation1.
    Vec3 rotation1_angleaxis_params(
      0.068924613882066, 0.213225926957886, 0.288748939228675);
    Rotation rotation2(Vec3(-0.1, -0.2, -0.3));
    // rotation3 is the composition of rotation1 and rotation2.
    Rotation rotation3(
      Vec3(-0.065223037770317, 0.020616131403543, -0.013176146513587));

    Vec3 log_params = rotation3.manifoldMinus(rotation2);
    Vec3 error = rotation1_angleaxis_params - log_params;
    ASSERT_LE(error.norm(), this->comparision_threshold);
}

// JACOBIAN TESTING
// Compare the rotateAndJacobian derivatives numerically.
TEST_F(RotationTestFixture, testRotateAndJacobian) {
    // Rotation and a vector which we will rotate.
    Rotation rotation1(Vec3(0.1, 0.2, 0.3));
    Vec3 v1(1, 1, 1);

    // First get the analytical Jacobians.
    Mat3 Jparam_analytical, Jpoint_analytical;
    rotation1.rotateAndJacobian(v1, Jpoint_analytical, Jparam_analytical);

    // Now get the numerical Jacobians
    Mat3 Jparam_numerical(3, 3), Jpoint_numerical(3, 3);

    // Jacobian wrt the point.
    RotateAndJacobianJpointFunctor Jpoint_functor(rotation1);
    numerical_jacobian(Jpoint_functor, v1, Jpoint_numerical);
    wave::MatX error_matrix = Jpoint_analytical - Jpoint_numerical;
    ASSERT_LE(error_matrix.norm(), this->comparision_threshold);

    // Jacobian wrt the rotation params.
    RotateAndJacobianJparamFunctor Jparam_functor(rotation1, v1);

    // Perturbations are about zero vector in the tangent space.
    Vec3 perturbation_vec(0, 0, 0);
    numerical_jacobian(Jparam_functor, perturbation_vec, Jparam_numerical);
    error_matrix = Jparam_analytical - Jparam_numerical;
    ASSERT_LE(error_matrix.norm(), this->comparision_threshold);

    // Test non-finite input values.
    Vec3 input_vector_nan(std::numeric_limits<double>::quiet_NaN(), 0, 0);
    ASSERT_THROW(rotation1.rotateAndJacobian(
                   input_vector_nan, Jpoint_analytical, Jparam_analytical),
                 std::invalid_argument);
    Vec3 input_vector_inf(std::numeric_limits<double>::infinity(), 0, 0);
    ASSERT_THROW(rotation1.rotateAndJacobian(
                   input_vector_inf, Jpoint_analytical, Jparam_analytical),
                 std::invalid_argument);
}

// Compare the composeAndJacobian derivatives numerically.
TEST_F(RotationTestFixture, testComposeAndJacobian) {
    Rotation rotation1(Vec3(0.1, 0.2, 0.3));
    Rotation rotation2(Vec3(-0.1, -0.2, -0.3));
    // rotation3 is the composition of rotation1 and rotation2.
    Rotation rotation3(
      Vec3(-0.065223037770317, 0.020616131403543, -0.013176146513587));

    // First get the analytical Jacobians and composition.
    Mat3 J_left_analytical, J_right_analytical;
    Rotation composed_rotation = rotation1.composeAndJacobian(
      rotation2, J_left_analytical, J_right_analytical);
    ASSERT_TRUE(
      composed_rotation.isNear(rotation3, this->comparision_threshold));


    // Now get the numerical Jacobians
    Mat3 J_left_numerical(3, 3), J_right_numerical(3, 3);
    // Perturbations are about zero vector in the tangent space.
    Vec3 perturbation_vec(0, 0, 0);

    // Jacobian wrt the left rotation.
    ComposeAndJacobianJLeftFunctor J_left_functor(rotation1, rotation2);
    numerical_jacobian(J_left_functor, perturbation_vec, J_left_numerical);

    wave::MatX error_matrix = J_left_analytical - J_left_numerical;
    ASSERT_LE(error_matrix.norm(), this->comparision_threshold);


    // Jacobian wrt the right rotation.
    ComposeAndJacobianJRightFunctor J_right_functor(rotation1, rotation2);
    numerical_jacobian(J_right_functor, perturbation_vec, J_right_numerical);

    error_matrix = J_right_analytical - J_right_numerical;
    ASSERT_LE(error_matrix.norm(), this->comparision_threshold);
}

// Compare the inverseAndJacobian derivatives numerically.
TEST_F(RotationTestFixture, testInverseAndJacobian) {
    Rotation rotation1(Vec3(0.1, 0.2, 0.3));
    // rotation1_inverse is the inverse of rotation1.
    Rotation rotation1_inverse(
      Vec3(-0.037879880513201, -0.220124031212965, -0.285771700628461));

    Mat3 J_analytical;
    Rotation rotation2 = rotation1.inverseAndJacobian(J_analytical);
    ASSERT_TRUE(
      rotation2.isNear(rotation1_inverse, this->comparision_threshold));

    // Now get the numerical Jacobians
    Mat3 J_numerical(3, 3);
    // Perturbations are about zero vector in the tangent space.
    Vec3 perturbation_vec(0, 0, 0);

    InverseAndJacobianFunctor J_inverse_functor(rotation1);
    numerical_jacobian(J_inverse_functor, perturbation_vec, J_numerical);

    wave::MatX error_matrix = J_numerical - J_analytical;
    ASSERT_LE(error_matrix.norm(), this->comparision_threshold);
}

// Compare the logMapAndJacobian derivatives numerically.
TEST_F(RotationTestFixture, testLogMapAndJacobian) {
    // True angle axis params for rotation1.
    Vec3 rotation1_angleaxis_params(
      0.068924613882066, 0.213225926957886, 0.288748939228675);

    Mat3 J_analytical;
    Vec3 log_params =
      Rotation::logMapAndJacobian(this->rotation1_expected, J_analytical);
    Vec3 error = rotation1_angleaxis_params - log_params;
    ASSERT_LE(error.norm(), this->comparision_threshold);

    // Now get the numerical Jacobians
    Mat3 J_numerical(3, 3);
    // Perturbations are about zero vector in the tangent space.
    Vec3 perturbation_vec(0, 0, 0);

    // Jacobian wrt the rotation.
    LogMapAndJacobianFunctor J_log_functor(this->rotation1_expected);
    numerical_jacobian(J_log_functor, perturbation_vec, J_numerical);

    wave::MatX error_matrix = J_numerical - J_analytical;
    ASSERT_LE(error_matrix.norm(), this->comparision_threshold);
}

// Compare the manifoldMinusAndJacobian derivatives numerically.
TEST_F(RotationTestFixture, testManifoldMinusAndJacobian) {
    // We know that rotation3 = rotation1*rotation2
    // Therefore rotation3 * inv(rotation2) = rotation1
    // log(rotation3 * inv(rotation2)) = log(rotation1)
    // which is exactly rotation3 \boxminus rotation2.

    // True angle axis params for rotation1.
    Vec3 rotation1_angleaxis_params(
      0.068924613882066, 0.213225926957886, 0.288748939228675);
    Rotation rotation2(Vec3(-0.1, -0.2, -0.3));
    // rotation3 is the composition of rotation1 and rotation2.
    Rotation rotation3(
      Vec3(-0.065223037770317, 0.020616131403543, -0.013176146513587));

    Mat3 J_left_analytical, J_right_analytical;
    Vec3 log_params = rotation3.manifoldMinusAndJacobian(
      rotation2, J_left_analytical, J_right_analytical);
    Vec3 error = rotation1_angleaxis_params - log_params;
    ASSERT_LE(error.norm(), this->comparision_threshold);

    // Now get the numerical Jacobians
    Mat3 J_left_numerical(3, 3), J_right_numerical(3, 3);
    // Perturbations are about zero vector in the tangent space.
    Vec3 perturbation_vec(0, 0, 0);

    // Jacobian wrt the left rotation.
    ManifoldMinusAndJacobianJLeftFunctor J_left_functor(rotation3, rotation2);
    numerical_jacobian(J_left_functor, perturbation_vec, J_left_numerical);

    wave::MatX error_matrix = J_left_analytical - J_left_numerical;
    ASSERT_LE(error_matrix.norm(), this->comparision_threshold);

    // Jacobian wrt the right rotation.
    ManifoldMinusAndJacobianJRightFunctor J_right_functor(rotation3, rotation2);
    numerical_jacobian(J_right_functor, perturbation_vec, J_right_numerical);

    error_matrix = J_right_analytical - J_right_numerical;
    ASSERT_LE(error_matrix.norm(), this->comparision_threshold);
}

}  // namespace wave

