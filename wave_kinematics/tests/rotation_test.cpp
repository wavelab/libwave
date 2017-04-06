#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/kinematics/numerical_test_functors.hpp"
#include "wave/kinematics/rotation.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

class RotationTestFixture : public ::testing::Test {
 public:
    // Data members used for testing.
    Rotation rotation1;
    Rotation rotation1_truth;
    Rotation rotation2;
    Rotation rotation3;
    Rotation rotation1_inverse;
    Rotation eye3;

    // A vector v1.
    Vec3 v1;

    // v1 rotated by rotation1.
    Vec3 v1_rotated;

    // True angle axis params for rotation1.
    Vec3 rotation1_angleaxis_params;

    void SetUp() {
        Rotation::fromEulerXYZ(0.1, 0.2, 0.3, rotation1);
        Rotation::fromEulerXYZ(-0.1, -0.2, -0.3, rotation2);

        // rotation3 is the composition of rotation1 and rotation2.
        Rotation::fromEulerXYZ(-0.065223037770317,
                               0.020616131403543,
                               -0.013176146513587,
                               rotation3);

        // rotation1_inverse is the inverse of rotation1.
        Rotation::fromEulerXYZ(-0.037879880513201,
                               -0.220124031212965,
                               -0.285771700628461,
                               rotation1_inverse);
        eye3.rotation_object.setMatrix(1, 0, 0, 0, 1, 0, 0, 0, 1);

        // rotation1_true is set using the underlying element wise
        // initializer.
        // It is the expected rotation matrix, using the parameters from
        // rotation1.

        rotation1_truth.rotation_object.setMatrix(0.936293363584199,
                                                  -0.275095847318244,
                                                  0.218350663146334,
                                                  0.289629477625516,
                                                  0.956425085849232,
                                                  -0.036957013524625,
                                                  -0.198669330795061,
                                                  0.097843395007256,
                                                  0.975170327201816);


        v1 << 1, 1, 1;
        v1_rotated << 0.879548179412290, 1.209097549950123, 0.874344391414010;
        rotation1_angleaxis_params << 0.068924613882066, 0.213225926957886,
          0.288748939228675;
    }
};


// Construction tests.

// Test that fromEulerXYZ returns the correct matrix.
TEST_F(RotationTestFixture, test_from_euler_xyz) {
    ASSERT_TRUE(rotation1_truth.isNear(rotation1));
}


// Test that fromAngleAxis returns the correct matrix.
TEST_F(RotationTestFixture, test_from_angle_axis) {
    Rotation rotation_from_angle_axis;
    Rotation::fromAngleAxis(rotation1_angleaxis_params.norm(),
                            rotation1_angleaxis_params.normalized(),
                            rotation_from_angle_axis);
    ASSERT_TRUE(rotation1.isNear(rotation_from_angle_axis));
}
// Test that fromExpMap returns the correct matrix.
TEST_F(RotationTestFixture, test_from_exp_map) {
    Rotation rotation_from_exp_map;
    Rotation::fromExpMap(rotation1_angleaxis_params, rotation_from_exp_map);
    ASSERT_TRUE(rotation1.isNear(rotation_from_exp_map));
}

// Test that logMap returns the correct parameters.
TEST_F(RotationTestFixture, test_log_map) {
    Vec3 se3_params = rotation1.logMap();
    Vec3 error = rotation1_angleaxis_params - se3_params;
    ASSERT_LE(error.norm(), Rotation::comparision_threshold);
}

// Test to ensure default constructor produces identity.
TEST_F(RotationTestFixture, test_default_constructor) {
    Rotation R;
    ASSERT_TRUE(R.isNear(eye3));
}

// Test to ensure setToIdentity returns identity.
TEST_F(RotationTestFixture, test_set_to_identity) {
    // First make sure the rotation is not already identity.
    ASSERT_FALSE(rotation1.isNear(eye3));
    rotation1.setToIdentity();
    ASSERT_TRUE(rotation1.isNear(eye3));
}

// Test that rotation composition returns expected product.
TEST_F(RotationTestFixture, test_compose) {
    rotation1.compose(rotation2);
    ASSERT_TRUE(rotation1.isNear(rotation3));
}

// Test that rotation inverse returns correct matrix.
TEST_F(RotationTestFixture, test_inverse) {
    rotation1.invert();
    ASSERT_TRUE(rotation1.isNear(rotation1_inverse));
}

// Test that rotation of a vector returns correct value.
TEST_F(RotationTestFixture, test_rotate) {
    Vec3 v1_rotated_test = rotation1.rotate(v1);
    Vec3 error = v1_rotated_test - v1_rotated;
    ASSERT_LE(error.norm(), Rotation::comparision_threshold);
}

// Test that inverse rotation of a vector returns correct value.
TEST_F(RotationTestFixture, test_inverse_rotate) {
    Vec3 v1_inverse_rotated_test = rotation1_inverse.inverseRotate(v1);
    Vec3 error = v1_inverse_rotated_test - v1_rotated;
    ASSERT_LE(error.norm(), Rotation::comparision_threshold);
}

// Test functionality of the boxplus operator.
TEST_F(RotationTestFixture, test_manifold_plus) {
    // We know that rotation3 = rotation1*rotation2
    // Therefore rotation3 = exp(rotation1_angleaxis_params)*rotation2,
    // which is exactly the boxplus operation.

    // Make sure rotation2 and rotation3 are not the same initially
    ASSERT_FALSE(rotation2.isNear(rotation3));

    // Use manifold plus to make rotation2==rotation3.
    rotation2.manifoldPlus(rotation1_angleaxis_params);
    ASSERT_TRUE(rotation2.isNear(rotation3));
}

// JACOBIAN TESTING
// Compare the rotateAndJacobian derivatives numerically.
TEST_F(RotationTestFixture, test_rotate_and_jacobian) {
    // First get the analytical Jacobians.
    Mat3 Jparam_analytical, Jpoint_analytical;
    rotation1.rotateAndJacobian(v1, Jpoint_analytical, Jparam_analytical);

    // Now get the numerical Jacobians
    Mat3 Jparam_numerical(3, 3), Jpoint_numerical(3, 3);

    // Jacobian wrt the point.
    RotateAndJacobianJpointFunctor Jpoint_functor(rotation1);
    numerical_jacobian(Jpoint_functor, v1, Jpoint_numerical);
    Eigen::MatrixXd error_matrix = Jpoint_analytical - Jpoint_numerical;
    ASSERT_LE(error_matrix.norm(), Rotation::comparision_threshold);

    // Jacobian wrt the rotation params.
    RotateAndJacobianJparamFunctor Jparam_functor(rotation1, v1);

    // Perturbations are about zero vector in the tangent space.
    Vec3 perturbation_vec(0, 0, 0);
    numerical_jacobian(Jparam_functor, perturbation_vec, Jparam_numerical);
    error_matrix = Jparam_analytical - Jparam_numerical;
    ASSERT_LE(error_matrix.norm(), Rotation::comparision_threshold);
}

}  // end of wave namespace

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
