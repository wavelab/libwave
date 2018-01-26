#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/kinematics/constant_velocity_gp_prior.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/geometry/numerical_test_functors.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

class TransformationTestFixture : public ::testing::Test {
 public:
    Transformation transform_expected;
    Vec6 transformation_twist_parameters;
    double comparison_threshold = 1e-5;

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


TEST_F(TransformationTestFixture, testInterpolation) {
    Transformation T_kp1 = this->transform_expected;
    Vec6 vel_k, vel_kp1;
    // Jacobian approximations rely on small rotation
    vel_k << 0, 0, 0, 1, 0.2, -0.1;
    vel_kp1 = vel_k;
    T_kp1.manifoldPlus(vel_k);

    double tk = 0;
    double tkp1 = 1;
    double tau = 0.75;

    Mat6 Qc = Mat6::Identity();

    Eigen::Matrix<double, 12, 12> hat, candle;

    wave_kinematics::ConstantVelocityPrior prior(tk, tkp1, &tau, Qc);

    prior.calculateStuff(hat, candle);

    Mat6 J_T_k_num, J_T_kp1_num, J_vel_k_num, J_vel_kp1_num;
    Mat6 J_T_k_an, J_T_kp1_an, J_vel_k_an, J_vel_kp1_an;
    Vec6 perturbation_vec = Vec6::Zero();

    Vec6 increment;

    auto Tint = Transformation::interpolateAndJacobians(
      this->transform_expected, T_kp1, vel_k, vel_kp1, hat, candle, J_T_k_an, J_T_kp1_an, J_vel_k_an, J_vel_kp1_an);

    TInterpolatedJTLeftFunctor J_Tleft_functor(this->transform_expected, T_kp1, vel_k, vel_kp1, hat, candle);
    numerical_jacobian(J_Tleft_functor, perturbation_vec, J_T_k_num);

    TInterpolatedJTRightFunctor J_Tright_functor(this->transform_expected, T_kp1, vel_k, vel_kp1, hat, candle);
    numerical_jacobian(J_Tright_functor, perturbation_vec, J_T_kp1_num);

    TInterpolatedJVLeftFunctor J_Vleft_functor(this->transform_expected, T_kp1, vel_k, vel_kp1, hat, candle);
    numerical_jacobian(J_Vleft_functor, perturbation_vec, J_vel_k_num);

    TInterpolatedJVRightFunctor J_Vright_functor(this->transform_expected, T_kp1, vel_k, vel_kp1, hat, candle);
    numerical_jacobian(J_Vright_functor, perturbation_vec, J_vel_kp1_num);

    std::cout << "Tk analytical: " << std::endl;
    std::cout << J_T_k_an << std::endl;
    std::cout << "Tk numerical: " << std::endl;
    std::cout << J_T_k_num << std::endl;

    std::cout << "Tkp1 analytical: " << std::endl;
    std::cout << J_T_kp1_an << std::endl;
    std::cout << "Tkp1 numerical: " << std::endl;
    std::cout << J_T_kp1_num << std::endl;

    wave::MatX errmat = J_T_k_an - J_T_k_num;
    EXPECT_LE(errmat.norm(), this->comparison_threshold);

    errmat = J_T_kp1_an - J_T_kp1_num;
    EXPECT_LE(errmat.norm(), this->comparison_threshold);

    errmat = J_vel_k_an - J_vel_k_num;
    EXPECT_LE(errmat.norm(), this->comparison_threshold);

    errmat = J_vel_kp1_an - J_vel_kp1_num;
    EXPECT_LE(errmat.norm(), this->comparison_threshold);
}
}