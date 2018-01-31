#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/kinematics/constant_velocity_gp_prior.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/geometry/numerical_test_functors.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

namespace {
using T_Type = Transformation<Eigen::Matrix<double, 3, 4>>;
}

class TransformationTestFixture : public ::testing::Test {
 public:
    T_Type transform_expected;
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
    T_Type T_k, T_kp1;

    Vec6 vel_k, vel_kp1;
    // Jacobian approximations rely on small rotation
    vel_k << 0.0, -0.0, 0.0, 5, 1, -1;
    vel_kp1 = vel_k;

    double tk = 0;
    double tkp1 = 0.5;
    double tau = 0.34;

    T_k.setIdentity();
    T_kp1.setIdentity();

    T_kp1.manifoldPlus(tkp1 * vel_k);

    Mat6 Qc = Mat6::Identity();

    Eigen::Matrix<double, 12, 12> hat, candle;

    wave_kinematics::ConstantVelocityPrior prior(tk, tkp1, &tau, Qc);

    prior.calculateStuff(hat, candle);

    Mat6 J_T_k_num, J_T_kp1_num, J_vel_k_num, J_vel_kp1_num;
    Mat6 J_T_k_an, J_T_kp1_an, J_vel_k_an, J_vel_kp1_an;
    Vec6 perturbation_vec = Vec6::Zero();

    Vec6 increment;

    auto Tint = T_Type::interpolateAndJacobians(
      T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0), J_T_k_an, J_T_kp1_an, J_vel_k_an, J_vel_kp1_an);

    TInterpolatedJTLeftFunctor J_Tleft_functor(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));
    numerical_jacobian(J_Tleft_functor, perturbation_vec, J_T_k_num);
    perturbation_vec = Vec6::Zero();

    TInterpolatedJTRightFunctor J_Tright_functor(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));
    numerical_jacobian(J_Tright_functor, perturbation_vec, J_T_kp1_num);
    perturbation_vec = Vec6::Zero();

    TInterpolatedJVLeftFunctor J_Vleft_functor(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));
    numerical_jacobian(J_Vleft_functor, perturbation_vec, J_vel_k_num);
    perturbation_vec = Vec6::Zero();

    TInterpolatedJVRightFunctor J_Vright_functor(T_k, T_kp1, vel_k, vel_kp1, hat.block<6,12>(0,0), candle.block<6,12>(0,0));
    numerical_jacobian(J_Vright_functor, perturbation_vec, J_vel_kp1_num);

    wave::MatX errmat = J_T_k_an - J_T_k_num;
    if(errmat.norm() > this->comparison_threshold) {
        std::cout << J_T_k_an << std::endl;
        std::cout << J_T_k_num << std::endl;
        EXPECT_TRUE(false);
    }

    errmat = J_T_kp1_an - J_T_kp1_num;
    if(errmat.norm() > this->comparison_threshold) {
        std::cout << J_T_kp1_an << std::endl;
        std::cout << J_T_kp1_num << std::endl;
        EXPECT_TRUE(false);
    }

    errmat = J_vel_k_an - J_vel_k_num;
    if(errmat.norm() > this->comparison_threshold) {
        std::cout << J_vel_k_an << std::endl;
        std::cout << J_vel_k_num << std::endl;
        EXPECT_TRUE(false);
    }

    errmat = J_vel_kp1_an - J_vel_kp1_num;
    if(errmat.norm() > this->comparison_threshold) {
        std::cout << J_vel_kp1_an << std::endl;
        std::cout << J_vel_kp1_num << std::endl;
        EXPECT_TRUE(false);
    }
}
}