#include <ceres/ceres.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "wave/optimization/ceres/odom_gp/point_to_plane_gp.hpp"
#include "wave/optimization/ceres/odom_gp/point_to_line_gp.hpp"
#include "wave/wave_test.hpp"
#include "wave/utils/math.hpp"
#include "wave/geometry_og/transformation.hpp"
#include "wave/kinematics/constant_velocity_gp_prior.hpp"

// This is a numerical check for some residuals where ceres gradient checker is not helpful

namespace wave {

TEST(point_to_line, jacobian) {

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double pt[3] = {1, 2, -4};

    const double delta_T = 0.5;
    const double **params;
    params = new const double *[4];

    Transformation<Eigen::Matrix<double, 3, 4>, false> T_k, T_kp1;
    Vec6 vel_k, vel_kp1;

    params[0] = T_k.storage.data();
    params[1] = T_kp1.storage.data();
    params[2] = vel_k.data();
    params[3] = vel_kp1.data();

    T_k.setIdentity();
    vel_k << 0.1, -0.1, 0.1, 5, 1, -1;
    vel_kp1 = vel_k;
    T_kp1 = T_k;
    T_kp1.manifoldPlus(delta_T * vel_k);

    double zero = 0;
    double tau = 0.34;
    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    wave_kinematics::ConstantVelocityPrior motion_prior(zero, delta_T, &tau, Qc, inv_Qc);

    Eigen::Matrix<double, 12, 12> hat, candle;

    SE3PointToLineGPObjects objects;

    motion_prior.calculateStuff(hat, candle);

    objects.hat = hat;
    objects.candle = candle;

    ceres::CostFunction *cost_function = new SE3PointToLineGP(pt,
                                                              ptA,
                                                              ptB,
                                                              objects,
                                                              Mat3::Identity(),
                                                              true);
    double **jacobian;
    jacobian = new double *[4];
    jacobian[0] = new double[24];
    jacobian[1] = new double[24];
    jacobian[2] = new double[12];
    jacobian[3] = new double[12];

    Vec2 op_result;

    cost_function->Evaluate(params, op_result.data(), jacobian);

    double const step_size = 1e-9;
    Transformation<Eigen::Matrix<double, 3, 4>, false> Tk_perturbed, Tkp1_perturbed;
    Vec6 vel_k_perturbed, vel_kp1_perturbed;

    Tk_perturbed = T_k;
    Tkp1_perturbed = T_kp1;
    vel_k_perturbed = vel_k;
    vel_kp1_perturbed = vel_kp1;

    params[0] = Tk_perturbed.storage.data();
    params[1] = Tkp1_perturbed.storage.data();
    params[2] = vel_k_perturbed.data();
    params[3] = vel_kp1_perturbed.data();

    std::vector<Eigen::Matrix<double, 2, 6>> an_jacs, num_jacs;
    an_jacs.resize(4);
    num_jacs.resize(4);

    Vec6 delta;
    delta.setZero();

    Vec2 result;
    Vec2 diff;

    double inv_step = 1.0 / step_size;

    for (uint32_t i = 0; i < 6; i++) {
        delta(i) = step_size;
        // First parameter
        Tk_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(0).block<2,1>(0,i) = inv_step * diff;
        Tk_perturbed = (T_k);
        // Second parameter
        Tkp1_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(1).block<2,1>(0,i) = inv_step * diff;
        Tkp1_perturbed = (T_kp1);
        // Third parameter
        vel_k_perturbed = vel_k_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(2).block<2,1>(0,i) = inv_step * diff;
        vel_k_perturbed = vel_k;
        // Fourth parameter
        vel_kp1_perturbed = vel_kp1_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(3).block<2,1>(0,i) = inv_step * diff;
        vel_kp1_perturbed = vel_kp1;

        delta.setZero();
    }

    // now get the analytical jacobians
    Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> an_jac_1(jacobian[0], 2, 12);
    an_jacs.at(0) = an_jac_1.block<2,6>(0,0);

    Eigen::Map<Eigen::Matrix<double, 2, 12, Eigen::RowMajor>> an_jac_2(jacobian[1], 2, 12);
    an_jacs.at(1) = an_jac_2.block<2,6>(0,0);

    Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> an_jac_3(jacobian[2], 2, 6);
    an_jacs.at(2) = an_jac_3;

    Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> an_jac_4(jacobian[3], 2, 6);
    an_jacs.at(3) = an_jac_4;

    for (uint32_t i = 0; i < 4; i++) {
        double err = (num_jacs.at(i) - an_jacs.at(i)).norm();
        if (err > 1e-10) {
            std::cout << "Index " << i << " with error = " << err << std::endl
                     << "Numerical: " << std::endl << num_jacs.at(i) << std::endl
                     << "Analytical:" << std::endl << an_jacs.at(i) << std::endl << std::endl;
        }
        EXPECT_NEAR(err, 0.0, 1e-6);
    }
}

TEST(point_to_plane, jacobian) {

    double ptA[3] = {1, 1, 0};
    double ptB[3] = {1, 3, -4};
    double ptC[3] = {4, -1, 0};
    double pt[3] = {1, 2, -4};

    const double delta_T = 0.5;
    const double **params;
    params = new const double *[4];

    Transformation<Eigen::Matrix<double, 3, 4>, false> T_k, T_kp1;
    Vec6 vel_k, vel_kp1;

    params[0] = T_k.storage.data();
    params[1] = T_kp1.storage.data();
    params[2] = vel_k.data();
    params[3] = vel_kp1.data();

    T_k.setIdentity();
    vel_k << 0.1, -0.1, 0.2, 5, 1, -1;
    vel_kp1 = vel_k;
    T_kp1 = (T_k);
    T_kp1.manifoldPlus(delta_T * vel_k);

    double zero = 0;
    double tau = 0.34;
    Mat6 Qc = Mat6::Identity();
    Mat6 inv_Qc = Qc.inverse();

    wave_kinematics::ConstantVelocityPrior motion_prior(zero, delta_T, &tau, Qc, inv_Qc);

    Eigen::Matrix<double, 12, 12> hat, candle;

    motion_prior.calculateStuff(hat, candle);

    SE3PointToPlaneGPObjects objects;
    objects.hat = hat;
    objects.candle = candle;

    ceres::CostFunction *cost_function = new SE3PointToPlaneGP(pt,
                                                              ptA,
                                                              ptB,
                                                              ptC,
                                                              objects,
                                                              Mat3::Identity(),
                                                              false);
    double **jacobian;
    jacobian = new double *[4];
    jacobian[0] = new double[12];
    jacobian[1] = new double[12];
    jacobian[2] = new double[6];
    jacobian[3] = new double[6];

    Eigen::Matrix<double, 1, 1> op_result;

    cost_function->Evaluate(params, op_result.data(), jacobian);

    double const step_size = 1e-9;
    Transformation<Eigen::Matrix<double, 3, 4>> Tk_perturbed, Tkp1_perturbed;
    Vec6 vel_k_perturbed, vel_kp1_perturbed;

    Tk_perturbed = (T_k);
    Tkp1_perturbed = (T_kp1);
    vel_k_perturbed = vel_k;
    vel_kp1_perturbed = vel_kp1;

    params[0] = Tk_perturbed.storage.data();
    params[1] = Tkp1_perturbed.storage.data();
    params[2] = vel_k_perturbed.data();
    params[3] = vel_kp1_perturbed.data();

    std::vector<Eigen::Matrix<double, 1, 6>> an_jacs, num_jacs;
    an_jacs.resize(4);
    num_jacs.resize(4);

    Vec6 delta;
    delta.setZero();

    Eigen::Matrix<double, 1, 1> diff, result;

    double inv_step = 1.0 / step_size;

    for (uint32_t i = 0; i < 6; i++) {
        delta(i) = step_size;
        // First parameter
        Tk_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(0).block<1,1>(0,i) = inv_step * diff;
        Tk_perturbed = (T_k);
        // Second parameter
        Tkp1_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(1).block<1,1>(0,i) = inv_step * diff;
        Tkp1_perturbed = (T_kp1);
        // Third parameter
        vel_k_perturbed = vel_k_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(2).block<1,1>(0,i) = inv_step * diff;
        vel_k_perturbed = vel_k;
        // Fourth parameter
        vel_kp1_perturbed = vel_kp1_perturbed + delta;
        cost_function->Evaluate(params, result.data(), nullptr);
        diff = result - op_result;
        num_jacs.at(3).block<1,1>(0,i) = inv_step * diff;
        vel_kp1_perturbed = vel_kp1;

        delta.setZero();
    }

    // now get the analytical jacobians
    Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> an_jac_1(jacobian[0], 1, 12);
    an_jacs.at(0) = an_jac_1.block<1,6>(0,0);

    Eigen::Map<Eigen::Matrix<double, 1, 12, Eigen::RowMajor>> an_jac_2(jacobian[1], 1, 12);
    an_jacs.at(1) = an_jac_2.block<1,6>(0,0);

    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> an_jac_3(jacobian[2], 1, 6);
    an_jacs.at(2) = an_jac_3;

    Eigen::Map<Eigen::Matrix<double, 1, 6, Eigen::RowMajor>> an_jac_4(jacobian[3], 1, 6);
    an_jacs.at(3) = an_jac_4;

    for (uint32_t i = 0; i < 4; i++) {
        double err = (num_jacs.at(i) - an_jacs.at(i)).norm();
        std::cout << "Index " << i << " has error: " << err << std::endl
                  << "Numerical: " << std::endl << num_jacs.at(i) << std::endl
                  << "Analytical:" << std::endl << an_jacs.at(i) << std::endl << std::endl;
        EXPECT_NEAR(err, 0.0, 1e-6);
    }
}

}