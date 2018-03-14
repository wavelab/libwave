#include <ceres/gradient_checker.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "wave/wave_test.hpp"
#include "wave/utils/math.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/optimization/ceres/odom_gp/constant_velocity.hpp"
#include "wave/optimization/ceres/odom_gp_coupled_states/constant_velocity.hpp"
#include "wave/optimization/ceres/local_params/SE3Parameterization.hpp"

namespace {
/// Helper functions
Eigen::Matrix<double, 12, 12> calculateCVIntegrand(const wave::Mat6 &Qc,
                                                   const double &delta_T,
                                                   const wave::Vec6 &velocity) {
    Eigen::Matrix<double, 12, 12> retval;
    auto jacobian = wave::Transformation<void>::SE3LeftJacobian(delta_T * velocity, 1e-4);
    retval.block<6, 6>(0, 0) = delta_T * delta_T * jacobian * Qc * jacobian.transpose();
    retval.block<6, 6>(6, 0) = delta_T * jacobian.transpose();
    retval.block<6, 6>(0, 6) = delta_T * jacobian;
    retval.block<6, 6>(6, 6) = Qc;
    return retval;
}

void calculateTransitionMatrix(const double &delta_T,
                               const wave::Vec6 &velocity,
                               Eigen::Matrix<double, 12, 12> &transition_matrix) {
    transition_matrix.setIdentity();
    transition_matrix.block<6, 6>(0, 0) = wave::Transformation<void>::expMapAdjoint(delta_T * velocity, 1e-4);
    transition_matrix.block<6, 6>(0, 6) = delta_T * wave::Transformation<void>::SE3LeftJacobian(delta_T * velocity, 1e-4);
}

Eigen::Matrix<double, 12, 12> integrateCovariance(const wave::Mat6 &Qc,
                                                  const double &delta_T,
                                                  const wave::Vec6 &velocity,
                                                  const int &steps) {
    Eigen::Matrix<double, 12, 12> Qtotal;
    Qtotal.setZero();
    double step_size = delta_T / (double) steps;
    Eigen::Matrix<double, 12, 12> Qincremental;
    double step_delta_T;
    for (int i = 0; i <= steps; i++) {
        step_delta_T = delta_T - ((double) i) * step_size;
        Qincremental = calculateCVIntegrand(Qc, step_delta_T, velocity);
        if (i == 0 || i == steps) {
            Qtotal.noalias() += 0.5 * Qincremental;
        } else {
            Qtotal.noalias() += Qincremental;
        }
    }
    Qtotal = Qtotal * step_size;
    return Qtotal;
}
}

namespace wave {

/**
 * Due to hack to workaround Ceres local parameterization for speedup
 * the built-in gradient checker can't be used
 */
TEST(ConstantVelocity, Jacobians) {
    Transformation<Eigen::Matrix<double, 3, 4>> start, end;
    Mat4 t_matrix;
    t_matrix << 0.936293363584199, -0.275095847318244, 0.218350663146334, 1, 0.289629477625516, 0.956425085849232,
      -0.036957013524625, 2, -0.198669330795061, 0.097843395007256, 0.975170327201816, 3, 0, 0, 0, 1;

    start.setFromMatrix(t_matrix);
    end.setFromMatrix(t_matrix);

    const double delta_T = 0.1;
    Vec6 start_vel, end_vel;

    start_vel << -0, 0, 0.3, 20, 5, -0.1;
    end_vel = start_vel;

    end.manifoldPlus(delta_T * start_vel);

    // Calculate transition matrix and weight matrix for residual
    Mat6 Qc = Mat6::Identity();
    Eigen::Matrix<double, 12, 12> covariance, weight;
    covariance = integrateCovariance(Qc, delta_T, start_vel, 20);

    weight = Eigen::Matrix<double, 12, 12>::Identity();

    ceres::CostFunction *cost_function = new ConstantVelocityPrior(weight, delta_T);

    const double **parameters;
    parameters = new const double *[4];
    parameters[0] = start.storage.data();
    parameters[1] = end.storage.data();
    parameters[2] = start_vel.data();
    parameters[3] = end_vel.data();

    double *residuals;
    residuals = new double[12];

    double **jacobians;
    jacobians = new double *[4];
    jacobians[0] = new double[144];
    jacobians[1] = new double[144];
    jacobians[2] = new double[72];
    jacobians[3] = new double[72];

    cost_function->Evaluate(parameters, residuals, jacobians);

    Eigen::Matrix<double, 12, 1> residual_vec;
    residual_vec = Eigen::Map<Eigen::Matrix<double, 12, 1>>(residuals);

    EXPECT_NEAR(residual_vec.norm(), 0, 1e-6);

    Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> Jr_Ti(jacobians[0]);
    Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> Jr_Tip1(jacobians[1]);
    Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> Jr_Wi(jacobians[2]);
    Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> Jr_Wip1(jacobians[3]);

    const double step_size = 1.4901e-07;
    const double inv_step = 1.0 / step_size;
    Vec6 delta;
    delta.setZero();

    Transformation<> Tk_perturbed, Tkp1_perturbed;
    Vec6 vel_k_perturbed, vel_kp1_perturbed;

    Tk_perturbed.deepCopy(start);
    Tkp1_perturbed.deepCopy(end);
    vel_k_perturbed = start_vel;
    vel_kp1_perturbed = end_vel;

    parameters[0] = Tk_perturbed.storage.data();
    parameters[1] = Tkp1_perturbed.storage.data();
    parameters[2] = vel_k_perturbed.data();
    parameters[3] = vel_kp1_perturbed.data();

    Eigen::Matrix<double, 12, 1> diff, result;

    std::vector<Eigen::Matrix<double, 12, 6>> an_jacs, num_jacs;
    num_jacs.resize(4);
    an_jacs.resize(4);
    an_jacs.at(0) = Jr_Ti.block<12,6>(0,0);
    an_jacs.at(1) = Jr_Tip1.block<12,6>(0,0);
    an_jacs.at(2) = Jr_Wi;
    an_jacs.at(3) = Jr_Wip1;

    for (uint32_t i = 0; i < 6; i++) {
        delta(i) = step_size;
        // First parameter
        Tk_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(parameters, result.data(), nullptr);
        diff = result - residual_vec;
        num_jacs.at(0).block<12,1>(0,i) = inv_step * diff;
        Tk_perturbed.deepCopy(start);
        // Second parameter
        Tkp1_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(parameters, result.data(), nullptr);
        diff = result - residual_vec;
        num_jacs.at(1).block<12,1>(0,i) = inv_step * diff;
        Tkp1_perturbed.deepCopy(end);
        // Third parameter
        vel_k_perturbed = vel_k_perturbed + delta;
        cost_function->Evaluate(parameters, result.data(), nullptr);
        diff = result - residual_vec;
        num_jacs.at(2).block<12,1>(0,i) = inv_step * diff;
        vel_k_perturbed = start_vel;
        // Fourth parameter
        vel_kp1_perturbed = vel_kp1_perturbed + delta;
        cost_function->Evaluate(parameters, result.data(), nullptr);
        diff = result - residual_vec;
        num_jacs.at(3).block<12,1>(0,i) = inv_step * diff;
        vel_kp1_perturbed = end_vel;

        delta.setZero();
    }

    for (uint32_t i = 0; i < 4; i++) {
        double err = (num_jacs.at(i) - an_jacs.at(i)).norm();
        EXPECT_NEAR(err, 0.0, 1e-6);
        if (err > 1e-6) {
            std::cout << "Failed on index " << i << std::endl
                      << "Numerical: " << std::endl << num_jacs.at(i) << std::endl
                      << "Analytical:" << std::endl << an_jacs.at(i) << std::endl << std::endl;
        }
    }
}

TEST(ConstantVelocity, CoupledStates) {
    // Ceres needs a contiguous memory block for a single parameter. So supply one.
    double memblock[30];
    Eigen::Map<Vec6> vel(memblock);
    auto tk_ptr = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock + 6, 3, 4);
    auto tkp1_ptr = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock + 18, 3, 4);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> Tk(tk_ptr);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> Tkp1(tkp1_ptr);

    Mat4 t_matrix;
    t_matrix << 0.936293363584199, -0.275095847318244, 0.218350663146334, 1, 0.289629477625516, 0.956425085849232,
            -0.036957013524625, 2, -0.198669330795061, 0.097843395007256, 0.975170327201816, 3, 0, 0, 0, 1;

    Tk.setFromMatrix(t_matrix);
    Tkp1.setFromMatrix(t_matrix);

    const double delta_T = 0.1;

    vel << -0, 0, 0.3, 20, 5, -0.1;

    Tkp1.manifoldPlus(delta_T * vel);

    Eigen::Matrix<double, 6, 6> weight;

    weight.setIdentity();

    ceres::CostFunction *cost_function = new ConstantVelocityPriorCoupled<30>(weight, delta_T, 0);

    const double **parameters;
    parameters = new const double *[4];
    parameters[0] = memblock;

    double *residuals;
    residuals = new double[6];

    double **jacobians;
    jacobians = new double *[1];
    jacobians[0] = new double[180];  // 6 residuals * 30 dimension state

    cost_function->Evaluate(parameters, residuals, jacobians);

    Eigen::Matrix<double, 6, 1> residual_vec;
    residual_vec = Eigen::Map<Eigen::Matrix<double, 6, 1>>(residuals);

    EXPECT_NEAR(residual_vec.norm(), 0, 1e-6);

    Eigen::Map<Eigen::Matrix<double, 6, 30, Eigen::RowMajor>> J_total(jacobians[0]);

    const double step_size = 1.4901e-07;
    const double inv_step = 1.0 / step_size;
    Vec6 delta;
    delta.setZero();

    double memblock_perturb[30];

    Eigen::Map<Vec6> vel_perturbed(memblock_perturb);
    auto tk_ptr_perturb = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock_perturb + 6, 3, 4);
    auto tkp1_ptr_perturb = std::make_shared<Eigen::Map<Eigen::Matrix<double, 3, 4>>>(memblock_perturb + 18, 3, 4);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> Tk_perturbed(tk_ptr_perturb);
    Transformation<Eigen::Map<Eigen::Matrix<double, 3, 4>>> Tkp1_perturbed(tkp1_ptr_perturb);

    Tk_perturbed.deepCopy(Tk);
    Tkp1_perturbed.deepCopy(Tkp1);
    vel_perturbed = vel;

    Eigen::Matrix<double, 6, 1> diff, result;

    std::vector<Eigen::Matrix<double, 6, 6>> an_jacs, num_jacs;
    num_jacs.resize(3);
    an_jacs.resize(3);
    an_jacs.at(0) = J_total.block<6,6>(0,0);
    an_jacs.at(1) = J_total.block<6,6>(0,6);
    an_jacs.at(2) = J_total.block<6,6>(0,18);

    parameters[0] = memblock_perturb;
    for (uint32_t i = 0; i < 6; i++) {
        delta(i) = step_size;
        // First parameter
        vel_perturbed = vel_perturbed + delta;
        cost_function->Evaluate(parameters, result.data(), nullptr);
        diff = result - residual_vec;
        num_jacs.at(0).block<6,1>(0,i) = inv_step * diff;
        vel_perturbed = vel;

        // Second parameter
        Tk_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(parameters, result.data(), nullptr);
        diff = result - residual_vec;
        num_jacs.at(1).block<6,1>(0,i) = inv_step * diff;
        Tk_perturbed.deepCopy(Tk);

        // Third parameter
        Tkp1_perturbed.manifoldPlus(delta);
        cost_function->Evaluate(parameters, result.data(), nullptr);
        diff = result - residual_vec;
        num_jacs.at(2).block<6,1>(0,i) = inv_step * diff;
        Tkp1_perturbed.deepCopy(Tkp1);

        delta.setZero();
    }

    for (uint32_t i = 0; i < 3; i++) {
        double err = (num_jacs.at(i) - an_jacs.at(i)).norm();
        EXPECT_NEAR(err, 0.0, 1e-6);
        if (err > 1e-6) {
            std::cout << "Failed on index " << i << std::endl
                      << "Numerical: " << std::endl << num_jacs.at(i) << std::endl
                      << "Analytical:" << std::endl << an_jacs.at(i) << std::endl << std::endl;
        }
    }
}

}
