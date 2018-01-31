#include <ceres/gradient_checker.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "wave/wave_test.hpp"
#include "wave/utils/math.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/optimization/ceres/constant_velocity.hpp"
#include "wave/optimization/ceres/SE3Parameterization.hpp"

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
 * this test only checks against values from Matlab implementation
 */
TEST(ConstantVelocity, Jacobians) {
    Transformation<Eigen::Matrix<double, 3, 4>> start, end;
    Mat4 t_matrix;
    t_matrix << 0.936293363584199, -0.275095847318244, 0.218350663146334, 1, 0.289629477625516, 0.956425085849232,
      -0.036957013524625, 2, -0.198669330795061, 0.097843395007256, 0.975170327201816, 3, 0, 0, 0, 1;

    start.setFromMatrix(t_matrix);
    end.setFromMatrix(t_matrix);

    const double delta_T = 1;
    Vec6 start_vel, end_vel;

    start_vel << 0, 0, 0, 1, 0.2, -0.1;
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
    parameters[0] = start.getInternalMatrix().derived().data();
    parameters[1] = end.getInternalMatrix().derived().data();
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

    Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> Jr_Ti(jacobians[0]);
    Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> Jr_Tip1(jacobians[1]);
    Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> Jr_Wi(jacobians[2]);
    Eigen::Map<Eigen::Matrix<double, 12, 6, Eigen::RowMajor>> Jr_Wip1(jacobians[3]);

    /// Known values
    Eigen::Matrix<double, 12, 6> Jr_Ti_known, Jr_Tip1_known, Jr_Wi_known, Jr_Wip1_known;
    Jr_Ti_known << -1, -2.77555756156289e-16, 2.46330733588707e-16, 0, 0, 0, -2.77555756156289e-16, -0.999999999999999,
      -2.91433543964104e-16, 0, 0, 0, 2.46330733588707e-16, -2.91433543964104e-16, -1, 0, 0, 0, 1.37737043992558e-15,
      -0.050000000000001, -0.100000000000001, -1, -2.77555756156289e-16, 2.46330733588707e-16, 0.0499999999999992,
      -1.17961196366423e-16, 0.500000000000001, -2.77555756156289e-16, -0.999999999999999, -2.91433543964104e-16,
      0.0999999999999996, -0.499999999999998, -9.5062846483529e-16, 2.46330733588707e-16, -2.91433543964104e-16, -1, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.07552855510562e-17, -0.05, -0.1, 0, 0, 0, 0.0499999999999999,
      1.59594559789866e-16, 0.5, 0, 0, 0, 0.0999999999999998, -0.5, -1.70349845340922e-16, 0, 0, 0;

    Jr_Tip1_known << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -0.0500000000000003, -0.1, 1, 0, 0,
      0.0500000000000003, 0, 0.5, 0, 1, 0, 0.1, -0.5, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0.05, 0.1, 0, 0, 0, -0.05, 0, -0.5, 0, 0, 0, -0.1, 0.5, 0, 0, 0, 0;

    Jr_Wi_known << -1, -0, -0, -0, -0, -0, -0, -1, -0, -0, -0, -0, -0, -0, -1, -0, -0, -0, -0, -0, -0, -1, -0, -0, -0,
      -0, -0, -0, -1, -0, -0, -0, -0, -0, -0, -1, -1, -0, -0, -0, -0, -0, -0, -1, -0, -0, -0, -0, -0, -0, -1, -0, -0,
      -0, -0, -0, -0, -1, -0, -0, -0, -0, -0, -0, -1, -0, -0, -0, -0, -0, -0, -1;

    Jr_Wip1_known << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, -0.0500000000000003, -0.1, 1, 0, 0,
      0.0500000000000003, 0, 0.5, 0, 1, 0, 0.1, -0.5, 0, 0, 0, 1;

    Eigen::Matrix<double, 12, 1> residual_vec;
    residual_vec = Eigen::Map<Eigen::Matrix<double, 12, 1>>(residuals);

    EXPECT_NEAR(residual_vec.norm(), 0, 1e-6);

    Eigen::Matrix<double, 12, 6> errmat;

    errmat = Jr_Ti.block<12,6>(0,0) - Jr_Ti_known;
    EXPECT_NEAR(errmat.norm(), 0, 1e-6);
    errmat = Jr_Tip1.block<12,6>(0,0) - Jr_Tip1_known;
    EXPECT_NEAR(errmat.norm(), 0, 1e-6);
    errmat = Jr_Wi - Jr_Wi_known;
    EXPECT_NEAR(errmat.norm(), 0, 1e-6);
    errmat = Jr_Wip1 - Jr_Wip1_known;
    EXPECT_NEAR(errmat.norm(), 0, 1e-6);
}
}
