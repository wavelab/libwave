#include "wave/optimization/ceres/ba.hpp"

namespace wave {

int BundleAdjustment::addCamera(const Mat3 &K,
                                const MatX &features,
                                const VecX &landmark_ids,
                                double *cam_t,
                                double *cam_q,
                                double **landmarks) {
    // create a residual block for each image feature
    for (int i = 0; i < features.rows(); i++) {
        // build residual
        Vec2 feature{features(i, 0), features(i, 1)};
        auto residual = new BAResidual(K, feature);

        // build cost function
        auto cost_func = new ceres::AutoDiffCostFunction<
          BAResidual,  // Residual type
          2,           // size of residual
          4,           // size of 1st parameter - quaternion
          3,           // size of 2nd parameter - camera center (x, y, z)
          3            // size of 3rd parameter - 3d point in world (x, y, z)
          >(residual);

        // add residual block to problem
        this->problem.AddResidualBlock(
          cost_func,                          // cost function
          NULL,                               // loss function
          cam_q,                              // camera quaternion
          cam_t,                              // camera translation
          landmarks[(int) landmark_ids(i)]);  // landmark
    }

    // add quaternion local parameterization
    ceres::LocalParameterization *quat_param;
    quat_param = new ceres::EigenQuaternionParameterization();
    this->problem.SetParameterization(cam_q, quat_param);

    return 0;
}

int BundleAdjustment::solve() {
    // set options
    this->options.max_num_iterations = 200;
    this->options.use_nonmonotonic_steps = false;
    this->options.use_inner_iterations = true;
    this->options.preconditioner_type = ceres::SCHUR_JACOBI;
    this->options.linear_solver_type = ceres::SPARSE_SCHUR;
    this->options.parameter_tolerance = 1e-10;
    this->options.num_threads = 8;
    this->options.num_linear_solver_threads = 8;
    this->options.minimizer_progress_to_stdout = true;

    // solve
    ceres::Solve(this->options, &this->problem, &this->summary);
    std::cout << summary.FullReport() << "\n";

    return 0;
}

}  // namespace wave
