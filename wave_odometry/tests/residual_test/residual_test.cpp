#include <ceres/solver.h>
#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>

#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/implicit_geometry/implicit_line.hpp"
#include "wave/odometry/implicit_geometry/implicit_plane.hpp"
#include "wave/optimization/ceres/local_params/line_parameterization.hpp"
#include "wave/optimization/ceres/local_params/plane_parameterization.hpp"
#include "wave/utils/math.hpp"
#include "wave/wave_test.hpp"

struct TestEvalCallback : ceres::EvaluationCallback {
    explicit TestEvalCallback(wave::Vec3 *state_point,
                              std::vector<std::vector<Eigen::Map<wave::MatXf>>> *feat_points) :
            ceres::EvaluationCallback(), state_point(state_point), feat_points(feat_points) {}

    virtual void PrepareForEvaluation(bool, bool) {
        feat_points->at(0).at(0).block<3, 1>(0, 2) = state_point->cast<float>();
    }
    wave::Vec3 *state_point;
    std::vector<std::vector<Eigen::Map<wave::MatXf>>> *feat_points;
};

namespace wave {

TEST(implicit_line, simple) {
    std::vector<std::vector<Eigen::Map<MatXf>>> feat_points;
    Vec<Vec<VecE<Eigen::Tensor<double, 3>>>> jacs;

    uint32_t n_pts = 3;

    // one "scan"
    feat_points.resize(1);
    jacs.resize(1);
    // one "feature type"
    MatXf feature_points;
    feature_points.resize(3, n_pts);
    feat_points.at(0).emplace_back(Eigen::Map<MatXf>(feature_points.data(), feature_points.rows(), feature_points.cols()));
    jacs.at(0).resize(1);

    // one "state"
    jacs.at(0).at(0).resize(1);
    jacs.at(0).at(0).at(0) = Eigen::Tensor<double, 3>(3, 3, n_pts);
    jacs.at(0).at(0).at(0).setZero();

    FeatureTrack track;
    track.mapping.resize(n_pts);
    track.mapping.resize(n_pts);
//    track.featT_idx = 0;

    track.state_ids.resize(n_pts);

    Vec3 state_point;
    for (uint32_t i = 0; i < n_pts; i++) {
        track.mapping.at(i).pt_idx = i;
        track.mapping.at(i).scan_idx = 0;

        track.state_ids.at(i).emplace_back(0);
        if (i == 2) {
            jacs.at(0).at(0).at(0)(0,0,i) = 1.0;
            jacs.at(0).at(0).at(0)(1,1,i) = 1.0;
            jacs.at(0).at(0).at(0)(2,2,i) = 1.0;

            feat_points.at(0).at(0).block<3, 1>(0, i) = Vec3f::Random();
        } else {
            Vec3f temp;
            temp << 0.0, 0.0, 2 * static_cast<float>(i);
            feat_points.at(0).at(0).block<3, 1>(0, i) = temp;
        }
    }
    state_point = feat_points.at(0).at(0).block<3, 1>(0, 2).cast<double>();

    track.geometry.block<3, 1>(0,0) = Vec3::Random();
    if (track.geometry(2) < 0) {
        track.geometry.block<3, 1>(0,0) *= -1.0;
    }
    track.geometry.block<3, 1>(0,0).normalize();
    track.geometry.block<3, 1>(3,0).setZero();

    ceres::Problem::Options options;
    options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(options);

    track.jacs = &(jacs.at(0));

    std::list<ImplicitLineResidual<3>> costs;
    for (uint32_t i = 0; i < 3; ++i) {
        costs.emplace_back(i, 0, &track, &feat_points);
    }

    for (auto &cost : costs) {
        problem.AddResidualBlock(&cost, nullptr, track.geometry.data(), state_point.data());
    }

    ceres::LocalParameterization *local_param = new LineParameterization();
    problem.SetParameterization(track.geometry.data(), local_param);

    TestEvalCallback callback(&state_point, &feat_points);

    ceres::Solver::Options solve_options;
    solve_options.evaluation_callback = &callback;
    solve_options.use_nonmonotonic_steps = true;

    ceres::Solver::Summary summary;
    ceres::Solve(solve_options, &problem, &summary);

    std::cout << "\n" << summary.FullReport() << "\n";

    /// Checking that all points are on the line defined by geometry in track
    std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>> normals;
    normals.emplace_back(feat_points.at(0).at(0).block<3, 1>(0, 0) - track.geometry.block<3, 1>(3,0).cast<float>());
    normals.emplace_back(feat_points.at(0).at(0).block<3, 1>(0, 1) - track.geometry.block<3, 1>(3,0).cast<float>());
    normals.emplace_back(feat_points.at(0).at(0).block<3, 1>(0, 2) - track.geometry.block<3, 1>(3,0).cast<float>());

    for (auto &dir : normals) {
        dir.normalize();
        float dot = dir.transpose() * track.geometry.block<3, 1>(0,0).cast<float>();
        EXPECT_NEAR(std::abs(dot), 1.0, 1.0e-6);
    }
}

TEST(implicit_plane, simple) {
    std::vector<std::vector<Eigen::Map<MatXf>>> feat_points;
    Vec<Vec<VecE<Eigen::Tensor<double, 3>>>> jacs;

    uint32_t n_pts = 4;

    // one "scan"
    feat_points.resize(1);
    jacs.resize(1);
    // one "feature type"
    MatXf feature_points;
    feature_points.resize(3, n_pts);
    feat_points.at(0).emplace_back(Eigen::Map<MatXf>(feature_points.data(), feature_points.rows(), feature_points.cols()));
    jacs.at(0).resize(1);

    // one state
    jacs.at(0).at(0).resize(1);
    jacs.at(0).at(0).at(0) = Eigen::Tensor<double, 3>(3, 3, n_pts);
    jacs.at(0).at(0).at(0).setZero();

    FeatureTrack track;
    track.mapping.resize(n_pts);
    track.mapping.resize(n_pts);
//    track.featT_idx = 0;

    track.state_ids.resize(n_pts);

    Vec3 state_point;
    for (uint32_t i = 0; i < n_pts; i++) {
        track.mapping.at(i).pt_idx = i;
        track.mapping.at(i).scan_idx = 0;

        track.state_ids.at(i).emplace_back(0);
        if (i == 2) {
            jacs.at(0).at(0).at(0)(0,0,i) = 1.0;
            jacs.at(0).at(0).at(0)(1,1,i) = 1.0;
            jacs.at(0).at(0).at(0)(2,2,i) = 1.0;
            feat_points.at(0).at(0).block<3, 1>(0, i) = Vec3f::Random();
        } else {
            feat_points.at(0).at(0).block<3, 1>(0, i) = Vec3f::Random();
        }
    }
    state_point = feat_points.at(0).at(0).block<3, 1>(0, 2).cast<double>();

    track.geometry.block<3, 1>(0,0) = (feat_points.at(0).at(0).block<3, 1>(0, 1) - feat_points.at(0).at(0).block<3, 1>(0, 0)).cross(
            feat_points.at(0).at(0).block<3, 1>(0, 2) - feat_points.at(0).at(0).block<3, 1>(0, 0)).cast<double>();
    if (track.geometry(2) < 0) {
        track.geometry.block<3, 1>(0,0) *= -1.0;
    }
    track.geometry.block<3, 1>(0,0).normalize();
    track.geometry.block<3, 1>(3,0).setZero();

    track.jacs = &(jacs.at(0));

    ceres::Problem::Options options;
    options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(options);

    std::list<ImplicitPlaneResidual<3>> costs;
    for (uint32_t i = 0; i < 4; ++i) {
        costs.emplace_back(i, 0, &track, &feat_points);
    }

    for (auto &cost : costs) {
        problem.AddResidualBlock(&cost, nullptr, track.geometry.data(), state_point.data());
    }

    ceres::LocalParameterization *local_param = new PlaneParameterization();
    problem.SetParameterization(track.geometry.data(), local_param);

    TestEvalCallback callback(&state_point, &feat_points);

    ceres::Solver::Options solve_options;
    solve_options.evaluation_callback = &callback;
    // provides roughly 1.5-2x speedup in this case. Presumably because plane is finicky
    solve_options.use_nonmonotonic_steps = true;

    ceres::Solver::Summary summary;
    ceres::Solve(solve_options, &problem, &summary);

    std::cout << "\n" << summary.FullReport() << "\n";

    /// Checking that all points are on the plane
    std::vector<Vec3f, Eigen::aligned_allocator<Vec3f>> normals;
    normals.emplace_back(feat_points.at(0).at(0).block<3, 1>(0, 0) - track.geometry.block<3, 1>(3,0).cast<float>());
    normals.emplace_back(feat_points.at(0).at(0).block<3, 1>(0, 1) - track.geometry.block<3, 1>(3,0).cast<float>());
    normals.emplace_back(feat_points.at(0).at(0).block<3, 1>(0, 2) - track.geometry.block<3, 1>(3,0).cast<float>());
    normals.emplace_back(feat_points.at(0).at(0).block<3, 1>(0, 3) - track.geometry.block<3, 1>(3,0).cast<float>());

    for (auto &dir : normals) {
        dir.normalize();
        float dot = dir.transpose() * track.geometry.block<3, 1>(0,0).cast<float>();
        EXPECT_NEAR(std::abs(dot), 0, 1.0e-6);
    }
}

/**
 * Normally, the analytical Jacobians would be tested with numerical differentiation, but the reliance on
 * evaluation callback to update the operating point makes that difficult.
 */

}