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

    // one "scan"
    feat_points.resize(1);
    // one "feature type"
    MatXf feature_points;
    feature_points.resize(3, 3);
    feat_points.at(0).emplace_back(Eigen::Map<MatXf>(feature_points.data(), feature_points.rows(), feature_points.cols()));

    FeatureTrack track;
    track.mapping.resize(3);
    track.mapping.resize(3);
    track.featT_idx = 0;

    track.state_ids.resize(3);
    track.jacs.resize(3);

    Vec3 state_point;
    for (uint32_t i = 0; i < 3; i++) {
        track.mapping.at(i).pt_idx = i;
        track.mapping.at(i).scan_idx = 0;

        track.state_ids.at(i).emplace_back(0);
        if (i == 2) {
            track.jacs.at(i).emplace_back(Mat3::Identity());
            feat_points.at(0).at(0).block<3, 1>(0, i) = Vec3f::Random();
        } else {
            track.jacs.at(i).emplace_back(Mat3::Zero());
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

    std::list<ImplicitLineResidual<3>> costs;
    for (uint32_t i = 0; i < 3; ++i) {
        costs.emplace_back(i, &track, &feat_points);
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

    // one "scan"
    feat_points.resize(1);
    // one "feature type"
    MatXf feature_points;
    feature_points.resize(3, 4);
    feat_points.at(0).emplace_back(Eigen::Map<MatXf>(feature_points.data(), feature_points.rows(), feature_points.cols()));

    FeatureTrack track;
    track.mapping.resize(4);
    track.mapping.resize(4);
    track.featT_idx = 0;

    track.state_ids.resize(4);
    track.jacs.resize(4);

    Vec3 state_point;
    for (uint32_t i = 0; i < 4; i++) {
        track.mapping.at(i).pt_idx = i;
        track.mapping.at(i).scan_idx = 0;

        track.state_ids.at(i).emplace_back(0);
        if (i == 2) {
            track.jacs.at(i).emplace_back(Mat3::Identity());
            feat_points.at(0).at(0).block<3, 1>(0, i) = Vec3f::Random();
        } else {
            track.jacs.at(i).emplace_back(Mat3::Zero());
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

    ceres::Problem::Options options;
    options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(options);

    std::list<ImplicitPlaneResidual<3>> costs;
    for (uint32_t i = 0; i < 4; ++i) {
        costs.emplace_back(i, &track, &feat_points);
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