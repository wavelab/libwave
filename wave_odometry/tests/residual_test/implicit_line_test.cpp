#include <ceres/solver.h>
#include <ceres/ceres.h>

#include "wave/odometry/feature_track.hpp"
#include "wave/odometry/implicit_geometry/implicit_line.hpp"
#include "wave/optimization/ceres/local_params/spherical_parameterization.hpp"
#include "wave/wave_test.hpp"

struct TestEvalCallback : ceres::EvaluationCallback {
    explicit TestEvalCallback(wave::FeatureTrack<3> *track) : ceres::EvaluationCallback(), track(track) {}

    virtual void PrepareForEvaluation(bool, bool) {
        for (uint32_t i = 0; i < this->track->pts.size(); ++i) {
            Eigen::Map<const wave::Vec3> vec(this->track->pts.at(i));
            this->track->tpts.at(i) = vec;
        }
    }
    wave::FeatureTrack<3> *track;
};


namespace wave {

TEST(implicit_line, simple) {
    FeatureTrack<3> track;
    track.pts.resize(3);
    track.tpts.resize(3);

    std::vector<Vec3, Eigen::aligned_allocator<Vec3>> test_pts;
    test_pts.resize(3);
    for (int i = 0; i < 3; i++) {
        test_pts.at(i) = Vec3::Random();
        track.pts.at(i) = test_pts.at(i).data();

        track.n_states.emplace_back(0);
        track.p_states.emplace_back(0);
        if (i == 2) {
            track.prev_jac.emplace_back(Mat3::Identity());
        } else {
            track.prev_jac.emplace_back(Mat3::Zero());
        }
        track.next_jac.emplace_back(Mat3::Zero());
    }

    Vec3 normal = test_pts.at(0) - test_pts.at(2);
    normal.normalize();

    ImplicitLineResidual<3, 3, 3, 3> cost_function(track);
    ceres::Problem::Options options;
    options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    ceres::Problem problem(options);
    problem.AddResidualBlock(&cost_function, nullptr, normal.data(), test_pts.at(2).data());

    ceres::LocalParameterization *spherical = new SphericalParameterization();
    problem.SetParameterization(normal.data(), spherical);

    TestEvalCallback callback(&track);

    ceres::Solver::Options solve_options;
    solve_options.evaluation_callback = &callback;

    ceres::Solver::Summary summary;
    ceres::Solve(solve_options, &problem, &summary);

    summary.FullReport();
}





}