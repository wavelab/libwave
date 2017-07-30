#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>


#include "wave/wave_test.hpp"
#include "wave/vision/dataset.hpp"

namespace wave {

/** Convert pose to gtsam type */
inline gtsam::Pose3 gtsamPoseFromEigen(const Quaternion &q, const Vec3 &p) {
    return gtsam::Pose3{gtsam::Rot3{q}, gtsam::Point3{p}};
}

inline gtsam::Pose3 gtsamPoseFromState(const VOTestInstant &state) {
    // Transform from robot to camera frame
    // This involves the rotation sequence: -90 deg about initial x axis,
    // 0, then -90 deg about initial z axis.
    const auto q_BC = Quaternion{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                                 Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};

    const Quaternion q_GC = state.robot_q_GB * q_BC;
    const auto &G_p_GC = state.robot_G_p_GB;

    return gtsamPoseFromEigen(q_GC, G_p_GC);
}

/** Get the tranformation between two gtsam poses */
inline gtsam::Pose3 poseBetween(const gtsam::Pose3 &from,
                                const gtsam::Pose3 &to) {
    auto between = from.inverse() * to;

    // demonstrate the math
    auto res = from * between;
    assert(to.translation().isApprox(res.translation()));

    return between;
}

inline gtsam::Pose3 poseBetweenStates(const VOTestInstant &from,
                                      const VOTestInstant &to) {
    auto pose = gtsamPoseFromState(to);
    auto prev_pose = gtsamPoseFromState(from);

    return poseBetween(prev_pose, pose);
}

class GtsamExample : public ::testing::Test {
 protected:
    VOTestDatasetGenerator generator;
    VOTestDataset dataset;
    gtsam::Cal3_S2::shared_ptr kParams =
      boost::make_shared<gtsam::Cal3_S2>(200, 200, 0, 320, 240);

    GtsamExample() {
        generator.camera.image_width = 640;
        generator.camera.image_height = 480;
        generator.camera.K = kParams->matrix();
        generator.camera.hz = 10.0;
        generator.nb_landmarks = 100;
        generator.landmark_x_bounds << -10, 10;
        generator.landmark_y_bounds << -10, 10;
        generator.landmark_z_bounds << -1, 1;
        dataset = generator.generate();

        // Remove landmarks seen fewer than 3 times
        dataset.removeUnobservedLandmarks(3);
    }
};

TEST_F(GtsamExample, run) {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;

    std::vector<gtsam::Pose3> true_poses;

    // Noise: one pixel in u and v
    auto measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

    // Loop over the different poses, adding the observations to iSAM
    // incrementally
    for (auto i = 0u; i < this->dataset.states.size(); ++i) {
        const auto &state = this->dataset.states[i];
        const auto &observations = state.features_observed;

        // We have to convert things to gtsam types
        // @todo better integration

        const auto pose = gtsamPoseFromState(state);
        true_poses.push_back(pose);

        if (i > 0) {
            // Add odometry factor
            auto between = poseBetweenStates(dataset.states[i - 1], state);

            Vec6 noise_vec;
            noise_vec << Vec3::Constant(0.001), Vec3::Constant(0.0001);

            auto noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
            auto odometry_factor = gtsam::BetweenFactor<gtsam::Pose3>{
              gtsam::Symbol{'x', i - 1}, gtsam::Symbol{'x', i}, between, noise};
            graph.push_back(odometry_factor);
        }

        // Add a purposely offset initial estimate
        const auto offset =
          gtsam::Pose3{gtsam::Rot3::Rodrigues(-0.02, 0.02, 0.22),
                       gtsam::Point3{0.05, -0.10, 0.20}};

        auto pose_estimate = pose;
        pose_estimate = pose_estimate.compose(offset);
        initial_estimate.insert(gtsam::Symbol{'x', i}, pose_estimate);


        // Add factors for each landmark observation
        for (size_t j = 0; j < observations.size(); ++j) {
            // Convert observation
            const auto measurement = gtsam::Point2{observations[j].second};
            const auto landmark_id = observations[j].first;

            // Double check that our measurement matches gtsam's projection fn
            const auto &G_p_GF = this->dataset.landmarks[landmark_id];
            gtsam::SimpleCamera camera(pose, *this->kParams);
            gtsam::Point2 gt_measurement =
              camera.project(gtsam::Point3{G_p_GF});
            ASSERT_PRED2(VectorsNear, gt_measurement, measurement);

            auto projection_factor =
              gtsam::GenericProjectionFactor<gtsam::Pose3,
                                             gtsam::Point3,
                                             gtsam::Cal3_S2>{
                gt_measurement,
                measurement_noise,
                gtsam::Symbol{'x', i},
                gtsam::Symbol{'l', landmark_id},
                this->kParams};

            graph.push_back(projection_factor);

            // Add a purposely offset initial estimate the first time we see it
            const auto key = gtsam::Symbol{'l', landmark_id};
            if (!initial_estimate.exists(key)) {
                auto offset = gtsam::Point3{-0.25, 0.20, 0.15};
                offset = gtsam::Point3{0, 0, 0};
                initial_estimate.insert<gtsam::Point3>(key, G_p_GF + offset);
            }
        }

        // Add priors on the first two poses to fix the origin and scale
        if (i < 2) {
            VecX noise_vec{6};
            noise_vec << Vec3::Constant(1e-5), Vec3::Constant(1e-6);
            auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
            graph.push_back(gtsam::PriorFactor<gtsam::Pose3>{
              gtsam::Symbol{'x', i}, pose, poseNoise});
        }
    }

    gtsam::LevenbergMarquardtOptimizer optimizer{graph, initial_estimate};
    auto result = optimizer.optimize();


    // Uncomment to print estimated values
    // result.print("Final result:\n");

    // Check camera poses
    for (auto i = 0u; i < this->dataset.states.size(); ++i) {
        const auto key = gtsam::Symbol{'x', i};
        // We have to cast the generic Value back to the known type
        const auto estimated_pose = result.at(key).cast<gtsam::Pose3>();
        const auto &true_pose = true_poses[i];

        EXPECT_PRED3(VectorsNearPrec,
                     true_pose.translation(),
                     estimated_pose.translation(),
                     1e-3)
          << "x" << i;

        const auto true_q = true_poses[i].rotation().toQuaternion();
        const auto estimated_q = estimated_pose.rotation().toQuaternion();
        const auto rotation_error = true_q.angularDistance(estimated_q);
        EXPECT_LT(rotation_error, 1e-4) << "x" << i;
    }

    // Check landmarks
    // @todo a few landmarks have a bad estimate. For now, just check that most
    // are correct.
    auto num_outliers = 0u;
    for (const auto &l : this->dataset.landmarks) {
        const auto &landmark_id = l.first;
        const auto &true_pos = l.second;

        const auto key = gtsam::Symbol{'l', landmark_id};
        const auto estimated_pos = result.at(key).cast<gtsam::Point3>();

        const auto dist = (true_pos - estimated_pos).norm();
        if (dist > 1e-3) {
            ++num_outliers;
        }
    }
    EXPECT_LT(num_outliers, this->dataset.landmarks.size() / 10);

    std::cout << "gtsam_offline_example results:" << std::endl;
    std::cout << "initial error = " << graph.error(initial_estimate)
              << std::endl;
    std::cout << "final error = " << graph.error(result) << std::endl;
}

}  // namespace wave
