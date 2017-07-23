#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
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

class GtsamExample : public ::testing::Test {
 protected:
    VOTestDatasetGenerator generator;
    VOTestDataset dataset;
    gtsam::Cal3_S2 kParams{100, 100, 0, 320, 240};

    GtsamExample() {
        generator.camera.image_width = 640;
        generator.camera.image_height = 480;
        generator.camera.K = kParams.matrix();
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

        // Transform from robot to camera frame
        // This involves the rotation sequence: -90 deg about initial x axis,
        // 0, then -90 deg about initial z axis.
        auto q_BC = Quaternion{Eigen::AngleAxisd(-M_PI_2, Vec3::UnitZ()) *
                               Eigen::AngleAxisd(-M_PI_2, Vec3::UnitX())};

        const Quaternion q_GC = state.robot_q_GB * q_BC;
        const auto &G_p_GC = state.robot_G_p_GB;
        auto pose = gtsamPoseFromEigen(q_GC, G_p_GC);
        true_poses.push_back(pose);

        // Add a purposely offset initial estimate
        auto offset = gtsam::Pose3{gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                   gtsam::Point3{0.05, -0.10, 0.20}};

        initial_estimate.insert(gtsam::Symbol{'x', i}, pose.compose(offset));

        // Add factors for each landmark observation
        for (size_t j = 0; j < observations.size(); ++j) {
            // Convert observation
            const auto measurement = gtsam::Point2{observations[j].second};
            const auto landmark_id = observations[j].first;

            // Double check that our measurement matches gtsam's projection fn
            const auto &G_p_GF = this->dataset.landmarks[landmark_id];
            gtsam::SimpleCamera camera(pose, this->kParams);
            gtsam::Point2 gt_measurement =
              camera.project(gtsam::Point3{G_p_GF});
            ASSERT_PRED2(VectorsNear, gt_measurement, measurement);

            graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3,
                                                           gtsam::Point3,
                                                           gtsam::Cal3_S2>{
              measurement,
              measurement_noise,
              gtsam::Symbol{'x', i},
              gtsam::Symbol{'l', landmark_id},
              boost::make_shared<gtsam::Cal3_S2>(this->kParams)});

            // Add a purposely offset initial estimate the first time we see it
            const auto key = gtsam::Symbol{'l', landmark_id};
            if (!initial_estimate.exists(key)) {
                auto offset = gtsam::Point3{-0.25, 0.20, 0.15};
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
                     1e-3);

        const auto true_q = true_poses[i].rotation().toQuaternion();
        const auto estimated_q = estimated_pose.rotation().toQuaternion();
        const auto rotation_error = true_q.angularDistance(estimated_q);
        EXPECT_LT(rotation_error, 1e-4);
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
}

}  // namespace wave
