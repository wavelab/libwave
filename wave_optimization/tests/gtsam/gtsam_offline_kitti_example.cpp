#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

#include "wave/wave_test.hpp"
#include "wave/vision/dataset.hpp"
#include "gtsam_helpers.hpp"

const auto DATASET_DIR = "tests/data/vo_data_drive_0036";

namespace wave {

class GtsamExample : public ::testing::Test {
 protected:
    VOTestDataset dataset = VOTestDataset::loadFromDirectory(DATASET_DIR);
    gtsam::Cal3_S2::shared_ptr kParams =
      boost::make_shared<gtsam::Cal3_S2>(dataset.camera_K(0, 0),
                                         dataset.camera_K(1, 1),
                                         0.,
                                         dataset.camera_K(0, 2),
                                         dataset.camera_K(1, 2));
};

TEST_F(GtsamExample, run) {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;

    std::vector<gtsam::Pose3> true_poses;

    // Noise: one pixel in u and v
    const auto measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, 10.0);

    // Loop over the different poses, adding the observations to iSAM
    // incrementally
    for (auto i = 0u; i < this->dataset.states.size(); ++i) {
        const auto &state = this->dataset.states[i];
        const auto &observations = state.features_observed;

        // We have to convert things to gtsam types
        // @todo better integration

        const auto pose = gtsamPoseFromState(state);
        true_poses.push_back(pose);

        // Add a purposely offset initial estimate
        const auto offset = gtsam::Pose3{gtsam::Rot3::Rodrigues(-0.1, 0.1, 0.1),
                                         gtsam::Point3{0.05, -0.10, 0.20}};

        auto pose_estimate = pose;
        pose_estimate = pose_estimate.compose(offset);
        initial_estimate.insert(gtsam::Symbol{'x', i}, pose_estimate);

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

        // Add factors for each landmark observation
        for (size_t j = 0; j < observations.size(); ++j) {
            // Convert observation
            const auto measurement = gtsam::Point2{observations[j].second};
            const auto landmark_id = observations[j].first;

            auto projection_factor =
              gtsam::GenericProjectionFactor<gtsam::Pose3,
                                             gtsam::Point3,
                                             gtsam::Cal3_S2>{
                measurement,
                measurement_noise,
                gtsam::Symbol{'x', i},
                gtsam::Symbol{'l', landmark_id},
                this->kParams};
            graph.push_back(projection_factor);

            // Initialize the landmark just in front of the camera
            const auto key = gtsam::Symbol{'l', landmark_id};
            if (!initial_estimate.exists(key)) {
                auto camera = gtsam::SimpleCamera{pose, *this->kParams};
                auto est = camera.backproject_from_camera(measurement, 3.0);
                initial_estimate.insert<gtsam::Point3>(key, est);
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

        double linear_dist =
          (true_pose.translation() - estimated_pose.translation()).norm();
        EXPECT_LT(linear_dist, 1e-2) << "x" << i;

        const auto true_q = true_poses[i].rotation().toQuaternion();
        const auto estimated_q = estimated_pose.rotation().toQuaternion();
        const auto rotation_error = true_q.angularDistance(estimated_q);
        EXPECT_LT(rotation_error, 1e-3) << "x" << i;
    }

    // No landmark ground truth to check in this case

    std::cout << "gtsam_offline_kitti_example results:" << std::endl;
    std::cout << "initial error = " << graph.error(initial_estimate)
              << std::endl;
    std::cout << "final error = " << graph.error(result) << std::endl;
}

}  // namespace wave
