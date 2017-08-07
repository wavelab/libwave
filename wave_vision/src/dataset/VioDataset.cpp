#include "wave/vision/dataset/VioDataset.hpp"

namespace wave {

// Helper functions for this file
namespace {
// Add measurements from VoDataset's landmark measurement map to a container
void addVoMeasurementsToContainer(
  ImageNum i,
  TimePoint time_point,
  VioDataset::Camera sensor_id,
  const std::vector<LandmarkObservation> observations,
  VioDataset::LandmarkMeasurementContainer &container) {
    for (const auto &obs : observations) {
        const auto &landmark_id = obs.first;
        const auto &measurement = obs.second;
        container.emplace(time_point, sensor_id, landmark_id, i, measurement);
    }
}

// Handle one VoDataset state, adding measurements to containers
void addVoStateToDataset(ImageNum i,
                         TimePoint time_start,
                         const VoInstant &state,
                         VioDataset &dataset) {
    auto d = std::chrono::duration<double>(state.time);

    TimePoint time_point =
      time_start + std::chrono::duration_cast<TimePoint::duration>(d);

    auto pose = VioDataset::PoseValue{};
    pose.G_p_GI = state.robot_G_p_GB;
    pose.R_GI = Rotation{}.setFromMatrix(state.robot_q_GB.matrix());

    dataset.poses.emplace(time_point, VioDataset::PoseSensor::Pose, pose);

    // Reorganize camera measurements
    addVoMeasurementsToContainer(i,
                                 time_point,
                                 VioDataset::Camera::Left,
                                 state.features_observed,
                                 dataset.feature_measurements);
}

}  // namespace

VioDataset VioDatasetGenerator::generate() {
    VioDataset dataset;

    // Use the existing VO generator for now, just change the format
    VoDataset vo = VoDatasetGenerator::generate();

    dataset.landmarks = std::move(vo.landmarks);

    // Choose arbitrary start time
    auto time_start = std::chrono::steady_clock::now();

    // Handle measurements
    for (auto i = 0u; i < vo.states.size(); ++i) {
        addVoStateToDataset(i, time_start, vo.states[i], dataset);
    }

    // Handle calibration
    dataset.camera_K = vo.camera_K;
    // In VoDatasetGenerator, the transformation from body to camera is just a
    // rotation from NWU to EDN
    dataset.I_p_IC = Vec3::Zero();
    dataset.R_IC = Rotation{Vec3{-M_PI_2, 0, -M_PI_2}};

    return dataset;
}

}  // namespace wave
