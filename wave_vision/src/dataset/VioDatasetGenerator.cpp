#include "wave/vision/dataset/VioDatasetGenerator.hpp"

namespace wave {

// @todo another commit (not in master yet) makes this definition elsewhere
using TimePoint = std::chrono::steady_clock::time_point;

// Helper functions used only in this file
namespace {
// Add measurements from VoDataset's landmark measurement map to a container
void addVoMeasurementsToContainer(
  ImageNum i,
  TimePoint time_point,
  VioDataset::Camera sensor_id,
  const std::vector<LandmarkObservation> observations,
  VioDataset::ObsContainer &container) {
    for (const auto &obs : observations) {
        const auto &landmark_id = obs.first;
        const auto &measurement = obs.second;
        container.emplace(time_point, sensor_id, landmark_id, i, measurement);
    }
}

// Convert one VoDataset state to a PoseValue
VioDataset::PoseValue poseFromVoState(const VoInstant &state) {
    auto pose = VioDataset::PoseValue{};
    pose.G_p_GI = state.robot_G_p_GB;
    pose.R_GI = Rotation{}.setFromMatrix(state.robot_q_GB.matrix());
    return pose;
}

// Get a TimePoint given a start time and duration in seconds
TimePoint timePointAfterStartTime(TimePoint time_start, double dt_seconds) {
    const auto d = std::chrono::duration<double>(dt_seconds);
    return time_start + std::chrono::duration_cast<TimePoint::duration>(d);
}

// Handle one VoDataset state, adding measurements to containers
void addVoStateToDataset(ImageNum i,
                         TimePoint time_start,
                         const VoInstant &state,
                         VioDataset &dataset) {
    const auto time_point = timePointAfterStartTime(time_start, state.time);

    auto pose = poseFromVoState(state);

    dataset.poses.emplace(time_point, VioDataset::PoseSensor::Pose, pose);

    // Reorganize camera measurements
    addVoMeasurementsToContainer(i,
                                 time_point,
                                 VioDataset::Camera::Left,
                                 state.features_observed,
                                 dataset.feature_measurements);
}

// Produce imu measurements from two poses
VioDataset::ImuValue imuFromPoses(const VioDataset::PoseValue &p1,
                                  const VioDataset::PoseValue &p2,
                                  double dt_seconds) {
    VioDataset::ImuValue res;

    // Convert motion expressed in world frame to imu frame
    // @todo this is what happens when there is no Rotation.inverse() method
    auto R_I1_G = p1.R_GI;  // Not correct until the next line!
    R_I1_G.invert();        // OK, now R_IG == R_GI.inverse().
    auto R_I2_G = p2.R_GI;  // Not correct until the next line!
    R_I2_G.invert();        // OK, now R_IG == R_GI.inverse().

    const auto I_p_I1_G = R_I1_G.rotate(p1.G_p_GI);
    const auto I_p_I2_G = R_I2_G.rotate(p2.G_p_GI);

    // Angular velocity
    res.I_ang_vel_GI = R_I2_G.manifoldMinus(R_I1_G) / dt_seconds;

    // Linear velocity
    res.I_vel_GI = (I_p_I2_G - I_p_I1_G) / dt_seconds;

    return res;
}

// Produce imu measurements from two VoDataset states, and add them to container
void addImuStatesToDataset(TimePoint time_start,
                           const VoInstant &state1,
                           const VoInstant &state2,
                           VioDataset &dataset) {
    const auto p1 = poseFromVoState(state1);
    const auto p2 = poseFromVoState(state2);
    const auto dt_seconds = state1.time - state2.time;
    const auto imu_value = imuFromPoses(p1, p2, dt_seconds);
    const auto time_point = timePointAfterStartTime(time_start, state1.time);
    dataset.imu_measurements.emplace(
      time_point, VioDataset::ImuSensor::Imu, imu_value);
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

        // Calculate imu measurements
        // (the last one is missing)
        if (i > 0) {
            addImuStatesToDataset(
              time_start, vo.states[i - 1], vo.states[i], dataset);
        }
    }

    // Handle calibration
    dataset.camera.K = vo.camera_K;
    dataset.camera.image_width = 2 * dataset.camera.cx();
    dataset.camera.image_height = 2 * dataset.camera.cy();

    // In VoDatasetGenerator, the transformation from body to camera is just a
    // rotation from NWU to EDN
    dataset.I_p_IC = Vec3::Zero();
    dataset.R_IC = Rotation{Vec3{-M_PI_2, 0, -M_PI_2}};

    return dataset;
}

}  // namespace wave
