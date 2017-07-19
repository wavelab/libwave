#include "wave/benchmark/trajectory_compare.hpp"

namespace wave {

BenchmarkPose poseError(const MeasurementContainer<PoseMeasurement> &truth,
                        const PoseMeasurement &measurement) {
    // Look up the true pose, interpolating if necessary and possible
    auto true_pose =
      truth.get(measurement.time_point, ComparisonKey::GROUND_TRUTH);
    true_pose.rotation.invert();

    auto error_pose = BenchmarkPose{};
    error_pose.rotation = true_pose.rotation * measurement.value.rotation;
    error_pose.translation =
      measurement.value.translation - true_pose.translation;
    return error_pose;
}

MeasurementContainer<PoseMeasurement> trajectoryError(
  const MeasurementContainer<PoseMeasurement> &truth,
  const MeasurementContainer<PoseMeasurement> &measurements) {
    auto errors = MeasurementContainer<PoseMeasurement>{};
    for (const auto &meas : measurements) {
        auto error_pose = poseError(truth, meas);
        errors.emplace(meas.time_point, ComparisonKey::ERROR, error_pose);
    }
    return errors;
}

void TrajectoryCompare::reset() {
    this->ground_truth.clear();
    this->measurements.clear();
    this->error.clear();
}

void TrajectoryCompare::pushMeasurement(const BenchmarkPose &pose,
                                        const TimeType time) {
    measurements.insert(
      PoseMeasurement(time, ComparisonKey::MEASUREMENT, pose));
}

void TrajectoryCompare::pushTruth(const BenchmarkPose &pose,
                                  const TimeType time) {
    ground_truth.insert(
      PoseMeasurement(time, ComparisonKey::GROUND_TRUTH, pose));
}

void TrajectoryCompare::calculateError() {
    this->error = trajectoryError(this->ground_truth, this->measurements);
}

void TrajectoryCompare::outputCSV(const std::string &path) {
    this->file.open(path);

    if (this->file.is_open()) {
        for (auto iter = error.begin(); iter != error.end(); iter++) {
            this->file << iter->value.translation << std::endl;
            this->file << iter->value.rotation.logMap();
            this->file << std::endl;
        }
    }
}

}  // end of namespace wave
