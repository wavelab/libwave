#include "wave/benchmark/trajectory_compare.hpp"

namespace wave {

void TrajectoryCompare::reset() {
    this->ground_truth.clear();
    this->measurements.clear();
    this->error.clear();
}

void TrajectoryCompare::pushMeasurement(BenchmarkPose pose,
                                        const TimeType time) {
    measurements.insert(PoseMeasurement(time, this->measurement_key, pose));
}

void TrajectoryCompare::pushTruth(BenchmarkPose pose, const TimeType time) {
    ground_truth.insert(PoseMeasurement(time, this->ground_truth_key, pose));
}

void TrajectoryCompare::calculateError() {
    for (auto iter = measurements.begin(); iter != measurements.end(); iter++) {
        BenchmarkPose truth =
          ground_truth.get(iter->time_point, this->ground_truth_key);
        BenchmarkPose error;
        truth.rotation.invert();
        error.rotation = truth.rotation * iter->value.rotation;
        error.translation = iter->value.translation - truth.translation;
        this->error.emplace(iter->time_point, this->error_key, error);
    }
}

void TrajectoryCompare::calculateError(
  const MeasurementContainer<PoseMeasurement> &truth,
  const MeasurementContainer<PoseMeasurement> &measured) {
    this->reset();
    this->ground_truth = truth;
    this->measurements = measured;
    this->calculateError();
}

void TrajectoryCompare::outputCSV(std::string path) {
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