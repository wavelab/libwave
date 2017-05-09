#include <wave/benchmark/trajectorycompare.hpp>

namespace wave {

void TrajectoryCompare::reset() {
    this->ground_truth.clear();
    this->measurements.clear();
    this->error.clear();
}

void TrajectoryCompare::push_measurement(Pose pose, const TimeType time) {
    measurements.insert(Measurement(time, this->measurement_key, pose));
}

void TrajectoryCompare::push_truth(Pose pose, const TimeType time) {
    ground_truth.insert(Measurement(time, this->ground_truth_key, pose));
}

void TrajectoryCompare::calculate_error() {
    for (auto iter = measurements.begin(); iter != measurements.end(); iter++) {
        Pose truth =
          ground_truth.get(iter->time_point, this->ground_truth_key);
        Pose error;
        truth.rotation.invert();
        error.rotation = iter->value.rotation * truth.rotation;
        error.translation = iter->value.translation - truth.translation;
        this->error.insert(
          Measurement(iter->time_point, this->error_key, error));
    }
}

void TrajectoryCompare::output_csv(std::string path, std::string filename) {
    file.open(path + filename);
    if (file.is_open()) {
        for (auto iter = error.begin(); iter != error.end(); iter++) {
            file << iter->value.translation;
            file << iter->value.rotation;
            file << std::endl;
        }
    } else {
        // cry about it
    }
}

}  // end of namespace wave