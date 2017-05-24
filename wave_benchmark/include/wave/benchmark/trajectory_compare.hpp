#ifndef WAVE_BENCHMARK_TRAJECTORY_COMPARE_HPP
#define WAVE_BENCHMARK_TRAJECTORY_COMPARE_HPP

#include <iostream>
#include <fstream>

#include "pose_measurement.hpp"
#include "wave/containers/measurement_container.hpp"
#include "wave/geometry/rotation.hpp"

namespace wave {

class TrajectoryCompare {
 public:
    void reset();
    void pushTruth(Pose pose, const TimeType time);
    void pushMeasurement(Pose pose, const TimeType time);
    void calculateError();
    void calculateError(const MeasurementContainer<PoseMeasurement> &truth,
                        const MeasurementContainer<PoseMeasurement> &measured);
    void outputCSV(std::string path);
    MeasurementContainer<PoseMeasurement> ground_truth;
    MeasurementContainer<PoseMeasurement> measurements;
    MeasurementContainer<PoseMeasurement> error;

 private:
    const int ground_truth_key = 0;
    const int measurement_key = 1;
    const int error_key = 2;
    std::ofstream file;
};

}  // namespace wave

#endif  // WAVE_BENCHMARK_TRAJECTORY_COMPARE_HPP