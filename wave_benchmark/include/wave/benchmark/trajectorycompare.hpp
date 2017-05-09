#ifndef POSE_COMPARE_LIBRARY_HPP
#define POSE_COMPARE_LIBRARY_HPP

#include <iostream>
#include <fstream>

#include "posemeasurement.hpp"
#include <wave/containers/measurement_container.hpp>
#include <wave/geometry/rotation.hpp>

namespace wave {

class TrajectoryCompare {
 public:
    void reset();
    void push_truth(Pose pose, const TimeType time);
    void push_measurement(Pose pose, const TimeType time);
    void calculate_error();
    void output_csv(std::string path, std::string filename);

 private:
    wave::MeasurementContainer<Measurement> ground_truth;
    wave::MeasurementContainer<Measurement> measurements;
    wave::MeasurementContainer<Measurement> error;
    const int ground_truth_key = 0;
    const int measurement_key = 1;
    const int error_key = 3;
    std::ofstream file;
};

}  // end of namespace wave

#endif