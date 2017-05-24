/** @file
 * @ingroup benchmark
 */

#ifndef WAVE_BENCHMARK_TRAJECTORY_COMPARE_HPP
#define WAVE_BENCHMARK_TRAJECTORY_COMPARE_HPP

#include <iostream>
#include <fstream>

#include "pose_measurement.hpp"
#include "wave/containers/measurement_container.hpp"
#include "wave/geometry/rotation.hpp"

namespace wave {
/** @addtogroup benchmark
 *  @{ */

class TrajectoryCompare {
 public:
    /** Clears the internal measurement containers
     *
     */
    void reset();
    /** Adds a ground truth observation
     *
     */
    void pushTruth(BenchmarkPose pose, const TimeType time);
    /** Adds a pose measurement
     *
     */
    void pushMeasurement(BenchmarkPose pose, const TimeType time);
    /** Finds the difference between the measurements and ground truth
     * and places the result into another measurement container.
     * The error is calculated at the time of each measurement. Ground truth
     * is linearly interpolated.
     *
     */
    void calculateError();
    /** Helper function that takes complete measurement containers and calculates the
     * error immediately.
     *
     */
    void calculateError(const MeasurementContainer<PoseMeasurement> &truth,
                        const MeasurementContainer<PoseMeasurement> &measured);
    /** Outputs content of error container into a csv
     * The format of the csv is to output the translational error followed by the
     * rotational error as a so(3) vector. Newlines are delimiters
     *
     */
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

/** @} end of group */
}  // namespace wave

#endif  // WAVE_BENCHMARK_TRAJECTORY_COMPARE_HPP