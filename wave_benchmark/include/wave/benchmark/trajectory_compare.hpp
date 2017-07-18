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
#include "enum_class.hpp"

namespace wave {
/** @addtogroup benchmark
 *  @{ */

/** Finds the difference between a single pose measurement and the contents of a
 * MeasurementContainer.
 *
 * If `truth` does not hold a value at the exact time points of `measurement`,
 * it is linearly interpolated.
 *
 * @return a pose corresponding to the transformation from the true pose to
 * the measurement pose.
 * @throw std::out_of_range if truth cannot be interpolated
 */
BenchmarkPose poseError(const MeasurementContainer<PoseMeasurement> &truth,
                        const PoseMeasurement &measurement);


/** Finds the difference between the measured poses and ground truth.
 *
 * If the time points of the two input containers don't match, `truth` is
 * linearly interpolated to give one difference for each `measured` pose.
 *
 * @return a container holding transformations between the poses in `truth` and
 * the poses in `measured`.
 * @throw std::out_of_range if truth cannot be interpolated
 */
MeasurementContainer<PoseMeasurement> trajectoryError(
  const MeasurementContainer<PoseMeasurement> &truth,
  const MeasurementContainer<PoseMeasurement> &measured);

class TrajectoryCompare {
 public:
    /** Clears the internal measurement containers */
    void reset();

    /** Adds a ground truth observation */
    void pushTruth(const BenchmarkPose &pose, const TimeType time);

    /** Adds a pose measurement */
    void pushMeasurement(const BenchmarkPose &pose, const TimeType time);

    /** Finds the difference between the measurements and ground truth
     * using `trajectoryError` and places the result into `this->error`.
     *
     * The error is calculated at the time of each measurement. Ground truth
     * is linearly interpolated.
     */
    void calculateError();

    /** Outputs content of error container into a csv
     * The format of the csv is to output the translational error followed by
     * the rotational error as a so(3) vector. Newlines are delimiters
     */
    void outputCSV(const std::string &path);

    MeasurementContainer<PoseMeasurement> ground_truth;
    MeasurementContainer<PoseMeasurement> measurements;
    MeasurementContainer<PoseMeasurement> error;

 private:
    std::ofstream file;
};

/** @} end of group */
}  // namespace wave

#endif  // WAVE_BENCHMARK_TRAJECTORY_COMPARE_HPP
