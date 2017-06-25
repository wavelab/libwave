/**
 * @file
 * @ingroup optimization
 * Examples of Factor and FactorVariable instances
 *
 * @todo move elsewhere when other instances are implemented
 */
#ifndef WAVE_EXAMPLE_INSTANCES_HPP_HPP
#define WAVE_EXAMPLE_INSTANCES_HPP_HPP

#include "wave/optimization/factor_graph/Factor.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/FactorMeasurement.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */


template <typename T>
using Position2D = FactorValue<T, 2>;

template <typename T>
using Orientation2D = FactorValue<T, 1>;

template <typename T>
using Distance = FactorValue<T, 1>;

/**
 * Specialized variable representing a 2D pose.
 *
 * In this example, references are assigned to parts of the underlying data
 * vector, allowing factor functions to operate on clearly named parameters.
 *
 * In future changes, it may be possible to save the Variable implementer even
 * this work, by automatically generating these mappings using some fancy boost
 * libraries.
 */
template <typename T>
struct Pose2D : public ComposedValue<T, Position2D, Orientation2D> {
    // Use base class constructors
    using ComposedValue<T, Position2D, Orientation2D>::ComposedValue;

    Position2D<T>& position = std::get<0>(this->elements);
    Orientation2D<T>& orientation = std::get<1>(this->elements);
};

/** Define variable types for each value type */
using Pose2DVar = FactorVariable<Pose2D>;
using Landmark2DVar = FactorVariable<Position2D>;
using DistanceMeasurement = FactorMeasurement<Distance>;


/** Calculate distance and jacobians
 *
 * Notice how each parameter corresponds to strongly-typed variables and
 * matrices, instead of something like `double **`.
 *
 * @param[in] pose
 * @param[in] landmark
 * @param[out] residual
 * @param[out] j_pose
 * @param[out] j_landmark
 * @return true on success
 */
template <typename T>
inline bool distanceMeasurementFunction(const Pose2D<T> &pose,
                                        const Position2D<T> &landmark_pos,
                                        Distance<T> &result) noexcept {
    Vec2 diff = pose.position - landmark_pos;
    double distance = diff.norm();
    result = distance;
    return true;
}

/**
 * Factor representing a distance measurement between a 2D pose and landmark.
 */
class DistanceToLandmarkFactor
  : public Factor<DistanceMeasurement, Pose2DVar, Landmark2DVar> {
 public:
    explicit DistanceToLandmarkFactor(DistanceMeasurement meas,
                                      std::shared_ptr<Pose2DVar> p,
                                      std::shared_ptr<Landmark2DVar> l)
        : Factor<DistanceMeasurement, Pose2DVar, Landmark2DVar>{
            distanceMeasurementFunction, meas, std::move(p), std::move(l)} {}
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_EXAMPLE_INSTANCES_HPP_HPP
