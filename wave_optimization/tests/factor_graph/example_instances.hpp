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
#include "wave/optimization/factor_graph/ComposedValue.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */


template <typename T, typename O = void>
using Position2D = FactorValue<T, O, 2>;

template <typename T, typename O = void>
using Orientation2D = FactorValue<T, O, 1>;

template <typename T, typename O = void>
using Distance = FactorValue<T, O, 1>;

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
template <typename T, typename O = void>
struct Pose2D : public ComposedValue<Pose2D<T, O>, Position2D, Orientation2D> {
    // Use base class constructors
    using Base = ComposedValue<Pose2D<T, O>, Position2D, Orientation2D>;
    using Base::Base;
    Pose2D() = default;
    Pose2D(const Pose2D &rhs) : Base{rhs} {}

    Ref<Position2D<T, O>> position = this->template block<0>();
    Ref<Orientation2D<T, O>> orientation = this->template block<1>();
};

template <typename T, typename O = void>
struct RangeBearing
  : public ComposedValue<RangeBearing<T, O>, Distance, Orientation2D> {
    // Use base class constructors
    using Base = ComposedValue<RangeBearing<T, O>, Distance, Orientation2D>;
    using Base::Base;
    RangeBearing() = default;
    RangeBearing(const RangeBearing &rhs) : Base{rhs} {}

    Ref<Distance<T, O>> range = this->template block<0>();
    Ref<Orientation2D<T, O>> bearing = this->template block<1>();
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
struct DistanceMeasurementFunctor {
    template <typename T, typename O = void>
    static bool evaluate(const Pose2D<T, O> &pose,
                         const Position2D<T, O> &landmark_pos,
                         Distance<T, O> &result) noexcept {
        Eigen::Matrix<T, 2, 1> diff = pose.position - landmark_pos;
        result[0] = diff.norm();
        return true;
    }
};


struct RangeBearingMeasurementFunctor {
    template <typename T, typename O = void>
    static bool evaluate(const Pose2D<T, O> &pose,
                         const Position2D<T, O> &landmark,
                         RangeBearing<T, O> &result) noexcept {
        Eigen::Matrix<T, 2, 1> diff = landmark.position - pose.position;
        result.range = diff.norm();
        result.bearing = atan2(diff.y(), diff.x()) - pose.orientation.value();
    }
};


/**
 * Factor representing a distance measurement between a 2D pose and landmark.
 */
using DistanceToLandmarkFactor =
  Factor<DistanceMeasurementFunctor, Distance, Pose2D, Position2D>;

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_EXAMPLE_INSTANCES_HPP_HPP
