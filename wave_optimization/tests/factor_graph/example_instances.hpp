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
WAVE_DEFINE_COMPOSED_VALUE(Pose2D,
                           (Position2D, position)        //
                           (Orientation2D, orientation)  //
                           );
WAVE_DEFINE_COMPOSED_VALUE(RangeBearing,
                           (Distance, range)         //
                           (Orientation2D, bearing)  //
                           );

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
    bool operator()(const Pose2D<T, O> &pose,
                    const Position2D<T, O> &landmark_pos,
                    Distance<T, O> &result) const noexcept {
        Eigen::Matrix<T, 2, 1> diff = pose.position - landmark_pos;
        result[0] = diff.norm();
        return true;
    }
};


struct RangeBearingMeasurementFunctor {
    template <typename T, typename O = void>
    bool operator()(const Pose2D<T, O> &pose,
                    const Position2D<T, O> &landmark,
                    RangeBearing<T, O> &result) const noexcept {
        Eigen::Matrix<T, 2, 1> diff = landmark - pose.position;
        result.range[0] = diff.norm();
        result.bearing[0] =
          atan2(diff.y(), diff.x()) - pose.orientation.value();
        return true;
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
