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
struct Pose2D : public ValueView<3> {
    // Use base class constructor
    // We can't inherit constructors due to bug in gcc
    // (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67054)
    explicit Pose2D(double *d) : ValueView<3>{d} {}
    explicit Pose2D(MappedType &m) : ValueView<3>{m} {}
    Pose2D &operator=(const Pose2D &other) {
        this->ValueView<3>::operator=(other);
        return *this;
    }
    using Vec1 = Eigen::Matrix<double, 1, 1>;

    Eigen::Map<const Vec2> position{dataptr};
    Eigen::Map<const Vec1> orientation{dataptr + 2};
};

/**
 * Specialized variable representing a 2D landmark position.
 *
 * In this example, references are assigned to parts of the underlying data
 * vector, allowing factor functions to operate on clearly named parameters.
 */
struct Landmark2D : public ValueView<2> {
    // Use base class constructor
    // We can't inherit constructors due to bug in gcc
    // (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=67054)
    explicit Landmark2D(double *d) : ValueView<2>{d} {}
    explicit Landmark2D(MappedType &m) : ValueView<2>{m} {}
    Landmark2D &operator=(const Landmark2D &other) {
        this->ValueView<2>::operator=(other);
        return *this;
    }

    Eigen::Map<const Vec2> position{dataptr};
};

/** Define variable types for each value type */
using Pose2DVar = FactorVariable<Pose2D>;
using Landmark2DVar = FactorVariable<Landmark2D>;


using DistanceMeasurement = FactorMeasurement<double>;


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
inline bool distanceMeasurementFunction(const Pose2D &pose,
                                        const Landmark2D &landmark,
                                        ResultOut<1> result,
                                        JacobianOut<1, 3> j_pose,
                                        JacobianOut<1, 2> j_landmark) noexcept {
    Vec2 diff = pose.position - landmark.position;
    double distance = diff.norm();
    result(0) = distance;

    // For each jacobian, check that optimizer requested it
    // (just as you would check pointers from ceres)
    if (j_pose) {
        j_pose << diff.transpose() / distance, 0;
    }
    if (j_landmark) {
        j_landmark << diff.transpose() / distance;
    }

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
