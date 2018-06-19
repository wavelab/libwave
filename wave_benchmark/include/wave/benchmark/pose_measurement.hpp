/** @file
 * @ingroup benchmark
 */

#ifndef WAVE_BENCHMARK_POSE_MEASUREMENT_HPP
#define WAVE_BENCHMARK_POSE_MEASUREMENT_HPP

#include <chrono>
#include "wave/geometry/geometry.hpp"
#include "wave/utils/math.hpp"
#include "wave/containers/measurement.hpp"
#include "wave/benchmark/enum_class.hpp"

namespace wave {
/** @addtogroup benchmark
 *  @{ */

struct BenchmarkPose {
    RotationMd rotation;
    Vec3 translation;

    BenchmarkPose() : rotation(RotationMd()), translation{0.0, 0.0, 0.0} {}
    BenchmarkPose(RotationMd rot, Vec3 trans)
        : rotation(rot), translation(trans) {}
};

using PoseMeasurement = Measurement<BenchmarkPose, ComparisonKey>;

/** Perform interpolation (or extrapolation) between two measurements.
*/
inline BenchmarkPose interpolate(const PoseMeasurement &m1,
                                 const PoseMeasurement &m2,
                                 const TimePoint &t) {
    auto w2 = 1.0 * (t - m1.time_point) / (m2.time_point - m1.time_point);

    auto m1inverse = inverse(m1.value.rotation);

    auto relative = eval(log(m2.value.rotation * m1inverse));
    relative.value() *= w2;  // operator* not done for wave_geometry, so cheating by accessing Eigen object

    auto trans = (1 - w2) * m1.value.translation + w2 * m2.value.translation;

    BenchmarkPose retval;
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(relative.value()(2,0), Vec3::UnitZ()) *
         Eigen::AngleAxisd(relative.value()(1,0), Vec3::UnitY()) *
         Eigen::AngleAxisd(relative.value()(0,0), Vec3::UnitX());
    retval.rotation = RotationMd(rot * m1.value.rotation.value());
    retval.translation = trans;
    return retval;
};

/** @} end of group */
}  // namespace wave

#endif  // WAVE_BENCHMARK_POSE_MEASUREMENT_HPP
