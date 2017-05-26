/** @file
 * @ingroup benchmark
 */

#ifndef WAVE_BENCHMARK_POSE_MEASUREMENT_HPP
#define WAVE_BENCHMARK_POSE_MEASUREMENT_HPP

#include <chrono>
#include "wave/geometry/rotation.hpp"
#include "wave/utils/math.hpp"
#include "wave/containers/measurement.hpp"
#include "enum_class.hpp"

namespace wave {
/** @addtogroup benchmark
 *  @{ */

struct BenchmarkPose {
    Rotation rotation;
    Vec3 translation;
    BenchmarkPose() : rotation(Rotation()), translation{0.0, 0.0, 0.0} {}
    BenchmarkPose(Rotation rot, Vec3 trans) : rotation(rot), translation(trans) {}
};

using PoseMeasurement = Measurement<BenchmarkPose, ComparisonKey>;

/** Perform interpolation (or extrapolation) between two measurements.
*/
inline BenchmarkPose interpolate(const PoseMeasurement &m1,
                        const PoseMeasurement &m2,
                        const TimeType &t) {
    auto w2 = 1.0 * (t - m1.time_point) / (m2.time_point - m1.time_point);

    auto m1inverse = m1.value.rotation;
    m1inverse.invert();
    auto relative = m2.value.rotation * m1inverse;
    wave::Vec3 wvec = w2 * relative.logMap();
    wave::Rotation interpolated;
    interpolated.setFromExpMap(wvec);
    auto trans = (1 - w2) * m1.value.translation + w2 * m2.value.translation;
    BenchmarkPose retval;
    retval.rotation = interpolated * m1.value.rotation;
    retval.translation = trans;
    return retval;
};

/** @} end of group */
}  // namespace wave

#endif  // WAVE_BENCHMARK_POSE_MEASUREMENT_HPP
