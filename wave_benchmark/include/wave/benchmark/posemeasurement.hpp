#ifndef POSE_COMPARE_POSEMEASUREMENT_HPP_H
#define POSE_COMPARE_POSEMEASUREMENT_HPP_H

#include <chrono>
#include <wave/geometry/rotation.hpp>
#include <wave/utils/math.hpp>

using TimeType = std::chrono::steady_clock::time_point;

struct Pose {
    wave::Rotation rotation;
    wave::Vec3 translation;
};

struct PoseMeasurement {
    TimeType time_point;
    int sensor_id;
    Pose value;
    PoseMeasurement(const TimeType &t, const int &s, const Pose &v)
        : time_point{t}, sensor_id{s}, value{v} {}
};

/** Perform interpolation (or extrapolation) between two measurements.
*/
inline Pose interpolate(const PoseMeasurement &m1,
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
    Pose retval;
    retval.rotation = interpolated * m1.value.rotation;
    retval.translation = trans;
    return retval;
};


#endif  // POSE_COMPARE_POSEMEASUREMENT_HPP_H
