/**
 * @file
 * Generic measurement type meant to be used with MeasurementContainer
 * @ingroup containers
 */

#ifndef WAVE_CONTAINERS_MEASUREMENT_HPP
#define WAVE_CONTAINERS_MEASUREMENT_HPP

#include <chrono>

/** The wave namespace */
namespace wave {

/** @addtogroup containers
 *  @{ */

using TimePoint = std::chrono::steady_clock::time_point;

/** Generic measurement storage type designed to be used with
 * MeasurementContainer.
 * */
template <typename T, typename S>
struct Measurement {
    TimePoint time_point;
    S sensor_id;
    T value;

    Measurement(const TimePoint &t, const S &s, const T &v)
        : time_point{t}, sensor_id{s}, value{v} {}
};

/** Perform linear interpolation (or extrapolation) between two measurements.
 *
 *
 * This is the default interpolation function for generic Measurement types.
 * For measurements requiring non-linear interpolation, you may provide your own
 * specialization.
 *
 * The equation is @f$ y = y_0 + (x-x_0)\frac{y_1 - y_0}{x_1-x_0} @f$
 *
 */
template <typename T>
decltype(T::value) interpolate(const T &m1, const T &m2, const TimePoint &t) {
    auto w2 = 1.0 * (t - m1.time_point) / (m2.time_point - m1.time_point);
    return (1 - w2) * m1.value + w2 * m2.value;
}

/** @} group containers */
}  // namespace wave

#endif  // WAVE_CONTAINERS_MEASUREMENT_HPP
