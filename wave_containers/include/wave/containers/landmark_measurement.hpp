/**
 * @file Specialized measurement type for landmark observations
 * @ingroup containers
 */
#ifndef WAVE_CONTAINERS_LANDMARK_MEASUREMENT_HPP
#define WAVE_CONTAINERS_LANDMARK_MEASUREMENT_HPP

#include "wave/utils/math.hpp"

namespace wave {
/** @addtogroup containers
 *  @{ */

/** The integral type used to track a landmark across multiple measurements
 *
 * @todo: This is a weak typedef; consider a strong typedef using phantom types.
 */
using LandmarkId = std::size_t;
using TimeType = std::chrono::steady_clock::time_point;

/** Storage type for a landmark measurement as a 2D position.
 *
 * Presumably the landmark measurement has been extracted from a camera image,
 * but it is now a separate entity with no record of which camera image it came
 * from.
 *
 * This class template is similar to `Measurement`, with an additional member:
 * `landmark_id`, which represents a unique ID used for tracking a single
 * landmark across multiple images.
 *
 * The measurement value is 2D vector, representing the pixel position of the
 * landmark (u, v).
 *
 * @tparam SensorIdType The enum or integral type used to identify camera
 * sensors.
 */
template <typename SensorIdType>
struct LandmarkMeasurement {
    TimeType time_point;
    SensorIdType sensor_id;
    LandmarkId landmark_id;
    Vec2 value;

    LandmarkMeasurement(const TimeType &t,
                        const SensorIdType &s,
                        const LandmarkId &id,
                        const Vec2 &v)
        : time_point{t}, sensor_id{s}, landmark_id{id}, value{v} {}
};


/** @} group containers */
}  // namespace wave

#endif  // WAVE_CONTAINERS_LANDMARK_MEASUREMENT_HPP
