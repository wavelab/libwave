/**
 * @file Specialized measurement type for landmark observations
 * @ingroup containers
 */
#ifndef WAVE_CONTAINERS_LANDMARK_MEASUREMENT_HPP
#define WAVE_CONTAINERS_LANDMARK_MEASUREMENT_HPP

#include <chrono>
#include "wave/utils/math.hpp"

namespace wave {
/** @addtogroup containers
 *  @{ */

/** The integral type used to track a landmark across multiple measurements
 *
 * @todo: This is a weak typedef; consider a strong typedef using phantom types.
 */
using ImageNum = std::size_t;
using LandmarkId = std::size_t;
using TimePoint = std::chrono::steady_clock::time_point;

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
 * @tparam S The enum or integral type used to identify camera
 * sensors.
 */
template <typename T, typename S>
struct LandmarkMeasurement {
    TimePoint time_point;
    S sensor_id;
    LandmarkId landmark_id;
    ImageNum image;
    T value;

    LandmarkMeasurement(const TimePoint &t,
                        const S &s,
                        const LandmarkId &id,
                        const ImageNum &img,
                        const T &v)
        : time_point{t}, sensor_id{s}, landmark_id{id}, image{img}, value{v} {}
};


/** @} group containers */
}  // namespace wave

#endif  // WAVE_CONTAINERS_LANDMARK_MEASUREMENT_HPP
