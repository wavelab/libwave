/**
 * @file
 * Specialized measurement container for landmark observations
 */
#ifndef WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP
#define WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP

#include "wave/containers/measurement_container_base.hpp"

namespace wave {
/** @addtogroup containers
 *  @{ */

/** Container which stores landmark measurements.
 *
 * @tparam T is the stored measurement type. The `LandmarkMeasurement` class
 * template is designed to be used here.
 *
 * However, any class can be used that has the following public members:
 *   - `time_point` (of type \c wave::TimeType)
 *   - `sensor_id` (any type sortable by \c std::less)
 *   - `landmark_id` (any type sortable by \c std::less)
 *   - `value` (any type)
 *
 * The unique key for this container is the tuple (time_point, sensor_id,
 * landmark_id). That is, an insertion will have an effect only if there is not
 * already an element in the container with that key. When iterating over the
 * container, elements are sorted lexicographically: first by time, then by
 * sensor_id, then by landmark_id.
 */
template <typename T>
class LandmarkMeasurementContainer
  : public MeasurementContainerBase<LandmarkMeasurementContainer<T>> {
    using traits = internal::container_traits<LandmarkMeasurementContainer<T>>;

 public:
    using Base = MeasurementContainerBase<LandmarkMeasurementContainer<T>>;

    // Types

    /** Alias for the template parameter, giving the type of Measurement stored
     * in this container */
    using typename Base::MeasurementType;
    /** Alias for the measurement's value.
     * Note this does *not* correspond to a typical container's value_type. */
    using typename Base::ValueType;
    /** Alias for template parameter giving the type of the sensor id */
    using typename Base::SensorIdType;
    /** Alias for the type of the landmark id */
    using LandmarkIdType = decltype(MeasurementType::landmark_id);
    /** A vector representing landmark / feature measurements across images */
    using Track = std::vector<MeasurementType>;

    // Constructors

    /** Default construct an empty container */
    LandmarkMeasurementContainer();

    /** Inherit all non-default constuctors from base */
    using MeasurementContainerBase<
      LandmarkMeasurementContainer<T>>::MeasurementContainerBase;

    // Retrieval

    /** Gets the value of a landmark measurement.
     *
     * @throw std::out_of_range if a measurement with exactly matching time,
     * sensor, and landmark id does not exist.
     *
     * No interpolation is performed.
     */
    ValueType get(const TimeType &t, SensorIdType s, LandmarkIdType id) const;

    /** Get a list of all unique landmark IDs in the container */
    std::vector<LandmarkIdType> getLandmarkIDs() const;

    /** Get unique landmark IDs with measurements in the time window
     *
     * @return a vector of landmark IDs, in increasing order
     *
     * The window is inclusive. If `start > end`, the result will be empty.
     */
    std::vector<LandmarkIdType> getLandmarkIDsInWindow(
      const TimeType &start, const TimeType &end) const;

    /** Get a sequence of measurements of a landmark from one sensor
     * @return a vector of landmark measurements sorted by time
     *
     */
    Track getTrack(const SensorIdType &s, const LandmarkIdType &id) const
      noexcept;

    /** Get a sequence of measurements of a landmark from one sensor, in the
     * given time window.
     *
     * @return a vector of landmark measurements sorted by time
     *
     * The window is inclusive. If `start > end`, the result will be empty.
     */
    Track getTrackInWindow(const SensorIdType &s,
                           const LandmarkIdType &id,
                           const TimeType &start,
                           const TimeType &end) const noexcept;
};

/** @} group containers */
}  // namespace wave

#include "impl/landmark_measurement_container.hpp"

#endif  // WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP
