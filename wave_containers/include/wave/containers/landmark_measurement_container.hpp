/**
 * @file
 * Specialized measurement container for landmark observations
 */
#ifndef WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP
#define WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP

#include <chrono>

namespace wave {
/** @addtogroup containers
 *  @{ */

using TimeType = std::chrono::steady_clock::time_point;

/** Internal implementation details - for developers only */
namespace internal {

template <typename T>
struct landmark_container;

}  // namespace internal

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
 */
template <typename T>
class LandmarkMeasurementContainer {
 public:
    // Types

    /** Alias for the template parameter, giving the type of measurement stored
     * in this container */
    using MeasurementType = T;
    /** Alias for the measurement's value.
     * Note this does *not* correspond to a typical container's value_type. */
    using ValueType = decltype(MeasurementType::value);
    /** Alias for the type of the sensor id */
    using SensorIdType = decltype(MeasurementType::sensor_id);
    /** Alias for the type of the landmark id */
    using LandmarkIdType = decltype(MeasurementType::landmark_id);
    /** A vector representing landmark / feature measurements across images */
    using Track = std::vector<MeasurementType>;


    using iterator =
      typename internal::landmark_container<T>::composite_type::iterator;
    using const_iterator =
      typename internal::landmark_container<T>::composite_type::const_iterator;
    using sensor_iterator =
      typename internal::landmark_container<T>::sensor_composite_type::iterator;
    using size_type = std::size_t;

    // Constructors

    /** Default construct an empty container */
    LandmarkMeasurementContainer();

    /** Construct the container with the contents of the range [first, last) */
    template <typename InputIt>
    LandmarkMeasurementContainer(InputIt first, InputIt last);

    // Capacity

    /** Return true if the container has no elements. */
    bool empty() const noexcept;

    /** Return the number of elements in the container. */
    size_type size() const noexcept;

    // Modifiers

    /** Insert a Measurement if a measurement for the same time and sensor does
     * not already exist.
     *
     * @return a pair p. If and only if insertion occurred, p.second is true and
     * p.first points to the element inserted.
     */
    std::pair<iterator, bool> insert(const MeasurementType &m);

    /** For each element of the range [first, last), inserts a Measurement if a
     * measurement for the same time and sensor does not already exist.
     *
     * @param first, last iterators representing a valid range of Measurements,
     * but not iterators into this container
     */
    template <typename InputIt>
    void insert(InputIt first, InputIt last);

    /** Insert a Measurement constructed from the arguments if a measurement for
     * the same time and sensor does not already exist.
     *
     * @return a pair p. If and only if insertion occurred, p.second is true and
     * p.first points to the element inserted.
     */
    template <typename... Args>
    std::pair<iterator, bool> emplace(Args &&... args);

    /** Delete the element with the matching time, sensor, and landmark id if
     * one exists.
     *
     * @return the number of elements deleted.
     */
    size_type erase(const TimeType &t, SensorIdType s, LandmarkIdType id);

    /** Delete the element at `position`
     *
     * @param position a valid dereferenceable iterator of this container
     * @return An iterator pointing to the element following the deleted one, or
     * `end()` if it was the last.
     */
    iterator erase(iterator position) noexcept;


    /** Delete the elements in the range [first, last)
     *
     * @param first, last a valid range of this container
     * @return `last`
     */
    iterator erase(iterator first, iterator last) noexcept;

    /** Delete all elements */
    void clear() noexcept;

    // Retrieval

    /** Gets the value of a landmark measurement.
     *
     * @throw std::out_of_range if a measurement with exactly matching time,
     * sensor, and landmark id does not exist.
     *
     * No interpolation is performed.
     */
    ValueType get(const TimeType &t, SensorIdType s, LandmarkIdType id) const;

    /** Get all measurements from the given sensor
     *
     * @return a pair of iterators representing the start and end of the range.
     * If the range is empty, both iterators will be equal.
     *
     * @note because these iterators use the underlying ordered index of
     * sensor_ids, they are not the same type as those from `begin()`,
     * `getTimeWindow()`, etc.
     */
    std::pair<sensor_iterator, sensor_iterator> getAllFromSensor(
      const SensorIdType &s) const noexcept;

    /** Get all measurements between the given times.
     *
     * @param start, end an inclusive range of times, with start <= end
     *
     * @return a pair of iterators representing the start and end of the range.
     * If the range is empty, both iterators will be equal.
     */
    std::pair<iterator, iterator> getTimeWindow(const TimeType &start,
                                                const TimeType &end) const
      noexcept;

    /** Get a list of all unique landmark IDs in the container */
    std::vector<LandmarkIdType> getLandmarkIDs() const;

    /** Get unique landmark IDs with measurements in the time window */
    std::vector<LandmarkIdType> getLandmarkIDsInWindow(
      const TimeType &start, const TimeType &end) const;

    /** Get a sequence of measurements of a landmark from one sensor
     * @return a vector of landmark measurements sorted by time
     *
     */
    Track getTrack(const SensorIdType &s, const LandmarkIdType &id) const
      noexcept;

    /** Get a sequnce of measurements of a landmark from one sensor, in the
     * given
     * time window.
     *
     * @return a vector of landmark measurements sorted by time
      */
    Track getTrackInWindow(const SensorIdType &s,
                           const LandmarkIdType &id,
                           const TimeType &start,
                           const TimeType &end) const noexcept;

    // Iterators

    iterator begin() noexcept;
    iterator end() noexcept;
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;
    const_iterator cbegin() const noexcept;
    const_iterator cend() const noexcept;

 protected:
    using composite_type =
      typename internal::landmark_container<T>::composite_type;

    // Helper to get the composite index
    composite_type &composite() noexcept;
    const composite_type &composite() const noexcept;

    // Internal multi_index_container
    typename internal::landmark_container<T>::type storage;
};

/** @} group containers */
}  // namespace wave

#include "impl/landmark_measurement_container.hpp"

#endif  // WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP
