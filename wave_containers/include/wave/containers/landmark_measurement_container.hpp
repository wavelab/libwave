/**
 * @file
 * Specialized measurement container for landmark observations
 */
#ifndef WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP
#define WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/version.hpp>
#include <iterator>
#include <chrono>
#include <algorithm>

namespace wave {
/** @addtogroup containers
 *  @{ */


using TimeType = std::chrono::steady_clock::time_point;

/** Internal implementation details - for developers only */
namespace internal {

using boost::multi_index::member;
using boost::multi_index::indexed_by;
using boost::multi_index::ordered_unique;
using boost::multi_index::ordered_non_unique;
using boost::multi_index::composite_key;
using boost::multi_index::tag;

/**
 * The internal::landmark_container<T> holds all the type definitions
 * required for a multi_index_container holding measurements of type T.
 *
 * See `wave::internal::measurement_container`.
 */
template <typename T>
struct landmark_container {
    // First, define which members of the Measurement object are used as keys
    struct time_key : member<T, TimeType, &T::time_point> {};
    struct sensor_key : member<T, decltype(T::sensor_id), &T::sensor_id> {};
    struct landmark_key : member<T, decltype(T::landmark_id), &T::landmark_id> {
    };

    // Define a composite key which combines the above three keys
    // This lets us search elements sorted by time, then sensor id, then
    // landmark ID
    struct combined_key : composite_key<T, time_key, sensor_key, landmark_key> {
    };
    struct sensor_composite_key
      : composite_key<T, sensor_key, time_key, landmark_key> {};

    // These types are used as tags to retrieve each index after the
    // multi_index_container is generated
    struct time_index {};
    struct sensor_index {};
    struct sensor_composite_index {};
    struct landmark_index {};
    struct composite_index {};

    // Define an index for each key. Each index will be accessible via its tag
    struct indices
      : indexed_by<
          ordered_non_unique<tag<time_index>, time_key>,
          ordered_non_unique<tag<sensor_index>, sensor_key>,
          ordered_non_unique<tag<landmark_index>, landmark_key>,
          ordered_unique<tag<composite_index>, combined_key>,
          ordered_unique<tag<sensor_composite_index>, sensor_composite_key>> {};

    // Finally, define the multi_index_container type.
    // This is the container type which can actually be used to make objects
    using type = boost::multi_index_container<T, indices, std::allocator<T>>;

    // For convenience, get the type of the indices, using their tags
    using composite_type = typename type::template index<composite_index>::type;
    using time_type = typename type::template index<time_index>::type;
    using sensor_composite_type =
      typename type::template index<sensor_composite_index>::type;
    using landmark_type = typename type::template index<landmark_index>::type;

    // Define a view indexed by time, for complex searches
    struct const_time_index
      : indexed_by<
          ordered_non_unique<member<T, const TimeType, &T::time_point>>> {};
    using time_view = boost::multi_index_container<const T *, const_time_index>;
};
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

template <typename T>
LandmarkMeasurementContainer<T>::LandmarkMeasurementContainer() {}


template <typename T>
template <typename InputIt>
LandmarkMeasurementContainer<T>::LandmarkMeasurementContainer(InputIt first,
                                                              InputIt last) {
    this->composite().insert(first, last);
};

template <typename T>
std::pair<typename LandmarkMeasurementContainer<T>::iterator, bool>
LandmarkMeasurementContainer<T>::insert(const MeasurementType &m) {
    return this->composite().insert(m);
}

template <typename T>
template <typename InputIt>
void LandmarkMeasurementContainer<T>::insert(InputIt first, InputIt last) {
    return this->composite().insert(first, last);
}

template <typename T>
template <typename... Args>
std::pair<typename LandmarkMeasurementContainer<T>::iterator, bool>
LandmarkMeasurementContainer<T>::emplace(Args &&... args) {
// Support Boost.MultiIndex <= 1.54, which does not have emplace()
#if BOOST_VERSION < 105500
    return this->composite().insert(
      MeasurementType{std::forward<Args>(args)...});
#else
    return this->composite().emplace(std::forward<Args>(args)...);
#endif
}

template <typename T>
typename LandmarkMeasurementContainer<T>::size_type
LandmarkMeasurementContainer<T>::erase(const TimeType &t,
                                       SensorIdType s,
                                       LandmarkIdType id) {
    auto &composite = this->composite();
    auto it = composite.find(boost::make_tuple(t, s, id));
    if (it == composite.end()) {
        return 0;
    }
    composite.erase(it);
    return 1;
}

template <typename T>
typename LandmarkMeasurementContainer<T>::iterator
LandmarkMeasurementContainer<T>::erase(iterator position) noexcept {
    return this->composite().erase(position);
}

template <typename T>
typename LandmarkMeasurementContainer<T>::iterator
LandmarkMeasurementContainer<T>::erase(iterator first, iterator last) noexcept {
    return this->composite().erase(first, last);
}

template <typename T>
typename LandmarkMeasurementContainer<T>::ValueType
LandmarkMeasurementContainer<T>::get(const TimeType &t,
                                     SensorIdType s,
                                     LandmarkIdType id) const {
    const auto &composite = this->composite();

    auto iter = composite.find(boost::make_tuple(t, s, id));
    if (iter == composite.end()) {
        // Requested key is not in this container
        throw std::out_of_range("LandmarkMeasurementContainer::get");
    }
    return iter->value;
}

template <typename T>
std::pair<typename LandmarkMeasurementContainer<T>::sensor_iterator,
          typename LandmarkMeasurementContainer<T>::sensor_iterator>
LandmarkMeasurementContainer<T>::getAllFromSensor(const SensorIdType &s) const
  noexcept {
    // Get the measurements sorted by sensor_id
    const auto &sensor_composite_index = this->storage.template get<
      typename internal::landmark_container<T>::sensor_composite_index>();

    return sensor_composite_index.equal_range(s);
};

template <typename T>
std::pair<typename LandmarkMeasurementContainer<T>::iterator,
          typename LandmarkMeasurementContainer<T>::iterator>
LandmarkMeasurementContainer<T>::getTimeWindow(const TimeType &start,
                                               const TimeType &end) const
  noexcept {
    // Consider a "backward" window empty
    if (start > end) {
        return {this->end(), this->end()};
    }

    // The composite index is already sorted by time first, thus it's enough to
    // do a partial search. Find the start and end of the range.
    const auto &composite = this->composite();
    auto iter_begin = composite.lower_bound(boost::make_tuple(start));
    auto iter_end = composite.upper_bound(boost::make_tuple(end));

    return {iter_begin, iter_end};
}

template <typename T>
std::vector<typename LandmarkMeasurementContainer<T>::LandmarkIdType>
LandmarkMeasurementContainer<T>::getLandmarkIDs() const {
    return this->getLandmarkIDsInWindow(TimeType::min(), TimeType::max());
}

template <typename T>
std::vector<typename LandmarkMeasurementContainer<T>::LandmarkIdType>
LandmarkMeasurementContainer<T>::getLandmarkIDsInWindow(
  const TimeType &start, const TimeType &end) const {
    // Use the index sorted by landmark id
    const auto &landmark_index = this->storage.template get<
      typename internal::landmark_container<T>::landmark_index>();
    auto unique_ids = std::vector<LandmarkIdType>{};

    // Iterate over all measurements sorted by time, first then landmark_id.
    // Copy landmark ids into a vector, skipping consecutive equal elements and
    // elements outside the desired time window.
    for (auto &meas : landmark_index) {
        if (unique_ids.empty() || meas.landmark_id != unique_ids.back()) {
            if (meas.time_point >= start && meas.time_point <= end) {
                unique_ids.push_back(meas.landmark_id);
            }
        }
    }

    // Note there are other ways to do this, e.g. keeping another index, or
    // performing a "complex search" by generating a view. Pending performance
    // analysis, this method seems simplest.

    return unique_ids;
}

template <typename T>
typename LandmarkMeasurementContainer<T>::Track
LandmarkMeasurementContainer<T>::getTrack(const SensorIdType &s,
                                          const LandmarkIdType &id) const
  noexcept {
    return this->getTrackInWindow(s, id, TimeType::min(), TimeType::max());
};

template <typename T>
typename LandmarkMeasurementContainer<T>::Track
LandmarkMeasurementContainer<T>::getTrackInWindow(const SensorIdType &s,
                                                  const LandmarkIdType &id,
                                                  const TimeType &start,
                                                  const TimeType &end) const
  noexcept {
    // Consider a "backwards" window empty
    if (start >= end) {
        return Track{};
    }

    const auto &landmark_index = this->storage.template get<
      typename internal::landmark_container<T>::landmark_index>();

    // Get all measurements with desired landmark id
    const auto res = landmark_index.equal_range(id);

    // We need to get a track sorted by time, but landmark_index is not.
    // Construct a view, indexed by time, with only those measurements
    //
    // This method is based on one described here:
    // http://www.boost.org/doc/libs/1_63_0/libs/multi_index/doc/examples.html#example6
    //
    // While iterating, pick the measurements with desired sensor_id
    auto time_view = typename internal::landmark_container<T>::time_view{};
    for (auto it = res.first; it != res.second; ++it) {
        if (it->sensor_id == s) {
            // insert a pointer to the measurement
            time_view.insert(&*it);
        }
    }

    // Narrow down to the desired time window
    const auto iter_begin = time_view.lower_bound(start);
    const auto iter_end = time_view.upper_bound(end);

    // Build a vector holding copies of the measurements
    // Remember time_view holds pointers, so we can't copy it directly
    auto track = Track{};
    for (auto it = iter_begin; it != iter_end; ++it) {
        track.push_back(**it);
    }
    return track;
};

template <typename T>
bool LandmarkMeasurementContainer<T>::empty() const noexcept {
    return this->composite().empty();
}

template <typename T>
typename LandmarkMeasurementContainer<T>::size_type
LandmarkMeasurementContainer<T>::size() const noexcept {
    return this->composite().size();
}

template <typename T>
void LandmarkMeasurementContainer<T>::clear() noexcept {
    return this->composite().clear();
}

template <typename T>
typename LandmarkMeasurementContainer<T>::iterator
LandmarkMeasurementContainer<T>::begin() noexcept {
    return this->composite().begin();
}

template <typename T>
typename LandmarkMeasurementContainer<T>::iterator
LandmarkMeasurementContainer<T>::end() noexcept {
    return this->composite().end();
}

template <typename T>
typename LandmarkMeasurementContainer<T>::const_iterator
LandmarkMeasurementContainer<T>::begin() const noexcept {
    return this->composite().begin();
}

template <typename T>
typename LandmarkMeasurementContainer<T>::const_iterator
LandmarkMeasurementContainer<T>::end() const noexcept {
    return this->composite().end();
}

template <typename T>
typename LandmarkMeasurementContainer<T>::const_iterator
LandmarkMeasurementContainer<T>::cbegin() const noexcept {
    return this->composite().cbegin();
}

template <typename T>
typename LandmarkMeasurementContainer<T>::const_iterator
LandmarkMeasurementContainer<T>::cend() const noexcept {
    return this->composite().cend();
}

template <typename T>
typename LandmarkMeasurementContainer<T>::composite_type &
LandmarkMeasurementContainer<T>::composite() noexcept {
    return this->storage.template get<
      typename internal::landmark_container<T>::composite_index>();
}

template <typename T>
const typename LandmarkMeasurementContainer<T>::composite_type &
LandmarkMeasurementContainer<T>::composite() const noexcept {
    return this->storage.template get<
      typename internal::landmark_container<T>::composite_index>();
}

/** @} end of group */
}  // namespace wave

#endif  // WAVE_CONTAINERS_LANDMARK_MEASUREMENT_CONTAINER_HPP
