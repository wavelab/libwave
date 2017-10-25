#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/version.hpp>

namespace wave {

namespace internal {

using boost::multi_index::member;
using boost::multi_index::indexed_by;
using boost::multi_index::ordered_unique;
using boost::multi_index::composite_key;
using boost::multi_index::tag;

/** Holds all the type definitions required for a boost::multi_index_container
 * holding measurements of type T.
 *
 * To easily accomplish this container's goal - provide access to measurements
 * sorted by time, by sensor, or both - we wrap a Boost.MultiIndex container.
 *
 * The multi_index_container template generates code allowing us to index the
 * stored elements in different ways. Because these indices are specified at
 * compile-time, the multi_index_container is configured using template
 * parameters, which are defined here.
 *
 * See http://www.boost.org/doc/libs/1_63_0/libs/multi_index/doc/index.html for
 * details on Boost.MultiIndex.
 *
 * Note this template is for convenience only, no objects are constructed.
 */
template <typename T>
struct measurement_container {
    // First, define which members of the Measurement object are used as keys
    // Specify that the time key corresponds to the time_point member
    struct time_key : member<T, decltype(T::time_point), &T::time_point> {};
    // Specify that the sensor key corresponds to the sensor_id member
    struct sensor_key : member<T, decltype(T::sensor_id), &T::sensor_id> {};

    // Define a composite key sorted by time first, then sensor
    struct time_and_sensor_key : composite_key<T, time_key, sensor_key> {};

    // Define a composite key sorted by sensor first, then time
    struct sensor_and_time_key : composite_key<T, sensor_key, time_key> {};

    // These types are used as tags to retrieve each index after the
    // multi_index_container is generated
    struct sensor_index {};
    struct composite_index {};

    // Define an index for each key. Each index will be accessible via its tag
    struct indices
      : indexed_by<ordered_unique<tag<composite_index>, time_and_sensor_key>,
                   ordered_unique<tag<sensor_index>, sensor_and_time_key>> {};

    // Note we use separate struct definitions above, instead of typedefs, to
    // reduce the length of the name printed by the compiler.

    // Finally, define the multi_index_container type.
    // This is the container type which can actually be used to make objects
    using type = boost::multi_index_container<T, indices, std::allocator<T>>;

    // For convenience, get the type of some indices, using their tags
    using composite_type = typename type::template index<composite_index>::type;
    using sensor_type = typename type::template index<sensor_index>::type;
};
}  // namespace internal

template <typename T>
MeasurementContainer<T>::MeasurementContainer() {}

/** Construct the container with the contents of a range */
template <typename T>
template <typename InputIt>
MeasurementContainer<T>::MeasurementContainer(InputIt first, InputIt last) {
    this->composite().insert(first, last);
};

template <typename T>
std::pair<typename MeasurementContainer<T>::iterator, bool>
MeasurementContainer<T>::insert(const MeasurementType &m) {
    return this->composite().insert(m);
}

template <typename T>
template <typename InputIt>
void MeasurementContainer<T>::insert(InputIt first, InputIt last) {
    return this->composite().insert(first, last);
}

template <typename T>
template <typename... Args>
std::pair<typename MeasurementContainer<T>::iterator, bool>
MeasurementContainer<T>::emplace(Args &&... args) {
// Support Boost.MultiIndex <= 1.54, which does not have emplace()
#if BOOST_VERSION < 105500
    return this->composite().insert(
      MeasurementType{std::forward<Args>(args)...});
#else
    return this->composite().emplace(std::forward<Args>(args)...);
#endif
}

template <typename T>
typename MeasurementContainer<T>::size_type MeasurementContainer<T>::erase(
  const TimeType &t, const SensorIdType &s) {
    auto &composite = this->composite();
    auto it = composite.find(boost::make_tuple(t, s));
    if (it == composite.end()) {
        return 0;
    }
    composite.erase(it);
    return 1;
}

template <typename T>
typename MeasurementContainer<T>::iterator MeasurementContainer<T>::erase(
  iterator position) noexcept {
    return this->composite().erase(position);
}

template <typename T>
typename MeasurementContainer<T>::iterator MeasurementContainer<T>::erase(
  iterator first, iterator last) noexcept {
    return this->composite().erase(first, last);
}

template <typename T>
typename MeasurementContainer<T>::ValueType MeasurementContainer<T>::get(
  const TimeType &t, const SensorIdType &s) const {
    // To interpolate measurements from one sensor only, we use the index sorted
    // by sensor_id first
    const auto &sensor_index = this->storage.template get<
      typename internal::measurement_container<T>::sensor_index>();

    // find the first element with sensor>=s and time >= t
    auto i_next = sensor_index.lower_bound(boost::make_tuple(s, t));

    if (i_next == sensor_index.end() || i_next->sensor_id != s) {
        // Requested time is not between two measurements for this sensor
        throw std::out_of_range("MeasurementContainer::get");
    }

    if (t == i_next->time_point) {
        // Requested time exactly matches
        return i_next->value;
    }

    // If no exact match, need at least one previous measurement to interpolate
    const auto i_prev = std::prev(i_next);

    if (i_next == sensor_index.begin() || i_prev->sensor_id != s) {
        // Requested time is not between two measurements for this sensor
        throw std::out_of_range("MeasurementContainer::get");
    }

    // Requested time is between two applicable measurements. Can interpolate.
    // Get pointer to the first element < t (since keys are unique)
    return interpolate(*i_prev, *i_next, t);
}

template <typename T>
std::pair<typename MeasurementContainer<T>::sensor_iterator,
          typename MeasurementContainer<T>::sensor_iterator>
MeasurementContainer<T>::getAllFromSensor(const SensorIdType &s) const
  noexcept {
    // Get the measurements sorted by sensor_id
    const auto &sensor_index = this->storage.template get<
      typename internal::measurement_container<T>::sensor_index>();

    return sensor_index.equal_range(s);
};

template <typename T>
std::pair<typename MeasurementContainer<T>::iterator,
          typename MeasurementContainer<T>::iterator>
MeasurementContainer<T>::getTimeWindow(const TimeType &start,
                                       const TimeType &end) const noexcept {
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
bool MeasurementContainer<T>::empty() const noexcept {
    return this->composite().empty();
}

template <typename T>
typename MeasurementContainer<T>::size_type MeasurementContainer<T>::size()
  const noexcept {
    return this->composite().size();
}

template <typename T>
void MeasurementContainer<T>::clear() noexcept {
    return this->composite().clear();
}

template <typename T>
typename MeasurementContainer<T>::iterator
MeasurementContainer<T>::begin() noexcept {
    return this->composite().begin();
}

template <typename T>
typename MeasurementContainer<T>::iterator
MeasurementContainer<T>::end() noexcept {
    return this->composite().end();
}

template <typename T>
typename MeasurementContainer<T>::const_iterator
MeasurementContainer<T>::begin() const noexcept {
    return this->composite().begin();
}

template <typename T>
typename MeasurementContainer<T>::const_iterator MeasurementContainer<T>::end()
  const noexcept {
    return this->composite().end();
}

template <typename T>
typename MeasurementContainer<T>::const_iterator
MeasurementContainer<T>::cbegin() const noexcept {
    return this->composite().cbegin();
}

template <typename T>
typename MeasurementContainer<T>::const_iterator MeasurementContainer<T>::cend()
  const noexcept {
    return this->composite().cend();
}

template <typename T>
typename MeasurementContainer<T>::composite_type &
MeasurementContainer<T>::composite() noexcept {
    return this->storage.template get<
      typename internal::measurement_container<T>::composite_index>();
}

template <typename T>
const typename MeasurementContainer<T>::composite_type &
MeasurementContainer<T>::composite() const noexcept {
    return this->storage.template get<
      typename internal::measurement_container<T>::composite_index>();
}


}  // namespace wave
