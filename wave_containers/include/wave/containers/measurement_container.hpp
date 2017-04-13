#ifndef WAVE_CONTAINERS_MEASUREMENT_CONTAINER_HPP
#define WAVE_CONTAINERS_MEASUREMENT_CONTAINER_HPP

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/version.hpp>
#include <iterator>
#include <chrono>

namespace wave {

using TimeType = std::chrono::steady_clock::time_point;

/** Internal implementation details - for developers only */
namespace internal {

using boost::multi_index::member;
using boost::multi_index::indexed_by;
using boost::multi_index::ordered_unique;
using boost::multi_index::ordered_non_unique;
using boost::multi_index::composite_key;
using boost::multi_index::tag;

/** To easily accomplish this container's goal - provide access to measurements
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
 * The internal::measurement_container<T> holds all the type definitions
 * required for a multi_index_container holding measurements of type T. Note
 * this template is for convenience only, no objects are constructed.
 */
template<typename T>
struct measurement_container {
    // First, define which members of the Measurement object are used as keys
    // Specify that the time key corresponds to the time_point member
    struct time_key : member<T, TimeType, &T::time_point> {};
    // Specify that the sensor key corresponds to the sensor_id member
    struct sensor_key : member<T, decltype(T::sensor_id), &T::sensor_id> {};

    // Define a composite key which combines the above two keys
    // This lets us search elements sorted by sensor id first, then time
    struct sensor_and_time_key : composite_key<T, sensor_key, time_key> {};

    // These types are used as tags to retrieve each index after the
    // multi_index_container is generated
    struct time_index {};
    struct sensor_index {};
    struct composite_index {};

    // Define an index for each key. Each index will be accessible via its tag
    struct indices : indexed_by<
            ordered_non_unique<tag<time_index>, time_key>,
            ordered_non_unique<tag<sensor_index>, sensor_key>,
            ordered_unique<tag<composite_index>, sensor_and_time_key>> {
    };

    // Note we use separate struct definitions above, instead of typedefs, to
    // reduce the length of the name printed by the compiler.

    // Finally, define the multi_index_container type.
    // This is the container type which can actually be used to make objects
    using type = boost::multi_index_container<T, indices, std::allocator<T>>;

    // For convenience, get the type of the composite index, using its tag
    using composite_type = typename type::template index<composite_index>::type;
};
}  // namespace internal

/** Container which stores and transparently interpolates measurements.
 *
 * @tparam T is the stored measurement type. The Measurement class template
 * in wave/containers/measurement.hpp is designed to be used here.
 *
 * However, any class can be used that has the following public members:
 *   \li \c  time_point (of type \c wave::TimeType)
 *   \li \c sensor_id (any type sortable by \c std::less)
 *   \li \c value (any type)
 *
 * Additionally, the non-member function
 *   \code{.cpp}
 *   interpolate(const T&, const T&, const TimeType&)
 *   \endcode
 * must be defined for type \c T.
 */
template<typename T>
class MeasurementContainer {
 public:
    // Types

    /** Alias for the template parameter, giving the type of Measurement stored
     * in this container */
    using MeasurementType = T;
    /** Alias for the measurement's value.
     * Note this does *not* correspond to a typical container's value_type. */
    using ValueType = decltype(MeasurementType::value);
    /** Alias for template parameter giving the type of the sensor id */
    using SensorIdType = decltype(MeasurementType::sensor_id);

    using iterator = typename
    internal::measurement_container<T>::composite_type::iterator;
    using const_iterator = typename
    internal::measurement_container<T>::composite_type::const_iterator;
    using size_type = std::size_t;

    // Constructors

    /** Default construct an empty container */
    MeasurementContainer();

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
    std::pair<iterator, bool> insert(const MeasurementType &);

    /** Insert a Measurement constructed from the arguments if a measurement for
     * the same time and sensor does not already exist.
     *
     * @return a pair p. If and only if insertion occurred, p.second is true and
     * p.first points to the element inserted.
     */
    template<typename... Args>
    std::pair<iterator, bool> emplace(Args &&... args);

    /** Delete the element with the matching time and sensor id, if one exists.
     *
     * @return the number of elements deleted.
     */
    size_type erase(const TimeType &, const SensorIdType &);

    /** Delete all elements */
    void clear() noexcept;

    // Retrieval

    /** Get the value of a measurement with corresponding time and sensor id */
    const ValueType get(const TimeType &, const SensorIdType &) const;

    // Iterators

    iterator begin() noexcept;
    iterator end() noexcept;
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;
    const_iterator cbegin() const noexcept;
    const_iterator cend() const noexcept;

 private:
    using composite_type = typename
    internal::measurement_container<T>::composite_type;

    // Helper to get the composite index
    composite_type &composite() noexcept;
    const composite_type &composite() const noexcept;

    // Internal multi_index_container
    typename internal::measurement_container<T>::type storage;
};

template<typename T>
MeasurementContainer<T>::MeasurementContainer() {

}

template<typename T>
std::pair<typename MeasurementContainer<T>::iterator, bool>
MeasurementContainer<T>::insert(const MeasurementType &m) {
    return this->composite().insert(m);
}

template<typename T>
template<typename... Args>
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

template<typename T>
typename MeasurementContainer<T>::size_type
MeasurementContainer<T>::erase(const TimeType &t, const SensorIdType &s) {
    auto &composite = this->composite();
    auto it = composite.find(boost::make_tuple(s, t));
    if (it == composite.end()) {
        return 0;
    }
    composite.erase(it);
    return 1;
}

template<typename T>
const typename MeasurementContainer<T>::ValueType
MeasurementContainer<T>::get(const TimeType &t, const SensorIdType &s) const {
    const auto &composite = this->composite();

    // lower_bound is pointer to the first element >= key
    auto i_next = composite.lower_bound(boost::make_tuple(s, t));

    // Pointer to the first element < key (since keys are unique)
    auto i_prev = i_next;
    --i_prev;

    if (t == i_next->time_point && s == i_next->sensor_id) {
        // Requested time exactly matches
        return i_next->value;
    }

    // The search looks at sensor_id first, then time. Must check both.
    if (i_next == composite.end() || i_next == composite.begin() ||
            s != i_next->sensor_id || s != i_prev->sensor_id) {
        // Requested time is not between two measurements for this sensor
        throw std::out_of_range("MeasurementContainer::get");
    }

    // Requested time is between two applicable measurements
    return interpolate(*i_prev, *i_next, t);
}

template<typename T>
bool MeasurementContainer<T>::empty() const noexcept {
    return this->composite().empty();
}

template<typename T>
typename MeasurementContainer<T>::size_type
MeasurementContainer<T>::size() const noexcept {
    return this->composite().size();
}

template<typename T>
void MeasurementContainer<T>::clear() noexcept {
    return this->composite().clear();
}

template<typename T>
typename MeasurementContainer<T>::iterator
MeasurementContainer<T>::begin() noexcept {
    return this->composite().begin();
}

template<typename T>
typename MeasurementContainer<T>::iterator
MeasurementContainer<T>::end() noexcept {
    return this->composite().end();
}

template<typename T>
typename MeasurementContainer<T>::const_iterator
MeasurementContainer<T>::begin() const noexcept {
    return this->composite().begin();
}

template<typename T>
typename MeasurementContainer<T>::const_iterator
MeasurementContainer<T>::end() const noexcept {
    return this->composite().end();
}

template<typename T>
typename MeasurementContainer<T>::const_iterator
MeasurementContainer<T>::cbegin() const noexcept {
    return this->composite().cbegin();
}

template<typename T>
typename MeasurementContainer<T>::const_iterator
MeasurementContainer<T>::cend() const noexcept {
    return this->composite().cend();
}

template<typename T>
typename MeasurementContainer<T>::composite_type &
MeasurementContainer<T>::composite() noexcept {
    return this->storage.template
            get<typename internal::measurement_container<T>::composite_index>();
}

template<typename T>
const typename MeasurementContainer<T>::composite_type &
MeasurementContainer<T>::composite() const noexcept {
    return this->storage.template
            get<typename internal::measurement_container<T>::composite_index>();
}

}  // namespace wave

#endif  // WAVE_CONTAINERS_MEASUREMENT_CONTAINER_HPP
