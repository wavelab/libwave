#ifndef WAVE_UTILS_MEASUREMENT_HPP
#define WAVE_UTILS_MEASUREMENT_HPP

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/version.hpp>
#include <iterator>
#include <chrono>

namespace wave {

using boost::multi_index::member;
using boost::multi_index::indexed_by;
using boost::multi_index::ordered_unique;
using boost::multi_index::ordered_non_unique;
using boost::multi_index::composite_key;
using boost::multi_index::composite_key_compare;
using boost::multi_index::tag;

using TimeType = std::chrono::steady_clock::time_point;

/** Measurement type to store in the container */
template<typename T, typename S>
struct Measurement {
    TimeType time_point;
    S sensor_id;
    T value;

    Measurement(const TimeType &t, const S &s, const T &v) :
            time_point{t},
            sensor_id{s},
            value{v} {}
};

/** Perform linear interpolation between two measurements
 *
 * This template can be specialized for measurement types requiring a
 * different calculation
 */
template<typename T>
decltype(T::value) interpolate(const T &m1, const T &m2, const TimeType &t) {
    auto w2 = 1.0 * (t - m1.time_point) / (m2.time_point - m1.time_point);
    return (1 - w2) * m1.value + w2 * m2.value;
};

/** Container which stores and transparently interpolates measurements.
 *
 * @tparam T is the measurement type which must have the fields time_point (of
 * type TimeType), sensor_id (of any type), and value (of any type).
 *
 * The function interpolate(const T&, const T&, const TimeType&) must be defined
 * for type T.
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

 private:
    /* For now, some types used in the public interface are derived from the
     * private storage_type. That's why this private block appears here */
    // Tags for accessing indices in the multi_index_container
    struct time_index {};
    struct sensor_index {};
    struct composite_index {};

    // Define a container able to index by time, sensor id, or both
    struct time_member : member<MeasurementType,
                                TimeType,
                                &MeasurementType::time_point> {
    };
    struct sensor_id_member : member<MeasurementType,
                                     SensorIdType,
                                     &MeasurementType::sensor_id> {
    };
    // Note the composite index is sorted by sensor id first, then time
    struct composite_key_type : composite_key<MeasurementType,
                                              sensor_id_member,
                                              time_member> {
    };
    struct storage_type_indices : indexed_by<
            ordered_non_unique<tag<time_index>, time_member>,
            ordered_non_unique<tag<sensor_index>, sensor_id_member>,
            ordered_unique<tag<composite_index>, composite_key_type>> {
    };
    // Finally, define actual multi_index_container type. The separate struct
    // definitions above reduce the length of the name printed by the compiler
    using storage_type = boost::multi_index_container<
            MeasurementType,
            storage_type_indices,
            std::allocator<MeasurementType>>;
    using composite_type = typename storage_type::template index<composite_index>::type;

 public:
    // More types

    using iterator = typename composite_type::iterator;
    using const_iterator = typename composite_type::const_iterator;
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
    // Helper to get the composite index
    composite_type &composite() noexcept;
    const composite_type &composite() const noexcept;

    // Internal multi_index_container
    storage_type storage;
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
    return this->storage.template get<composite_index>();
}

template<typename T>
const typename MeasurementContainer<T>::composite_type &
MeasurementContainer<T>::composite() const noexcept {
    return this->storage.template get<composite_index>();
}

}  // namespace wave

#endif  // WAVE_MEASUREMENT_HPP
