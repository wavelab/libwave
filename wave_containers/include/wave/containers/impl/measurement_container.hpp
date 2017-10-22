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
using boost::multi_index::ordered_non_unique;
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
struct container_traits<::wave::MeasurementContainer<T>> {
    // Define to give MeasurementContainerBase access to its derived type
    using MeasurementType = T;

    // Then, set up boost::multi_index_container
    // First, define which members of the Measurement object are used as keys
    // Specify that the time key corresponds to the time_point member
    struct time_key : member<T, TimeType, &T::time_point> {};
    // Specify that the sensor key corresponds to the sensor_id member
    struct sensor_key : member<T, decltype(T::sensor_id), &T::sensor_id> {};

    // Define a composite key which combines the above two keys
    // This lets us search elements sorted by time first, then  sensor id
    struct sensor_and_time_key : composite_key<T, time_key, sensor_key> {};

    // These types are used as tags to retrieve each index after the
    // multi_index_container is generated
    struct time_index {};
    struct sensor_index {};
    struct composite_index {};

    // Define an index for each key. Each index will be accessible via its tag
    struct indices
      : indexed_by<ordered_non_unique<tag<time_index>, time_key>,
                   ordered_non_unique<tag<sensor_index>, sensor_key>,
                   ordered_unique<tag<composite_index>, sensor_and_time_key>> {
    };
    // Finally, define the multi_index_container type.
    // This is the container type which can actually be used to make objects
    using type = boost::multi_index_container<T, indices, std::allocator<T>>;

    // For convenience, get the type of some indices, using their tags
    using composite_type = typename type::template index<composite_index>::type;
    using sensor_type = typename type::template index<sensor_index>::type;


    // Define a view indexed by time, for complex searches
    struct const_time_index
      : indexed_by<ordered_unique<member<T, const TimeType, &T::time_point>>> {
    };
    using time_view = boost::multi_index_container<const T *, const_time_index>;
};
}  // namespace internal

template <typename T>
MeasurementContainer<T>::MeasurementContainer() {}

template <typename T>
typename MeasurementContainer<T>::ValueType MeasurementContainer<T>::get(
  const TimeType &t, const SensorIdType &s) const {
    // To interpolate measurements from one sensor only, we first extract all
    // measurements from that sensor, then search by time. This method is based
    // on one described here:
    // http://www.boost.org/doc/libs/1_63_0/libs/multi_index/doc/examples.html#example6

    // Find all measurements from this sensor
    const auto &sensor_index =
      this->storage.template get<typename traits::sensor_index>();
    const auto sensor_it = sensor_index.equal_range(s);

    // Construct a view, indexed by time, with only those measurements
    auto time_view = typename traits::time_view{};
    for (auto it = sensor_it.first; it != sensor_it.second; ++it) {
        time_view.insert(&*it);  // the view holds pointers to the measurements
    }

    // find the first element with time >= t
    auto i_next = time_view.lower_bound(t);

    if (i_next == time_view.end()) {
        // Requested time is not between two measurements for this sensor
        throw std::out_of_range("MeasurementContainer::get");
    }

    const auto p_next = *i_next;

    if (t == p_next->time_point) {
        // Requested time exactly matches
        return p_next->value;
    }

    // If no exact match, need at least one more measurement to interpolate
    if (i_next == time_view.begin()) {
        // Requested time is not between two measurements for this sensor
        throw std::out_of_range("MeasurementContainer::get");
    }

    // Requested time is between two applicable measurements. Can interpolate.
    // Get pointer to the first element < t (since keys are unique)
    const auto i_prev = std::prev(i_next);
    const auto p_prev = *i_prev;
    return interpolate(*p_prev, *p_next, t);
}

}  // namespace wave
