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
 * holding measurements of type T. See `wave::internal::measurement_container`.
 */
template <typename T>
struct landmark_container {
    // First, define which members of the Measurement object are used as keys
    struct time_key : member<T, decltype(T::time_point), &T::time_point> {};
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
      : indexed_by<ordered_non_unique<
          member<T, const decltype(T::time_point), &T::time_point>>> {};
    using time_view = boost::multi_index_container<const T *, const_time_index>;
};
}  // namespace internal

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

    // Iterate over all measurements sorted by time first, then landmark_id.
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
    if (start > end) {
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

}  // namespace wave
