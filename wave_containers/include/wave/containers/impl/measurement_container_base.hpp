#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/version.hpp>

namespace wave {

/** Construct the container with the contents of a range */
template <typename T>
template <typename InputIt>
MeasurementContainerBase<T>::MeasurementContainerBase(InputIt first,
                                                      InputIt last) {
    this->composite().insert(first, last);
};

template <typename T>
bool MeasurementContainerBase<T>::empty() const noexcept {
    return this->composite().empty();
}

template <typename T>
typename MeasurementContainerBase<T>::size_type
MeasurementContainerBase<T>::size() const noexcept {
    return this->composite().size();
}

template <typename T>
std::pair<typename MeasurementContainerBase<T>::iterator, bool>
MeasurementContainerBase<T>::insert(const MeasurementType &m) {
    return this->composite().insert(m);
}

template <typename T>
template <typename InputIt>
void MeasurementContainerBase<T>::insert(InputIt first, InputIt last) {
    return this->composite().insert(first, last);
}

template <typename T>
template <typename... Args>
std::pair<typename MeasurementContainerBase<T>::iterator, bool>
MeasurementContainerBase<T>::emplace(Args &&... args) {
// Support Boost.MultiIndex <= 1.54, which does not have emplace()
#if BOOST_VERSION < 105500
    return this->composite().insert(
      MeasurementType{std::forward<Args>(args)...});
#else
    return this->composite().emplace(std::forward<Args>(args)...);
#endif
}

template <typename T>
template <typename... Args>
typename MeasurementContainerBase<T>::size_type
MeasurementContainerBase<T>::erase(Args &&... args) {
    auto &composite = this->composite();
    auto it = composite.find(boost::make_tuple(std::forward<Args>(args)...));
    if (it == composite.end()) {
        return 0;
    }
    composite.erase(it);
    return 1;
}

template <typename T>
typename MeasurementContainerBase<T>::iterator
MeasurementContainerBase<T>::erase(iterator position) noexcept {
    return this->composite().erase(position);
}

template <typename T>
typename MeasurementContainerBase<T>::iterator
MeasurementContainerBase<T>::erase(iterator first, iterator last) noexcept {
    return this->composite().erase(first, last);
}

template <typename T>
void MeasurementContainerBase<T>::clear() noexcept {
    return this->composite().clear();
}

template <typename T>
std::pair<typename MeasurementContainerBase<T>::sensor_iterator,
          typename MeasurementContainerBase<T>::sensor_iterator>
MeasurementContainerBase<T>::getAllFromSensor(const SensorIdType &s) const
  noexcept {
    // Get the measurements sorted by sensor_id
    const auto &sensor_index =
      this->storage.template get<typename traits::sensor_index>();

    return sensor_index.equal_range(s);
};

template <typename T>
std::pair<typename MeasurementContainerBase<T>::iterator,
          typename MeasurementContainerBase<T>::iterator>
MeasurementContainerBase<T>::getTimeWindow(const TimeType &start,
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
typename MeasurementContainerBase<T>::iterator
MeasurementContainerBase<T>::begin() noexcept {
    return this->composite().begin();
}

template <typename T>
typename MeasurementContainerBase<T>::iterator
MeasurementContainerBase<T>::end() noexcept {
    return this->composite().end();
}

template <typename T>
typename MeasurementContainerBase<T>::const_iterator
MeasurementContainerBase<T>::begin() const noexcept {
    return this->composite().begin();
}

template <typename T>
typename MeasurementContainerBase<T>::const_iterator
MeasurementContainerBase<T>::end() const noexcept {
    return this->composite().end();
}

template <typename T>
typename MeasurementContainerBase<T>::const_iterator
MeasurementContainerBase<T>::cbegin() const noexcept {
    return this->composite().cbegin();
}

template <typename T>
typename MeasurementContainerBase<T>::const_iterator
MeasurementContainerBase<T>::cend() const noexcept {
    return this->composite().cend();
}

template <typename T>
typename MeasurementContainerBase<T>::composite_type &
MeasurementContainerBase<T>::composite() noexcept {
    return this->storage.template get<typename traits::composite_index>();
}

template <typename T>
const typename MeasurementContainerBase<T>::composite_type &
MeasurementContainerBase<T>::composite() const noexcept {
    return this->storage.template get<typename traits::composite_index>();
}


}  // namespace wave
