#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/composite_key.hpp>
#include <boost/version.hpp>

namespace wave {

/** Construct the container with the contents of a range */
template <typename Derived>
template <typename InputIt>
MeasurementContainerBase<Derived>::MeasurementContainerBase(InputIt first,
                                                            InputIt last) {
    this->composite().insert(first, last);
};

template <typename Derived>
bool MeasurementContainerBase<Derived>::empty() const noexcept {
    return this->composite().empty();
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::size_type
MeasurementContainerBase<Derived>::size() const noexcept {
    return this->composite().size();
}

template <typename Derived>
std::pair<typename MeasurementContainerBase<Derived>::iterator, bool>
MeasurementContainerBase<Derived>::insert(const MeasurementType &m) {
    return this->composite().insert(m);
}

template <typename Derived>
template <typename InputIt>
void MeasurementContainerBase<Derived>::insert(InputIt first, InputIt last) {
    return this->composite().insert(first, last);
}

template <typename Derived>
template <typename... Args>
std::pair<typename MeasurementContainerBase<Derived>::iterator, bool>
MeasurementContainerBase<Derived>::emplace(Args &&... args) {
// Support Boost.MultiIndex <= 1.54, which does not have emplace()
#if BOOST_VERSION < 105500
    return this->composite().insert(
      MeasurementType{std::forward<Args>(args)...});
#else
    return this->composite().emplace(std::forward<Args>(args)...);
#endif
}

template <typename Derived>
template <typename... Args>
typename MeasurementContainerBase<Derived>::size_type
MeasurementContainerBase<Derived>::erase(Args &&... args) {
    auto &composite = this->composite();
    auto it = composite.find(boost::make_tuple(std::forward<Args>(args)...));
    if (it == composite.end()) {
        return 0;
    }
    composite.erase(it);
    return 1;
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::iterator
MeasurementContainerBase<Derived>::erase(iterator position) noexcept {
    return this->composite().erase(position);
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::iterator
MeasurementContainerBase<Derived>::erase(iterator first,
                                         iterator last) noexcept {
    return this->composite().erase(first, last);
}

template <typename Derived>
void MeasurementContainerBase<Derived>::clear() noexcept {
    return this->composite().clear();
}

template <typename Derived>
std::pair<typename MeasurementContainerBase<Derived>::sensor_iterator,
          typename MeasurementContainerBase<Derived>::sensor_iterator>
MeasurementContainerBase<Derived>::getAllFromSensor(const SensorIdType &s) const
  noexcept {
    // Get the measurements sorted by sensor_id
    const auto &sensor_index =
      this->storage.template get<typename traits::sensor_index>();

    return sensor_index.equal_range(s);
};

template <typename Derived>
std::pair<typename MeasurementContainerBase<Derived>::iterator,
          typename MeasurementContainerBase<Derived>::iterator>
MeasurementContainerBase<Derived>::getTimeWindow(const TimeType &start,
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

template <typename Derived>
typename MeasurementContainerBase<Derived>::iterator
MeasurementContainerBase<Derived>::begin() noexcept {
    return this->composite().begin();
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::iterator
MeasurementContainerBase<Derived>::end() noexcept {
    return this->composite().end();
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::const_iterator
MeasurementContainerBase<Derived>::begin() const noexcept {
    return this->composite().begin();
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::const_iterator
MeasurementContainerBase<Derived>::end() const noexcept {
    return this->composite().end();
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::const_iterator
MeasurementContainerBase<Derived>::cbegin() const noexcept {
    return this->composite().cbegin();
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::const_iterator
MeasurementContainerBase<Derived>::cend() const noexcept {
    return this->composite().cend();
}

template <typename Derived>
typename MeasurementContainerBase<Derived>::composite_type &
MeasurementContainerBase<Derived>::composite() noexcept {
    return this->storage.template get<typename traits::composite_index>();
}

template <typename Derived>
const typename MeasurementContainerBase<Derived>::composite_type &
MeasurementContainerBase<Derived>::composite() const noexcept {
    return this->storage.template get<typename traits::composite_index>();
}


}  // namespace wave
