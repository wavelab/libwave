namespace wave {

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
    // To interpolate measurements from one sensor only, we first extract all
    // measurements from that sensor, then search by time. This method is based
    // on one described here:
    // http://www.boost.org/doc/libs/1_63_0/libs/multi_index/doc/examples.html#example6

    // Find all measurements from this sensor
    const auto &sensor_index = this->storage.template get<
      typename internal::measurement_container<T>::sensor_index>();
    const auto sensor_it = sensor_index.equal_range(s);

    // Construct a view, indexed by time, with only those measurements
    auto time_view = typename internal::measurement_container<T>::time_view{};
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
