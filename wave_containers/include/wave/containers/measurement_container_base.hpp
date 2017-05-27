/**
 * @file
 * @ingroup containers
 */

#ifndef WAVE_CONTAINERS_MEASUREMENT_CONTAINER_BASE_HPP
#define WAVE_CONTAINERS_MEASUREMENT_CONTAINER_BASE_HPP

#include <chrono>

namespace wave {

/** @addtogroup containers
 *  @{ */

using TimeType = std::chrono::steady_clock::time_point;

/** Internal implementation details - for developers only */
namespace internal {

/** Internal traits class template. Must be specialized for each derived type
 * of MeasurementContainerBase. */
template <typename Derived>
struct container_traits;

}  // namespace internal

/** Base class for containers which store measurements.
 *
 * @tparam Derived is the derived type, e.g. a landmark measurement container
 */
template <typename Derived>
class MeasurementContainerBase {
    using traits = internal::container_traits<Derived>;

 public:
    // Types

    /** Alias for the template parameter, giving the type of Measurement stored
     * in this container */
    using MeasurementType = typename traits::MeasurementType;
    /** Alias for the measurement's value.
     * Note this does *not* correspond to a typical container's value_type. */
    using ValueType = decltype(MeasurementType::value);
    /** Alias for template parameter giving the type of the sensor id */
    using SensorIdType = decltype(MeasurementType::sensor_id);

    using iterator = typename traits::composite_type::iterator;
    using const_iterator = typename internal::container_traits<
      Derived>::composite_type::const_iterator;
    using sensor_iterator = typename traits::sensor_type::iterator;
    using size_type = std::size_t;

    // Constructors

    MeasurementContainerBase() = default;

    /** Construct the container with the contents of the range [first, last) */
    template <typename InputIt>
    MeasurementContainerBase(InputIt first, InputIt last);

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

    /** Delete the element with the matching time and sensor id, if one exists.
     *
     * @return the number of elements deleted.
     */
    size_type erase(const TimeType &t, const SensorIdType &s);

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


    // Iterators

    iterator begin() noexcept;
    iterator end() noexcept;
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;
    const_iterator cbegin() const noexcept;
    const_iterator cend() const noexcept;

 protected:
    using composite_type = typename traits::composite_type;

    // Helper to get the composite index
    composite_type &composite() noexcept;
    const composite_type &composite() const noexcept;

    // Internal multi_index_container
    typename traits::type storage;
};

/** @} group containers */
}  // namespace wave

#include "impl/measurement_container_base.hpp"

#endif  // WAVE_CONTAINERS_MEASUREMENT_CONTAINER_BASE_HPP
