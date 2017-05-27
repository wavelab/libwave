/**
 * @file
 * @ingroup containers
 *
 * Container which stores and transparently interpolates measurements from
 * multiple sensors.
 *
 * @defgroup containers
 * Storage types for data used by other modules.
 */

#ifndef WAVE_CONTAINERS_MEASUREMENT_CONTAINER_HPP
#define WAVE_CONTAINERS_MEASUREMENT_CONTAINER_HPP

#include "wave/containers/measurement_container_base.hpp"

namespace wave {
/** @addtogroup containers
 *  @{ */

/** Container which stores and transparently interpolates measurements.
 *
 * @tparam T is the stored measurement type. The Measurement class template
 * in wave/containers/measurement.hpp is designed to be used here.
 *
 * However, any class can be used that has the following public members:
 *   - `time_point` (of type `wave::TimeType`)
 *   - `sensor_id` (any type sortable by `std::less`)
 *   - `value` (any type)
 *
 * Additionally, the non-member function
 *   ```
 *   interpolate(const T&, const T&, const TimeType&)
 *   ```
 * must be defined for type `T`.
 */
template <typename T>
class MeasurementContainer
  : public MeasurementContainerBase<MeasurementContainer<T>> {
 public:
    // Constructors

    /** Default construct an empty container */
    MeasurementContainer();

    /** Inherit all non-default constuctors from base */
    using MeasurementContainerBase<
      MeasurementContainer<T>>::MeasurementContainerBase;

    // Types

    /** Alias for the template parameter, giving the type of Measurement stored
     * in this container */
    using MeasurementType = T;
    /** Alias for the measurement's value.
     * Note this does *not* correspond to a typical container's value_type. */
    using ValueType = decltype(MeasurementType::value);
    /** Alias for template parameter giving the type of the sensor id */
    using SensorIdType = decltype(MeasurementType::sensor_id);

    using iterator = typename internal::container_traits<
      MeasurementContainer<T>>::composite_type::iterator;
    using const_iterator = typename internal::container_traits<
      MeasurementContainer<T>>::composite_type::const_iterator;
    using sensor_iterator = typename internal::container_traits<
      MeasurementContainer<T>>::sensor_type::iterator;
    using size_type = std::size_t;

    // Retrieval

    /** Get the value of a measurement with corresponding time and sensor id */
    ValueType get(const TimeType &t, const SensorIdType &s) const;
};

/** @} group containers */
}  // namespace wave

#include "impl/measurement_container.hpp"

#endif  // WAVE_CONTAINERS_MEASUREMENT_CONTAINER_HPP
