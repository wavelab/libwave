/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP

#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/Noise.hpp"
#include "wave/optimization/factor_graph/OutputMap.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

/** The default NoiseType parameter to FactorMeasurement */
template <typename V>
using DefaultNoiseType = DiagonalNoise<FactorVariable<V>::Size>;

/**
 * A measurement, with associated noise, associated with a Factor
 * @tparam V the type of ValueView representing the measurement's value
 * @tparam NoiseTmpl the type of noise
 */
template <typename V, typename N = DefaultNoiseType<V>>
class FactorMeasurement : public FactorVariable<V> {
    using Base = FactorVariable<V>;

 public:
    using VarType = Base;
    using ViewType = typename Base::ViewType;
    using NoiseType = N;
    using MappedType = typename Base::MappedType;
    constexpr static int Size = Base::Size;

    /** Construct with initial value and initial noise value*/
    explicit FactorMeasurement(MappedType initial,
                               typename NoiseType::InitType noise_value)
        : Base{std::move(initial)}, noise{std::move(noise_value)} {}

    /** Construct with initial value, only for variables of size one*/
    explicit FactorMeasurement(double initial,
                               typename NoiseType::InitType noise_value)
        : Base{initial}, noise{std::move(noise_value)} {}

    /** Construct with initial value and initial noise object*/
    explicit FactorMeasurement(MappedType initial, NoiseType noise)
        : Base{std::move(initial)}, noise{std::move(noise)} {}

    /** Construct with initial value and no noise
     * Only allowed when NoiseType is void*/
    explicit FactorMeasurement(MappedType initial) : Base{std::move(initial)} {
        static_assert(std::is_void<NoiseType>::value,
                      "A noise value must be provided as the second argument");
    }

    NoiseType noise;
};


/**
 * Partially specialized FactorMeasurement with zero noise
 *
 * This specialization can be constructed with a measured value only, without
 * needing to specify noise.
 */
template <typename V>
class FactorMeasurement<V, void> : public FactorVariable<V> {
    using Base = FactorVariable<V>;

 public:
    using VarType = Base;
    using ViewType = typename Base::ViewType;
    using NoiseType = void;
    using MappedType = typename Base::MappedType;
    constexpr static int Size = Base::Size;

    /** Construct with initial value and no noise */
    explicit FactorMeasurement(MappedType initial) : Base{std::move(initial)} {}

    /** Construct with initial value, only for variables of size one*/
    explicit FactorMeasurement(double initial) : Base{initial} {}
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP
