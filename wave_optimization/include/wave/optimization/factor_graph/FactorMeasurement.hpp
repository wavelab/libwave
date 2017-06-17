/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP

#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/Noise.hpp"

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
    explicit FactorMeasurement(MappedType &&initial,
                               typename NoiseType::InitType &&noise)
        : Base{std::move(initial)}, noise{std::move(noise)} {}

    /** Construct copying initial value and initial noise value*/
    explicit FactorMeasurement(const MappedType &initial,
                               const typename NoiseType::InitType &noise)
        : Base{initial}, noise{noise} {}

    /** Construct with initial value and initial noise value*/
    explicit FactorMeasurement(MappedType &&initial, NoiseType &&noise)
        : Base{std::move(initial)}, noise{std::move(noise)} {}

    /** Construct copying initial value and initial noise value*/
    explicit FactorMeasurement(const MappedType &initial,
                               const NoiseType &noise)
        : Base{initial}, noise{noise} {}

    NoiseType noise;
};

/**
 * Partially specialized FactorMeasurement with zero noise
 */
template <typename V>
class FactorMeasurement<V, ZeroNoise> : public FactorVariable<V> {
    using Base = FactorVariable<V>;

 public:
    using ViewType = typename Base::ViewType;
    using NoiseType = ZeroNoise;
    using MappedType = typename Base::MappedType;
    constexpr static int Size = Base::Size;

    /** Construct with initial value */
    explicit FactorMeasurement(MappedType &&initial)
        : Base{std::move(initial)} {
        this->setFixed(true);
    }

    /** Construct copying initial value  */
    explicit FactorMeasurement(const MappedType &initial) : Base{initial} {}

    ZeroNoise noise;
};


template <typename V, typename N>
inline Eigen::Matrix<double, FactorMeasurement<V, N>::Size, 1> operator-(
  const Eigen::Ref<Eigen::Matrix<double, FactorMeasurement<V, N>::Size, 1>>
    &lhs,
  const FactorMeasurement<V, N> &rhs) {
    return lhs -
           Eigen::Map<
             const Eigen::Matrix<double, FactorMeasurement<V, N>::Size, 1>>{
             rhs.data()};
}


/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP
