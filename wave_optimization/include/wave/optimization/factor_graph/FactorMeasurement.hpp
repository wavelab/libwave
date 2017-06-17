/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP

#include "wave/optimization/factor_graph/FactorVariable.hpp"


namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * A measurement associated with a Factor
 *
 * @tparam V the type of ValueView representing the measurement's value
 * @todo a Noise parameter will be added here
 */
template <typename V>
class FactorMeasurement : public FactorVariable<V> {
    using Base = FactorVariable<V>;

 public:
    using ViewType = typename Base::ViewType;
    using MappedType = typename Base::MappedType;
    constexpr static int Size = Base::Size;

    /** Construct with initial value */
    explicit FactorMeasurement(MappedType &&initial)
        : Base{std::move(initial)} {}

    /** Construct copying initial value  */
    explicit FactorMeasurement(const MappedType &initial) : Base{initial} {}
};

/** Subtract a measurement from the result of a measurement function
 * @return the difference: the non-normalized residual
 */
template <typename V>
inline Eigen::Matrix<double, 1, FactorMeasurement<V>::Size> operator-(
  const ResultOut<FactorMeasurement<V>::Size> &lhs,
  const FactorMeasurement<V> &rhs) {
    // We need to accept the specific types and manually convert them, as Eigen
    // 3.2 is not as great at automatically converting things. See #151
    return lhs - Eigen::Map<
                   const Eigen::Matrix<double, 1, FactorMeasurement<V>::Size>>{
                   rhs.data()};
}

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_MEASUREMENT_HPP
