#include "wave/optimization/factor_graph/template_helpers.hpp"

namespace wave {

namespace internal {

template <typename T, typename O, int S>
struct FactorValueAlias {
    using type = Eigen::Matrix<T, S, 1>;
};

template <typename T, int S>
struct FactorValueAlias<T, FactorValueOptions::Map, S> {
    using type = Eigen::Map<Eigen::Matrix<T, S, 1>>;
};

}  // namespace internal


}  // namespace wave
