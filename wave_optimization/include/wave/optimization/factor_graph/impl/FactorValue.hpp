#include "wave/utils/template_helpers.hpp"

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


namespace Eigen {
namespace internal {

template <typename T, typename O, int S>
struct traits<wave::FactorValue<T, O, S>>
  : traits<typename wave::FactorValue<T, O, S>::Base> {};

template <typename T, typename O, int S>
struct evaluator<wave::FactorValue<T, O, S>>
  : evaluator<typename wave::FactorValue<T, O, S>::Base> {};

}  // namespace internal
}  // namespace Eigen
