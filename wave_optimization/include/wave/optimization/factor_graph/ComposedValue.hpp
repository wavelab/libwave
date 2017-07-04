/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_COMPOSEDVALUE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_COMPOSEDVALUE_HPP

#include "wave/optimization/factor_graph/FactorValue.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

template <typename Derived, template <typename...> class... V>
class ComposedValue;

template <template <typename, typename> class D,
          typename Scalar,
          typename Options,
          template <typename...> class... V>
class ComposedValue<D<Scalar, Options>, V...> {
    using Derived = D<Scalar, Options>;

 public:
    using BlockSizes = tmp::index_sequence<V<Scalar>::SizeAtCompileTime...>;
    constexpr static int Size = tmp::sum_index_sequence<BlockSizes>::value;
    using ComposedMatrix = FactorValue<Scalar, Options, Size>;
    using ValueTuple = std::tuple<V<Scalar, Options>...>;
    using ValueIndices = typename tmp::cumulative_index<BlockSizes>::type;

    ComposedValue() : mat{ComposedMatrix::Zero()} {}

    explicit ComposedValue(V<Scalar, Options>... args) {
        this->initMatrix(tmp::make_index_sequence<sizeof...(V)>{},
                         std::move(args)...);
    }

    /** Initialize from pointer to raw array (for Map variant only) */
    explicit ComposedValue(Scalar *dataptr) : mat{dataptr} {}

    Scalar *data() noexcept {
        return this->mat.data();
    }

    // Arithmetic operators

    Derived &operator-=(const Derived &rhs) {
        this->mat -= rhs.mat;
        return static_cast<Derived &>(*this);
    }

    const Derived operator-(const Derived &rhs) const {
        return Derived{*static_cast<Derived const *>(this)} -= rhs;
    }

 protected:
    template <int I>
    Eigen::Ref<typename std::tuple_element<I, ValueTuple>::type>
    block() noexcept {
        const auto i = tmp::index_sequence_element<I, ValueIndices>::value;
        const auto size = tmp::index_sequence_element<I, BlockSizes>::value;
        return this->mat.template segment<size>(i);
    }

 private:
    template <int... Is>
    void initMatrix(tmp::index_sequence<Is...>, V<Scalar, Options>... args) {
        auto loop = {(this->block<Is>() = std::move(args), 0)...};
        (void) loop;
    }

    ComposedMatrix mat;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_COMPOSEDVALUE_HPP
