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

template <typename Derived,
          typename Scalar,
          typename Options,
          template <typename...> class... V>
class ComposedValue {
 public:
    using ValueTuple = std::tuple<V<Scalar, Options>...>;

    ComposedValue() : blocks{V<Scalar, Options>::Zero()...} {}

    explicit ComposedValue(V<Scalar, Options>... args)
        : blocks{std::move(args)...} {}

    explicit ComposedValue(tmp::replacet<Scalar *, V>... args)
        : blocks{V<Scalar, FactorValueOptions::Map>{args}...} {}

    std::vector<Scalar *> blockData() noexcept {
        return this->blockDataImpl(tmp::make_index_sequence<sizeof...(V)>{});
    }

    // Arithmetic operators

    Derived &operator-=(const Derived &rhs) {
        this->blocks =
          tmp::transformTupleTmpl<std::minus>(this->blocks, rhs.blocks);
        return static_cast<Derived &>(*this);
    }

    const Derived operator-(const Derived &rhs) const {
        return Derived{*static_cast<Derived const *>(this)} -= rhs;
    }

 protected:
    template <int I>
    typename std::tuple_element<I, ValueTuple>::type &block() noexcept {
        return std::get<I>(this->blocks);
    }

 private:
    template <int... Is>
    std::vector<Scalar *> blockDataImpl(tmp::index_sequence<Is...>) noexcept {
        return {std::get<Is>(this->blocks).data()...};
    }

    ValueTuple blocks;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_COMPOSEDVALUE_HPP
