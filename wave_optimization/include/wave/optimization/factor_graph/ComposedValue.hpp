/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_COMPOSEDVALUE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_COMPOSEDVALUE_HPP

#include "wave/optimization/factor_graph/FactorValue.hpp"
#include "wave/optimization/factor_graph/impl/define_composed_value.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

template <typename Derived, template <typename...> class... V>
class ComposedValue;

template <template <typename, typename> class D,
          typename Scalar,
          typename Options,
          template <typename...> class... V>
class ComposedValue<D<Scalar, Options>, V...>
  : public FactorValue<Scalar,
                       Options,
                       tmp::sum<V<Scalar>::RowsAtCompileTime...>::value> {
    using Derived = D<Scalar, Options>;

 public:
    using Base = FactorValue<Scalar,
                             Options,
                             tmp::sum<V<Scalar>::RowsAtCompileTime...>::value>;
    using MatrixType = typename Base::Base;
    constexpr static std::array<int, sizeof...(V)> BlockSizes{
      {V<Scalar>::RowsAtCompileTime...}};
    constexpr static int Size = tmp::array_sum(BlockSizes);
    constexpr static std::array<int, sizeof...(V)> BlockIndices =
      tmp::cumulative_array(BlockSizes);
    using ValueTuple = std::tuple<V<Scalar, Options>...>;

    // Inherit base constructor
    using Base::Base;
    using Base::operator=;

    ComposedValue() : Base{Base::Zero()} {}

    explicit ComposedValue(V<Scalar, Options>... args) {
        this->initMatrix(tmp::make_index_sequence<sizeof...(V)>{},
                         std::move(args)...);
    }

    // Make public some privately inherited methods
    using Base::data;

    template <int I>
    Eigen::Ref<typename std::tuple_element<I, ValueTuple>::type>
    blockAtIndex() noexcept {
        constexpr auto i = std::get<I>(BlockIndices);
        constexpr auto size = std::get<I>(BlockSizes);
        return this->template segment<size>(i);
    }

 protected:
    template <int... Is>
    void initMatrix(tmp::index_sequence<Is...>, V<Scalar, Options>... args) {
        auto loop = {(this->blockAtIndex<Is>() = std::move(args), 0)...};
        (void) loop;
    }
};

/** Specialization for square option */
template <template <typename, typename> class D,
          typename Scalar,
          template <typename...> class... V>
class ComposedValue<D<Scalar, FactorValueOptions::Square>, V...>
  : public FactorValue<Scalar,
                       FactorValueOptions::Square,
                       tmp::sum<V<Scalar>::RowsAtCompileTime...>::value> {
    using Derived = D<Scalar, FactorValueOptions::Square>;

 public:
    using Base = FactorValue<Scalar,
                             FactorValueOptions::Square,
                             tmp::sum<V<Scalar>::RowsAtCompileTime...>::value>;
    using MatrixType = typename Base::Base;
    constexpr static std::array<int, sizeof...(V)> BlockSizes{
      {V<Scalar>::RowsAtCompileTime...}};
    constexpr static int Size = tmp::array_sum(BlockSizes);
    constexpr static std::array<int, sizeof...(V)> BlockIndices =
      tmp::cumulative_array(BlockSizes);
    using ValueTuple = std::tuple<V<Scalar, FactorValueOptions::Square>...>;

    // Inherit base constructor
    using Base::Base;
    using Base::operator=;

    ComposedValue() : Base{Base::Zero()} {}

    /** Given a reference to a diagonal block of `mat`, determine the index */
    template <typename T>
    int indexFromRef(const Eigen::Ref<T> &ref) const {
        // Use low-level math for now
        const auto diff = ref.data() - this->data();
        if (diff < 0 || diff >= this->size()) {
            throw std::logic_error(
              "ComposedValue::indexFromRef: invalid reference");
        }
        return diff % Size;
    }

    template <int I>
    Eigen::Ref<typename std::tuple_element<I, ValueTuple>::type>
    blockAtIndex() noexcept {
        constexpr auto i = std::get<I>(BlockIndices);
        constexpr auto size = std::get<I>(BlockSizes);
        return this->template block<size, size>(i, i);
    }

    template <int I, int J>
    Eigen::Block<MatrixType, BlockSizes[I], BlockSizes[J]>
    blockAtIndex() noexcept {
        constexpr auto i = std::get<I>(BlockIndices);
        constexpr auto j = std::get<J>(BlockIndices);
        constexpr auto rows = std::get<I>(BlockSizes);
        constexpr auto cols = std::get<J>(BlockSizes);
        return this->template block<rows, cols>(i, j);
    }

    /** Return a blockAtIndex given references to diagonal blocks */
    template <typename T, typename O, int R, int C>
    Eigen::Block<MatrixType, R, C> blockAtIndex(
      const Eigen::Ref<FactorValue<T, O, R>> &row_block,
      const Eigen::Ref<FactorValue<T, O, C>> &col_block) {
        const auto i = this->indexFromRef(row_block);
        const auto j = this->indexFromRef(col_block);
        return this->template block<R, C>(i, j);
    };
};

/** @} group optimization */
}  // namespace wave

#define WAVE_DEFINE_COMPOSED_VALUE(NAME, ATTRIBUTES) \
    WAVE_DEFINE_COMPOSED_VALUE_IMPL(NAME, ATTRIBUTES)

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_COMPOSEDVALUE_HPP
