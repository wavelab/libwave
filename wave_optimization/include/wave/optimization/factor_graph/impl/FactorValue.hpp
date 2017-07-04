namespace wave {
namespace internal {

/** Specialization of factor_value_traits for types of ComposedValue. */
template <typename, typename>
struct factor_value_traits_composed;

template <typename V, typename... Nested>
struct factor_value_traits_composed<V, std::tuple<Nested...>> {
    using MatrixType = typename V::ComposedMatrix;
};

/** Specialization of factor_value_traits for types of FactorValue */
template <typename V, typename>
struct factor_value_traits {
    using MatrixType = typename V::Base;
};

/** Uses factor_value_traits_nested if V::ValueTuple exists. See "SFINAE". */
template <typename V>
struct factor_value_traits<
  V,
  typename std::conditional<false, typename V::ValueTuple, void>::type>
  : factor_value_traits_composed<V, typename V::ValueTuple> {};


}  // namespace internal
}  // namespace wave


namespace Eigen {
namespace internal {

// These definitions tell Eigen to treat FactorValue identically to it's Matrix
// base class. See https://eigen.tuxfamily.org/dox/TopicNewExpressionType.html
template <typename T, typename O, int S>
struct traits<wave::FactorValue<T, O, S>>
  : traits<typename wave::FactorValue<T, O, S>::Base> {};

template <typename T, typename O, int S>
struct evaluator<wave::FactorValue<T, O, S>>
  : evaluator<typename wave::FactorValue<T, O, S>::Base> {};

}  // namespace internal
}  // namespace Eigen
