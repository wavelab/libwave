namespace wave {

namespace internal {
// Template helpers used only in this file

/** We have an integer `N`, known at compiler time, and want the compiler to
 * generate a function call `f(1, ..., N)`. For example, if N is 3, we want to
 * call `f(1, 2, 3)`.
 *
 * The solution involves variadic templates. This template, `index_sequence`,
 * holds the sequence 1..N as template parameters. When we pass an instance of
 * it to a variadic function template, the sequence can become a parameter pack.
 * For example,
 *
 * ```
 * template<int... Is>
 * void callF(index_sequence<Is...> indices) { f(Is...); }
 *
 * int main() {
 *     callF(index_sequence<1, 2, 3>{});
 *     f(1, 2, 3);
 * }
 * ```
 *
 * The call above is equivalent to f(1, 2, 3). Of course, it's not useful if you
 * have to type out index_sequence<1, 2, 3>. That's why we define the template
 * `make_index_sequence`, which lets us call `callF(make_index_sequence<3>{});`.
 *
 * This approach is common enough that `index_sequence` and
 * `make_index_sequence` were added to the standard library in C++14. Since
 * they are not in C++11, we define a simple implementation here.
 *
 * See also:
 *
 * - http://en.cppreference.com/w/cpp/utility/integer_sequence
 * - The "indices trick": http://loungecpp.wikidot.com/tips-and-tricks:indices
 * - Variadic templates:
 * http://eli.thegreenplace.net/2014/variadic-templates-in-c
 */
template <int... Indices>
struct index_sequence {};

/** Generates an index_sequence<1, ..., N> via recursion */
template <int N, int... Indices>
struct make_index_sequence : make_index_sequence<N - 1, N - 1, Indices...> {};

/** Final stop on the recursion train */
template <int... Indices>
struct make_index_sequence<0, Indices...> : index_sequence<Indices...> {};

/** Constructs a FactorValue mapping the given raw array.
 *
 * @tparam V the type of FactorVariable
 * @tparam I the index of the variable in this factor
 * @param parameters the array of array pointers from ceres
 * @return a FactorValue mapping the Ith array
 */
template <typename V, int I>
const typename V::ViewType make_value(double const *const *parameters) {
    /* The array is const, but ValueView maps non-const arrays. It would be
     * complicated to have two variants of ValueView, one for const and one
     * for non-const arrays. In this case, we know that the ValueView itself
     * is const, and it won't modify the array itself - thus, nothing will
     * modify the array, and it's "safe" to cast away the constness. Still,
     * @todo: reconsider this cast */
    const auto ptr = const_cast<double *>(parameters[I]);
    return typename V::ViewType{ptr};
}

/** Constructs a jacobian output object mapping the given raw array.
 *
 * @tparam R size of residuals (height of the jacobian matrix)
 * @tparam V size of variable (width of the jacobian matrix)
 * @tparam I index of the variable in this factor
 * @param jacobians the array of array pointers from ceres
 * @return a jacobian mapping the Ith array
 */
template <int R, int C, int I>
JacobianOut<R, C> make_jacobian(double **jacobians) {
    // Note jacobians may be null, meaning no jacobians were requested.
    return JacobianOut<R, C>{jacobians ? jacobians[I] : nullptr};
}

/** Calls Factor::evaluate() with the necessary arguments generated from the
 * Factor type (known at compile time) and the provided array pointers (known
 * at run time).
 */
template <int R, typename... V, int... Is>
bool callEvaluate(const Factor<R, V...> &factor,
                  double const *const *parameters,
                  double *residuals,
                  double **jacobians,
                  index_sequence<Is...>) noexcept {
    // Call evaluate, generating the correct type and number of the three kinds
    // of arguments: input values, output residuals, and output jacobians.
    return factor.evaluate(make_value<V, Is>(parameters)...,
                           ResidualsOut<R>{residuals},
                           make_jacobian<R, V::Size, Is>(jacobians)...);
}

}  // namespace internal

template <int R, typename... V>
bool Factor<R, V...>::evaluateRaw(double const *const *parameters,
                                  double *residuals,
                                  double **jacobians) const noexcept {
    // Call a helper function. We need to do this to expand the index sequence
    // which we generate here. This approach is explained above.
    return internal::callEvaluate<R, V...>(
      *this,
      parameters,
      residuals,
      jacobians,
      internal::make_index_sequence<sizeof...(V)>());
}

template <int R, typename... V>
void Factor<R, V...>::print(std::ostream &os) const {
    os << "[";
    os << "Factor arity " << NumVars << ", ";
    os << "variables: ";
    for (auto i = 0; i < NumVars; ++i) {
        const auto &v = this->variable_ptrs[i];
        os << *v << "(" << v << ")";
        if (i < NumVars - 1) {
            os << ", ";
        }
    }
    os << "]";
}


}  // namespace wave
