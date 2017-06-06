#include <boost/fusion/algorithm.hpp>
#include <boost/fusion/functional/invocation/invoke.hpp>
#include <boost/mpl/range_c.hpp>

namespace wave {

namespace internal {
// Template helpers used only in this file

/** Reinitialize each Variable with the parameters pointer
 * Use with fusion::fold */
struct set_variables {
    using result_type = const double *const *;

    template <typename VarType>
    result_type operator()(result_type const &parameters,
                           const std::shared_ptr<VarType> &v_ptr) const {
        v_ptr = std::make_shared<VarType>(*parameters);
        return parameters + 1;
    }

    // This second function is less a "const", needed for error with Boost 1.54
    template <typename VarType>
    result_type operator()(result_type const &parameters,
                           std::shared_ptr<VarType> &v_ptr) const {
        v_ptr = std::make_shared<VarType>(*parameters);
        return parameters + 1;
    }
};

/** Get the jacobian map type
 * @tparam PtrType type of std::shared_ptr<> to the Variable
 * @tparam N number of residuals (equivalently, rows of the jacobian) */
template <typename PtrType, int N>
using OutMapType = JacobianOut<N, PtrType::element_type::SizeAtCompileTime>;

/** Return a map to the given jacobian array
 * Use with fusion::transform */
template <int N>
struct make_jacobian_map {
    double **const jacobians;

    template <typename T>
    OutMapType<T, N> operator()(T &, int i) const {
        return OutMapType<T, N>(jacobians[i]);
    }
};

/** Return a reference to the Variable, given a shared_ptr to it
 * Use with fusion::transform */
struct ref_from_shared_ptr {
    template <typename T>
    T &operator()(const std::shared_ptr<T> &t) const {
        return *t;
    }
};

/** Print variables to a stream, with commas
 * Use with fusion::for_each
 */
struct print_variable {
    std::ostream &os;
    template <typename T>
    int operator()(int index, T &v) const {
        if (index > 0) {
            os << ", ";
        }
        os << *v << "(" << v << ")";
        return index + 1;
    }
};

}  // namespace internal

template <int R, typename... V>
bool Factor<R, V...>::evaluateRaw(double const *const *parameters,
                                  double *residuals,
                                  double **jacobians) noexcept {
    namespace fusion = boost::fusion;
    using boost::mpl::range_c;

    // This function is given C-style arrays. The number and size of the arrays
    // is encoded in this class' template parameters. Our purpose is to map
    // Eigen objects to these arrays.

    // Construct new Variable objects which map the given parameter arrays
    fusion::fold(this->variables, parameters, internal::set_variables{});

    // Create a sequence of Eigen Maps to the given jacobian arrays
    // @todo check row-major vs column major map
    auto jacobian_maps =
      fusion::transform(this->variables,
                        range_c<int, 0, NumVars>(),
                        internal::make_jacobian_map<ResidualSize>{jacobians});

    // Create a map to the given residual array
    ResidualsOut<ResidualSize> r{residuals};

    // Put together the args for a call to this->calculate()
    const auto args = fusion::push_front(
      fusion::join(
        fusion::push_back(
          fusion::transform(this->variables, internal::ref_from_shared_ptr{}),
          r),
        jacobian_maps),
      this);

    // Call the function
    fusion::invoke(&FactorType::evaluate, args);

    return true;
}

template <int R, typename... V>
void Factor<R, V...>::print(std::ostream &os) const {
    os << "[";
    os << "Factor arity " << NumVars << ", ";
    os << "variables: ";
    boost::fusion::fold(this->variables, 0, internal::print_variable{os});
    os << "]";
}

}  // namespace wave
