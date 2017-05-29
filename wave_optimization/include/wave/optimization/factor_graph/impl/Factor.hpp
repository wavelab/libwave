#include <boost/fusion/algorithm.hpp>
#include <boost/fusion/functional/invocation/invoke.hpp>
#include <boost/mpl/range_c.hpp>

namespace wave {

namespace {
// Template helpers used only here

struct set_variables {
    using result_type = double const *const *;
    template <typename VarType>
    result_type operator()(result_type parameters,
                           std::shared_ptr<VarType> &v_ptr) {
        v_ptr.reset(new VarType(*parameters));
        return parameters + 1;
    }
};

template <typename PtrType, int NumResiduals>
struct jac_map_type {
    using VariableType = typename PtrType::element_type;
    using type = Eigen::Map<
      Eigen::Matrix<double, NumResiduals, VariableType::SizeAtCompileTime>>;
};

template <int N>
struct make_jacobian_maps {
    double **const jacobians;

    template <typename T>
    typename jac_map_type<T, N>::type operator()(T &ptr_to_var, int i) const {
        using Map = typename jac_map_type<T, N>::type;
        return Map(jacobians[i]);
    }
};

struct dereference {
    template <typename>
    struct result;

    template <typename T>
    struct result<dereference(const std::shared_ptr<T> &)> {
        using type = T &;
    };

    template <typename T>
    T &operator()(const std::shared_ptr<T> &t) const {
        return *t;
    }
};

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

}  // namespace

template <typename R, typename... V>
bool Factor<R, V...>::evaluate(double const *const *parameters,
                               double *residuals,
                               double **jacobians) noexcept {
    namespace fusion = boost::fusion;
    using boost::mpl::range_c;

    // This function is given C-style arrays. The number and size of the arrays
    // is encoded in this class' template parameters. Our purpose is to map
    // Eigen objects to these arrays.

    // Construct new Variable objects which map the given parameter arrays
    fusion::fold(this->variables, parameters, set_variables{});

    // Create a sequence of Eigen Maps to the given jacobian arrays
    // @todo check row-major vs column major map
    auto jacobian_maps =
      fusion::transform(this->variables,
                        range_c<int, 0, NumVars>(),
                        make_jacobian_maps<NumResiduals>{jacobians});

    // Create a map to the given residual array
    Eigen::Map<R> r{residuals};

    // Put together the args for a call to this->calculate()
    const auto args = fusion::push_front(
      fusion::join(
        fusion::push_back(fusion::transform(this->variables, dereference{}), r),
        jacobian_maps),
      this);

    // Call the function
    fusion::invoke(&FactorType::calculate, args);

    return true;
}

template <typename R, typename... V>
void Factor<R, V...>::print(std::ostream &os) const {
    os << "[";
    os << "Factor arity " << NumVars << ", ";
    os << "variables: ";
    boost::fusion::fold(this->variables, 0, print_variable{os});
    os << "]";
}

}  // namespace wave
