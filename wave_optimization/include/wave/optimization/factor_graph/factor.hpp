#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP

#include <memory>
//#include <boost/bind.hpp>

#include "wave/utils/utils.hpp"
#include "wave/optimization/factor_graph/variable.hpp"

namespace wave {

/**
 *
 * @tparam VariableTypes
 * @tparam ResidualType
 */

class FactorBase {
 public:
    virtual bool evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) noexcept = 0;
    virtual void print(std::ostream &os) const = 0;
    virtual ~FactorBase() = default;
};


template <int I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type set_variables(
  std::tuple<Tp...> &, double const *const *) {}
template <int I = 0, typename... Tp>
  inline typename std::enable_if <
  I<sizeof...(Tp), void>::type set_variables(std::tuple<Tp...> &t,
                                             double const *const *parameters) {
    auto &ref = std::get<I>(t);
    using VariableType = typename std::remove_reference<decltype(*ref)>::type;
    ref.reset(new VariableType(parameters[I]));
    set_variables<I + 1, Tp...>(t, parameters);
}


template <int...>
struct seq {};

template <int N, int... S>
struct gens : gens<N - 1, N - 1, S...> {};

template <int... S>
struct gens<0, S...> {
    typedef seq<S...> type;
};


template <typename ResidualType, typename... VariableTypes>
class Factor : public FactorBase {
 public:
    constexpr static int num_variables = sizeof...(VariableTypes);
    constexpr static int num_residuals = ResidualType::SizeAtCompileTime;
    using ResidualRefType = Eigen::Ref<ResidualType>;

    Factor(std::shared_ptr<VariableTypes>... variables)
        : variables{variables...} {}


    virtual bool calculate(
      const VariableTypes &...,
      ResidualRefType,
      Eigen::Map<
        Eigen::Matrix<double,
                      num_residuals,
                      VariableTypes::SizeAtCompileTime>>...) noexcept = 0;

    using JacobianMapType = std::tuple<Eigen::Map<
      Eigen::
        Matrix<double, num_residuals, VariableTypes::SizeAtCompileTime>>...>;

    bool evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) noexcept override {
        set_variables(this->variables, parameters);

        auto out_jacobians = std::make_tuple<
          Eigen::Map<Eigen::Matrix<double,
                                   num_residuals,
                                   VariableTypes::SizeAtCompileTime>>...>(
          Eigen::Map<Eigen::Matrix<double, 1, 3>>{jacobians[0]},
          Eigen::Map<Eigen::Matrix<double, 1, 2>>{jacobians[1]});


        this->callCalculate(typename gens<num_variables>::type(),
                            residuals,
                            typename gens<num_variables>::type(),
                            out_jacobians);
        return true;
    }

    template <int... R, int... S>
    bool callCalculate(seq<R...>, double *r, seq<S...>, JacobianMapType &j) {
        Eigen::Map<ResidualType> out_res{r};
        return this->calculate(
          *std::get<R>(this->variables)..., out_res, std::get<S>(j)...);
    }

    virtual void print(std::ostream &os) const override {
        os << "[";
        os << "arity: " << this->num_variables << ", ";
        os << "variables: ";
        for (size_t i = 0; i < this->num_variables; i++) {
            if (i != 0) {
                os << ", ";
            }
            //            os << *std::get<i>(this->variables);
        }
        os << "]";
    }
    std::tuple<std::shared_ptr<VariableTypes>...> variables;
};

template <typename ResidualType, typename... VariableTypes>
std::ostream &operator<<(std::ostream &os, const FactorBase &factor) {
    factor.print(os);
    return os;
}

class DistanceToLandmarkFactor : public Factor<Vec1, Pose2DVar, Landmark2DVar> {
 public:
    explicit DistanceToLandmarkFactor(double measurement,
                                      std::shared_ptr<Pose2DVar> p,
                                      std::shared_ptr<Landmark2DVar> l)
        : Factor<Vec1, Pose2DVar, Landmark2DVar>{p, l}, meas{measurement} {}

    virtual bool calculate(
      const Pose2DVar &pose,
      const Landmark2DVar &landmark,
      Eigen::Ref<Vec1> residual,
      Eigen::Map<Eigen::Matrix<double, 1, 3>> j_pose,
      Eigen::Map<Eigen::Matrix<double, 1, 2>> j_landmark) noexcept override {
        double distance = (pose.position - landmark.position).norm();
        residual(0) = distance - this->meas;

        if (j_pose.data()) {
            j_pose << (pose.position - landmark.position).transpose() /
                        distance,
              0;
        }
        if (j_landmark.data()) {
            j_landmark << (pose.position - landmark.position).transpose() /
                            distance;
        }

        return true;
    }

    double meas;
};


}  // end of wave namespace
#endif
