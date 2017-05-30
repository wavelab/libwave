/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP

#include <boost/fusion/include/vector.hpp>
#include <memory>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/FactorBase.hpp"
#include "wave/optimization/factor_graph/OutputMap.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * Abstract base for factors of different variable types.
 *
 * Any factor stored in a FactorGraph must define an `evaluate` function which
 * works with pointers to pointers of C-style arrays supplied by the optimizer.
 * For clarity and type safety, it would be preferable to work with
 * better-defined types such as fixed-size Eigen vectors, which requires
 * re-interpreting the arrays as Eigen types of the expected size. This template
 * does that work.
 *
 * To define a factor, derive an instance of this template and define
 * `calculate()` with the appropriate parameters.
 *
 * @tparam NumResiduals The dimension of the residual vector
 * @tparam VariableTypes The list of variables typed operated on by this factor
 */
template <int NumResiduals, typename... VariableTypes>
class Factor : public FactorBase {
    using FactorType = Factor<NumResiduals, VariableTypes...>;

 public:
    constexpr static int NumVars = sizeof...(VariableTypes);
    constexpr static int ResidualSize = NumResiduals;
    using ResidualType = Eigen::Matrix<double, NumResiduals, 1>;

    explicit Factor(std::shared_ptr<VariableTypes>... variables)
        : variables{variables...} {}
    virtual ~Factor() = default;

    /**
     * Calculate residuals and jacobians at the given variable values.
     * Override this function with the correct parameters for your derived type.
     *
     * @param[in] variables each variable this factor links
     * @param[out] residuals residuals calculated using the given variables
     * @param[out] jacobians
     * @parblock
     * Jacobian matrices with respect to each variable. There is one jacobian
     * parameter for each variable parameter, in the same order.
     * @endparblock
     *
     * @note  Each residual and jacobian parameter is a special map object which
     * may evaluate to false to indicate the jacobian should not be calculated
     * on this call. Wrap calculation and assignment of each parameter `j` in a
     * conditional, `if (j) {...}`.
     *
     * @return true if calculation was successful, false on failure
     */
    virtual bool evaluate(
      const VariableTypes &... variables,
      ResidualsOut<NumResiduals> residuals,
      JacobianOut<NumResiduals,
                  VariableTypes::SizeAtCompileTime>... jacobians) noexcept = 0;

    bool evaluateRaw(double const *const *parameters,
                     double *residuals,
                     double **jacobians) noexcept override;

    void print(std::ostream &os) const override;

 private:
    boost::fusion::vector<std::shared_ptr<VariableTypes>...> variables;
};

/**
 * Factor representing a distance measurement between a 2D pose and landmark.
 *
 * @todo Move elsewhere (just an example here for now)
 */
class DistanceToLandmarkFactor : public Factor<1, Pose2DVar, Landmark2DVar> {
 public:
    explicit DistanceToLandmarkFactor(double measurement,
                                      std::shared_ptr<Pose2DVar> p,
                                      std::shared_ptr<Landmark2DVar> l)
        : Factor<1, Pose2DVar, Landmark2DVar>{std::move(p), std::move(l)},
          meas{measurement} {}

    bool evaluate(const Pose2DVar &pose,
                  const Landmark2DVar &landmark,
                  ResidualsOut<1> residual,
                  JacobianOut<1, 3> j_pose,
                  JacobianOut<1, 2> j_landmark) noexcept override {
        double distance = (pose.position - landmark.position).norm();
        residual(0) = distance - this->meas;

        if (j_pose) {
            j_pose << (pose.position - landmark.position).transpose() /
                        distance,
              0;
        }
        if (j_landmark) {
            j_landmark << (pose.position - landmark.position).transpose() /
                            distance;
        }

        return true;
    }

    double meas;
};

/** @} group optimization */
}  // namespace wave

#include "impl/Factor.hpp"

#endif
