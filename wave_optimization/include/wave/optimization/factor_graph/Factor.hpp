/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTOR_HPP

#include <boost/fusion/include/vector.hpp>

#include "wave/utils/utils.hpp"
#include "wave/optimization/factor_graph/FactorVariable.hpp"
#include "wave/optimization/factor_graph/FactorBase.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

/** Type of Jacobian output parameters for `Factor::calculate()`. */
template<int Rows, int Cols>
using Jacobian = Eigen::Map<Eigen::Matrix<double, Rows, Cols>>;

/**
 * Abstract base for factors of different variable types.
 *
 * Any factor stored in a FactorGraph must define an `evaluate` function which
 * works with pointers to pointers of C-style arrays supplied by the optimizer.
 * For clarity and type safety, it would be preferable to work with
 * better-defined
 * types such as fixed-size Eigen vectors, which requires re-interpreting the
 * arrays as Eigen types of the expected size. This template does that work.
 *
 * To define a factor, derive an instance of this template and define
 * `calculate()` with the appropriate parameters.
 *
 * @tparam ResidualType The Eigen vector type of the residuals
 * @tparam VariableTypes The list of variables typed operated on by this factor
 */
template <typename ResidualType, typename... VariableTypes>
class Factor : public FactorBase {
    using FactorType = Factor<ResidualType, VariableTypes...>;

 public:
    constexpr static int NumVars = sizeof...(VariableTypes);
    constexpr static int NumResiduals = ResidualType::SizeAtCompileTime;

    Factor(std::shared_ptr<VariableTypes>... variables)
        : variables{variables...} {}

    /**
     * Calculate residuals and jacobians at the given variable values.
     * Override this function with the correct parameters for your derived type.
     *
     * @param[in] variables each variable this factor links
     * @param[out] residuals residuals calculated using the given variables
     * @param[out] jacobians
     * @parblock
     * Jacobian matrices with respect to each variable. There is one
     * jacobian parameter for each variable parameter, in the same order. Each
     * parameter `j` is an `Eigen::Map` which may be null, indicating jacobian
     * should not be calculated on this call. If `j.data() == 0`, do not assign
     * to `j`.
     * @endparblock
     *
     * @return true if calculation was successful, false on failure
     */
    virtual bool calculate(
      const VariableTypes &... variables,
      Eigen::Map<ResidualType> residuals,
      Eigen::Map<Eigen::Matrix<
        double,
        NumResiduals,
        VariableTypes::SizeAtCompileTime>>... jacobians) noexcept = 0;

    bool evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) noexcept override;

    virtual void print(std::ostream &os) const override;

 private:
    boost::fusion::vector<std::shared_ptr<VariableTypes>...> variables;
};

/**
 * Factor representing a distance measurement between a 2D pose and landmark.
 */
class DistanceToLandmarkFactor : public Factor<Vec1, Pose2DVar, Landmark2DVar> {
 public:
    explicit DistanceToLandmarkFactor(double measurement,
                                      std::shared_ptr<Pose2DVar> p,
                                      std::shared_ptr<Landmark2DVar> l)
        : Factor<Vec1, Pose2DVar, Landmark2DVar>{p, l}, meas{measurement} {}

    virtual bool calculate(
      const Pose2DVar &pose,
      const Landmark2DVar &landmark,
      Eigen::Map<Vec1> residual,
      Jacobian<1, 3> j_pose,
      Jacobian<1, 2> j_landmark) noexcept override {
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

/** @} group optimization */
}  // namespace wave

#include "impl/Factor.hpp"

#endif
