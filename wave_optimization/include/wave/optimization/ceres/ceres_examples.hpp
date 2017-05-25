/** @file
 * @ingroup optimization
 *
 * This file contains code examples on how to use [Ceres-Solver][ceres-solver],
 * in particular it contains examples for how to implement a:
 * - Cost function with automatic differentiation - Cost function with numerical
 * differentiation - Cost function with analytical differentiation
 * For more information on how to implement your own cost function see this
 * [tutorial][tutorial] and [ceres website][ceres-tutorial].
 *
 * The functions `AutoDiffCostFunctor`, `NumericalDiffFunctor` and
 * `AnalyticalCostFunction` run the automatic differentiation, numerical
 * differentiation and analytical differentiation nonlinear least squares
 * optimization problem on the following cost function:
 * @f[
 * f(x) = (10 - x)
 * @f]
 * where the minimum is located at $x = 10$.
 *
 * [tutorial]: #ref/optimizers/ceres_examples
 * [ceres-tutorial]: http://ceres-solver.org/tutorial.html
 * [ceres-solver]: http://ceres-solver.org/
 */

#ifndef WAVE_OPTIMIZATION_CERES_CERES_EXAMPLES_HPP
#define WAVE_OPTIMIZATION_CERES_CERES_EXAMPLES_HPP

#include <ceres/ceres.h>


namespace wave {
/** @addtogroup optimization
 *  @{ */

struct AutoDiffCostFunctor {
    template <typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

struct NumericalDiffCostFunctor {
    template <typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
    }
};

class AnalyticalCostFunction : public ceres::SizedCostFunction<1, 1> {
 public:
    virtual ~AnalyticalCostFunction() {}
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {
        const double x = parameters[0][0];
        residuals[0] = 10 - x;

        // Compute the Jacobian if asked for.
        if (jacobians != NULL && jacobians[0] != NULL) {
            jacobians[0][0] = -1;
        }
        return true;
    }
};

struct CurveFittingResidual {
    CurveFittingResidual(double x, double y) : x(x), y(y) {}
    template <typename T>
    bool operator()(const T *const m, const T *const c, T *residual) const {
        residual[0] = this->y - exp(m[0] * this->x + c[0]);
        return true;
    }

 private:
    const double x;
    const double y;
};

void runAutoDiffExample(void);
void runNumericalDiffExample(void);
void runAnalyticalDiffExample(void);
void runCurveFittingExample(void);

/** @} end of group */
}  // end of wave namespace
#endif
