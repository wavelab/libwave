#ifndef WAVE_OPTIMIZATION_CERES_CERES_EXAMPLES_HPP
#define WAVE_OPTIMIZATION_CERES_CERES_EXAMPLES_HPP

#include <ceres/ceres.h>


namespace wave {
namespace ceres {
namespace examples {

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

class AnalyticalCostFunction : public ::ceres::SizedCostFunction<1, 1> {
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

void runAutoDiffExample(void);
void runNumericalDiffExample(void);
void runAnalyticalDiffExample(void);

}  // end of examples namespace
}  // end of ceres namespace
}  // end of wave namespace
#endif
