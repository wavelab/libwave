# wave.ceres.ceres_examples

This module contains code examples on how to use [Ceres-Solver][ceres-solver], in particular it contains exampls for how to implement a:

- Cost function with automatic differentiation
- Cost function with numerical differentiation
- Cost function with analytical differentiation

For more information on how to implement your own cost function see this [tutorial][tutorial] and [ceres website][ceres-tutorial].


## Functors / Classes

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

`AutoDiffCostFunctor`, `NumericalDiffFunctor` and `AnalyticalCostFunction` all implmement the cost function for solving the nonlinear least squares of:

\begin{equation}
  f(x) = (10 - x)
\end{equation}

Where the minimum is located at $x = 10$.


## Functions

    void runAutoDiffExample(void);
    void runNumericalDiffExample(void);
    void runAnalyticalDiffExample(void);

`wave.ceres.ceres_examples` contains three functions
`void runAutoDiffExample(void);`, `void runNumericalDiffExample(void);` and `void runAnalyticalDiffExample(void);` that run the automatic differentiation, numerical differentiation and analytical differentiation nonlinear least squares optimization problem on the following cost function.

\begin{equation}
  f(x) = (10 - x)
\end{equation}

Where the minimum is located at $x = 10$.

[tutorial]: #ref/optimizers/ceres_examples
[ceres-tutorial]: http://ceres-solver.org/tutorial.html
[ceres-solver]: http://ceres-solver.org/
