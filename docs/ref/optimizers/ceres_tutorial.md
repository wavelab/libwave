# Ceres Tutorial
As with all nonlinear-optimizers they can be difficult to master, in the following we give examples on how to effectively implement custom cost functions in Ceres-Solver. In particular we will cover several different topics:

- How to define a cost function?
- How to define the optimization problem?
- How do I define a numerical cost function?
- How do I define an analytical cost function?
- How do I load the optimization data?


## How to define a cost function?
Ceres an solve bounds constrained robustified non-linear least squares problems of the form:

\begin{equation}
  \min_{x} \dfrac{1}{2} \sum_{i} \rho_{i} (|| f_{i} (x_{i}, \dots, x_{i_{k}}) ||^{2})
\end{equation}

The expression $\rho_{i} (|| f_{i} (x_{i}, \dots, x_{i_{k}}) ||^{2})$ is known in ceres as a **`ResidualBlock`**, where $f_{i}(\cdot)$ is a cost function that depends on the parameter blocks $[x_{i}, \dots, x_{i_{k}}]$.

To get started, consider the problem of finding the minimum of the function

\begin{equation}
  \dfrac{1}{2} (10 - x)^{2}
\end{equation}

The cost function in this case is:

\begin{equation}
  f(x) = (10 - x)
\end{equation}

This is a trivial problem, whose minimum is located at $x = 10$. The first step
is to write a functor that will evaluate this function:

    struct CostFunctor {
      template <typename T>
      bool operator()(const T* const x, T* residual) const {
        residual[0] = T(10.0) - x[0];
        return true;
      }
    };

Keypoints to the example code:

- In Ceres a cost function has to be a functor (function objects: a plain old C++ object plus the `()` operator), this is essentially so that the ceres solver can call your cost function with the overridded `()` operator you implemented, this is where you will add your custom cost function, for more information on functors see [this][stack_overflow-functors].
- The output of your cost function is written to `residual[0]`, in all cases `residual` is always an array of type `T`, further in this example we are only writing to `residual[0]` because the output dimension for our scalar cost function is 1.
- `x[0]` is of size 1 for this problem because the dimension of `x` is 1.
- The `operator()` is a templated method, which assumes that all its inputs and outputs are of some type `T`. The use of templating here allows Ceres to call `CostFunctor::operator<T>()`, with `T=double` when just the value of the residual is needed, and with a special type `T=Jet` when the Jacobians are needed.
- Because of the special `Jet` type that Ceres uses, one cannot pass in `Eigen::Vector` or `Eigen::Matrix`, the type of template `T` has to be a C++ primitive.
- The functor can be implemented using a `struct` as above, or with `class` either will work (as we will see later).
- Since the cost function contains template code it has to reside in a header file.


## How to define the optimization problem?
In the previous section we have showed how one defines a cost function, here we will describe how one uses the cost function by setting up the optimization problem in Ceres.

    // The variable to solve for with its initial value.
    double initial_x = 5.0;
    double x = initial_x;

    // Build the problem.
    ceres::Problem problem;

    // Set up the only cost function (also known as residual). This uses
    // auto-differentiation to obtain the derivative (jacobian).
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver!
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initial_x << " -> " << x << "\n";

Note that:

    ceres::CostFunction *cost_function
        = new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
                               ^              ^        ^  ^
                               |              |        |  |
    Automatic Differentiation -+              |        |  |
    Your custom cost functor  ----------------+        |  |
    Dimension of residual -----------------------------+  |
    Dimension of x ---------------------------------------+

The `ceres::AutoDiffCostFunction` takes our `CostFunctor` previously defined as input, automatically differentiates it and gives it a `ceres::CostFunction` interface.

Running the above code example gives us:

    iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
      0  4.512500e+01    0.00e+00    9.50e+00   0.00e+00   0.00e+00  1.00e+04       0    5.33e-04    3.46e-03
      1  4.511598e-07    4.51e+01    9.50e-04   9.50e+00   1.00e+00  3.00e+04       1    5.00e-04    4.05e-03
      2  5.012552e-16    4.51e-07    3.17e-08   9.50e-04   1.00e+00  9.00e+04       1    1.60e-05    4.09e-03
    Ceres Solver Report: Iterations: 2, Initial cost: 4.512500e+01, Final cost: 5.012552e-16, Termination: CONVERGENCE
    x : 0.5 -> 10



## How do I define a numerical cost function?
Defining a numerical cost function is just a matter of using `ceres::NumericDiffCostFunction`, for more info see this [doc][ceres-numerical_diff].

    ceres::CostFunction *cost_function
        = new ceres::NumericDiffCostFunction<MyScalarCostFunctor, CENTRAL, 1, 2, 2>(
            new MyScalarCostFunctor(1.0));                           ^     ^  ^  ^
                                                                     |     |  |  |
                                  Finite Differencing Scheme --------+     |  |  |
                                  Dimension of residual -------------------+  |  |
                                  Dimension of x -----------------------------+  |
                                  Dimension of y --------------------------------+


## How do I define an analytical cost function?
The implementation for an analytical cost function is more involving compared to a numerical or automatic differentiated cost function. First we have to implement the analytical cost function using `ceres::SizedCostFunction`:

    class QuadraticCostFunction
      : public SizedCostFunction<1 /* number of residuals */,
                                 1 /* size of first parameter */> {
    public:
      virtual ~QuadraticCostFunction() {}
      virtual bool Evaluate(double const * const * parameters,
                            double *residuals,
                            double **jacobians) const {
        double x = parameters[0][0];
        // f(x) = 10 - x.
        residuals[0] = 10 - x;

        // f'(x) = -1. Since there's only 1 parameter and that parameter
        // has 1 dimension, there is only 1 element to fill in the
        // jacobians.
        //
        // Since the Evaluate function can be called with the jacobians
        // pointer equal to NULL, the Evaluate function must check to see
        // if jacobians need to be computed.
        //
        // For this simple problem it is overkill to check if jacobians[0]
        // is NULL, but in general when writing more complex
        // CostFunctions, it is possible that Ceres may only demand the
        // derivatives w.r.t. a subset of the parameter blocks.
        if (jacobians != NULL && jacobians[0] != NULL) {
          jacobians[0][0] = -1;
        }
        return true;
      }
    };

Keypoints:

- Instead of overriding the `()` operator, with a `SizedCostFunction` we override the `Evaluate()` method.
- `parameters` contains all the data for the cost function.
- The jacobian for the cost function should be calculated when `jacobians` is not `NULL`
- Dimensions for residuals and parameters are now defined in the inherited class typecast

After defining the cost function to use it you simply do:

    ceres::CostFunction *cost_function = new AnalyticalCostFunction;


## How do I load the optimization data?
The examples we have seen until now are simple optimization problems with no data. The original purpose of least squares and non-linear least squares analysis was fitting curves to data. It is only appropriate that we now consider an example of such a problem. Lets consider a new cost function:

\begin{equation}
    y = e^{mx + c}
\end{equation}

We can implement the cost functor for the above cost function as:

    struct ExponentialResidual {
      ExponentialResidual(double x, double y)
          : x(x), y(y) {}

      template <typename T>
      bool operator()(const T* const m, const T* const c, T* residual) const {
        residual[0] = T(this->y) - exp(m[0] * T(this->x) + c[0]);
        return true;
      }

    private:
      // Observations for a sample.
      const double x;
      const double y;
    };

The key-point to note is we are using the private `x` and `y` member variables to store an observation. Assuming the observations are in a `2n` sized array called `data` the problem construction is a simple matter of creating a `ceres::CostFunction` for every observation.

    double m = 0.0;
    double c = 0.0;

    ceres::Problem problem;
    for (int i = 0; i < kNumObservations; ++i) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
              new ExponentialResidual(data[2 * i], data[2 * i + 1]));
      problem.AddResidualBlock(cost_function, NULL, &m, &c);
    }



[stack_overflow-functors]: http://stackoverflow.com/questions/356950/c-functors-and-their-uses
[ceres-numerical_diff]: http://ceres-solver.org/nnls_modeling.html#numericdiffcostfunction
