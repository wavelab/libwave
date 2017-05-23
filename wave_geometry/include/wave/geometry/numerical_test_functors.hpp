#ifndef WAVE_NUMERICAL_TEST_FUNCTORS_HPP
#define WAVE_NUMERICAL_TEST_FUNCTORS_HPP

#include "wave/geometry/rotation.hpp"

namespace wave {

// The Functors are used for computing finite difference Jacobians
// of rotation expressions.

class RotateAndJacobianJpointFunctor {
 public:
    Rotation R;
    Vec3 P;
    RotateAndJacobianJpointFunctor(Rotation input_rotation) {
        this->R = input_rotation;
    }

    Vec3 operator()(Vec3 input_point) {
        Vec3 output_point = R.rotate(input_point);
        return output_point;
    }
};


class RotateAndJacobianJparamFunctor {
 public:
    Rotation R;
    Vec3 P;
    RotateAndJacobianJparamFunctor(Rotation input_rotation, Vec3 input_point) {
        this->R = input_rotation;
        this->P = input_point;
    }

    Vec3 operator()(Vec3 input_point) {
        Rotation Rp = this->R;
        Rp = Rp.manifoldPlus(input_point);
        Vec3 output_point = Rp.rotate(P);
        return output_point;
    }
};


// Implement the numerical differentiation, following procedure
// for a 5 point stencil https://en.wikipedia.org/wiki/Five-point_stencil
// Exact step size representation is outlined in
// "Numerical Recipes by Brian P. Flannery,
// Saul Teukolsky, and William H. Press, chapter 5.7.


// Compute the perturbation points as recommended in Numerical Recipes.
// Modify the step size so that the perturbations are exactly
// representable numbers by adding then subtracting the same value.
double get_perturbation_point(double evaluation_point, double step_size) {
    // Use volatile to avoid compiler optimizations.
    volatile double perturbation_point = evaluation_point + step_size;
    double exact_step_size = (perturbation_point - evaluation_point);
    return evaluation_point + exact_step_size;
}

template <typename MatrixType, typename FunctorType>
void numerical_jacobian(FunctorType F,
                        Vec3 evaluation_point,
                        Eigen::MatrixBase<MatrixType> &jac) {
    for (int i = 0; i < evaluation_point.size(); i++) {
        // Select the step size as recommended in Numerical Recipes.
        double step_size = cbrt(std::numeric_limits<double>::epsilon()) *
                           std::fabs(evaluation_point[i]);
        // If the step size is exactly equal to zero, just select it as
        // cbrt(eps).
        if (step_size == 0.0)
            step_size = cbrt(std::numeric_limits<double>::epsilon());

        Vec3 perturbation_point = evaluation_point;
        // Compute the function value using positive perturbations.
        perturbation_point[i] =
          get_perturbation_point(evaluation_point[i], step_size);
        auto F_xp1 = F(perturbation_point);
        perturbation_point = evaluation_point;
        perturbation_point[i] =
          get_perturbation_point(evaluation_point[i], 2 * step_size);
        auto F_xp2 = F(perturbation_point);

        // Perform same operations for negative perturbation.
        perturbation_point = evaluation_point;
        perturbation_point[i] =
          get_perturbation_point(evaluation_point[i], -step_size);
        auto F_xm1 = F(perturbation_point);
        perturbation_point = evaluation_point;
        perturbation_point[i] =
          get_perturbation_point(evaluation_point[i], -2 * step_size);
        auto F_xm2 = F(perturbation_point);

        // Finally compute the finite difference.
        Vec3 finite_difference =
          (-F_xp2 + 8 * F_xp1 - 8 * F_xm1 + F_xm2) / (12 * step_size);
        jac.col(i) = finite_difference;
    }
}

}  // end namespace wave

#endif
