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

class ComposeAndJacobianJLeftFunctor {
 public:
    Rotation R_left;
    Rotation R_right;
    ComposeAndJacobianJLeftFunctor(Rotation R_left, Rotation R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Rotation operator()(Vec3 perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R_left;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.composeAndJacobian(R_right, J, J);
    }
};

class ComposeAndJacobianJRightFunctor {
 public:
    Rotation R_left;
    Rotation R_right;
    ComposeAndJacobianJRightFunctor(Rotation R_left, Rotation R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Rotation operator()(Vec3 perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R_right;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_left.composeAndJacobian(R_perturbed, J, J);
    }
};

class InverseAndJacobianFunctor {
 public:
    Rotation R;
    InverseAndJacobianFunctor(Rotation R) {
        this->R = R;
    }

    Rotation operator()(Vec3 perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.inverseAndJacobian(J);
    }
};

class LogMapAndJacobianFunctor {
 public:
    Rotation R;
    LogMapAndJacobianFunctor(Rotation R) {
        this->R = R;
    }

    Vec3 operator()(Vec3 perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return Rotation::logMapAndJacobian(R_perturbed, J);
    }
};


class ManifoldMinusAndJacobianJLeftFunctor {
 public:
    Rotation R_left;
    Rotation R_right;
    ManifoldMinusAndJacobianJLeftFunctor(Rotation R_left, Rotation R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Vec3 operator()(Vec3 perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R_left;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.manifoldMinus(this->R_right);
    }
};

class ManifoldMinusAndJacobianJRightFunctor {
 public:
    Rotation R_left;
    Rotation R_right;
    ManifoldMinusAndJacobianJRightFunctor(Rotation R_left, Rotation R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Vec3 operator()(Vec3 perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R_right;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return this->R_left.manifoldMinus(R_perturbed);
    }
};

// Compute the perturbation points as recommended in Numerical Recipes.
// Modify the step size so that the perturbations are exactly
// representable numbers by adding then subtracting the same value.
double get_perturbation_point(double evaluation_point, double step_size) {
    // Use volatile to avoid compiler optimizations.
    volatile double perturbation_point = evaluation_point + step_size;
    double exact_step_size = (perturbation_point - evaluation_point);
    return evaluation_point + exact_step_size;
}


// Numerical difference using simple forward difference method.
// Requires overloaded -operator to perform manifoldMinus for
// Manifold quantities.

template <typename MatrixType, typename FunctorType>
void numerical_jacobian(FunctorType F,
                        Vec3 evaluation_point,
                        Eigen::MatrixBase<MatrixType> &jac) {
    for (int i = 0; i < evaluation_point.size(); i++) {
        // Select the step size as recommended in Numerical Recipes.
        double step_size = sqrt(std::numeric_limits<double>::epsilon()) *
                           std::fabs(evaluation_point[i]);
        // If the step size is exactly equal to zero, just select it as
        // sqrt(eps).
        if (step_size == 0.0) {
            step_size = sqrt(std::numeric_limits<double>::epsilon());
        }

        Vec3 perturbation_point = evaluation_point;
        // Compute the function value using positive perturbations.
        perturbation_point[i] =
          get_perturbation_point(evaluation_point[i], step_size);
        auto F_xp1 = F(perturbation_point);

        // Perform same operations for zero perturbation.
        auto F_x = F(evaluation_point);

        // Finally compute the finite difference.
        Vec3 finite_difference = (F_xp1 - F_x) / (step_size);
        jac.col(i) = finite_difference;
    }
}

}  // end namespace wave

#endif
