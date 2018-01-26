/** @file
 * @ingroup geometry
 */

#ifndef WAVE_GEOMETRY_NUMERICAL_TEST_FUNCTORS_HPP
#define WAVE_GEOMETRY_NUMERICAL_TEST_FUNCTORS_HPP

#include "wave/geometry/rotation.hpp"
#include "wave/geometry/transformation.hpp"

namespace wave {
/** @addtogroup geometry
 *  @{ */

// The Functors are used for computing finite difference Jacobians
// of rotation and/or translation expressions.

class RotateAndJacobianJpointFunctor {
 public:
    Rotation R;
    Vec3 P;
    RotateAndJacobianJpointFunctor(const Rotation &input_rotation) {
        this->R = input_rotation;
    }

    Vec3 operator()(const Vec3 &input_point) {
        Vec3 output_point = R.rotate(input_point);
        return output_point;
    }
};


class RotateAndJacobianJparamFunctor {
 public:
    Rotation R;
    Vec3 P;
    RotateAndJacobianJparamFunctor(const Rotation &input_rotation, const Vec3 &input_point) {
        this->R = input_rotation;
        this->P = input_point;
    }

    Vec3 operator()(const Vec3 &input_point) {
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
    ComposeAndJacobianJLeftFunctor(const Rotation &R_left, const Rotation &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Rotation operator()(const Vec3 &perturbation) {
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
    ComposeAndJacobianJRightFunctor(const Rotation &R_left, const Rotation &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Rotation operator()(const Vec3 &perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R_right;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_left.composeAndJacobian(R_perturbed, J, J);
    }
};

class InverseAndJacobianFunctor {
 public:
    Rotation R;
    InverseAndJacobianFunctor(const Rotation &R) {
        this->R = R;
    }

    Rotation operator()(const Vec3 &perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.inverseAndJacobian(J);
    }
};

class LogMapAndJacobianFunctor {
 public:
    Rotation R;
    LogMapAndJacobianFunctor(const Rotation &R) {
        this->R = R;
    }

    Vec3 operator()(const Vec3 &perturbation) {
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
    ManifoldMinusAndJacobianJLeftFunctor(const Rotation &R_left, const Rotation &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Vec3 operator()(const Vec3 &perturbation) {
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
    ManifoldMinusAndJacobianJRightFunctor(const Rotation &R_left, const Rotation &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Vec3 operator()(const Vec3 &perturbation) {
        Mat3 J;
        Rotation R_perturbed = this->R_right;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return this->R_left.manifoldMinus(R_perturbed);
    }
};

// Transformation functors
class TransformAndJacobianJpointFunctor {
 public:
    Transformation T;
    Vec3 P;
    TransformAndJacobianJpointFunctor(const Transformation &input_transformation) {
        this->T = input_transformation;
    }

    Vec3 operator()(const Vec3 &input_point) {
        Vec3 output_point = this->T.transform(input_point);
        return output_point;
    }
};


class TransformAndJacobianJparamFunctor {
 public:
    Transformation T;
    Vec3 P;
    TransformAndJacobianJparamFunctor(const Transformation &input_transformation, const Vec3 &input_point) {
        this->T = input_transformation;
        this->P = input_point;
    }

    Vec3 operator()(const Vec6 &wvec) {
        Transformation Tp = this->T;
        Tp = Tp.manifoldPlus(wvec);
        Vec3 output_point = Tp.transform(this->P);
        return output_point;
    }
};

class TComposeAndJacobianJLeftFunctor {
 public:
    Transformation R_left;
    Transformation R_right;
    TComposeAndJacobianJLeftFunctor(const Transformation &R_left, const Transformation &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Transformation operator()(const Vec6 &perturbation) {
        Mat6 J;
        Transformation R_perturbed = this->R_left;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.composeAndJacobian(R_right, J, J);
    }
};

class TComposeAndJacobianJRightFunctor {
 public:
    Transformation R_left;
    Transformation R_right;
    TComposeAndJacobianJRightFunctor(const Transformation &R_left, const Transformation &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Transformation operator()(const Vec6 &perturbation) {
        Mat6 J;
        Transformation R_perturbed = this->R_right;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_left.composeAndJacobian(R_perturbed, J, J);
    }
};

class TInverseAndJacobianFunctor {
 public:
    Transformation R;
    TInverseAndJacobianFunctor(const Transformation &R) {
        this->R = R;
    }

    Transformation operator()(const Vec6 &perturbation) {
        Transformation R_perturbed = this->R;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.inverse();
    }
};

class TLogMapAndJacobianFunctor {
 public:
    Transformation R;
    TLogMapAndJacobianFunctor(const Transformation &R) {
        this->R = R;
    }

    Vec6 operator()(const Vec6 &perturbation) {
        Mat6 J;
        Transformation R_perturbed = this->R;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.logMap();
    }
};

class TManifoldMinusAndJacobianJLeftFunctor {
 public:
    Transformation R_left;
    Transformation R_right;
    TManifoldMinusAndJacobianJLeftFunctor(const Transformation &R_left, const Transformation &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Vec6 operator()(const Vec6 &perturbation) {
        Mat6 J;
        Transformation R_perturbed = this->R_left;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.manifoldMinus(this->R_right);
    }
};

class TManifoldMinusAndJacobianJRightFunctor {
 public:
    Transformation R_left;
    Transformation R_right;
    TManifoldMinusAndJacobianJRightFunctor(const Transformation &R_left, const Transformation &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Vec6 operator()(const Vec6 &perturbation) {
        Mat6 J;
        Transformation R_perturbed = this->R_right;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return this->R_left.manifoldMinus(R_perturbed);
    }
};

class TInterpolatedJTLeftFunctor {
 public:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    Transformation T_k;
    Transformation T_kp1;
    Vec6 vel_k;
    Vec6 vel_kp1;

    Eigen::Matrix<double, 6, 12> hat, candle;

    Transformation T_t;
    TInterpolatedJTLeftFunctor(const Transformation &T_k,
                               const Transformation &T_kp1,
                               const Vec6 &vel_k,
                               const Vec6 &vel_kp1,
                               const Eigen::Matrix<double, 6, 12> &hat,
                               const Eigen::Matrix<double, 6, 12> &candle)
        : T_k(T_k), T_kp1(T_kp1), vel_k(vel_k), vel_kp1(vel_kp1), hat(hat), candle(candle) {
        T_t = Transformation::interpolate(this->T_k, this->T_kp1, this->vel_k, this->vel_kp1, this->hat, this->candle);
    }

    Transformation operator()(const Vec6 &perturbation) {
        Transformation T_k_perturb = this->T_k;
        T_k_perturb.manifoldPlus(perturbation);
        Transformation T_int;
        T_int = Transformation::interpolate(T_k_perturb, this->T_kp1, this->vel_k, this->vel_kp1, this->hat, this->candle);
        return T_int;
    }
};

class TInterpolatedJTRightFunctor {
 public:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    Transformation T_k;
    Transformation T_kp1;
    Vec6 vel_k;
    Vec6 vel_kp1;

    Eigen::Matrix<double, 6, 12> hat, candle;

    Transformation T_t;
    TInterpolatedJTRightFunctor(const Transformation &T_k,
                               const Transformation &T_kp1,
                               const Vec6 &vel_k,
                               const Vec6 &vel_kp1,
                               const Eigen::Matrix<double, 6, 12> &hat,
                               const Eigen::Matrix<double, 6, 12> &candle)
            : T_k(T_k), T_kp1(T_kp1), vel_k(vel_k), vel_kp1(vel_kp1), hat(hat), candle(candle) {
        T_t = Transformation::interpolate(this->T_k, this->T_kp1, this->vel_k, this->vel_kp1, this->hat, this->candle);
    }

    Transformation operator()(const Vec6 &perturbation) {
        Transformation T_kp1_perturb = this->T_kp1;
        T_kp1_perturb.manifoldPlus(perturbation);
        Transformation T_int;
        T_int = Transformation::interpolate(this->T_k, T_kp1_perturb, this->vel_k, this->vel_kp1, this->hat, this->candle);
        return T_int;
    }
};

class TInterpolatedJVLeftFunctor {
 public:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    Transformation T_k;
    Transformation T_kp1;
    Vec6 vel_k;
    Vec6 vel_kp1;

    Eigen::Matrix<double, 6, 12> hat, candle;

    Transformation T_t;
    TInterpolatedJVLeftFunctor(const Transformation &T_k,
                                const Transformation &T_kp1,
                                const Vec6 &vel_k,
                                const Vec6 &vel_kp1,
                                const Eigen::Matrix<double, 6, 12> &hat,
                                const Eigen::Matrix<double, 6, 12> &candle)
            : T_k(T_k), T_kp1(T_kp1), vel_k(vel_k), vel_kp1(vel_kp1), hat(hat), candle(candle) {
        T_t = Transformation::interpolate(this->T_k, this->T_kp1, this->vel_k, this->vel_kp1, this->hat, this->candle);
    }

    Transformation operator()(const Vec6 &perturbation) {
        Vec6 vel_k_perturb = this->vel_k + perturbation;
        Transformation T_int;
        T_int = Transformation::interpolate(this->T_k, this->T_kp1, vel_k_perturb, this->vel_kp1, this->hat, this->candle);
        return T_int;
    }
};

class TInterpolatedJVRightFunctor {
 public:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    Transformation T_k;
    Transformation T_kp1;
    Vec6 vel_k;
    Vec6 vel_kp1;

    Eigen::Matrix<double, 6, 12> hat, candle;

    Transformation T_t;
    TInterpolatedJVRightFunctor(const Transformation &T_k,
                               const Transformation &T_kp1,
                               const Vec6 &vel_k,
                               const Vec6 &vel_kp1,
                               const Eigen::Matrix<double, 6, 12> &hat,
                               const Eigen::Matrix<double, 6, 12> &candle)
            : T_k(T_k), T_kp1(T_kp1), vel_k(vel_k), vel_kp1(vel_kp1), hat(hat), candle(candle) {
        T_t = Transformation::interpolate(this->T_k, this->T_kp1, this->vel_k, this->vel_kp1, this->hat, this->candle);
    }

    Transformation operator()(const Vec6 &perturbation) {
        Vec6 vel_kp1_perturb = this->vel_kp1 + perturbation;
        Transformation T_int;
        T_int = Transformation::interpolate(this->T_k, this->T_kp1, this->vel_k, vel_kp1_perturb, this->hat, this->candle);
        return T_int;
    }
};

// Compute the perturbation points as recommended in Numerical Recipes.
// Modify the step size so that the perturbations are exactly
// representable numbers by adding then subtracting the same value.
inline double get_perturbation_point(double evaluation_point, double step_size) {
    // Use volatile to avoid compiler optimizations.
    volatile double perturbation_point = evaluation_point + step_size;
    double exact_step_size = (perturbation_point - evaluation_point);
    return evaluation_point + exact_step_size;
}


// Numerical difference using simple forward difference method.
// Requires overloaded -operator to perform manifoldMinus for
// Manifold quantities.

template <typename MatrixType, typename FunctorType, typename TangentType>
void numerical_jacobian(FunctorType F, const TangentType &evaluation_point, Eigen::MatrixBase<MatrixType> &jac) {
    for (int i = 0; i < evaluation_point.size(); i++) {
        // Select the step size as recommended in Numerical Recipes.
        double step_size = sqrt(std::numeric_limits<double>::epsilon()) * std::fabs(evaluation_point[i]);
        // If the step size is exactly equal to zero, just select it as
        // sqrt(eps).
        if (step_size == 0.0) {
            step_size = sqrt(std::numeric_limits<double>::epsilon());
        }

        TangentType perturbation_point = evaluation_point;
        // Compute the function value using positive perturbations.
        perturbation_point[i] = get_perturbation_point(evaluation_point[i], step_size);
        auto F_xp1 = F(perturbation_point);

        // Perform same operations for zero perturbation.
        auto F_x = F(evaluation_point);

        // Finally compute the finite difference.
        VecX finite_difference = (F_xp1 - F_x) / (step_size);
        jac.col(i) = finite_difference;
    }
}

/** @} group geometry */
}  // namespace wave

#endif  // WAVE_GEOMETRY_NUMERICAL_TEST_FUNCTORS_HPP
