/** @file
 * @ingroup geometry
 */

#ifndef WAVE_GEOMETRY_NUMERICAL_TEST_FUNCTORS_HPP
#define WAVE_GEOMETRY_NUMERICAL_TEST_FUNCTORS_HPP

#include "wave/geometry_og/transformation.hpp"

namespace wave {
/** @addtogroup geometry
 *  @{ */

// The Functors are used for computing finite difference Jacobians
// of rotation and/or translation expressions.

namespace {
    using T_Type = Transformation<Mat34, false>;
}
// Transformation functors
class TransformAndJacobianJpointFunctor {
 public:
    T_Type T;
    TransformAndJacobianJpointFunctor(const T_Type &input_transformation) {
        this->T = input_transformation;
    }

    Vec3 operator()(const Vec3 &input_point) {
        Vec3 output_point;
        this->T.transform(input_point, output_point);
        return output_point;
    }
};


class TransformAndJacobianJparamFunctor {
 public:
    T_Type T;
    Vec3 P;
    TransformAndJacobianJparamFunctor(const T_Type &input_transformation, const Vec3 &input_point) {
//        this->T.deepCopy(input_transformation);
        this->T = input_transformation;
        this->P = input_point;
    }

    Vec3 operator()(const Vec6 &wvec) {
        T_Type Tp;
//        Tp.deepCopy(this->T);
        Tp = this->T;
        Tp.manifoldPlus(wvec);
        Vec3 output_point;
        Tp.transform(this->P, output_point);
        return output_point;
    }
};

class TComposeAndJacobianJLeftFunctor {
 public:
    T_Type R_left;
    T_Type R_right;
    TComposeAndJacobianJLeftFunctor(const T_Type &R_left, const T_Type &R_right) {
//        this->R_left.deepCopy(R_left);
//        this->R_right.deepCopy(R_right);
        this->R_left = R_left;
        this->R_right = R_right;
    }

    T_Type operator()(const Vec6 &perturbation) {
        Mat6 J;
        T_Type R_perturbed;
//        R_perturbed.deepCopy(this->R_left);
        R_perturbed = this->R_left;
        R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.composeAndJacobian(R_right, J, J);
    }
};

class TComposeAndJacobianJRightFunctor {
 public:
    T_Type R_left;
    T_Type R_right;
    TComposeAndJacobianJRightFunctor(const T_Type &R_left, const T_Type &R_right) {
//        this->R_left.deepCopy(R_left);
//        this->R_right.deepCopy(R_right);
        this->R_left = R_left;
        this->R_right = R_right;
    }

    T_Type operator()(const Vec6 &perturbation) {
        Mat6 J;
        T_Type R_perturbed;
//        R_perturbed.deepCopy(this->R_right);
        R_perturbed = this->R_right;
        R_perturbed.manifoldPlus(perturbation);
        return R_left.composeAndJacobian(R_perturbed, J, J);
    }
};

class TInverseAndJacobianFunctor {
 public:
    T_Type R;
    TInverseAndJacobianFunctor(const T_Type &R) {
        this->R = R;
    }

    T_Type operator()(const Vec6 &perturbation) {
        T_Type R_perturbed;
//        R_perturbed.deepCopy(this->R);
        R_perturbed = this->R;
        R_perturbed.manifoldPlus(perturbation);
        T_Type retval;
        R_perturbed.transformInverse(retval);
        return retval;
    }
};

class TLogMapAndJacobianFunctor {
 public:
    T_Type R;
    TLogMapAndJacobianFunctor(const T_Type &R) {
        this->R = R;
    }

    Vec6 operator()(const Vec6 &perturbation) {
        Mat6 J;
        T_Type R_perturbed;
//        R_perturbed.deepCopy(this->R);
        R_perturbed = this->R;
        R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.logMap();
    }
};

class TManifoldMinusAndJacobianJLeftFunctor {
 public:
    T_Type R_left;
    T_Type R_right;
    TManifoldMinusAndJacobianJLeftFunctor(const T_Type &R_left, const T_Type &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Vec6 operator()(const Vec6 &perturbation) {
        Mat6 J;
        T_Type R_perturbed;
        R_perturbed = this->R_left;
        R_perturbed.manifoldPlus(perturbation);
        return R_perturbed.manifoldMinus(this->R_right);
    }
};

class TManifoldMinusAndJacobianJRightFunctor {
 public:
    T_Type R_left;
    T_Type R_right;
    TManifoldMinusAndJacobianJRightFunctor(const T_Type &R_left, const T_Type &R_right) {
        this->R_left = R_left;
        this->R_right = R_right;
    }

    Vec6 operator()(const Vec6 &perturbation) {
        Mat6 J;
        T_Type R_perturbed;
        R_perturbed = this->R_right;
        R_perturbed = R_perturbed.manifoldPlus(perturbation);
        return this->R_left.manifoldMinus(R_perturbed);
    }
};

template<typename T>
class TInterpolatedJTLeftFunctor {
 public:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    T T_k;
    T T_kp1;
    Vec6 vel_k;
    Vec6 vel_kp1;

    Mat12 hat, candle;
    
    TInterpolatedJTLeftFunctor(const T &T_k,
                               const T &T_kp1,
                               const Vec6 &vel_k,
                               const Vec6 &vel_kp1,
                               const Mat12 &hat,
                               const Mat12 &candle)
        : vel_k(vel_k), vel_kp1(vel_kp1), hat(hat), candle(candle) {
        this->T_k = T_k;
        this->T_kp1 = T_kp1;
    }

    T operator()(const Vec6 &perturbation) {
        T T_k_perturb;
        T_k_perturb = this->T_k;
        T_k_perturb.manifoldPlus(perturbation);
        T T_int;
        T::interpolate(T_k_perturb, this->T_kp1, this->vel_k, this->vel_kp1, this->hat, this->candle, T_int);
        return T_int;
    }
};

template<typename T>
class TInterpolatedJTRightFunctor {
 public:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    T T_k;
    T T_kp1;
    Vec6 vel_k;
    Vec6 vel_kp1;

    Mat12 hat, candle;
    
    TInterpolatedJTRightFunctor(const T &T_k,
                               const T &T_kp1,
                               const Vec6 &vel_k,
                               const Vec6 &vel_kp1,
                               const Mat12 &hat,
                               const Mat12 &candle)
            : vel_k(vel_k), vel_kp1(vel_kp1), hat(hat), candle(candle) {
        this->T_k = T_k;
        this->T_kp1 = T_kp1;
    }

    T operator()(const Vec6 &perturbation) {
        T T_kp1_perturb;
        T_kp1_perturb = this->T_kp1;
        T_kp1_perturb.manifoldPlus(perturbation);
        T T_int;
        T::interpolate(this->T_k, T_kp1_perturb, this->vel_k, this->vel_kp1, this->hat, this->candle, T_int);
        return T_int;
    }
};

template<typename T>
class TInterpolatedJVLeftFunctor {
 public:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    T T_k;
    T T_kp1;
    Vec6 vel_k;
    Vec6 vel_kp1;

    Mat12 hat, candle;
    
    TInterpolatedJVLeftFunctor(const T &T_k,
                                const T &T_kp1,
                                const Vec6 &vel_k,
                                const Vec6 &vel_kp1,
                                const Mat12 &hat,
                                const Mat12 &candle)
            : vel_k(vel_k), vel_kp1(vel_kp1), hat(hat), candle(candle) {
        this->T_k = T_k;
        this->T_kp1 = T_kp1;
    }

    T operator()(const Vec6 &perturbation) {
        Vec6 vel_k_perturb = this->vel_k + perturbation;
        T T_int;
        T::interpolate(this->T_k, this->T_kp1, vel_k_perturb, this->vel_kp1, this->hat, this->candle, T_int);
        return T_int;
    }
};

template<typename T>
class TInterpolatedJVRightFunctor {
 public:
    using Mat12 = Eigen::Matrix<double, 12, 12>;
    T T_k;
    T T_kp1;
    Vec6 vel_k;
    Vec6 vel_kp1;

    Mat12 hat, candle;
    
    TInterpolatedJVRightFunctor(const T &T_k,
                               const T &T_kp1,
                               const Vec6 &vel_k,
                               const Vec6 &vel_kp1,
                               const Mat12 &hat,
                               const Mat12 &candle)
            : vel_k(vel_k), vel_kp1(vel_kp1), hat(hat), candle(candle) {
        this->T_k = T_k;
        this->T_kp1 = T_kp1;
    }

    T operator()(const Vec6 &perturbation) {
        Vec6 vel_kp1_perturb = this->vel_kp1 + perturbation;
        T T_int;
        T::interpolate(this->T_k, this->T_kp1, this->vel_k, vel_kp1_perturb, this->hat, this->candle, T_int);
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
