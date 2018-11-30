#ifndef WAVE_TRANSFORMATION_HPP
#define WAVE_TRANSFORMATION_HPP

#include <memory>
#include <Eigen/Core>
#include "unsupported/Eigen/MatrixFunctions"

#include "wave/utils/utils.hpp"

namespace wave {

template <typename T_str = Mat34, bool approximate = false>
class Transformation {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr const double tol = 1.0e-5;
    T_str storage;

    Transformation() {
        this->setIdentity();
    }

    Transformation(Transformation const &other) : storage(other.storage) {}

    Transformation(const T_str &storage) : storage(storage) {}
    Transformation(T_str &&storage) noexcept : storage(std::move(storage)) {}

    template <typename Other, bool OtherApprox>
    Transformation &operator=(const Transformation<Other, OtherApprox> &other) {
        this->storage = other.storage;
        return *this;
    }

    template <typename Other, bool OtherApprox>
    Transformation &operator=(Transformation<Other, OtherApprox> &&other) noexcept {
        this->storage = std::move(other.storage);
        return *this;
    }

    /** Constructs from Euler angles.
     *
     * @param eulers X-Y-Z Euler angles in radians, such that
     * @param translation Values to initialize transformation matrix
     * @f[
     * R = R_z R_y R_x
     * @f]
     * where @f$ R_x, R_y @f$, and @f$ R_z @f$, are rotation matrices
     * constructed using the rotation about @f x, y @f, and @fz@f, respectively.
     */
    explicit Transformation(const Vec3 &eulers, const Vec3 &translation);

    /** Sets from Euler angles.
     *
     * @param input_vector X-Y-Z Euler angles; see `Rotation(const Vec3 &)`.
     * @param translation Values to initialize transformation matrix
     * @return a reference to `*this`
     */
    Transformation &setFromEulerXYZ(const Vec3 &eulers, const Vec3 &translation);

    /** Sets from exponential map.
     *
     * Converts from the se(3) Lie algebra, to the SE(3) Lie group using the
     * exponential map
     * @f[
     * R = exp(w_{\times})
     * @f]
     * where @f$ w_{\times} @f$ is the input `se3_vector` represented in
     * skew-symmetric form.
     *
     * @param se3_vector Twist vector corresponding to the se3 lie algebra for the
     * transformation object. Parameters are rotation, then translation
     * @return a reference to `*this`
     */
    template <typename VType>
    Transformation &setFromExpMap(const Eigen::MatrixBase<VType> &se3_vector);

    /** Sets from a Transformation matrix.
     * @return a reference to `*this`
     */
    Transformation &setFromMatrix(const Mat4 &input_matrix);

    /** Sets to the identity Transformation.
     * @return a reference to `*this`
     */
    Transformation &setIdentity();

    /** Checks if matrix determinant is within given tolerance of 1 and normalizes
     * rotation component if not
     */
    Transformation &normalize();

    template <typename InType, typename Other, bool InApprox, bool O_approx>
    static void interpolate(const Transformation<InType, InApprox> &T_k,
                            const Transformation<InType, InApprox> &T_kp1,
                            const Vec6 &twist_k,
                            const Vec6 &twist_kp1,
                            const Mat12 &hat,
                            const Mat12 &candle,
                            Transformation<Other, O_approx> &T_int,
                            Vec6* interpTwist = nullptr);

    template <typename InType, typename Other, bool InApprox, bool O_approx>
    static void interpolateReduced(const Transformation<InType, InApprox> &T_k,
                            const Transformation<InType, InApprox> &T_kp1,
                            const Vec6 &twist_k,
                            const Vec6 &twist_kp1,
                            const Mat2 &hat,
                            const Mat2 &candle,
                            Transformation<Other, O_approx> &T_int,
                            Vec6* interpTwist = nullptr);

    /**
     * Returns an interpolated tranform at time t where t is between t_k and t_kp1.
     * Method is the one proposed in "Full STEAM Ahead" Anderson & Barfoot
     * @param T_k: Transform at time t_k
     * @param T_kp1: Transform at time t_kp1
     * @param twist_k: Twist at time t_k
     * @param twist_kp1: Twist at time t_kp1
     * @param hat: first interpolation factor
     * @param candle: second interpolation factor
     * @return Transformation at time t
     */
    template <typename InType, typename Other, bool InApprox, bool O_approx>
    static void interpolateAndJacobians(const Transformation<InType, InApprox> &T_k,
                                        const Transformation<InType, InApprox> &T_kp1,
                                        const Vec6 &twist_k,
                                        const Vec6 &twist_kp1,
                                        const Mat12 &hat,
                                        const Mat12 &candle,
                                        Transformation<Other, O_approx> &Tint,
                                        Mat6 &J_Tk,
                                        Mat6 &J_Tkp1,
                                        Mat6 &J_twist_k,
                                        Mat6 &J_twist_kp1);

    /** Returns the se3 Lie algebra parameters for this rotation.
     * @return the vector @f$ w @f$ given by
     * @f[
     * w = log(R)_{\times}
     * @f]
     * where @f$ w_\times @f$ is the skew-symmetric matrix form of the Lie
     * algebra parameters @f$ w \in \mathbb{R}^3 @f$.
     */
    Vec6 logMap(double tolerance = 1e-4) const;

    Mat6 adjointRep() const;

    /** Computes the exp map of the input and computes the Jacobian
     * of the exp map wrt @f$ w @f$.
     *
     * @param[in] W The input vector we are taking the expmap of.
     * @param[in]TOL Tolerance to switch between exact and taylor approximation (avoids singularity at zero)
     * @f$ W @f$.
     */
    template <typename VType, typename MType>
    static void expMap(const Eigen::MatrixBase<VType> &W, Eigen::MatrixBase<MType> &retval);

    /** Computes a first order approximation of exp map of the input and computes the Jacobian
     * of the exp map wrt @f$ w @f$.
     *
     * @param[in] W The input vector we are taking the expmap of.
     * @param[in]TOL Tolerance to switch between exact and taylor approximation (avoids singularity at zero)
     * @f$ W @f$.
     */
    template <typename VType, typename MType>
    static void expMap1st(const Eigen::MatrixBase<VType> &W, Eigen::MatrixBase<MType> &retval);

    /**
     * Computes the exponential map of the for the adjoint
     * @param W The input vector we are taking the expmap of.
     * @param TOL Tolerance to switch between exact and taylor approximation (avoids singularity at zero)
     * @return
     */
    static Mat6 expMapAdjoint(const Vec6 &W);

    /** Computes the log map of the input and computes the Jacobian
     * of the log map wrt @f$ R @f$.
     *
     * @param[in] R The input rotation we are taking the log of.
     * @f$ R @f$.
     */
    static Vec6 logMap(const Transformation &T);

    /**
     * Computes the left side Jacobian of SE3
     * @param W The twist location to calculate the jacobian of
     * @param[in]TOL Tolerance to switch between exact and taylor approximation
     * @return Left SE3 jacobian
     */
    static Mat6 SE3LeftJacobian(const Vec6 &W);

    /**
     * Computes the left side Jacobian using a 1st order Taylor Expansion
     * @param W The twist location to calculate the jacobian of
     * @return Left SE3 jacobian
     */
    template<typename Derived, typename OtherDerived>
    static void SE3ApproxLeftJacobian(const Eigen::MatrixBase<Derived> &W, Eigen::MatrixBase<OtherDerived> &J);

    /**
     * Computes the inverse left side Jacobian using a 1st order Taylor Expansion
     * @param W The twist
     * @return Inverse Left SE3 Jacobian
     */
     template<typename OtherDerived>
    static Eigen::Matrix<typename OtherDerived::Scalar, 6, 6> SE3ApproxInvLeftJacobian(const Eigen::MatrixBase<OtherDerived> &W);

    /**
     * Implements ^ operator mapping a vector to a skew symmetric matrix
     * @param V vector
     * @return V^
     */
    template <typename Derived, typename OtherDerived>
    static void skewSymmetric3(const Eigen::MatrixBase<Derived> &V, Eigen::MatrixBase<OtherDerived> &retval);

    template <typename Derived, typename OtherDerived>
    static void skewSymmetric3(const Eigen::MatrixBase<Derived> &V, Eigen::MatrixBase<OtherDerived> &&retval);

    template <typename Derived>
    static Mat3 skewSymmetric3(const Eigen::MatrixBase<Derived> &V);

    template <typename Derived, typename OtherDerived>
    static void skewSymmetric6(const Eigen::MatrixBase<Derived> &W, Eigen::MatrixBase<OtherDerived> &retval);

    /**
     * Returns the approximate Jacobian of an interpolated transform wrt to
     * the end perturbation. Called A(alpha, epsilon) in Barfoot State Estimation
     * twist quantity must be small for approximation to hold
     * @param twist of transform
     * @return A(alpha, twist)
     */
    static void Jinterpolated(const Vec6 &twist, const double &alpha, Mat6 &retval);

    static void Adjoint(const Vec6 &twist, Mat6 &retval);

    /**
     * Returns the "lift" Jacobian for use with Ceres solver
     * @return
     */
    void J_lift(Eigen::Matrix<double, 12, 6> &retval) const;

    /** transforms the input vector by this transformation.
     *
     * @return the vector @f$p_o@f$, the rotated result given by @f[
     * \label{eqn:rot}
     * p_o = R p_i
     * @f]
     * where @f$p_i@f$ is the input vector.
     */
    Vec3 transform(const Vec3 &input_vector) const;

    template <typename IP_T, typename OP_T>
    void transform(const Eigen::MatrixBase<IP_T> &ip_vec, Eigen::MatrixBase<OP_T> &op_vec) const;

    /** Rotates the input vector and calculate Jacobians.
     *
     * @param[in] input_vector The vector to be rotated
     * @param[out] Jpoint Jacobian of rotation function with respect to the
     * point input
     * @param[out] Jparam Jacobian of rotation function with respect to this
     * rotation
     * @return the rotated vector as given by `rotate()`.
     */
    template <typename IP_T, typename OP_T, typename M_T1, typename M_T2>
    void transformAndJacobian(const Eigen::MatrixBase<IP_T> &input_vector,
                              Eigen::MatrixBase<OP_T> &output_vector,
                              Eigen::MatrixBase<M_T1> *Jpoint = nullptr,
                              Eigen::MatrixBase<M_T2> *Jparam = nullptr) const;

    /** transforms the input vector by the inverse of this transformation.
     *
     * @return the vector @f$p_o@f$, given by @f[
     * p_o = T^{-1} p_i
     * @f]
     * where @f$p_i@f$ is the input vector.
     */
    Vec3 inverseTransform(const Vec3 &input_vector) const;

    template <typename IP_T, typename OP_T>
    void inverseTransform(const Eigen::MatrixBase<IP_T> &ip_vec, Eigen::MatrixBase<OP_T> &op_vec) const;

    /** Invert the transformation in place. */
    Transformation &invert();

    /**
     * Return the inverse of the transformation matrix, while preserving original
     * @return the inverse of the transformation object;
     */
    template <typename T_OUT, bool approx>
    void transformInverse(Transformation<T_OUT, approx> &T_inv) const;

    Transformation<Mat34, approximate> transformInverse() const;

    /** Checks if the input transformation is sufficiently close to this transformation
     *
     * The function computes a disparity transformation @f$R_d@f$ as
     * @f[
     * R_d = T_{other} T^{-1}.
     * @f]
     * If the transformations are (approximately) equal, @f$R_d@f$ will be
     * (approximately) the identity matrix,
     * or equivalently, the log map, @f$ w_d = log(T_d) @f$, will be
     * (approximately) zero.
     *
     * @return true if @f$ ||w_d|| \leq \epsilon @f$,
     * where @f$ \epsilon @f$ is the precision given by `comparison_threshold`.
     */
    bool isNear(const Transformation &other, double comparison_threshold) const;

    /** Calculates the result of the manifold-plus operation @f$ \boxplus @f$.
     *
     * @param omega @f$ \omega \in \mathbb{R}^6 @f$, a vector in the tangent
     * space of @f$ T @f$
     * @return The result, @f$ T = T \boxplus \omega @f$
     * @note the @f$ \boxplus @f$ operator uses the **left** composition with
     * the exponential map.
     * @f[
     * \boxplus: \mathbb{SE}(3) \times \mathbb{R}^6 \mapsto \mathbb{SE}(3) \\
     * T \boxplus \omega = exp(\omega)T
     * @f]
     */
    template <typename VType>
    Transformation &manifoldPlus(const Eigen::MatrixBase<VType> &omega);

    /** Calculates the result of the manifold-minus operation @f$ \boxminus @f$.
     *
     * @param T @f$ T \in \mathbb{SE}3 @f$, a transformation
     * @return The result, @f$ \omega = T_t \boxminus T @f$
     * @note the @f$ \boxminus @f$ operator is implemented as follows:
     * @f[
     * \boxminus: \mathbb{SE}(3) \times \mathbb{SE}(3) \mapsto \mathbb{R}^6  \\
     * T_t \boxminus T = log(T_t T^{-1})
     * @f]
     * where @f$ T_t @f$ corresponds to the rotation of **this** object.
     */
    template <typename Other, bool OtherApprox>
    Vec6 manifoldMinus(const Transformation<Other, OtherApprox> &R) const;

    template <typename Other, bool OtherApprox, typename OtherDerived>
    void manifoldMinus(const Transformation<Other, OtherApprox> &R, Eigen::MatrixBase<OtherDerived> &retval) const;

    /** Same as manifoldMinus, except also computes the Jacobians with respect
     * to the left and right arguments of manifoldMinus.  For manifold minus,
     * @f$ \omega = T_t \boxminus T @f$.
     *
     * @param[in] T @f$ T \in \mathbb{SE}3 @f$, a transformation
     * @param[out] J_left The Jacobian of manifoldMinus wrt the left argument,
     * @f$ T_t @f$, which is also **this** transformation.
     * @param[out] J_right The Jacobian of manifoldMinus wrt the right argument,
     * @f$ T @f$, which is also the input transformation.
     * @return The result, @f$ \omega = T_t \boxminus T @f$
     */
    template<typename otherStorage, bool otherApprox>
    Vec6 manifoldMinusAndJacobian(const Transformation<otherStorage, otherApprox> &T, Mat6 *J_left = nullptr, Mat6 *J_right = nullptr) const;

    template <typename Other, typename OutT>
    void compose(const Other &T_Right, OutT &T_out) const;

    /** Composes two transformations and computes the Jacobians wrt the left
     * and right transformations:
     * @f$ T_{out} = T_{left}*T_{right} @f$  Note that **this** transformation
     * corresponds with @f$ T_{left} @f$ and the input transformation is @f$
     * T_{right}@f$.
     * @param[in] T_right @f$ T_{right} \in \mathbb{SE}3 @f$, the input
     * transformation.
     * @param[out] J_left The Jacobian of compose wrt the left argument,
     * @f$ T_{left} @f$ , which is also **this** transformation.
     * @param[out] J_right The Jacobian of compose wrt the right argument,
     * @f$ T_{right} @f$, which is also the input transformation.
     * @return The resulting composition @f$ T_{out} @f$.
     */
    template <typename Other, bool approx = approximate>
    Transformation<T_str, approximate> composeAndJacobian(const Transformation<Other, approx> &T_right,
                                                          Mat6 &J_left,
                                                          Mat6 &J_right) const;

    /** Compute the inverse of **this** transformation and computes the Jacobian
     * of the inverse mapping wrt **this** transformation.
     * @param[out] J_transformation The Jacobian of the inverse mapping wrt **this**
     * transformation.
     * @return The inverse of **this** transformation.
     */
    Transformation inverseAndJacobian(Mat6 &J_transformation) const;

    /** @return the corresponding 3x3 rotation matrix */
    Mat3 getRotationMatrix() const;

    /** @return the 3x1 translation vector */
    Vec3 getTranslation() const;

    /** @return the 4x4 transformation matrix */
    Mat4 getMatrix() const;

    /** Implements transformation composition. */
    template <typename Other, bool OtherApprox>
    Transformation<Mat34, approximate> operator*(const Transformation<Other, OtherApprox> &T) const;

    /** Overload operator for manifold - */
    Vec6 operator-(const Transformation &T) const;
};
}

#include "wave/geometry_og/impl/transformation_impl.hpp"

#endif  // WAVE_TRANSFORMATION_HPP
