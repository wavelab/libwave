#ifndef WAVE_TRANSFORMATION_HPP
#define WAVE_TRANSFORMATION_HPP

#include <Eigen/Core>

#include "wave/utils/utils.hpp"

namespace wave {

class Transformation {
 private:
    Eigen::Matrix<double, 3, 4> matrix;

    double TOL = 1e-04;

 public:
    /** Default constructor, initializes to Identity. */
    Transformation();

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
    Transformation &setFromExpMap(const Vec6 &se3_vector);

    /** Sets from a Transformation matrix.
     * @return a reference to `*this`
     */
    Transformation &setFromMatrix(const Mat4 &input_matrix);

    /** Sets to the identity Transformation.
     * @return a reference to `*this`
     */
    Transformation &setIdentity();

    /** Returns the se3 Lie algebra parameters for this rotation.
     * @return the vector @f$ w @f$ given by
     * @f[
     * w = log(R)_{\times}
     * @f]
     * where @f$ w_\times @f$ is the skew-symmetric matrix form of the Lie
     * algebra parameters @f$ w \in \mathbb{R}^3 @f$.
     */
    Vec6 logMap() const;

    /** Computes the exp map of the input and computes the Jacobian
     * of the exp map wrt @f$ w @f$.
     *
     * @param[in] W The input vector we are taking the expmap of.
     * @param[in]TOL Tolerance to switch between exact and taylor approximation (avoids singularity at zero)
     * @f$ W @f$.
     */
    static Mat4 expMap(const Vec6 &W, double TOL);

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
    static Mat6 SE3LeftJacobian(const Vec6 &W, double TOL);

    /**
     * Computes the left side Jacobian using a 2nd order Taylor Expansion
     * @param W The twist location to calculate the jacobian of
     * @return Left SE3 jacobian
     */
    static Mat6 SE3ApproxLeftJacobian(const Vec6 &W);

    /**
     * Implements ^ operator mapping a vector to a skew symmetric matrix
     * @param V vector
     * @return V^
     */
    static Mat3 skewSymmetric3(const Vec3 &V);

    /**
     * Returns the approximate Jacobian of an interpolated transform wrt to
     * the end perturbation. Called A(alpha, epsilon) in Barfoot State Estimation
     * twist quantity must be small for approximation to hold
     * @param twist of transform
     * @return A(alpha, twist)
     */
    static Mat6 Jinterpolated(const Vec6 &twist, const double &alpha);

    /** transforms the input vector by this transformation.
     *
     * @return the vector @f$p_o@f$, the rotated result given by @f[
     * \label{eqn:rot}
     * p_o = R p_i
     * @f]
     * where @f$p_i@f$ is the input vector.
     */
    Vec3 transform(const Vec3 &input_vector) const;

    /** Rotates the input vector and calculate Jacobians.
     *
     * @param[in] input_vector The vector to be rotated
     * @param[out] Jpoint Jacobian of rotation function with respect to the
     * point input
     * @param[out] Jparam Jacobian of rotation function with respect to this
     * rotation
     * @return the rotated vector as given by `rotate()`.
     */
    Vec3 transformAndJacobian(const Vec3 &input_vector, Mat3 &Jpoint, Eigen::Matrix<double, 3, 6> &Jparam) const;

    /** transforms the input vector by the inverse of this transformation.
     *
     * @return the vector @f$p_o@f$, given by @f[
     * p_o = T^{-1} p_i
     * @f]
     * where @f$p_i@f$ is the input vector.
     */
    Vec3 inverseTransform(const Vec3 &input_vector) const;

    /** Invert the transformation in place. */
    Transformation &invert();

    /**
     * Return the inverse of the transformation matrix, while preserving original
     * @return the inverse of the transformation object;
     */
    Transformation inverse() const;

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
    Transformation &manifoldPlus(const Vec6 &omega);

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
    Vec6 manifoldMinus(const Transformation &R) const;

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
    Vec6 manifoldMinusAndJacobian(const Transformation &T, Mat6 &J_left, Mat6 &J_right) const;

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
    Transformation composeAndJacobian(const Transformation &T_right, Mat6 &J_left, Mat6 &J_right) const;

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

    /** @return a reference to the internal matrix object **/
    Eigen::Matrix<double, 3, 4>& getInternalMatrix() {return this->matrix;};

    /** Implements transformation composition. */
    Transformation operator*(const Transformation &T) const;

    /** Overload operator for manifold - */
    Vec6 operator-(const Transformation &T) const;
};
}

#endif  // WAVE_TRANSFORMATION_HPP
