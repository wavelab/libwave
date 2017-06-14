/** @file
 * @ingroup geometry
 */

#ifndef WAVE_GEOMETRY_ROTATION_HPP
#define WAVE_GEOMETRY_ROTATION_HPP

#include <functional>
#include <iostream>
#include <ostream>

#include <kindr/Core>

#include "wave/utils/utils.hpp"

namespace wave {
/** @addtogroup geometry
 *  @{ */

/**
 * Representation of a 3D rotation.
 *
 * Internally, this class uses the Kindr library's Rotation
 * class to perform elementary operations. For more information,
 * see [Kindr's documentation][kindr_doc].
 *
 * [kindr_doc]: http://docs.leggedrobotics.com/kindr/page_rotations.html
 */
class Rotation {
 private:
    // Internal storage as a kindr Rotation object.
    kindr::RotationMatrixD rotation_object;

 public:
    /** Default constructor, initializes to Identity. */
    Rotation();

    /** Constructs from Euler angles.
     *
     * @param input_vector X-Y-Z Euler angles in radians, such that
     * @f[
     * R = R_z R_y R_x
     * @f]
     * where @f$ R_x, R_y @f$, and @f$ R_z @f$, are rotation matrices
     * constructed using the rotation about @f x, y @f, and @fz@f, respectively.
     */
    explicit Rotation(const Vec3 &input_vector);

    /** Constructs from an angle-axis representation. */
    explicit Rotation(const double angle_magnitude, const Vec3 &rotation_axis);

    /** Sets from Euler angles.
     *
     * @param input_vector X-Y-Z Euler angles; see `Rotation(const Vec3 &)`.
     * @return a reference to `*this`
     */
    Rotation &setFromEulerXYZ(const Vec3 &input_vector);

    /** Sets from angle-axis representation.
     * @return a reference to `*this`
     */
    Rotation &setFromAngleAxis(const double angle_magnitude,
                               const Vec3 &rotation_axis);

    /** Sets from exponential map.
     *
     * Converts from the so(3) Lie algebra, to the SO(3) Lie group using the
     * exponential map
     * @f[
     * R = exp(w_{\times})
     * @f]
     * where @f$ w_{\times} @f$ is the input `se3_vector` represented in
     * skew-symmetric form.
     *
     * @param se3_vector Vector corresponding to the se3 lie algebra for the
     * rotation.
     * @return a reference to `*this`
     */
    Rotation &setFromExpMap(const Vec3 &se3_vector);

    /** Sets from a rotation matrix.
     * @return a reference to `*this`
     */
    Rotation &setFromMatrix(const Mat3 &input_matrix);

    /** Sets to the identity rotation.
     * @return a reference to `*this`
     */
    Rotation &setIdentity();

    /** Returns the se3 Lie algebra parameters for this rotation.
     * @return the vector @f$ w @f$ given by
     * @f[
     * w = log(R)_{\times}
     * @f]
     * where @f$ w_\times @f$ is the skew-symmetric matrix form of the Lie
     * algebra parameters @f$ w \in \mathbb{R}^3 @f$.
     */
    Vec3 logMap() const;

    /** Computes the log map of the input and computes the Jacobian
     * of the log map wrt @f$ R @f$.
     *
     * @param[in] R The input rotation we are taking the log of.
     * @param[out] J_logmap Jacobian of the log map wrt to the input
     * @f$ R @f$.
     */
    static Vec3 logMapAndJacobian(const Rotation &R, Mat3 &J_logmap);

    /** Rotates the input vector by this rotation.
     *
     * @return the vector @f$p_o@f$, the rotated result given by @f[
     * \label{eqn:rot}
     * p_o = R p_i
     * @f]
     * where @f$p_i@f$ is the input vector.
     */
    Vec3 rotate(const Vec3 &input_vector) const;

    /** Rotates the input vector and calculate Jacobians.
     *
     * @param[in] input_vector The vector to be rotated
     * @param[out] Jpoint Jacobian of rotation function with respect to the
     * point input
     * @param[out] Jparam Jacobian of rotation function with respect to this
     * rotation
     * @return the rotated vector as given by `rotate()`.
     */
    Vec3 rotateAndJacobian(const Vec3 &input_vector,
                           Mat3 &Jpoint,
                           Mat3 &Jparam) const;
    /** Rotates the input vector by the inverse of this rotation.
     *
     * @return the vector @f$p_o@f$, given by @f[
     * p_o = R^{-1} p_i
     * @f]
     * where @f$p_i@f$ is the input vector.
     */
    Vec3 inverseRotate(const Vec3 &input_vector) const;

    /** Invert the rotation in place. */
    void invert();

    /** Checks if the input rotation is sufficiently close to this rotation
     *
     * The function computes a disparity rotation @f$R_d@f$ as
     * @f[
     * R_d = R_{other} R^{-1}.
     * @f]
     * If the rotations are (approximately) equal, @f$R_d@f$ will be
     * (approximately) the identity matrix,
     * or equivalently, the log map, @f$ w_d = log(R_d) @f$, will be
     * (approximately) zero.
     *
     * @return true if @f$ ||w_d|| \leq \epsilon @f$,
     * where @f$ \epsilon @f$ is the precision given by `comparison_threshold`.
     */
    bool isNear(const Rotation &other, double comparison_threshold) const;

    /** Calculates the result of the manifold-plus operation @f$ \boxplus @f$.
     *
     * @param omega @f$ \omega \in \mathbb{R}^3 @f$, a vector in the tangent
     * space of @f$ R @f$
     * @return The result, @f$ R = R \boxplus \omega @f$
     * @note the @f$ \boxplus @f$ operator uses the **left** composition with
     * the exponential map.
     * @f[
     * \boxplus: \mathbb{SO}(3) \times \mathbb{R}^3 \mapsto \mathbb{SO}(3) \\
     * R \boxplus \omega = exp(\omega)R
     * @f]
     */
    Rotation &manifoldPlus(const Vec3 &omega);

    /** Calculates the result of the manifold-minus operation @f$ \boxminus @f$.
     *
     * @param R @f$ R \in \mathbb{SO}3 @f$, a rotation
     * @return The result, @f$ \omega = R_t \boxminus R @f$
     * @note the @f$ \boxminus @f$ operator is implemented as follows:
     * @f[
     * \boxminus: \mathbb{SO}(3) \times \mathbb{SO}(3) \mapsto \mathbb{R}^3  \\
     * R_t \boxminus R = log(R_t R^{-1})
     * @f]
     * where @f$ R_t @f$ corresponds to the rotation of **this** object.
     */
    Vec3 manifoldMinus(const Rotation &R) const;

    /** Same as manifoldMinus, except also computes the Jacobians with respect
     * to the left and right arguments of manifoldMinus.  For manifold minus,
     * @f$ \omega = R_t \boxminus R @f$.
     *
     * @param[in] R @f$ R \in \mathbb{SO}3 @f$, a rotation
     * @param[out] J_left The Jacobian of manifoldMinus wrt the left argument,
     * @f$ R_t @f$, which is also **this** rotation.
     * @param[out] J_right The Jacobian of manifoldMinus wrt the right argument,
     * @f$ R @f$, which is also the input rotation.
     * @return The result, @f$ \omega = R_t \boxminus R @f$
     */
    Vec3 manifoldMinusAndJacobian(const Rotation &R,
                                  Mat3 &J_left,
                                  Mat3 &J_right) const;

    /** Composes two rotations and computes the Jacobians wrt the left
     * and right rotations:
     * @f$ R_{out} = R_{left}*R_{right} @f$  Note that **this** rotation
     * corresponds with @f$ R_{left} @f$ and the input rotation is @f$
     * R_{right}@f$.
     * @param[in] R_right @f$ R_{right} \in \mathbb{SO}3 @f$, the input
     * rotation.
     * @param[out] J_left The Jacobian of compose wrt the left argument,
     * @f$ R_{left} @f$ , which is also **this** rotation.
     * @param[out] J_right The Jacobian of compose wrt the right argument,
     * @f$ R_{right} @f$, which is also the input rotation.
     * @return The resulting composition @f$ R_{out} @f$.
     */
    Rotation composeAndJacobian(const Rotation &R_right,
                                Mat3 &J_left,
                                Mat3 &J_right) const;

    /** Compute the inverse of **this** rotation and computes the Jacobian
     * of the inverse mapping wrt **this** rotation.
     * @param[out] J_rotation The Jacobian of the inverse mapping wrt **this**
     * rotation.
     * @return The inverse of **this** rotation.
     */
    Rotation inverseAndJacobian(Mat3 &J_rotation) const;


    /** @return the corresponding 3x3 rotation matrix */
    Mat3 toRotationMatrix() const;

    // Operator overloads.

    /** Implements rotation multiplication. */
    Rotation operator*(const Rotation &R) const;

    /** Implements manifoldMinus using - operator. */
    Vec3 operator-(const Rotation &R) const;
    friend std::ostream &operator<<(std::ostream &stream, const Rotation &R);
};

/** Checks to see if the input matrix is a valid rotation matrix. */
bool isValidRotationMatrix(const Mat3 &input_matrix);

/** @} group geometry */
}  // namespace wave

#endif  // WAVE_GEOMETRY_ROTATION_HPP
