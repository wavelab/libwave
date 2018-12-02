/** @file
 * @ingroup geometry
 */

#ifndef WAVE_GEOMETRY_EXCEPTION_HELPERS_HPP
#define WAVE_GEOMETRY_EXCEPTION_HELPERS_HPP

#include "wave/utils/math.hpp"

namespace wave {
/** @addtogroup geometry
 *  @{ */

/** Checks that all elements are finite (not NaN or +/-INF)
 *
 * @throws std::invalid_argument if any element is NaN or +/-INF
 */
template <typename MatrixType>
inline void checkMatrixFinite(const Eigen::MatrixBase<MatrixType> &mat) {
    if (!mat.allFinite()) {
        throw std::invalid_argument(
                "Array input contains non-finite elements.");
    }
}

/** Checks that scalar value is finite (not NaN or +/-INF)
 *
 * @throw std::invalid_argument if `input_scalar` is NaN or +/-INF
 */
template <typename Scalar>
inline void checkScalarFinite(Scalar input_scalar) {
    if (!std::isfinite(input_scalar)) {
        throw std::invalid_argument("Scalar input is non-finite.");
    }
}

/** Checks if the input vector is normalized.
 *
 * To be normalized, the norm must be within machine epsilon of 1.
 *
 * @throws std::invalid_argument if input is not normalized
 */
template <typename MatrixType>
inline void checkVectorNormalized(
        const Eigen::MatrixBase<MatrixType> &input_vector) {
    // fltcmp returns 0 if equal.
    if (fltcmp(input_vector.norm(),
               1.0,
               sqrt(std::numeric_limits<double>::epsilon()))) {
        throw std::invalid_argument("Vector input is not normalized.");
    }
}

/** Checks if a matrix is a valid rotation matrix.
 *
 * A matrix `R` is valid if:
 * 1. R*inv(R) == I,
 * 2. det(R) == 1;
 *
 * @returns true if valid
 */
inline bool isValidRotationMatrix(const Mat3 &R) {
    Mat3 eye3;
    eye3.setIdentity();
    Mat3 ortho_matrix = R * R.inverse();
    // fltcmp returns 0 if equal.
    if (ortho_matrix.isApprox(eye3) &&
        !fltcmp(
                R.determinant(), 1.0, sqrt(std::numeric_limits<double>::epsilon())))
        return true;
    else
        return false;
}

/** Checks if the input matrix is a valid rotation.
 *
 * @throws std::invalid_argument if `isValidRotationMatrix(R)` returns false
 */
inline void checkValidRotation(const Mat3 &R) {
    if (!isValidRotationMatrix(R)) {
        throw std::invalid_argument(
                "Matrix input is not a valid rotation matrix.");
    }
}

/** @} group geometry */
}  // namespace wave

#endif  // WAVE_GEOMETRY_EXCEPTION_HELPERS_HPP