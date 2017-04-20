#ifndef WAVE_EXCEPTION_HELPERS_HPP
#define WAVE_EXCEPTION_HELPERS_HPP

#include "wave/geometry/rotation.hpp"
#include "wave/utils/math.hpp"

namespace wave {


// This function checks to make sure all elements of the input
// Eigen array have finite values (not Nan or +/- Inf), as these
// values are invalid for rotation geometry.
template <typename MatrixType>
void checkArrayFinite(const Eigen::MatrixBase<MatrixType> &mat) {
    if (!mat.allFinite()) {
        throw std::invalid_argument(
          "Array input contains non-finite elements.");
    }
}

// Function that checks to see if a scalar value is finite,
// (not Nan or +/- Inf), as these values are invalid for
// rotation geometry.
void checkScalarFinite(double input_scalar) {
    if (!std::isfinite(input_scalar)) {
        throw std::invalid_argument("Scalar input is non-finite.");
    }
}

// Function that checks to see if the input vector is normalized.
template <typename MatrixType>
void checkVectorNormalized(const Eigen::MatrixBase<MatrixType> &input_vector) {
    // fltcmp returns 0 on successful comparision.
    if (fltcmp(input_vector.norm(), 1)) {
        throw std::invalid_argument("Vector input is not normalized.");
    }
}

// Function that checks to see if the input matrix is a valid rotation.
void checkValidRotation(const Mat3 &R) {
    // fltcmp returns 0 on successful comparision.
    if (!Rotation::isValidRotationMatrix(R)) {
        throw std::invalid_argument(
          "Matrix input is not a valid rotation matrix.");
    }
}
}

#endif