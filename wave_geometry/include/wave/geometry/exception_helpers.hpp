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
    // fltcmp returns 0 if equal.
    if (fltcmp(input_vector.norm(), 1.0, sqrt(std::numeric_limits<double>::epsilon()))) {
        throw std::invalid_argument("Vector input is not normalized.");
    }
}

// Function that checks if a rotation matrix is valid.  Requires that:
// 1) R*inv(R)==I,
// 2) det(R) == 1;
bool isValidRotationMatrix(const Mat3 &R) {

    Mat3 eye3;
    eye3.setIdentity();
    Mat3 ortho_matrix = R * R.inverse();
    // fltcmp returns 0 if equal.
    if (ortho_matrix.isApprox(eye3) && !fltcmp(R.determinant(), 1.0, sqrt(std::numeric_limits<double>::epsilon() ) ))
        return true;
    else
        return false;
}

// Function that checks to see if the input matrix is a valid rotation.
void checkValidRotation(const Mat3 &R) {
    
    if (!isValidRotationMatrix(R)) {
        throw std::invalid_argument(
          "Matrix input is not a valid rotation matrix.");
    }
}
}

#endif
