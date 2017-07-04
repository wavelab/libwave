/**
 * @file
 * Utility functions used in unit tests
 */

#ifndef WAVE_TEST_HPP
#define WAVE_TEST_HPP

#include <gtest/gtest.h>
#include "wave/utils/math.hpp"

namespace wave {

/** Predicate to check if vectors are approximately equal.
 * Use with EXPECT_PRED2 */
inline bool VectorsNear(const VecX &v1, const VecX &v2) {
    return v1.isApprox(v2);
}

/** Predicate to check if matrices are approximately equal.
 * Use with EXPECT_PRED2 */
inline bool MatricesNear(const MatX &m1, const MatX &m2) {
    return m1.isApprox(m2);
}

/** Predicate to check if vectors are equal within the given precision.
 * Use with EXPECT_PRED3
 * @note The comparison is multiplicative, and prec is not a linear tolerance.
 * See the documentation for Eigen's `isApprox()`.
 */
inline bool VectorsNearWithPrec(const VecX &v1, const VecX &v2, double prec) {
    return v1.isApprox(v2, prec);
}

/** Predicate to check if matrices are equal within the given precision.
 * Use with EXPECT_PRED3
 * @note The comparison is multiplicative, and prec is not a linear tolerance.
 * See the documentation for Eigen's `isApprox()`.
 */
inline bool MatricesNearWithPrec(const MatX &m1, const MatX &m2, double prec) {
    return m1.isApprox(m2, prec);
}

}  // namespace wave

#endif  // WAVE_TEST_HPP
