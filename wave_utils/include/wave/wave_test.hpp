/**
 * @file
 * Utility functions used in unit tests
 */

#ifndef WAVE_TEST_HPP
#define WAVE_TEST_HPP

#include <gtest/gtest.h>
#include "wave/utils/math.hpp"

namespace wave {

/** Predicate to check is vectors are approximately equal.
 * Use with EXPECT_PRED2 */
inline bool VectorsNear(const VecX &v1, const VecX &v2) {
    // The absolute comparison of isMuchSmallerThan works better for us than
    // relative comparison of isApprox
    return v1.isApprox(v2);
}

}  // namespace wave

#endif  // WAVE_TEST_HPP
