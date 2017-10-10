#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "wave/geometry/numerical_test_functors.hpp"
#include "wave/geometry/transformation.hpp"
#include "wave/utils/math.hpp"
#include "wave/utils/utils.hpp"

namespace wave {

class TransformationTestFixture : public ::testing::Test {
 public:
    Transformation transform_expected;

    double comparison_threshold = 1e-5;

    void SetUp() {
        Mat4 transform_matrix;
        transform_matrix << 0.936293363584199, -0.275095847318244,
                0.218350663146334, 1, 0.289629477625516, 0.956425085849232,
                -0.036957013524625, 2, -0.198669330795061, 0.097843395007256,
                0.975170327201816, 3, 0, 0, 0, 1;
        this->transform_expected.setFromMatrix(transform_matrix);
    }
};

// Test that fromEulerXYZ returns the correct matrix.
TEST_F(TransformationTestFixture, testFromEulerXYZ) {
    Transformation transformation(Vec3(0.1, 0.2, 0.3), Vec3(1, 2, 3));

    ASSERT_TRUE(this->transform_expected.isNear(transformation, this->comparison_threshold));
}

}