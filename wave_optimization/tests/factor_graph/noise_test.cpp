#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/Noise.hpp"

namespace wave {

struct NoiseTest : public ::testing::Test {
    // Define a sample composed value (use aliases for sized FactorValue)
    template <typename T, typename O = void>
    struct Composed
      : ComposedValue<Composed<T, O>, FactorValue1, FactorValue2> {
        using Base = ComposedValue<Composed<T, O>, FactorValue1, FactorValue2>;
        using Base::Base;
        Composed() = default;
        // The copy constructor must not copy the reference members
        Composed(const Composed &rhs) : Base{rhs} {}
        Ref<FactorValue<T, O, 1>> b0 = this->template blockAtIndex<0>();
        Ref<FactorValue<T, O, 2>> b1 = this->template blockAtIndex<1>();
    };
};


TEST_F(NoiseTest, diagonalNoise) {
    auto stddev = Composed<double>{1.1, {2.2, 3.3}};
    Mat3 expected_cov = Vec3{1.1 * 1.1, 2.2 * 2.2, 3.3 * 3.3}.asDiagonal();
    Mat3 expected_inv = Vec3{1 / 1.1, 1 / 2.2, 1 / 3.3}.asDiagonal();

    auto n = DiagonalNoise<Composed>{stddev};

    EXPECT_PRED2(MatricesNear, expected_cov, n.covariance());

    EXPECT_PRED2(MatricesNear, expected_inv, n.inverseSqrtCov());
}

TEST_F(NoiseTest, diagonalNoiseFromVector) {
    const auto stddev = Vec3{1.1, 2.2, 3.3};
    Mat3 expected_cov = Vec3{1.1 * 1.1, 2.2 * 2.2, 3.3 * 3.3}.asDiagonal();
    Mat3 expected_inv = Vec3{1 / 1.1, 1 / 2.2, 1 / 3.3}.asDiagonal();

    auto n = DiagonalNoise<Composed>{stddev};

    EXPECT_PRED2(MatricesNear, expected_cov, n.covariance());
    EXPECT_PRED2(MatricesNear, expected_inv, n.inverseSqrtCov());
}

TEST_F(NoiseTest, singleNoise) {
    auto stddev = 1.1;

    auto n = DiagonalNoise<FactorValue1>{stddev};
    EXPECT_DOUBLE_EQ(stddev * stddev, n.covariance()[0]);
    EXPECT_DOUBLE_EQ(1. / stddev, n.inverseSqrtCov()[0]);
}

TEST_F(NoiseTest, fullNoise) {
    FullNoise<Composed>::MatrixType cov = Mat3::Ones();
    cov(0, 0) = 3.3;
    cov(1, 1) = 4.4;
    cov(2, 2) = 5.5;

    auto n = FullNoise<Composed>{cov};

    // expected result calculated separately, using `inv(chol(cov, 'lower'))`
    Mat3 expected_inv;
    expected_inv << 0.550481882563180, 0, 0,     //
      -0.149711335415677, 0.494047406871736, 0,  //
      -0.111593441609686, -0.0754896810889053, 0.443748038400870;

    EXPECT_PRED2(MatricesNear, cov, n.covariance());
    EXPECT_PRED2(MatricesNear, expected_inv, n.inverseSqrtCov());
}

}  // namespace wave
