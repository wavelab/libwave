#include "wave/wave_test.hpp"
#include "wave/optimization/factor_graph/Noise.hpp"

namespace wave {

TEST(NoiseTest, diagonalNoise) {
    const DiagonalNoise<2>::InitType stddev = Vec2{1.1, 2.2};
    Eigen::Matrix2d expected_cov, expected_inv;
    expected_cov << 1.1 * 1.1, 0, 0, 2.2 * 2.2;
    expected_inv << 1 / 1.1, 0, 0, 1 / 2.2;

    auto n = DiagonalNoise<2>{stddev};

    auto res = Eigen::MatrixXd{n.covariance()};
    EXPECT_PRED2(MatricesNear, expected_cov, res);

    res = Eigen::MatrixXd{n.inverseSqrtCov()};
    EXPECT_PRED2(MatricesNear, expected_inv, res);
}

TEST(NoiseTest, singleNoise) {
    const DiagonalNoise<1>::InitType stddev = 1.1;

    auto n = DiagonalNoise<1>{stddev};
    EXPECT_DOUBLE_EQ(stddev * stddev, n.covariance());
    EXPECT_DOUBLE_EQ(1. / stddev, n.inverseSqrtCov());
}

TEST(NoiseTest, fullNoise) {
    FullNoise<2>::InitType cov = Mat2{};
    cov << 3.3, 1.1, 1.1, 4.4;

    auto n = FullNoise<2>{cov};

    // inverse of cholesky of cov calculated separately
    Mat2 expected_inv;
    expected_inv << 0.550481882563180, 0, -0.165976532577323, 0.497929597731969;

    auto res = Eigen::MatrixXd{n.covariance()};
    EXPECT_PRED2(MatricesNear, cov, res);

    res = Eigen::MatrixXd{n.inverseSqrtCov()};
    EXPECT_PRED2(MatricesNear, expected_inv, res);
}

}  // namespace wave
