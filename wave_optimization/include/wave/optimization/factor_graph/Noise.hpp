/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_NOISE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_NOISE_HPP

#include <iostream>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

template <typename T>
struct traits {};

/**
 * Gaussian noise with full covariance matrix
 * @tparam S dimension of the value
 */
template <int S>
class FullNoise {
    using SquareMat = Eigen::Matrix<double, S, S>;

 public:
    /** The type accepted by the constructor */
    using InitType = SquareMat;

    /** Construct with the given covariance matrix */
    explicit FullNoise(InitType cov)
        : covariance_mat{cov},
          // Pre-calculate inverse sqrt covariance, used in normalization
          inverse_sqrt_cov{SquareMat{cov.llt().matrixL()}.inverse()} {}

    Eigen::Matrix<double, S, S> inverseSqrtCov() const {
        return this->inverse_sqrt_cov;
    };

    Eigen::Matrix<double, S, S> covariance() const {
        return this->covariance_mat;
    };

 private:
    const SquareMat covariance_mat;
    const SquareMat inverse_sqrt_cov;
};

/**
 * Gaussian noise with diagonal covariance matrix
 * @tparam S dimension of the value
 */
template <int S>
class DiagonalNoise {
    using DiagonalMat = Eigen::DiagonalMatrix<double, S>;

 public:
    /** The type accepted by the constructor */
    using InitType = Eigen::Matrix<double, S, 1>;

    /** Construct with the given vector of standard devations (sigmas) */
    explicit DiagonalNoise(const InitType &stddev)
        : covariance_mat{stddev.cwiseProduct(stddev)},
          // Pre-calculate inverse sqrt covariance, used in normalization
          inverse_sqrt_cov{stddev.cwiseInverse()} {}

    DiagonalMat covariance() const {
        return this->covariance_mat;
    };

    DiagonalMat inverseSqrtCov() const {
        return this->inverse_sqrt_cov;
    };

 private:
    const DiagonalMat covariance_mat;
    const DiagonalMat inverse_sqrt_cov;
};

/**
 * Special case of noise for a single value
 */
template <>
class DiagonalNoise<1> {
 public:
    /** The type accepted by the constructor */
    using InitType = double;

    /** Construct with the given standard devations (sigma) */
    explicit DiagonalNoise<1>(double stddev) : stddev{stddev} {}

    double covariance() const {
        return this->stddev * this->stddev;
    };

    double inverseSqrtCov() const {
        return 1. / this->stddev;
    }

 private:
    double stddev;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_NOISE_HPP
