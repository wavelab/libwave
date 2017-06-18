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
 public:
    explicit FullNoise(Eigen::Matrix<double, S, S> cov) : covariance_mat{cov} {}

    Eigen::Matrix<double, S, S> inverseSqrtCov() const {
        const auto L =
          Eigen::Matrix<double, S, S>{this->covariance_mat.llt().matrixL()};
        return L.inverse();
    };

    Eigen::Matrix<double, S, S> covariance() const {
        return this->covariance_mat;
    };

 private:
    Eigen::Matrix<double, S, S> covariance_mat;
};

/**
 * Gaussian noise with diagonal covariance matrix
 * @tparam S dimension of the value
 */
template <int S>
class DiagonalNoise {
 public:
    // Decide which type must be passed in the constructor
    // Normally it's an Eigen vector, but for size-one values accept a double.
    using InitType = Eigen::Matrix<double, S, 1>;

    explicit DiagonalNoise(const InitType &sigma)
        : covariance_mat{sigma.cwiseProduct(sigma)} {}

    Eigen::DiagonalMatrix<double, S> inverseSqrtCov() const {
        return Eigen::DiagonalMatrix<double, S>{
          this->covariance_mat.diagonal().cwiseSqrt().cwiseInverse()};
    };

    Eigen::DiagonalMatrix<double, S> covariance() const {
        return this->covariance_mat;
    };

 private:
    Eigen::DiagonalMatrix<double, S> covariance_mat;
};

/**
 * Special case of noise for a single value
 */
template <>
class DiagonalNoise<1> {
 public:
    using InitType = double;

    explicit DiagonalNoise<1>(double stddev) : stddev{stddev} {}

    double inverseSqrtCov() const {
        return 1. / this->stddev;
    }

    double covariance() const {
        return this->stddev * this->stddev;
    };

 private:
    double stddev;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_NOISE_HPP
