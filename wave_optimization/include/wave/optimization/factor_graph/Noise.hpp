/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_NOISE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_NOISE_HPP

#include <iostream>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"
#include "wave/optimization/factor_graph/ComposedValue.hpp"


namespace wave {
/** @addtogroup optimization
 *  @{ */

template <typename T>
struct traits {};

/**
 * Gaussian noise with full covariance matrix
 * @tparam V FactorValue or ComposedValue template
 */
template <template <typename...> class V>
class FullNoise {
 public:
    using ValueType = V<double, FactorValueOptions::Square>;
    using MatrixType =
      typename internal::factor_value_traits<ValueType>::MatrixType;
    constexpr static int Size = ValueType::Size;

    /** Construct with the given covariance matrix */
    explicit FullNoise(MatrixType cov)
        : covariance_mat{cov},
          // Pre-calculate inverse sqrt covariance, used in normalization
          inverse_sqrt_cov{MatrixType{cov.llt().matrixL()}.inverse()} {}

    ValueType inverseSqrtCov() const {
        return this->inverse_sqrt_cov;
    };

    ValueType covariance() const {
        return this->covariance_mat;
    };

 private:
    const ValueType covariance_mat;
    const ValueType inverse_sqrt_cov;
};

/**
 * Gaussian noise with diagonal covariance matrix
 * @tparam V FactorValue or ComposedValue template
 */
template <template <typename...> class V>
class DiagonalNoise {
 public:
    using ValueType = V<double, void>;
    using MatrixType =
      typename internal::factor_value_traits<ValueType>::MatrixType;
    using SquareValueType = V<double, FactorValueOptions::Square>;
    constexpr static int Size = ValueType::Size;

    /** Construct with the given vector of standard devations (sigmas) */
    explicit DiagonalNoise(const MatrixType &stddev)
        : covariance_mat{stddev.cwiseProduct(stddev)},
          // Pre-calculate inverse sqrt covariance, used in normalization
          inverse_sqrt_cov{stddev.cwiseInverse()} {}

    /** Construct with the given value, holding standard deviations*/
    explicit DiagonalNoise(const ValueType &v) : DiagonalNoise{v.matrix()} {}

    /** Construct from double (for value of size 1 only) */
    explicit DiagonalNoise(double stddev)
        : DiagonalNoise{Eigen::Matrix<double, 1, 1>{stddev}} {}


    SquareValueType covariance() const {
        return SquareValueType{this->covariance_mat.asDiagonal()};
    }

    SquareValueType inverseSqrtCov() const {
        return SquareValueType{this->inverse_sqrt_cov.asDiagonal()};
    }

 private:
    const MatrixType covariance_mat;
    const MatrixType inverse_sqrt_cov;
};

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_NOISE_HPP
