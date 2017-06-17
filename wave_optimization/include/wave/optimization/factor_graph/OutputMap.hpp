/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_OUTPUTMAP_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_OUTPUTMAP_HPP

#include <Eigen/Dense>

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**
 * Specialization of Eigen::Map type with convenient validity checking.
 *
 * Used for output parameters of `Factor::evaluate()`. This object can be
 * checked as a bool to determine if the map is valid, and whether output should
 * be calculated.
 */
template <typename T>
class OutputMap : public Eigen::Map<T> {
 public:
    // Inherit constructors from base class
    using Eigen::Map<T>::Map;

    inline explicit operator bool() const noexcept {
        return this->data() != nullptr;
    }

    template <typename OtherDerived>
    OutputMap &operator=(const Eigen::MatrixBase<OtherDerived> &other) {
        this->Eigen::Map<T>::operator=(other);
        return *this;
    }
};

/** Type of jacobian output parameters for `Factor::evaluate()`. */
template <int Rows, int Cols>
using JacobianOut = OutputMap<Eigen::Matrix<double, Rows, Cols>>;

/** Type of measurement output parameter for `Factor::evaluate()`. */
template <int Size>
using ResultOut = OutputMap<Eigen::Matrix<double, Size, 1>>;

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_OUTPUTMAP_HPP
