/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP

#include <iostream>

#include "wave/utils/math.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

using VariableId = std::size_t;

/**
 * Representation of a variable in a factor graph.
 *
 * This class is meant to be derived by specialized variable types which assign
 * meaning to the underlying data vector.
 *
 * @todo nicer more intuitive interface
 */
template <int Size>
class FactorVariable : public FactorVariableBase {
 public:
    constexpr static int SizeAtCompileTime = Size;

    /** Default construct variable with no information.
     *
     * The variable's values are meaningless.
     *
     */
    FactorVariable() : map{nullptr} {}

    /** Construct to map the given array.
     *
     * The variable's values are valid only as long as `*data` is valid.
     *
     * @param data pointer to an array of size Size.
     */
    explicit FactorVariable(const double *data) : map{data} {}

    /** Return a pointer to the start of the internal vector.
     *
     * It may be invalid.
     */
    inline const double *data() const {
        return map.data();
    }

    void print(std::ostream &os) const override;

 protected:
    Eigen::Map<const Eigen::Matrix<double, Size, 1>> map;
};

/** @} group optimization */
}  // namespace wave

#include "impl/FactorVariable.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
