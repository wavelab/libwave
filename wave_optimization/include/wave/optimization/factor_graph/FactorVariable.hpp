/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP

#include <iostream>

#include "wave/utils/utils.hpp"
#include "wave/optimization/factor_graph/FactorVariableBase.hpp"

namespace wave {
/** @addtogroup optimization
 *  @{ */

using VariableId = std::size_t;
using Vec1 = Eigen::Matrix<double, 1, 1>;

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

    virtual void print(std::ostream &os) const override;

 protected:
    Eigen::Map<const Eigen::Matrix<double, Size, 1>> map;
};

/**
 * Specialized variable representing a 2D pose.
 *
 * In this example, references are assigned to parts of the underlying data
 * vector, allowing factor functions to operate on clearly named parameters.
 */
struct Pose2DVar : public FactorVariable<3> {
    using FactorVariable<3>::FactorVariable;

    Eigen::Ref<const Vec2> position{this->map.head<2>()};
    Eigen::Ref<const Vec1> orientation{this->map.tail<1>()};
};

/**
 * Specialized variable representing a 2D landmark position.
 *
 * In this example, references are assigned to parts of the underlying data
 * vector, allowing factor functions to operate on clearly named parameters.
 */
struct Landmark2DVar : public FactorVariable<2> {
    using FactorVariable<2>::FactorVariable;

    Eigen::Map<const Vec2> position{this->map.data()};
};


/** @} group optimization */
}  // namespace wave

#include "impl/FactorVariable.hpp"

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_VARIABLE_HPP
