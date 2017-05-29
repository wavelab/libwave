/**
 * @file
 * @ingroup optimization
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORVARIABLEBASE_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORVARIABLEBASE_HPP

namespace wave {
/** @addtogroup optimization
 *  @{ */

/**  Abstract base for all factor graph variables. */
class FactorVariableBase {
 public:
    virtual ~FactorVariableBase() = default;

    /** Print representation of the object for debugging.
     * Called by `operator<<`. */
    virtual void print(std::ostream &os) const = 0;
};

inline std::ostream &operator<<(std::ostream &os, const FactorVariableBase &v) {
    v.print(os);
    return os;
}

/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_FACTORVARIABLEBASE_HPP
