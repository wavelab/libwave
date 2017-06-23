/**
 * @file
 * @ingroup optimization
 * Utility functions for template metaprogramming
 */

#ifndef WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP
#define WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP


namespace wave {
/** @addtogroup optimization
 *  @{ */

namespace tmp {

/**
 * Directly replace a type.
 *
 * Meant to be used to make parameter packs of a single  type, of the same
 * number of arguments as another type parameter pack.
 *
 * For example, `replace<int, T>...` becomes `int...` of the same length.
 */
template <typename To, typename From>
using replace = To;

}  // namespace tmp
/** @} group optimization */
}  // namespace wave

#endif  // WAVE_OPTIMIZATION_FACTOR_GRAPH_TEMPLATE_HELPERS_HPP
