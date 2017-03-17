//
// Wrapper for the branch-and-bound version of the 3d scan matcher used in cartographer
// This method uses precomputed occupancy grids, so well-suited to multiple matches
// branch depth determines how many times the resolution is halved to generate coarse grids

#ifndef WAVE_MATCHING_FAST_CORRELATION_HPP
#define WAVE_MATCHING_FAST_CORRELATION_HPP

#include <cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h>
#include <cartographer/mapping_3d/laser_fan_inserter.h>
#include "wave/matching/cart_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {
namespace matching {

typedef cartographer::sensor::PointCloud CartPointCloud;
class FastCorrelationMatcher : public wave::matching::Matcher<CartPointCloud> {
  public:
    explicit FastCorrelationMatcher(float resolution);
    void setRef(const CartPointCloud& ref);
    void setTarget(const CartPointCloud& target);
    bool match();
  private:
    cartographer::mapping_3d::HybridGrid hybrid_grid;
    cartographer::transform::Rigid3d initial_pose, result_pose;
    CartPointCloud target;
    std::unique_ptr<cartographer::mapping_3d::scan_matching::FastCorrelativeScanMatcher>
            fast_correlative_scan_matcher;
    std::unique_ptr<cartographer::common::LuaParameterDictionary> dict;
    //some kind of criteria. no docs so who knows.
    const float kMinScore = 0.1f;
};

}  // namespace matching
}  // namespace wave

#endif //WAVE_MATCHING_FAST_CORRELATION_HPP
