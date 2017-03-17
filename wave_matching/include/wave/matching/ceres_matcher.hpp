//
// Wrapper for the exhaustive version of the 3d scan matcher used in cartographer
// Only use this for testing with small pointclouds. Too slow to use practically
//

#ifndef WAVE_MATCHING_CORRELATION_HPP
#define WAVE_MATCHING_CORRELATION_HPP

#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"
#include "wave/matching/cart_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {
namespace matching {

typedef cartographer::sensor::PointCloud CartPointCloud;
class CeresMatcher : public wave::matching::Matcher<CartPointCloud> {
  public:
    explicit CeresMatcher(float resolution);
    void setRef(const CartPointCloud& ref);
    void setTarget(const CartPointCloud& target);
    bool match();
    ceres::Solver::Summary summary;
  private:
    cartographer::mapping_3d::HybridGrid hybrid_grid;
    cartographer::transform::Rigid3d initial_pose, result_pose;
    CartPointCloud target;
    std::unique_ptr<cartographer::mapping_3d::scan_matching::CeresScanMatcher>
            ceres_matcher;
    std::unique_ptr<cartographer::common::LuaParameterDictionary> dict;
};

}  // namespace matching
}  // namespace wave

#endif //WAVE_MATCHING_CORRELATION_HPP
