//
// Wrapper for the exhaustive version of the 3d scan matcher used in cartographer
// Only use this for testing with small pointclouds. Too slow otherwise
//

#ifndef WAVE_MATCHING_CORRELATION_HPP
#define WAVE_MATCHING_CORRELATION_HPP

#include <cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h>
#include <cartographer/mapping_3d/scan_matching/real_time_correlative_scan_matcher.h>
#include <cartographer/mapping_3d/hybrid_grid.h>
#include <cartographer/sensor/point_cloud.h>
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"
#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "wave/matching/matcher.hpp"

namespace wave {
namespace matching {

typedef cartographer::sensor::PointCloud CartPointCloud;
class Correlation_Matcher : public wave::matching::Matcher<CartPointCloud> {
  public:
    explicit Correlation_Matcher(float resolution);
    void setRef(const CartPointCloud& ref);
    void setTarget(const CartPointCloud& target);
    bool match();
  private:
    cartographer::mapping_3d::HybridGrid hybrid_grid;
    cartographer::transform::Rigid3d initial_pose, result_pose;
    CartPointCloud target;
    std::unique_ptr<cartographer::mapping_3d::scan_matching::RealTimeCorrelativeScanMatcher>
            real_time_correlative_scan_matcher;
};
}
}

#endif //WAVE_MATCHING_CORRELATION_HPP
