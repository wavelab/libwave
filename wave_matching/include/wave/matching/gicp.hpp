/** @file
 * @ingroup matching
 *
 * Wrapper of ICP in PCL
 *
 * There are a few parameters that may be changed specific to this algorithm.
 * They can be set in the yaml config file.
 *
 * - corr_rand: nearest neighbour correspondences used to calculate
 * distributions
 * - max_iter: Limits number of ICP iterations
 * - t_eps: Criteria to stop iterating. If the difference between consecutive
 * transformations is less than this, stop.
 * - fit_eps: Criteria to stop iterating. If the cost function does not improve
 * by more than this quantity, stop.
 */

#ifndef WAVE_MATCHING_GICP_HPP
#define WAVE_MATCHING_GICP_HPP

#include <pcl/registration/gicp.h>

#include "wave/matching/pcl_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {
/** @addtogroup matching
 *  @{ */

struct GICPMatcherParams {
    GICPMatcherParams(const std::string &config_path);
    GICPMatcherParams() {}

    int corr_rand = 10;
    int max_iter = 100;
    double r_eps = 1e-8;
    double fit_eps = 1e-2;
    float res = 0.1;
};

class GICPMatcher : public Matcher<PCLPointCloudPtr> {
 public:
    explicit GICPMatcher(GICPMatcherParams params1);

    /** sets the reference pointcloud for the matcher
     * @param ref - Pointcloud
     */
    void setRef(const PCLPointCloudPtr &ref);

    /** sets the target (or scene) pointcloud for the matcher
     * @param targer - Pointcloud
     */
    void setTarget(const PCLPointCloudPtr &target);

    /** runs the matcher, blocks until finished.
     * Returns true if successful
     */
    bool match();

 private:
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    PCLPointCloudPtr ref, target, final;
    GICPMatcherParams params;
};

/** @} group matching */
}  // namespace wave

#endif  // WAVE_MATCHING_ICP_HPP
