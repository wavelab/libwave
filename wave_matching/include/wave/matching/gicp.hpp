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

class GICPMatcher : public Matcher<PCLPointCloud> {
 public:
    explicit GICPMatcher(float resolution, const std::string &config_path);
    /** hey there
     *
     * Its some function
     * @param ref
     */
    void setRef(const PCLPointCloud &ref);
    void setTarget(const PCLPointCloud &target);
    bool match();

 private:
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    PCLPointCloud ref, target, final;
};

/** @} group matching */
}  // namespace wave

#endif  // WAVE_MATCHING_ICP_HPP
