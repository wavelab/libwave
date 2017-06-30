/** @file
 * @ingroup matching
 *
 * Wrapper of ICP in PCL
 *
 * There are a few parameters that may be changed specific to this algorithm.
 * They can be set in the yaml config file.
 *
 * - max_corr: correspondences behind this many distance-units are
 * discarded
 * - max_iter: Limits number of ICP iterations
 * - t_eps: Criteria to stop iterating. If the difference between consecutive
 * transformations is less than this, stop.
 * - fit_eps: Criteria to stop iterating. If the cost function does not improve
 * by more than this quantity, stop.
 */

#ifndef WAVE_MATCHING_ICP_HPP
#define WAVE_MATCHING_ICP_HPP

#include <pcl/registration/icp.h>

#include "wave/matching/pcl_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {
/** @addtogroup matching
 *  @{ */

class ICPMatcher : public Matcher<PCLPointCloud> {
 public:
    /** This constructor takes an argument in order to adjust how much
     * downsampling is done before matching is attempted. Pointclouds are
     * downsampled using a voxel filter, the argument is the edge length of
     * each voxel. If resolution is non-positive, no downsampling is used.
     */
    explicit ICPMatcher(float resolution, const std::string &config_path);
    ICPMatcher() = delete;
    ~ICPMatcher();
    void setRef(const PCLPointCloud &ref);
    void setTarget(const PCLPointCloud &target);
    bool match();
    void estimateInfo();

    enum covar_method : int { LUM, CENSI };

 private:
    /** An instance of the G-ICP class from PCL */
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    /** An instance of a PCL voxel filter. It is used to downsample input. */
    pcl::VoxelGrid<pcl::PointXYZ> filter;

    /** Pointers to the reference and target pointclouds. The "final" pointcloud
     * is not exposed. PCL's ICP class creates an aligned verison of the target
     * pointcloud after matching, so the "final" member is used as a sink for
     * it. */
    PCLPointCloud ref, target, final;
    covar_method estimate_method;
    double lidar_ang_covar, lidar_lin_covar;
    void estimateLUM();
    void estimateCensi();
};

/** @} group matching */
}  // namespace wave

#endif  // WAVE_MATCHING_ICP_HPP
