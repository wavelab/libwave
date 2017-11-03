/** @file
 * @ingroup matching
 *
 * Wrapper of NDT in PCL.
 *
 * There are a few parameters that may be changed specific to this algorithm.
 * They can be set in the yaml config file. The path to this file should be
 * specified in the constructor for the class. A sample yaml file can be found
 * in the wave_matching/config directory.
 *
 * - step_size: Maximum Newton step size used in line search
 * - max_iter: Limits number of iterations
 * - t_eps: Criteria to stop iterating. If the difference between consecutive
 * transformations is less than this, stop.
 * - default_res: If the contructor is given an invalid resolution (too fine or
 * negative), use this resolution
 */

#ifndef WAVE_MATCHING_NDT_HPP
#define WAVE_MATCHING_NDT_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>

#include "wave/matching/matcher.hpp"
#include "wave/matching/pcl_common.hpp"

namespace wave {
/** @addtogroup matching
 *  @{ */

struct NDTMatcherParams {
    NDTMatcherParams(){};
    NDTMatcherParams(const std::string &config_path);

    int step_size = 3;
    int max_iter = 100;
    double t_eps = 1e-8;
    float res = 5;
    const float min_res = 0.05f;
};

class NDTMatcher : public Matcher<PCLPointCloudPtr> {
 public:
    /** This constructor takes an argument in order to specify resolution. The
     * resolution given takes precedence over the one in the config file. If
     * both resolutions are finer than the `min_res` class member, the
     * resolution is set to `min_res.`
     */
    explicit NDTMatcher(NDTMatcherParams params1);
    ~NDTMatcher();

    /** sets the reference pointcloud for the matcher
     * @param ref - Pointcloud
     */
    void setRef(const PCLPointCloudPtr &ref);

    /** sets the target (or scene) pointcloud for the matcher
     * @param targer - Pointcloud
     */
    void setTarget(const PCLPointCloudPtr &target);

    /** runs the matcher, blocks until finished.
     * Note that this version of ndt is SLOW
     * Returns true if successful
     */
    bool match();

 private:
    /** An instance of the NDT class from PCL */
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    /** Pointers to the reference and target pointclouds. The "final" pointcloud
     * is not exposed. PCL's NDT class creates an aligned verison of the target
     * pointcloud after matching, so the "final" member is used as a sink for
     * it. */
    PCLPointCloudPtr ref, target, final;
    NDTMatcherParams params;
};

/** @} group matching */
}  // namespace wave

#endif  // WAVE_MATCHING_ICP_HPP
