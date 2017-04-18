#ifndef WAVE_MATCHING_NDT_HPP
#define WAVE_MATCHING_NDT_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include "wave/matching/matcher.hpp"
#include "wave/matching/pcl_common.hpp"

namespace wave {

class NDTMatcher : public Matcher<PCLPointCloud> {
 public:
    explicit NDTMatcher(float resolution, const std::string &config_path);
    void setRef(const PCLPointCloud &ref);
    void setTarget(const PCLPointCloud &target);
    bool match();

 private:
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    PCLPointCloud ref, target, final;
    const float min_res = 0.05f;
};

}  // end of namespace wave

#endif  // WAVE_MATCHING_ICP_HPP
