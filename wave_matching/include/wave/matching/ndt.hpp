// Class to wrap PCL ICP matcher

#ifndef WAVE_MATCHING_NDT_HPP
#define WAVE_MATCHING_NDT_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
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
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
    PCLPointCloud ref, target, final;
};

}  // end of namespace wave

#endif  // WAVE_MATCHING_ICP_HPP
