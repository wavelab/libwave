// Class to wrap PCL ICP matcher

#ifndef WAVE_MATCHING_ICP_HPP
#define WAVE_MATCHING_ICP_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include "wave/matching/matcher.hpp"

namespace wave {

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloud;
class ICPMatcher : public Matcher<PCLPointCloud> {
 public:
    explicit ICPMatcher(float resolution);
    void setRef(const PCLPointCloud &ref);
    void setTarget(const PCLPointCloud &target);
    bool match();

 private:
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    PCLPointCloud ref, target, final;
};

}  // end of wave namespace

#endif  // WAVE_MATCHING_ICP_HPP
