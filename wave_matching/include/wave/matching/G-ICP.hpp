// Class to wrap PCL ICP matcher

#ifndef WAVE_MATCHING_GICP_HPP
#define WAVE_MATCHING_GICP_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include "wave/matching/matcher.hpp"

namespace wave {
namespace matching {

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloud;
class GICP_Matcher : public wave::matching::Matcher<PCLPointCloud> {
  public:
    explicit GICP_Matcher(float resolution);
    void setRef(const PCLPointCloud& ref);
    void setTarget(const PCLPointCloud& target);
    bool match();
  private:
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    PCLPointCloud ref, target, final;
};
}
}

#endif //WAVE_MATCHING_ICP_HPP
