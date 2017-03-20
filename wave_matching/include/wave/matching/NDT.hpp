// Class to wrap PCL ICP matcher

#ifndef WAVE_MATCHING_NDT_HPP
#define WAVE_MATCHING_NDT_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include "wave/matching/matcher.hpp"

namespace wave {
namespace matching {

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloud;
class NDT_Matcher : public wave::matching::Matcher<PCLPointCloud> {
  public:
    explicit NDT_Matcher(float resolution);
    void setRef(const PCLPointCloud& ref);
    void setTarget(const PCLPointCloud& target);
    bool match();
  private:
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> filter;
    PCLPointCloud ref, target, final;
};
}
}

#endif //WAVE_MATCHING_ICP_HPP
