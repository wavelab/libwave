// Class to wrap PCL ICP matcher

#ifndef WAVE_MATCHING_GICP_HPP
#define WAVE_MATCHING_GICP_HPP

#include <pcl/registration/gicp.h>
#include "wave/matching/pcl_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {

class GICPMatcher : public Matcher<PCLPointCloud> {
 public:
    explicit GICPMatcher(float resolution);
    void setRef(const PCLPointCloud &ref);
    void setTarget(const PCLPointCloud &target);
    bool match();

 private:
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    PCLPointCloud ref, target, final;
};

}  // end of wave namespace

#endif  // WAVE_MATCHING_ICP_HPP
