// Class to wrap PCL ICP matcher

#ifndef WAVE_MATCHING_ICP_HPP
#define WAVE_MATCHING_ICP_HPP

#include <pcl/registration/icp.h>
#include "wave/matching/pcl_common.hpp"
#include "wave/matching/matcher.hpp"

namespace wave {

class ICPMatcher : public Matcher<PCLPointCloud> {
    explicit ICPMatcher(float resolution, const std::string& config_path);
    ~ICPMatcher();
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
