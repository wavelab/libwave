#ifndef WAVE_PCL_COMMON_HPP
#define WAVE_PCL_COMMON_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

namespace wave {

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloud;

}  // end of namespace wave
#endif  // WAVE_PCL_COMMON_HPP
