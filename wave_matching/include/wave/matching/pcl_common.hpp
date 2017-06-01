/** @file
 * @ingroup matching
 */

#ifndef WAVE_PCL_COMMON_HPP
#define WAVE_PCL_COMMON_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

namespace wave {
/** @addtogroup matching
 *  @{ */

/* This type definition is used as a shorthand for the pointcloud object type
 * used
 * by the scan matching implementations in PCL. A number of those
 * implementations are
 * wrapped, so they use the same datatype for the pointcloud.
 */
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLPointCloud;

/** @} group matching */
}  // namespace wave

#endif  // WAVE_PCL_COMMON_HPP
