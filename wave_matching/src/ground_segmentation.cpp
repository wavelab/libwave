#include <pcl/impl/instantiate.hpp>

#include "wave/matching/ground_segmentation.hpp"
#include "wave/matching/impl/ground_segmentation.hpp"

namespace wave {
// Precompile our filter for common PCL point types
// See http://pointclouds.org/documentation/tutorials/writing_new_classes.php
PCL_INSTANTIATE(GroundSegmentation, PCL_XYZ_POINT_TYPES);

}  // namespace wave
