#include "wave/matching/ground_segmentation.hpp"
#include "wave/matching/impl/ground_segmentation.hpp"

#ifndef PCL_NO_PRECOMPILE
// Precompile our filter for common PCL point types
// See http://pointclouds.org/documentation/tutorials/writing_new_classes.php
#include <pcl/impl/instantiate.hpp>
PCL_INSTANTIATE(GroundSegmentation, PCL_XYZ_POINT_TYPES);
#endif  // PCL_NO_PRECOMPILE
